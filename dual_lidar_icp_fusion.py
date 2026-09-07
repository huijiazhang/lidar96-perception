#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
dual_lidar_icp_fusion.py

ROS Python version of dualLidarICPFusion.m.

功能：
1) 订阅左雷达点云和右雷达点云
2) 按 MATLAB 程序相同的核心逻辑进行预处理和 point-to-point ICP
3) 估计 right lidar -> left lidar 的外参 T_right_to_left
4) 发布 leftPC_disp、rightPC_aligned，便于 RViz 同时显示校对
5) 按 MATLAB disp 风格打印正矩阵、逆矩阵和 MATLAB affine3d 内部矩阵

兼容性：
- Python 2 / Python 3
- ROS 1 rospy
- 依赖 numpy；推荐安装 scipy，用 cKDTree 加速最近邻搜索

RViz 使用：
- Fixed Frame 设置为左点云 header.frame_id
- 添加 PointCloud2: /dual_lidar/leftPC_disp
- 添加 PointCloud2: /dual_lidar/rightPC_aligned
- 可选添加 PointCloud2: /dual_lidar/merged
"""
from __future__ import print_function, division

import sys
import math
import threading
import traceback

import numpy as np

try:
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
except Exception:
    rospy = None
    PointCloud2 = None
    PointField = None
    Header = None

try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


class Params(object):
    def __init__(self):
        # ===================== 参数区：默认值与 MATLAB 保持一致 =====================
        self.leftTopic = '/rslidar_points'
        self.rightTopic = '/gravity_aligned/points'

        # 发布给 RViz 校对的话题
        self.leftDispTopic = '/dual_lidar/leftPC_disp'
        self.rightAlignedTopic = '/dual_lidar/rightPC_aligned'
        self.mergedTopic = '/dual_lidar/merged'

        # 时间同步容忍度（秒）
        self.maxTimeDiff = 0.08

        # 点云过滤
        self.minRange = 1.0
        self.maxRange = 80.0
        self.zMin = -3.0
        self.zMax = 5.0

        # 自车附近盒状区域去除（按各自雷达坐标系）
        # [x_half, y_half, z_min, z_max]
        self.selfBox = np.array([1.5, 1.2, -2.0, 2.0], dtype=np.float64)

        # 可选 ROI
        self.useROI = False
        self.leftROI = np.array([[-30.0, 30.0], [-30.0, 30.0], [-3.0, 5.0]], dtype=np.float64)
        self.rightROI = np.array([[-30.0, 30.0], [-30.0, 30.0], [-3.0, 5.0]], dtype=np.float64)

        # ICP 降采样分辨率
        self.icpGridStep = 0.20
        self.mergeGridStep = 0.10

        # ICP 参数
        self.maxIterations = 50
        self.tolerance = np.array([0.001, 0.005], dtype=np.float64)  # [Tdiff, Rdiff]
        self.inlierRatio = 0.6
        self.metric = 'pointToPoint'

        # 打印频率
        self.printEveryN = 10
        self.printPrecision = 4  # 模拟 MATLAB 默认 format short 的 disp 风格

        # 初始外参猜测：T_right_to_left，标准机器人 SE3 形式 [R t; 0 0 0 1]
        self.initT_right_to_left = np.eye(4, dtype=np.float64)

        # 处理循环频率
        self.loopPause = 0.02

        # 发布 merged 点云；leftPC_disp/rightPC_aligned 总是发布
        self.publishMerged = True

        # 如果点太多，RViz/网络可能卡；0 表示不限制，保持核心逻辑完整
        self.maxPublishPoints = 0

        # 是否发布一个静态/动态 tf：right_lidar_aligned -> left_frame
        # 本脚本已直接把 rightPC_aligned 发布到 left frame，一般 RViz 不需要 tf。
        self.publishTf = False
        self.rightAlignedFrame = 'right_lidar_aligned'

    def load_from_ros_params(self):
        """允许用 rosparam 覆盖默认参数；没有参数时完全按 MATLAB 默认值运行。"""
        if rospy is None:
            return

        self.leftTopic = rospy.get_param('~left_topic', self.leftTopic)
        self.rightTopic = rospy.get_param('~right_topic', self.rightTopic)
        self.leftDispTopic = rospy.get_param('~left_disp_topic', self.leftDispTopic)
        self.rightAlignedTopic = rospy.get_param('~right_aligned_topic', self.rightAlignedTopic)
        self.mergedTopic = rospy.get_param('~merged_topic', self.mergedTopic)

        self.maxTimeDiff = float(rospy.get_param('~max_time_diff', self.maxTimeDiff))
        self.minRange = float(rospy.get_param('~min_range', self.minRange))
        self.maxRange = float(rospy.get_param('~max_range', self.maxRange))
        self.zMin = float(rospy.get_param('~z_min', self.zMin))
        self.zMax = float(rospy.get_param('~z_max', self.zMax))
        self.selfBox = np.array(rospy.get_param('~self_box', self.selfBox.tolist()), dtype=np.float64)

        self.useROI = bool(rospy.get_param('~use_roi', self.useROI))
        self.leftROI = np.array(rospy.get_param('~left_roi', self.leftROI.tolist()), dtype=np.float64)
        self.rightROI = np.array(rospy.get_param('~right_roi', self.rightROI.tolist()), dtype=np.float64)

        self.icpGridStep = float(rospy.get_param('~icp_grid_step', self.icpGridStep))
        self.mergeGridStep = float(rospy.get_param('~merge_grid_step', self.mergeGridStep))
        self.maxIterations = int(rospy.get_param('~max_iterations', self.maxIterations))
        self.tolerance = np.array(rospy.get_param('~tolerance', self.tolerance.tolist()), dtype=np.float64)
        self.inlierRatio = float(rospy.get_param('~inlier_ratio', self.inlierRatio))
        self.metric = rospy.get_param('~metric', self.metric)
        self.printEveryN = int(rospy.get_param('~print_every_n', self.printEveryN))
        self.printPrecision = int(rospy.get_param('~print_precision', self.printPrecision))
        self.loopPause = float(rospy.get_param('~loop_pause', self.loopPause))
        self.publishMerged = bool(rospy.get_param('~publish_merged', self.publishMerged))
        self.maxPublishPoints = int(rospy.get_param('~max_publish_points', self.maxPublishPoints))
        self.publishTf = bool(rospy.get_param('~publish_tf', self.publishTf))
        self.rightAlignedFrame = rospy.get_param('~right_aligned_frame', self.rightAlignedFrame)

        init_t = rospy.get_param('~init_T_right_to_left', self.initT_right_to_left.tolist())
        init_t = np.array(init_t, dtype=np.float64)
        if init_t.shape == (4, 4):
            self.initT_right_to_left = init_t
        else:
            rospy.logwarn('~init_T_right_to_left shape is not 4x4, using eye(4).')
            self.initT_right_to_left = np.eye(4, dtype=np.float64)


class LatestMsgBuffer(object):
    def __init__(self):
        self._lock = threading.Lock()
        self.left_msg = None
        self.right_msg = None

    def left_cb(self, msg):
        with self._lock:
            self.left_msg = msg

    def right_cb(self, msg):
        with self._lock:
            self.right_msg = msg

    def get(self):
        with self._lock:
            return self.left_msg, self.right_msg


def get_msg_time(msg):
    """将 ROS Header 时间戳转为秒。"""
    return msg.header.stamp.to_sec()


def read_xyz_compat(msg):
    """读取 PointCloud2 的 x/y/z 字段，兼容 organized/unorganized 点云。"""
    import sensor_msgs.point_cloud2 as pc2

    pts = []
    for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False):
        pts.append((p[0], p[1], p[2]))

    if len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float64)

    return np.asarray(pts, dtype=np.float64).reshape((-1, 3))


def preprocess_cloud(xyz, params, roi):
    """
    对应 MATLAB preprocessCloud：
    - 删除 NaN / Inf
    - 距离过滤
    - z 过滤
    - 去掉自车附近盒状区域
    - 可选 ROI
    - 生成显示点云和 ICP 降采样点云
    """
    if xyz is None or xyz.size == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return empty, empty

    xyz = np.asarray(xyz, dtype=np.float64).reshape((-1, 3))

    mask = np.isfinite(xyz).all(axis=1)
    xyz = xyz[mask]

    if xyz.shape[0] == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return empty, empty

    r = np.sqrt(np.sum(xyz * xyz, axis=1))
    mask = np.logical_and(r >= params.minRange, r <= params.maxRange)
    xyz = xyz[mask]

    if xyz.shape[0] == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return empty, empty

    mask = np.logical_and(xyz[:, 2] >= params.zMin, xyz[:, 2] <= params.zMax)
    xyz = xyz[mask]

    if xyz.shape[0] == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return empty, empty

    hx = params.selfBox[0]
    hy = params.selfBox[1]
    z0 = params.selfBox[2]
    z1 = params.selfBox[3]
    self_mask = np.logical_and.reduce((
        np.abs(xyz[:, 0]) <= hx,
        np.abs(xyz[:, 1]) <= hy,
        xyz[:, 2] >= z0,
        xyz[:, 2] <= z1
    ))
    xyz = xyz[np.logical_not(self_mask)]

    if params.useROI:
        mask = np.logical_and.reduce((
            xyz[:, 0] >= roi[0, 0], xyz[:, 0] <= roi[0, 1],
            xyz[:, 1] >= roi[1, 0], xyz[:, 1] <= roi[1, 1],
            xyz[:, 2] >= roi[2, 0], xyz[:, 2] <= roi[2, 1]
        ))
        xyz = xyz[mask]

    if xyz.shape[0] < 10:
        empty = np.zeros((0, 3), dtype=np.float64)
        return empty, empty

    pc_disp = xyz
    pc_icp = voxel_downsample(pc_disp, params.icpGridStep)
    return pc_disp, pc_icp


def voxel_downsample(points, voxel_size):
    """等价于 MATLAB pcdownsample(..., 'gridAverage', gridStep)。"""
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    if points.shape[0] == 0 or voxel_size <= 0:
        return points.copy()

    coords = np.floor(points / float(voxel_size)).astype(np.int64)
    coords = np.ascontiguousarray(coords)

    # Python2 / 老 numpy 兼容写法：不用 np.unique(axis=0)
    dtype = np.dtype((np.void, coords.dtype.itemsize * coords.shape[1]))
    _, inverse = np.unique(coords.view(dtype).ravel(), return_inverse=True)

    counts = np.bincount(inverse).astype(np.float64)
    sums = []
    for k in range(3):
        sums.append(np.bincount(inverse, weights=points[:, k]))
    out = np.vstack(sums).T / counts[:, None]
    return out.astype(np.float64)


def transform_points(points, T):
    """标准机器人 SE3：p_out = R * p_in + t。"""
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    if points.shape[0] == 0:
        return points.copy()
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    return np.dot(points, R.T) + t.reshape((1, 3))


def estimate_rigid_transform(src, dst):
    """
    使用 SVD 估计 src -> dst 的刚体变换。
    src/dst: Nx3，已是一一对应关系。
    """
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)

    src_mean = np.mean(src, axis=0)
    dst_mean = np.mean(dst, axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean

    H = np.dot(src_c.T, dst_c)
    U, _, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = np.dot(Vt.T, U.T)

    t = dst_mean - np.dot(R, src_mean)

    T = np.eye(4, dtype=np.float64)
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    return T


def nearest_neighbors(src, dst):
    """查询 src 中每个点在 dst 中的最近邻。"""
    if cKDTree is not None:
        tree = cKDTree(dst)
        dist, idx = tree.query(src, k=1)
        return dist, idx

    # 无 scipy 时的兼容兜底：分块暴力最近邻，速度较慢但不改变功能。
    block = 2048
    idx_all = np.empty((src.shape[0],), dtype=np.int64)
    dist_all = np.empty((src.shape[0],), dtype=np.float64)
    for i in range(0, src.shape[0], block):
        s = src[i:i + block]
        diff = s[:, None, :] - dst[None, :, :]
        d2 = np.sum(diff * diff, axis=2)
        idx = np.argmin(d2, axis=1)
        idx_all[i:i + block] = idx
        dist_all[i:i + block] = np.sqrt(d2[np.arange(idx.shape[0]), idx])
    return dist_all, idx_all


def rotation_angle(R):
    """旋转矩阵对应的角度，单位 rad。"""
    val = (np.trace(R) - 1.0) / 2.0
    val = max(-1.0, min(1.0, float(val)))
    return math.acos(val)


def pcregistericp_point_to_point(moving, fixed, initial_transform, params):
    """
    近似 MATLAB pcregistericp(..., 'Metric','pointToPoint') 的核心逻辑。
    moving = rightPC_icp, fixed = leftPC_icp
    返回：T_right_to_left, rmse
    """
    moving = np.asarray(moving, dtype=np.float64).reshape((-1, 3))
    fixed = np.asarray(fixed, dtype=np.float64).reshape((-1, 3))

    if moving.shape[0] < 3 or fixed.shape[0] < 3:
        raise RuntimeError('not enough points for ICP')

    T = np.array(initial_transform, dtype=np.float64, copy=True)
    last_rmse = np.inf

    for _ in range(params.maxIterations):
        moved = transform_points(moving, T)
        dist, idx = nearest_neighbors(moved, fixed)

        if dist.shape[0] < 3:
            raise RuntimeError('not enough correspondences for ICP')

        inlier_count = int(math.ceil(params.inlierRatio * dist.shape[0]))
        inlier_count = max(3, min(inlier_count, dist.shape[0]))

        # 取距离最小的一部分作为 inliers，对应 MATLAB InlierRatio 思路
        order = np.argsort(dist)
        keep = order[:inlier_count]

        src_in = moved[keep]
        dst_in = fixed[idx[keep]]

        dT = estimate_rigid_transform(src_in, dst_in)
        T = np.dot(dT, T)

        rmse = float(np.sqrt(np.mean(dist[keep] * dist[keep])))
        tdiff = float(np.linalg.norm(dT[0:3, 3]))
        rdiff = rotation_angle(dT[0:3, 0:3])

        if tdiff < params.tolerance[0] and rdiff < params.tolerance[1]:
            last_rmse = rmse
            break

        if abs(last_rmse - rmse) < 1e-12:
            last_rmse = rmse
            break

        last_rmse = rmse

    return T, last_rmse


def se3_to_matlab_internal(Tse3):
    """
    标准机器人 SE3 转 MATLAB affine3d 行向量内部矩阵：
    MATLAB 代码中 affine3d.T 等价于：
      [ R' 0
        t' 1 ]
    """
    Tm = np.eye(4, dtype=np.float64)
    Tm[0:3, 0:3] = Tse3[0:3, 0:3].T
    Tm[3, 0:3] = Tse3[0:3, 3].T
    return Tm


def merge_point_clouds(left_points, right_aligned_points, grid_step):
    """等价于 MATLAB pcmerge(leftPC_disp, rightPC_aligned, mergeGridStep)。"""
    if left_points.shape[0] == 0:
        return voxel_downsample(right_aligned_points, grid_step)
    if right_aligned_points.shape[0] == 0:
        return voxel_downsample(left_points, grid_step)
    merged = np.vstack((left_points, right_aligned_points))
    return voxel_downsample(merged, grid_step)


def make_header_like(src_msg, frame_id=None):
    header = Header()
    header.stamp = src_msg.header.stamp
    header.frame_id = frame_id if frame_id is not None else src_msg.header.frame_id
    return header


def points_to_cloud_msg(points, header, max_points=0):
    """高效创建只有 x/y/z 字段的 PointCloud2。"""
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))

    if max_points is not None and int(max_points) > 0 and points.shape[0] > int(max_points):
        # 均匀抽样，仅限制发布量；默认 max_points=0 不启用，保持完整显示点云。
        idx = np.linspace(0, points.shape[0] - 1, int(max_points)).astype(np.int64)
        points = points[idx]

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = False
    msg.data = points.tobytes() if hasattr(points, 'tobytes') else points.tostring()
    return msg


def matlab_disp(M, precision=4):
    """模拟 MATLAB disp 矩阵输出，不打印 numpy 的中括号。"""
    M = np.asarray(M, dtype=np.float64)
    fmt = '    ' + ('%12.' + str(int(precision)) + 'f') * M.shape[1]
    for i in range(M.shape[0]):
        print(fmt % tuple(M[i, :]))


def print_matrices(frame_count, rmse, T_right_to_left, print_precision):
    T_left_to_right = np.linalg.inv(T_right_to_left)
    T_matlab = se3_to_matlab_internal(T_right_to_left)

    print('====================================================')
    print('frame = %d, rmse = %.6f' % (frame_count, rmse))

    print('T_right_to_left (standard robotics SE3):')
    matlab_disp(T_right_to_left, print_precision)

    print('T_left_to_right (inverse):')
    matlab_disp(T_left_to_right, print_precision)

    print('MATLAB internal transform matrix (for pctransform):')
    matlab_disp(T_matlab, print_precision)


def maybe_publish_tf(params, left_msg, T_right_to_left):
    """可选发布 tf。默认关闭，因为 rightPC_aligned 已直接发布到 left frame。"""
    if not params.publishTf:
        return
    try:
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        import tf.transformations as tr

        br = maybe_publish_tf.br
    except AttributeError:
        import tf2_ros
        maybe_publish_tf.br = tf2_ros.TransformBroadcaster()
        br = maybe_publish_tf.br
    except Exception as exc:
        rospy.logwarn('publish_tf requested but tf2 is unavailable: %s', str(exc))
        return

    try:
        import tf.transformations as tr
        from geometry_msgs.msg import TransformStamped

        tf_msg = TransformStamped()
        tf_msg.header.stamp = left_msg.header.stamp
        tf_msg.header.frame_id = left_msg.header.frame_id
        tf_msg.child_frame_id = params.rightAlignedFrame
        tf_msg.transform.translation.x = float(T_right_to_left[0, 3])
        tf_msg.transform.translation.y = float(T_right_to_left[1, 3])
        tf_msg.transform.translation.z = float(T_right_to_left[2, 3])
        q = tr.quaternion_from_matrix(T_right_to_left)
        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])
        br.sendTransform(tf_msg)
    except Exception as exc:
        rospy.logwarn('failed to publish tf: %s', str(exc))


def run_node():
    if rospy is None:
        raise RuntimeError('This script must run in a ROS Python environment with rospy installed.')

    rospy.init_node('dual_lidar_icp_fusion', anonymous=False)

    params = Params()
    params.load_from_ros_params()

    if params.metric != 'pointToPoint':
        rospy.logwarn("Only pointToPoint ICP is implemented in this Python version; got metric=%s, using pointToPoint.", params.metric)
        params.metric = 'pointToPoint'

    if cKDTree is None:
        rospy.logwarn('scipy.spatial.cKDTree not found. Falling back to slow brute-force nearest neighbor search.')

    print('dualLidarICPFusion start...')
    print('Subscribing:')
    print('  left : %s' % params.leftTopic)
    print('  right: %s' % params.rightTopic)
    print('Publishing for RViz:')
    print('  leftPC_disp     : %s' % params.leftDispTopic)
    print('  rightPC_aligned : %s' % params.rightAlignedTopic)
    if params.publishMerged:
        print('  merged          : %s' % params.mergedTopic)

    buffer = LatestMsgBuffer()

    rospy.Subscriber(params.leftTopic, PointCloud2, buffer.left_cb, queue_size=1, buff_size=2 ** 24)
    rospy.Subscriber(params.rightTopic, PointCloud2, buffer.right_cb, queue_size=1, buff_size=2 ** 24)

    left_pub = rospy.Publisher(params.leftDispTopic, PointCloud2, queue_size=1)
    right_pub = rospy.Publisher(params.rightAlignedTopic, PointCloud2, queue_size=1)
    merged_pub = rospy.Publisher(params.mergedTopic, PointCloud2, queue_size=1) if params.publishMerged else None

    print('Waiting for point cloud topics...')
    rate_wait = rospy.Rate(10)
    while not rospy.is_shutdown():
        left_msg, right_msg = buffer.get()
        if left_msg is not None and right_msg is not None:
            break
        rate_wait.sleep()

    if rospy.is_shutdown():
        return

    print('Point cloud topics received.')
    print('')
    print('Running... Press Ctrl+C to stop.')
    print('')

    last_left_stamp = -float('inf')
    last_right_stamp = -float('inf')
    frame_count = 0
    prev_T_right_to_left = np.array(params.initT_right_to_left, dtype=np.float64, copy=True)

    rate_hz = 1.0 / params.loopPause if params.loopPause > 0 else 50.0
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        left_msg, right_msg = buffer.get()

        if left_msg is None or right_msg is None:
            rate.sleep()
            continue

        left_stamp = get_msg_time(left_msg)
        right_stamp = get_msg_time(right_msg)

        # 确保是新帧
        if left_stamp <= last_left_stamp or right_stamp <= last_right_stamp:
            rate.sleep()
            continue

        # 粗略时间同步检查
        if abs(left_stamp - right_stamp) > params.maxTimeDiff:
            rate.sleep()
            continue

        last_left_stamp = left_stamp
        last_right_stamp = right_stamp

        try:
            left_xyz = read_xyz_compat(left_msg)
            right_xyz = read_xyz_compat(right_msg)

            leftPC_disp, leftPC_icp = preprocess_cloud(left_xyz, params, params.leftROI)
            rightPC_disp, rightPC_icp = preprocess_cloud(right_xyz, params, params.rightROI)

            if leftPC_icp.shape[0] < 100 or rightPC_icp.shape[0] < 100:
                rospy.logwarn('[WARN] valid points too few, skip.')
                rate.sleep()
                continue

            # ICP 配准：moving = right, fixed = left
            T_right_to_left, rmse = pcregistericp_point_to_point(
                rightPC_icp, leftPC_icp, prev_T_right_to_left, params)
            prev_T_right_to_left = T_right_to_left

            # 用估计外参对齐右雷达显示点云
            rightPC_aligned = transform_points(rightPC_disp, T_right_to_left)
            mergedPC = merge_point_clouds(leftPC_disp, rightPC_aligned, params.mergeGridStep) if params.publishMerged else None

            # RViz 校对关键：两个点云都发布到 left frame
            left_header = make_header_like(left_msg, left_msg.header.frame_id)
            right_header = make_header_like(left_msg, left_msg.header.frame_id)

            left_pub.publish(points_to_cloud_msg(leftPC_disp, left_header, params.maxPublishPoints))
            right_pub.publish(points_to_cloud_msg(rightPC_aligned, right_header, params.maxPublishPoints))

            if params.publishMerged and merged_pub is not None:
                merged_pub.publish(points_to_cloud_msg(mergedPC, left_header, params.maxPublishPoints))

            maybe_publish_tf(params, left_msg, T_right_to_left)

            frame_count += 1
            if frame_count % params.printEveryN == 1:
                print_matrices(frame_count, rmse, T_right_to_left, params.printPrecision)

        except Exception as exc:
            rospy.logwarn('[WARN] ICP failed: %s', str(exc))
            rospy.logdebug(traceback.format_exc())

        rate.sleep()


def main():
    try:
        run_node()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print('[ERROR] %s' % str(exc), file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
