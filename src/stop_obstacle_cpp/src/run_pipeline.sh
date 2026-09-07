#!/usr/bin/env bash
# =============================================================
# run_pipeline.sh
# 96线激光雷达流水线 - 直接使用 /patchwork_refine/no_ground
# 防递归 / 防重复启动版本
#
# 流程：
#   1) patchworkpp ground_filter_pusher.launch
#   2) patchwork_refine_node
#   3) points_obstacle_m_96
#
# 已删除 fenced_lane_points_process_cpp：
#   points_obstacle_m_96 直接订阅 /patchwork_refine/no_ground
# =============================================================

set -u

PATCHWORK_WS="${HOME}/patchwork_ws"
FENCE_WS="${HOME}/fence_obstacle_cpp_ws"
ROS_SETUP="/opt/ros/melodic/setup.bash"
STARTUP_DELAY=15

# 防止车辆 auto.sh 中的 RUN_AFTER_BASHRC 被子终端继承后再次触发。
unset RUN_AFTER_BASHRC
unset RUN_AFTER_BASH

# 防止同一时刻重复执行本脚本。
LOCK_FILE="/tmp/lidar96_run_pipeline.lock"
exec 9>"${LOCK_FILE}"

if ! flock -n 9; then
    echo "[INFO] run_pipeline.sh 已有实例正在启动，本次退出。"
    exit 0
fi

fail()
{
    echo
    echo "[ERROR] $*"
    echo "[ERROR] 流水线未启动，不会创建三个终端。"
    exit 1
}

# -------------------------------------------------------------
# 1. 环境和编译结果检查
# -------------------------------------------------------------
[[ -f "${ROS_SETUP}" ]] \
    || fail "找不到 ${ROS_SETUP}"

[[ -f "${PATCHWORK_WS}/devel/setup.bash" ]] \
    || fail "patchwork_ws 未编译：缺少 ${PATCHWORK_WS}/devel/setup.bash"

[[ -f "${FENCE_WS}/devel/setup.bash" ]] \
    || fail "fence_obstacle_cpp_ws 未编译：缺少 ${FENCE_WS}/devel/setup.bash"

FENCE_BIN_DIR="${FENCE_WS}/devel/lib/stop_obstacle_cpp"

for node in \
    patchwork_refine_node \
    points_obstacle_m_96
do
    [[ -x "${FENCE_BIN_DIR}/${node}" ]] \
        || fail "缺少可执行程序 ${FENCE_BIN_DIR}/${node}，请先 catkin_make"
done

# -------------------------------------------------------------
# 2. 检查 Patchwork 包和 launch
# -------------------------------------------------------------
source "${ROS_SETUP}"
source "${PATCHWORK_WS}/devel/setup.bash"

rospack find patchworkpp >/dev/null 2>&1 \
    || fail "找不到 ROS 包 patchworkpp，请检查 patchwork_ws 是否编译成功"

roslaunch --files patchworkpp ground_filter_pusher.launch >/dev/null 2>&1 \
    || fail "找不到 patchworkpp/ground_filter_pusher.launch"

# -------------------------------------------------------------
# 3. 防止已经运行的流水线再次启动
# -------------------------------------------------------------
check_existing_pipeline()
{
    if rostopic list >/dev/null 2>&1; then
        RUNNING_NODES="$(rosnode list 2>/dev/null || true)"

        if echo "${RUNNING_NODES}" | grep -qE \
            '^/(points_obstacle_m_96_detector|patchwork_refine_node)$'
        then
            echo "[WARN] 检测到96线流水线节点已经运行，本次不重复启动："
            echo "${RUNNING_NODES}" | grep -E \
                '^/(points_obstacle_m_96_detector|patchwork_refine_node)$' \
                || true
            return 0
        fi
    fi

    # ROS Master 短暂异常或节点尚未完成注册时，再从系统进程检查。
    if pgrep -f "${FENCE_BIN_DIR}/patchwork_refine_node" >/dev/null 2>&1 \
       || pgrep -f "${FENCE_BIN_DIR}/points_obstacle_m_96" >/dev/null 2>&1 \
       || pgrep -f "roslaunch patchworkpp ground_filter_pusher.launch" >/dev/null 2>&1
    then
        echo "[WARN] 检测到96线流水线相关进程已经运行，本次不重复启动。"
        return 0
    fi

    return 1
}

if check_existing_pipeline; then
    exit 0
fi

# -------------------------------------------------------------
# 4. 延时启动
# -------------------------------------------------------------
echo "[INFO] 编译检查通过，当前没有重复节点。"
echo "[INFO] ${STARTUP_DELAY} 秒后启动96线流水线..."
sleep "${STARTUP_DELAY}"

# 延时期间可能被其他自启动机制拉起，再检查一次。
if check_existing_pipeline; then
    echo "[WARN] 延时期间流水线已被其他程序启动，本次取消。"
    exit 0
fi

WAIT_MASTER="until rostopic list >/dev/null 2>&1; do sleep 0.5; done"

# -------------------------------------------------------------
# 5. 三个终端
# -------------------------------------------------------------
CMD1="unset RUN_AFTER_BASHRC RUN_AFTER_BASH; source ${ROS_SETUP}; source ${PATCHWORK_WS}/devel/setup.bash; cd ${PATCHWORK_WS}; roslaunch patchworkpp ground_filter_pusher.launch; RC=\$?; echo; echo [EXIT] ground_filter exited code=\${RC}; exec bash --noprofile --norc -i"

CMD2="unset RUN_AFTER_BASHRC RUN_AFTER_BASH; source ${ROS_SETUP}; source ${FENCE_WS}/devel/setup.bash; cd ${FENCE_WS}; ${WAIT_MASTER}; sleep 1; rosrun stop_obstacle_cpp patchwork_refine_node; RC=\$?; echo; echo [EXIT] patchwork_refine_node exited code=\${RC}; exec bash --noprofile --norc -i"

CMD3="unset RUN_AFTER_BASHRC RUN_AFTER_BASH; source ${ROS_SETUP}; source ${FENCE_WS}/devel/setup.bash; cd ${FENCE_WS}; ${WAIT_MASTER}; sleep 2; rosrun stop_obstacle_cpp points_obstacle_m_96; RC=\$?; echo; echo [EXIT] points_obstacle_m_96 exited code=\${RC}; exec bash --noprofile --norc -i"

# env -u 再保险：确保 gnome-terminal-server 本身也拿不到自启动变量。
env -u RUN_AFTER_BASHRC -u RUN_AFTER_BASH \
gnome-terminal \
    --tab --title="1_ground_filter" -e "bash --noprofile --norc -c '${CMD1}'" \
    --tab --title="2_refine"        -e "bash --noprofile --norc -c '${CMD2}'" \
    --tab --title="3_obstacle_m96"  -e "bash --noprofile --norc -c '${CMD3}'"

# 给终端/节点一点时间创建，期间保持 flock 锁。
sleep 3
