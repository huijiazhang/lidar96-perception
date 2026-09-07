#!/bin/bash

# 合并脚本：auto_startup_tabs.sh + auto.sh
# 功能：等待 GNOME 桌面启动完成后，按原 auto.sh 的命令列表打开终端标签页/窗口。

# 等待桌面启动
until pgrep -x "gnome-shell" >/dev/null; do
    sleep 1
done

# 给桌面和终端服务一点初始化时间
sleep 3

commands=(
'sleep 5;  export RUN_AFTER_BASHRC="roscore"'
'sleep 15; export RUN_AFTER_BASHRC="/home/casp/fence_ws/monitor/feeder_status_listener"'
'sleep 15; export RUN_AFTER_BASHRC="/home/casp/fence_ws/monitor/display_planning_path"'
'sleep 15; export RUN_AFTER_BASHRC="rosrun szd rrcmc_pusher"'
'sleep 18; export RUN_AFTER_BASHRC="rosrun szd ycan_pusher"'
'sleep 15; export RUN_AFTER_BASHRC="roslaunch rslidar_sdk start.launch"'
'sleep 15; export RUN_AFTER_BASHRC="roslaunch szd teleop_joy.launch"'
'sleep 15; export RUN_AFTER_BASHRC="roslaunch rbx2_gui_pro rosbridge.launch"'
'sleep 15; export RUN_AFTER_BASHRC="rosrun szd ymonitor"'
'sleep 15; export RUN_AFTER_BASHRC="roslaunch szd object_cluster_pusher.launch"'
'sleep 15; export RUN_AFTER_BASHRC="roslaunch szd ground_filter_pusher.launch"'
'sleep 15; export RUN_AFTER_BASHRC="roslaunch szd map_loader.launch"'
'sleep 15; export RUN_AFTER_BASHRC="roslaunch szd nav_m_matching.launch"'
'sleep 30; export RUN_AFTER_BASHRC="roslaunch lakibeam1 lakibeam1_scan_casp.launch"'
'sleep 15; export RUN_AFTER_BASHRC="rosrun szd longju"'
'sleep 15; export RUN_AFTER_BASHRC="roslaunch szd obstacle_detected.launch"'
'sleep 15; export RUN_AFTER_BASHRC="rosrun szd ymove_pusher"'
'sleep 15; export RUN_AFTER_BASHRC="rosrun szd dyp_485"'
'sleep 15; export RUN_AFTER_BASHRC="/home/casp/fence_ws/feeder_control/fenceTrackingControl"'
'sleep 15; export RUN_AFTER_BASHRC="/home/casp/fence_ws/fence_detection/fence_tracking_gn_ros"'
'sleep 15; export RUN_AFTER_BASHRC="/home/casp/fence_ws/feeder_control/feeder_status_bridge_node"'
'sleep 15; export RUN_AFTER_BASHRC="/home/casp/fence_ws/slip_detection/vehicleSlipDetectorNode"'
#'sleep 15; export RUN_AFTER_BASHRC=" python /home/casp/fence_ws/imagecombined_detections_sender.py"'
'sleep 15; export RUN_AFTER_BASHRC="source /home/casp/robosence_airy/devel/setup.bash && roslaunch rslidar_sdk_airy start.launch"'
'sleep 15; export RUN_AFTER_BASHRC="cd /home/casp/fence_ws/gravity_align && ./gravity_align_yaml_18 _config_file:=default.yaml"'
'sleep 15; export RUN_AFTER_BASHRC="python /home/casp/fence_ws/channel/rectangle_checker_visual_fixed.py"'
#'sleep 15; /home/casp/docker/docker.sh'
'sleep 15; export RUN_AFTER_BASHRC="/home/casp/fence_ws/stacking_control/ycan_status_to_float64_array_node"'
'sleep 15; export RUN_AFTER_BASHRC="/home/casp/jwd/rtk/sinan/scripts/start_gnss_daily.sh"'
'sleep 15; export RUN_AFTER_BASHRC="/home/casp/rtk/sinan/scripts/start_gnss_daily.sh"'
#'sleep 15; export RUN_AFTER_BASHRC="/home/casp/fence_ws/stacking_control/rslidar_airy_dem_volume"'
'sleep 15; export RUN_AFTER_BASHRC="cd /home/casp/fence_ws/tools  && ./auto_record_bag_10min.sh"'
'sleep 15; export RUN_AFTER_BASHRC="roslaunch obstacle_detector charge_navigation.launch"'
'sleep 15; cd /home/casp/docker; export RUN_AFTER_BASHRC="python temp.py"'
'sleep 25; export RUN_AFTER_BASHRC="/home/casp/monitor_lakibeam.sh"'
#'sleep 15; export RUN_AFTER_BASHRC=/home/casp/catkin_yws/src/szd/scripts/robot_yaml_bridge/path_data_manager.py'
# >>> pusher_work_order_loader autostart >>>
'sleep 30; export PUSHER_PROJECT_ROOT="${PUSHER_PROJECT_ROOT:-$HOME/sys}"; export RUN_AFTER_BASHRC="/home/casp/jwd/pusher_work_order_loader/scripts/site_pusher_work_order_loader_ctl.sh start enable_rrcmc_sync:=true"'
'sleep 30; export RUN_AFTER_BASHRC="unset RUN_AFTER_BASHRC; cd /home/casp/fence_obstacle_cpp_ws/src/stop_obstacle_cpp/src && ./run_pipeline.sh"'
# <<< pusher_work_order_loader autostart <<<
)

# 需要打开新窗口的索引（0-based）
new_window_indices=(1 2 3 4)

tab_args=()    # 收集所有标签页
win_cmds=()    # 收集所有独立窗口的命令

for i in "${!commands[@]}"; do
    cmd="${commands[$i]}"
    if [[ " ${new_window_indices[*]} " =~ " $i " ]]; then
        win_cmds+=( "$cmd" )
    else
        tab_args+=( --tab -e "bash -c '$cmd; exec bash'" )
    fi
done

# 一次性开一个窗口，内含全部标签页（不依赖焦点，绝不会散开）
gnome-terminal --geometry=80x24 "${tab_args[@]}"

sleep 2

# 再单独开 4 个新窗口
for cmd in "${win_cmds[@]}"; do
    gnome-terminal --window --geometry=80x24 -- bash -c "$cmd; exec bash"
    sleep 1
done


