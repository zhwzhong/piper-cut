# 中文说明

详细说明已整理到同目录的 `README.md`，包括：

- 获取当前 TCP 坐标和位姿：`02_read_probe_tip_pose.py`
- 移动到某一个 TCP 点：`03_move_to_probe_tip_point.py`
- 移动到某一个 TCP 位姿：`04_move_to_probe_tip_pose.py`
- 恢复 SDK 可控模式：`05_restore_sdk_control_mode.py`
- 粗略扫描 TCP 可达范围：`06_scan_reachability_workspace.py`
- 使用 MoveIt2 `/compute_ik` 扫描 IK 可达范围，并支持探针尽量向下姿态搜索：`07_scan_moveit_ik_reachability.py`
- 将可达范围转换到相机坐标并画圈：`08_project_reachability_to_camera.py`
- 所有脚本参数、输入输出字段和共享函数说明

服务器使用：

```bash
source /home/radxa/box_sdk_env/bin/activate
cd /home/radxa/Desktop/piper_sdk_seam_probe_bundle_20260821
```

真实运动必须显式加确认参数，默认都是 dry-run 或只读检查。
