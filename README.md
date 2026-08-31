# PIPER + Orbbec 纸箱中缝与 TCP 控制工程

本工程是 SDK 版本，不依赖 ROS2/MoveIt2。当前已经包含相机外参标定和机械臂 TCP 标定，核心目标是把流程拆成几个稳定入口：

1. RGB-D 拍照并检测纸箱中缝，输出 `base_link` 下的探针尖端目标点。
2. 从 PIPER SDK 读取当前法兰反馈，并换算成 TCP 尖端坐标和位姿。
3. 给定一个 TCP 点，只移动到该点，姿态可用当前姿态或固定姿态。
4. 给定一个完整 TCP 位姿，移动到该位姿。
5. 从示教/异常状态恢复到 SDK 可控模式。

所有真实运动命令默认关闭。移动脚本不加 `--execute --confirm EXECUTE` 只做 dry-run，不会发送运动。

## 路径和环境

本地项目：

```bash
cd /Users/zhwzhong/Documents/拆快递箱子/piper_sdk_seam_probe_bundle_20260821
```

服务器项目：

```bash
ssh radxa
source /home/radxa/box_sdk_env/bin/activate
cd /home/radxa/Desktop/piper_sdk_seam_probe_bundle_20260821
```

默认硬件和坐标系：

```text
Orbbec Gemini 336L serial: CPC87630008B
PIPER CAN: can0
robot frame: base_link
camera frame: gemini336l_color_optical_frame
公开输出位置单位: meter，同时在关键结果里提供 mm 字段
公开输出姿态单位: degree
SDK EndPoseCtrl 位置单位: 0.001 mm
SDK EndPoseCtrl 姿态单位: 0.001 degree
```

> 说明：本工程内部沿用标定文件的米制单位，命令行手动输入点时可以通过 `--unit mm` 输入毫米。

## 标定文件

主要配置都在 `config/`：

- `runtime_config.yaml`：相机序列号、分辨率、CAN 名称、棋盘格参数和标定角点编号。
- `eye_to_hand_extrinsics.yaml`：相机到机械臂 `base_link` 的眼在手外外参。
- `calibration_bundle.yaml`：相机/棋盘格/机械臂标定结果打包文件。
- `depth_correction.yaml`：深度修正参数。
- `tcp_offset_m_rad.yaml`：TCP 标定结果，核心字段是 `result.tcp_offset_m_rad`。

更换探针后的 2026-08-31 六点物理验证原始数据和角点标注图保存在 `calibration_records/20260831_new_probe/`。

当前 TCP 修正公式：

```text
R_base_flange = Rz(rz) @ Ry(ry) @ Rx(rx)
p_base_tip = p_base_flange + R_base_flange @ t_flange_tip
```

其中 `t_flange_tip` 来自 `config/tcp_offset_m_rad.yaml`。因此读取到的 `corrected_probe_tip` 是软件修正后的真实探针尖端，不是 SDK 原始法兰点。

## 主脚本

### 1. 检测中缝并输出 TCP 目标点

脚本：

```bash
python 01_detect_seam_start_to_base.py
```

功能：

- 调用 Orbbec SDK 拍摄 `color.png`、`depth_mm.png`、`camera_info.yaml`。
- 在 RGB 图中检测纸箱中缝起点和终点。
- 使用相机外参、深度和 TCP 标定，把中缝点换算成 `base_link` 下的探针尖端目标点。
- 不连接机械臂，不发送任何运动命令。

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--snapshot-dir` | 空 | 使用已有 RGB-D 快照目录，不重新拍照。目录里需要有 `color.png`、`depth_mm.png`、`camera_info.yaml`。 |
| `--roi` | `470,190,380,320` | 检测区域，格式为 `x,y,width,height`，单位是 RGB 像素。 |
| `--mask-mode` | `rgbd` | 分割方式，可选 `rgbd`、`cardboard`、`depth`。当前场景如果 RGB 分割不稳定，可用 `depth`。 |
| `--output-root` | `outputs` | 输出根目录。 |
| `--depth-correction-mode` | `auto` | 深度修正方式，可选 `off`、`auto`、`force`。 |
| `--target-z-min-mm` | `99.0` | 生成起点/终点 TCP 目标时使用的固定 Z，单位 mm。检测得到的 X/Y 保留，Z 会统一改成该值。 |

输出：

```text
outputs/seam_run_YYYYMMDD_HHMMSS/
  center_seam_overlay_YYYYMMDD_HHMMSS.png
  center_seam_result_YYYYMMDD_HHMMSS.yaml
  probe_tip_targets_base.json
```

`probe_tip_targets_base.json` 关键字段：

```json
{
  "probe_tip_contact_targets_base": {
    "start_m": [0.201, -0.050, -0.001],
    "end_m": [0.352, -0.048, -0.001]
  }
}
```

`start_m` 和 `end_m` 是 `base_link` 下 TCP 尖端接触目标点，单位是米，不是法兰坐标。默认会应用 `--target-z-min-mm 99` 的固定 Z；原始未改写的转换结果保存在 `raw_probe_tip_contact_targets_base`。

### 2. 获取当前 TCP 坐标和位姿

脚本：

```bash
python 02_read_probe_tip_pose.py
```

功能：

- 只读取 PIPER SDK 当前法兰反馈。
- 应用 `config/tcp_offset_m_rad.yaml` 的 TCP 偏移。
- 输出当前 TCP 尖端坐标和当前姿态。
- 不发送运动命令。

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--can-name` | `can0` | CAN 设备名。 |
| `--tcp-calibration` | `config/tcp_offset_m_rad.yaml` | TCP 标定文件。 |
| `--output-root` | `outputs` | 保存 JSON 的目录。 |
| `--feedback-wait` | `0.5` | 连接后等待反馈稳定的秒数。 |

输出字段：

| 字段 | 含义 |
| --- | --- |
| `raw_code_output_flange.xyz_m` | SDK 原始法兰/J6 位置，单位米。 |
| `raw_code_output_flange.xyz_mm` | SDK 原始法兰/J6 位置，单位毫米。 |
| `raw_code_output_flange.rpy_deg` | SDK 原始法兰/J6 姿态，单位度。 |
| `corrected_probe_tip.xyz_m` | TCP 修正后的探针尖端位置，单位米。 |
| `corrected_probe_tip.xyz_mm` | TCP 修正后的探针尖端位置，单位毫米。 |
| `corrected_probe_tip.rpy_deg` | 当前末端姿态，单位度。 |
| `corrected_probe_tip.one_line_xyz_m_rpy_deg` | `[x_m, y_m, z_m, rx_deg, ry_deg, rz_deg]`，方便直接复制。 |
| `joint_feedback.joint_names` | 关节名称，固定为 `J1` 到 `J6`。 |
| `joint_feedback.raw_0p001deg` | SDK 读取的 6 个关节原始值，单位 `0.001 degree`。 |
| `joint_feedback.deg` | 6 个关节角，单位度。保存点位时也会记录这一项。 |
| `joint_feedback.valid_feedback` | 关节反馈时间戳是否有效。 |
| `tcp_correction.tcp_offset_flange_m` | TCP 在法兰坐标系下的偏移。 |
| `tcp_correction.tcp_offset_rotated_base_m` | TCP 偏移旋转到 `base_link` 后的值。 |
| `feedback_timestamp` | SDK 反馈时间戳。 |

示例输出：

```json
{
  "frame_id": "base_link",
  "corrected_probe_tip": {
    "xyz_m": [0.4095, -0.1687, -0.0049],
    "xyz_mm": [409.5, -168.7, -4.9],
    "rpy_deg": [177.234, -0.027, 145.415],
    "one_line_xyz_m_rpy_deg": [0.4095, -0.1687, -0.0049, 177.234, -0.027, 145.415]
  },
  "joint_feedback": {
    "joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
    "deg": [0.0, 12.3, -45.6, 0.0, 33.2, 10.1]
  }
}
```

### 3. 移动到某一个 TCP 点

脚本：

```bash
python 03_move_to_probe_tip_point.py --x 350 --y -80 --z 30 --unit mm
```

功能：

- 输入目标 TCP 点 `[x, y, z]`。
- 姿态默认使用当前机械臂姿态，也可以使用固定姿态或 JSON 中的姿态。
- 根据 TCP 标定反算 SDK 需要发送的法兰目标。
- 默认只输出 dry-run 结果，不移动。

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--point-json` | 空 | 读取目标点 JSON，可直接使用 `01` 输出的 `probe_tip_targets_base.json`。 |
| `--point-name` | `start` | 从 JSON 中选择 `start`、`end` 或 `mid`。 |
| `--x --y --z` | 空 | 手动输入 TCP 目标点。 |
| `--unit` | `m` | 手动输入点的单位，可选 `m` 或 `mm`。 |
| `--z-offset-m` | `0.0` | 在目标 TCP 的 z 上增加偏移，单位米。 |
| `--can-name` | `can0` | CAN 设备名。 |
| `--tcp-calibration` | `config/tcp_offset_m_rad.yaml` | TCP 标定文件。 |
| `--orientation-source` | `current` | 姿态来源，可选 `current`、`fixed`、`json`。 |
| `--orientation-json` | 空 | 当 `--orientation-source json` 时，从该 JSON 读取姿态。 |
| `--rx --ry --rz` | `177,0,145` | 当 `--orientation-source fixed` 时使用的姿态，单位度。 |
| `--motion-mode` | `moveL` | SDK 运动模式，可选 `moveP`、`moveL`；默认直线 TCP 运动。 |
| `--speed-percent` | `5` | 速度百分比，1 到 100。 |
| `--send-duration` | `3.0` | 连续发送目标命令的秒数。 |
| `--send-rate-hz` | `50.0` | 命令发送频率。 |
| `--execute` | 关闭 | 打开后才允许真实运动。 |
| `--confirm` | 空 | 真实运动必须写 `EXECUTE`。 |
| `--allow-not-normal-status` | 关闭 | 状态不正常时仍尝试发送，通常不要打开。 |
| `--x-min/x-max/y-min/y-max/z-min/z-max` | 见脚本默认值 | 工作空间保护范围，单位米。 |

从检测结果移动到中缝起点的 dry-run：

```bash
python 03_move_to_probe_tip_point.py \
  --point-json outputs/seam_run_YYYYMMDD_HHMMSS/probe_tip_targets_base.json \
  --point-name start
```

确认无误后，真实低速运动：

```bash
python 03_move_to_probe_tip_point.py \
  --point-json outputs/seam_run_YYYYMMDD_HHMMSS/probe_tip_targets_base.json \
  --point-name start \
  --execute \
  --confirm EXECUTE \
  --speed-percent 5
```

输出字段：

| 输出 | 含义 |
| --- | --- |
| `current_probe_tip` | 当前 TCP 尖端 `[x_m, y_m, z_m, rx_deg, ry_deg, rz_deg]`。 |
| `target_probe_tip_m` | 目标 TCP 点，单位米。 |
| `target_rpy_deg` | 本次采用的目标姿态，单位度。 |
| `command_flange_m` | TCP 反算后的法兰目标，单位米。 |
| `EndPoseCtrl cmd` | 最终发送给 SDK 的整数命令，位置单位 `0.001 mm`，角度单位 `0.001 degree`。 |
| `sent frames` | 真实运动时实际发送的命令帧数。 |
| `after_probe_tip` | 真实运动结束后再次读取的 TCP 尖端位姿。 |

### 4. 移动到某一个 TCP 位姿

脚本：

```bash
python 04_move_to_probe_tip_pose.py \
  --x 350 --y -80 --z 30 --unit mm \
  --rx 177 --ry 0 --rz 145
```

功能：

- 输入完整 TCP 位姿 `[x, y, z, rx, ry, rz]`。
- 根据 TCP 标定反算 SDK 法兰目标。
- 默认 dry-run，不移动。

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--pose-json` | 空 | 从 JSON 读取 TCP 位姿，支持 `02` 输出的 `corrected_probe_tip`。 |
| `--x --y --z` | 空 | 手动输入 TCP 目标位置。 |
| `--unit` | `m` | 手动输入位置单位，可选 `m` 或 `mm`。 |
| `--rx --ry --rz` | 空 | 手动输入 TCP 姿态，单位度。 |
| `--z-offset-m` | `0.0` | 在 TCP 目标 z 上增加偏移，单位米。 |
| `--can-name` | `can0` | CAN 设备名。 |
| `--tcp-calibration` | `config/tcp_offset_m_rad.yaml` | TCP 标定文件。 |
| `--motion-mode` | `moveL` | SDK 运动模式，可选 `moveP`、`moveL`；默认直线 TCP 运动。 |
| `--speed-percent` | `5` | 速度百分比。 |
| `--send-duration` | `3.0` | 连续发送目标命令的秒数。 |
| `--send-rate-hz` | `50.0` | 命令发送频率。 |
| `--execute` | 关闭 | 打开后才允许真实运动。 |
| `--confirm` | 空 | 真实运动必须写 `EXECUTE`。 |
| `--allow-not-normal-status` | 关闭 | 状态不正常时仍尝试发送，通常不要打开。 |

移动到当前已保存的 TCP 位姿 dry-run：

```bash
python 04_move_to_probe_tip_pose.py \
  --pose-json outputs/probe_tip_pose_YYYYMMDD_HHMMSS.json
```

真实低速运动：

```bash
python 04_move_to_probe_tip_pose.py \
  --x 350 --y -80 --z 30 --unit mm \
  --rx 177 --ry 0 --rz 145 \
  --execute \
  --confirm EXECUTE \
  --speed-percent 5
```

输出字段和 `03_move_to_probe_tip_point.py` 类似，区别是 `target_probe_tip_pose` 直接包含完整目标位姿。

### 5. 恢复 SDK 可控模式

脚本：

```bash
python 05_restore_sdk_control_mode.py
```

功能：

- 默认只读取当前机械臂控制状态。
- 如果机械臂仍在示教模式、录制状态或不可命令状态，脚本会提示原因。
- 只有加确认参数后才发送退出示教和切换 SDK 控制的命令。

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--can-name` | `can0` | CAN 设备名。 |
| `--speed-percent` | `5` | 切到 SDK 控制模式时使用的速度百分比。 |
| `--move-mode` | `j` | 默认控制模式，可选 `j`、`l`、`p`。 |
| `--repeat` | `3` | 退出示教命令重复次数。 |
| `--settle-time` | `0.5` | 每步命令后的等待秒数。 |
| `--execute` | 关闭 | 打开后才发送恢复控制命令。 |
| `--confirm` | 空 | 真实恢复必须写 `RESTORE_CONTROL_MODE`。 |

只检查状态：

```bash
python 05_restore_sdk_control_mode.py
```

执行恢复：

```bash
python 05_restore_sdk_control_mode.py \
  --execute \
  --confirm RESTORE_CONTROL_MODE
```

输出字段：

| 输出 | 含义 |
| --- | --- |
| `ctrl_mode` | 当前控制模式。出现 `TEACHING_MODE` 通常说明还在示教。 |
| `arm_status` | 机械臂状态。期望为 `NORMAL`。 |
| `mode_feed` | 当前运动模式反馈。 |
| `teach_status` | 示教/录制状态。 |
| `motion_status` | 当前运动状态。 |
| `err_code` | 控制器错误码，期望为 0。 |

### 6. 粗略扫描 TCP 可达范围

脚本：

```bash
python 06_scan_reachability_workspace.py
```

功能：

- 不发送运动命令，只做离线可达范围扫描。
- 输入 TCP 尖端的 X/Y/Z 范围和固定姿态 RPY。
- 使用当前 TCP 标定，把 TCP 目标反算成 SDK 法兰目标。
- 检查 TCP 目标、法兰目标、安全工作空间、半径范围和 `EndPoseCtrl` 命令范围。
- 输出 `outputs/reachability/*.json` 和 `*.csv`，用于查看哪些采样点通过了保守可达过滤。

常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--x-range-mm` | `150,650,25` | X 扫描范围，格式 `min,max,step`，单位 mm。 |
| `--y-range-mm` | `-300,300,25` | Y 扫描范围，格式 `min,max,step`，单位 mm。 |
| `--z-range-mm` | `80,220,20` | Z 扫描范围，格式 `min,max,step`，单位 mm。 |
| `--orientation-source` | `fixed` | 姿态来源，`fixed` 使用 `--rx/--ry/--rz`，`current` 需要连接机械臂读取当前 TCP 姿态。 |
| `--rx --ry --rz` | `173,-5,163` | 固定 TCP 姿态，单位度。 |
| `--connect` | 关闭 | 连接机械臂读取当前 TCP 位姿和状态；不会使能，也不会运动。 |
| `--tcp-calibration` | `config/tcp_offset_m_rad.yaml` | TCP 标定文件。 |
| `--x-min-mm ... --z-max-mm` | 项目安全范围 | TCP 和法兰目标的安全工作空间。 |
| `--min-radius-mm --max-radius-mm` | `80,626` | 粗略半径过滤，默认最大半径按 PiPER 约 626 mm 工作半径设置。 |

离线扫描箱子附近区域：

```bash
python 06_scan_reachability_workspace.py \
  --x-range-mm 250,550,10 \
  --y-range-mm -180,180,10 \
  --z-range-mm 99,160,10 \
  --rx 173 --ry -5 --rz 163
```

连接机械臂读取当前姿态后扫描：

```bash
python 06_scan_reachability_workspace.py \
  --connect \
  --x-range-mm 250,550,10 \
  --y-range-mm -180,180,10 \
  --z-range-mm 99,160,10
```

注意：这个脚本不是控制器内部 IK，也不会判断关节角连续性和奇异位形。它只能过滤明显不合理的 TCP/法兰目标。真正 IK 可达性需要 URDF/DH + IK 求解器，或在安全高度、小步长、低速条件下做真机验证。

### 7. 使用 MoveIt2 扫描 IK 可达范围

脚本：

```bash
python 07_scan_moveit_ik_reachability.py
```

功能：

- 不执行轨迹，只调用 MoveIt2 的 `/compute_ik` 服务。
- 输入 TCP 目标的 X/Y/Z 网格和固定 RPY 姿态。
- 输出每个采样点的 `ik_ok` 和 MoveIt 错误码。
- 可选 `--check-plan`，在 IK 成功后继续调用 `/plan_kinematic_path` 检查从当前状态到该 IK 解是否能规划。

在 spark 上使用前需要先加载 ROS2 和 PiPER 工作区：

```bash
source /opt/ros/jazzy/setup.bash
source ~/real_robot_control/piper_ws/install/setup.bash
```

还需要另一个终端先启动 MoveIt2/机械臂状态发布，使 `/compute_ik`、`/joint_states` 可用。可以参考 spark 上已有脚本：

```bash
~/Desktop/PIPER_Orbbec_Box_Cutting_Release_20260813_upload_20260815/box_cutting_pipeline/start_piper_moveit_follow.sh
```

扫描箱子附近区域：

```bash
python 07_scan_moveit_ik_reachability.py \
  --x-range-mm=250,550,20 \
  --y-range-mm=-180,180,20 \
  --z-range-mm=99,180,20 \
  --rx 173 --ry -5 --rz 163
```

同时检查规划：

```bash
python 07_scan_moveit_ik_reachability.py \
  --x-range-mm=250,550,20 \
  --y-range-mm=-180,180,20 \
  --z-range-mm=99,180,20 \
  --rx 173 --ry -5 --rz 163 \
  --check-plan
```

负数范围建议用等号写法，例如 `--y-range-mm=-180,180,20`，否则命令行解析可能把负数当成新参数。

## 共享函数说明

共享函数在 `lib/piper_sdk_control_utils.py`，移动和读取脚本都使用这里的 TCP 计算。

| 函数 | 输入 | 输出 | 功能 |
| --- | --- | --- | --- |
| `load_tcp_offset_m_rad(path)` | TCP YAML 路径 | `(tcp_offset, status)` | 读取 `tcp_offset_m_rad`，检查是否为 6 维且当前只支持平移 TCP。 |
| `euler_xyz_deg_to_matrix(values)` | `[rx, ry, rz]`，单位度 | `3x3 numpy.ndarray` | 生成 `Rz @ Ry @ Rx` 旋转矩阵。 |
| `tip_from_flange_pose_m(flange_xyz_m, rpy_deg, tcp_offset_m_rad)` | 法兰位置、姿态、TCP 偏移 | `(tip_xyz_m, rotated_tcp_m)` | 法兰坐标换算成 TCP 尖端坐标。 |
| `flange_from_tip_target_m(tip_xyz_m, rpy_deg, tcp_offset_m_rad)` | TCP 目标点、目标姿态、TCP 偏移 | `flange_xyz_m` | TCP 目标反算成 SDK 需要的法兰目标。 |
| `end_pose_cmd_from_flange_pose(flange_xyz_m, rpy_deg)` | 法兰目标位置和姿态 | `(X,Y,Z,RX,RY,RZ)` | 转成 `EndPoseCtrl` 的 SDK 整数单位。 |
| `connect_piper(can_name, enable, enable_timeout, piper_init)` | CAN 名称和连接参数 | `piper` 对象 | 连接 PIPER SDK，可选择是否使能。 |
| `read_flange_feedback(piper)` | SDK 对象 | `(flange_xyz_m, rpy_deg, timestamp)` | 读取 SDK 原始法兰反馈。 |
| `read_probe_tip_pose(piper, tcp_offset_m_rad)` | SDK 对象和 TCP 偏移 | `dict` | 读取当前反馈并输出 TCP 修正后的完整报告。 |
| `arm_status_summary(piper)` | SDK 对象 | `dict` | 读取控制模式、机械臂状态、错误码等。 |
| `command_ready_problems(status)` | `arm_status_summary` 输出 | `list[str]` | 判断当前是否适合发送 SDK 运动命令。 |
| `require_command_ready(piper, allow_not_normal)` | SDK 对象和是否强制允许 | 无 | 真实运动前的状态保护检查。 |
| `validate_xyz_workspace(label, xyz_m, limits)` | 点和工作空间上下限 | 无 | 检查目标是否超出配置的安全范围。 |
| `send_end_pose_repeated(...)` | SDK 对象、命令、运动模式、速度、持续时间 | `sent_count` | 按固定频率重复发送 `MotionCtrl_2 + EndPoseCtrl`。 |
| `load_pose_json(path)` | 位姿 JSON 路径 | `ProbePose` | 读取 `02` 输出或手写 JSON 中的 TCP 位姿。 |

## 推荐操作顺序

第一次运行或换装夹爪后：

```bash
python 02_read_probe_tip_pose.py
python 05_restore_sdk_control_mode.py
```

检测并验证中缝：

```bash
python 01_detect_seam_start_to_base.py --mask-mode depth --roi 435,417,460,230
```

先做 dry-run：

```bash
python 03_move_to_probe_tip_point.py \
  --point-json outputs/seam_run_YYYYMMDD_HHMMSS/probe_tip_targets_base.json \
  --point-name start
```

确认 `target_probe_tip_m`、`command_flange_m` 和工作空间范围都合理后，再低速执行。

## 网页控制面板

新增了一个独立网页入口，不改变现有脚本功能：

```bash
cd /home/radxa/Desktop/piper_sdk_seam_probe_bundle_20260821
/home/radxa/box_sdk_env/bin/python web_control_panel/server.py --host 0.0.0.0 --port 8080
```

浏览器打开：

```text
http://10.77.0.3:8080
```

页面中间显示相机画面和 ROI 区域，右侧按钮可以拍摄 ROI、检测中线、获取 TCP 坐标、移动到起点/终点，并提供 X/Y/Z 小步点动。真实运动默认锁定，必须勾选允许运动并输入确认词。

详细说明见：

```text
web_control_panel/README.md
```

## 当前验证状态

已验证：

- 本地和服务器代码可编译。
- 服务器可通过 SDK 读取法兰反馈，并输出 TCP 修正后的探针尖端位姿。
- 检测脚本在当前测试画面中使用 `--mask-mode depth` 和手动 ROI 可以跑通生成目标 JSON。
- 2026-08-31 更换探针后已重新标定 TCP，并在棋盘上使用 6 个分布点完成独立 RGB-D/探针验证；RMS 为 `1.896 mm`，最大误差为 `2.672 mm`，通过 `RMS <= 2.5 mm`、`max <= 4.0 mm` 的验收条件。

仍需现场确认：

- 当前测试画面不是标准纸箱场景时，中缝语义准确性不能只看代码通过，需要看 overlay 图片确认红线是否落在胶带中缝。
- 当前探针 TCP 标定文件状态为 `calibrated_and_validated`；探针、相机或机械臂基座重新安装后必须再次标定，真实切割前仍应执行低速、受保护的试运行。
- 真实运动前必须确认机械臂不在示教/录制状态，且 `err_code=0`。
