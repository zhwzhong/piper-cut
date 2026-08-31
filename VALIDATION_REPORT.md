# 验证报告：piper_sdk_seam_probe_bundle_20260821

## 2026-08-31 新探针 TCP 重新标定

结论：通过。更换探针后重新采集 20 个枢轴接触姿态，从中选取 12 个平衡样本求解 TCP，并使用棋盘上 6 个分布点完成独立 RGB-D/探针物理验证。新 TCP 已成为当前探针的权威配置。

### 当前 TCP

```text
tcp_offset_m_rad = [0.0014776104376773445,
                    0.0009592381545213136,
                    0.08428718645795262,
                    0.0, 0.0, 0.0]
status = calibrated_and_validated
```

### 枢轴标定质量

```text
选中样本（1-based） = 2, 4, 6, 7, 8, 10, 13, 14, 15, 17, 18, 20
接触残差 RMS/median/max = 0.731 / 0.753 / 0.913 mm
rank / condition = 6 / 7.376
最大姿态旋转跨度 = 76.152 deg
leave-one-out TCP spread RMS = 0.213 mm
```

### 6 点独立物理验证

```text
棋盘角点 ID = 13, 19, 38, 46, 52, 71
距离误差 RMS/median/max = 1.896 / 1.518 / 2.672 mm
XYZ 轴 RMS = [1.558, 0.534, 0.940] mm
验收条件 = RMS <= 2.5 mm 且 max <= 4.0 mm
结果 = PASS
```

本次验证采用“机械臂离开棋盘时缓存 RGB-D 点、关闭相机、再由操作者手动触碰角点”的流程；采集过程中未发送自动机械臂运动命令。仓库内对应记录：

```text
config/tcp_offset_m_rad.yaml
config/calibration_bundle.yaml
calibration_records/20260831_new_probe/comparison.yaml
calibration_records/20260831_new_probe/touch_plan.png
```

以下情况会使本标定失效：探针更换或重新安装、相机移动或重新安装、机械臂基座相对相机移动、相机序列号或成像参数变化，以及显著超出深度修正的已验证距离范围。标定通过不代表允许无人值守运动，切割时仍须保留受保护的接近和接触逻辑。

## 2026-08-21 原始项目验证记录

验证时间：2026-08-21

## 结论

这个包已经在本地和 radxa 服务器上创建为独立新项目，未改动旧的 `box_cutting_sdk_pipeline`。

代码完整性和运行链路结论：

```text
本地 zip manifest 校验：通过
本地 Python 编译检查：通过
远端 manifest 校验：通过
远端 Python 编译检查：通过
远端标定配置格式检查：通过
02_read_probe_tip_pose.py 只读 TCP 尖端位姿：通过
01_detect_seam_start_to_base.py 采集/检测/转换链路：使用 depth 模式通过
机械臂运动命令：未发送
```

准确性结论：

```text
标定和 TCP 转换链路可以运行，配置也能通过一致性检查。
当前现场画面主要是棋盘板和机械臂，不是快递箱胶带。
因此本次不能证明“纸箱胶带中缝检测语义准确”。
当前 overlay 中检测线落在棋盘/机械臂附近区域，后续需要放真实纸箱后重新验证。
```

## 本地项目

路径：

```text
/Users/zhwzhong/Documents/拆快递箱子/piper_sdk_seam_probe_bundle_20260821
```

执行过：

```bash
shasum -a 256 -c MANIFEST.sha256
python3 -m py_compile 01_detect_seam_start_to_base.py 02_read_probe_tip_pose.py lib/*.py
```

结果：通过。

## 服务器项目

路径：

```text
/home/radxa/Desktop/piper_sdk_seam_probe_bundle_20260821
```

执行过：

```bash
sha256sum -c MANIFEST.sha256
~/box_sdk_env/bin/python -m py_compile 01_detect_seam_start_to_base.py 02_read_probe_tip_pose.py lib/*.py
```

结果：通过。

## 配置检查

已检查：

```text
config/runtime_config.yaml
config/tcp_offset_m_rad.yaml
config/depth_correction.yaml
config/eye_to_hand_extrinsics.yaml
config/calibration_bundle.yaml
```

关键检查：

```text
T_base_camera 是 4x4 矩阵
T_base_camera 最后一行是 [0, 0, 0, 1]
旋转矩阵正交
旋转矩阵 det 接近 +1
tcp_offset_m_rad 长度为 6
```

结果：通过。

## 02 当前 TCP 尖端位姿读取

命令：

```bash
~/box_sdk_env/bin/python 02_read_probe_tip_pose.py
```

结果文件：

```text
outputs/probe_tip_pose_20260821_212050.json
```

关键输出：

```text
raw flange xyz_m = [0.407619, -0.17219, 0.057111]
corrected probe tip xyz_m = [0.40953629757716237, -0.16873724557891828, -0.004898507521884032]
corrected probe tip xyz_mm = [409.5362975771624, -168.73724557891828, -4.898507521884032]
rpy_deg = [177.234, -0.027, 145.415]
```

说明：该脚本只调用 `GetArmEndPoseMsgs()` 读取反馈并做 TCP 修正，不发送运动命令。

## 01 采集、检测和 base_link 转换

默认命令：

```bash
~/box_sdk_env/bin/python 01_detect_seam_start_to_base.py
```

结果：相机采集成功，但默认 `rgbd` 分割失败：

```text
Failed to segment cardboard-colored components.
```

当前现场使用下面参数可以跑通：

```bash
~/box_sdk_env/bin/python 01_detect_seam_start_to_base.py \
  --roi 435,417,460,230 \
  --mask-mode depth
```

结果文件：

```text
outputs/seam_run_20260821_212231/center_seam_overlay_20260821_212231.png
outputs/seam_run_20260821_212231/center_seam_result_20260821_212231.yaml
outputs/seam_run_20260821_212231/probe_tip_targets_base.json
```

关键输出：

```text
RGB seam start = [463.4176330566406, 536.7756958007812]
RGB seam end   = [886.1525268554688, 553.0347290039062]
depth used start/end = 690.816 mm -> 684.681 mm
depth fit RMS = 1.383 mm
base_link start tip = [0.2011487848924787, -0.22793417415759368, -0.001162799074377996] m
base_link end tip   = [0.6780955226124663, -0.2520097140853109, -0.01201139285849906] m
seam length = 477.677 mm
```

质量字段：

```text
calibration_bundle_accepted = true
depth_inlier_fraction = 0.7746478873239436
depth_fit_rms_mm = 1.3829630432393738
```

视觉检查：

```text
validation_remote_outputs/center_seam_overlay_20260821_212231.png
```

当前 overlay 显示检测线不在真实纸箱胶带上，而是在棋盘/机械臂附近区域。因此当前场景下不能认定中缝检测语义准确。

## 已做的小改动

只改了新项目内的一个入口脚本：

```text
01_detect_seam_start_to_base.py
```

改动：

```text
新增 --mask-mode 参数，支持 rgbd/cardboard/depth。
默认仍保持 rgbd，不改变原包默认行为。
```

原因：当前现场颜色/rgbd 分割失败，但 depth 分割可运行，便于后续继续验证。

## 后续建议

1. 放真实快递箱，并让胶带位于 ROI 内。
2. 先运行：

```bash
~/box_sdk_env/bin/python 01_detect_seam_start_to_base.py \
  --roi x,y,w,h \
  --mask-mode depth
```

3. 人工检查 overlay 中红线是否压在胶带中缝上，绿色 START 和蓝色 END 是否符合切割方向。
4. 再检查 `probe_tip_targets_base.json` 的起点、终点和长度是否合理。
5. 只有检测图和 base_link 点都合理后，再把功能迁移到主工程或接机械臂执行。
