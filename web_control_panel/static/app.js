const state = {
  busy: false,
  lastTarget: null,
  lastPose: null,
  streamTimer: null,
  manualMode: null,
  pendingLineStartPx: null,
  streamRetryCount: 0,
  stoppingStream: false,
  savedPoses: [],
  cutPoints: [],
  jogHeld: null,
  jogRunning: false,
  jogTimer: null,
  jogKeepaliveTimer: null,
};

const $ = (id) => document.getElementById(id);

const PARAMETER_HELP = `参数说明

[视觉/检测参数]
X, Y, W, H:
  ROI 区域，单位是像素。X/Y 是左上角坐标，W/H 是宽高。
  检测中线只在这个区域里找箱子，ROI 不准会直接影响检测结果。

Z最小 mm:
  全局 TCP 目标 Z 下限，默认 99 mm。
  检测、手动校准、移动到 XYZ、回到点位、到起点/终点、分段、A-F 和方向点动都会使用同一个值。
  如果某个待发送目标的 Z 小于这个值，后端会把实际发送的 Z 改为该最小值，并返回 z_clamped=true。

分段 mm:
  勾选“启用分段”后，从起点到终点时的最大插值段长，默认 30 mm。
  系统会把起点到终点拆成多段，每段成功后才继续下一段。

三线切割 / 端部线 px:
  勾选“三线切割”后，检测中线会额外生成左端短线、中间长线、右端短线。
  左边线段端点为 A/B，中间为 C/D，右边为 E/F。
  端部线 px 是左右短线的像素长度，默认 90 px，用于把胶带两端也切开。
  生成三线后，可通过“三线点位”下拉框选择 A-F 任一点并移动过去。
  “一键 A-F”会先移动到 point_2，再按 A->B -> B->C -> C->D -> D->E -> E->F 连续执行，并使用当前分段设置。

mask mode:
  rgbd      彩色图 + 深度图一起分割箱子，默认模式。
  depth     主要靠深度找箱子上表面，光照影响小，但容易受桌面/深度空洞影响。
  cardboard 主要靠纸箱颜色分割，深度参与少，适合深度不稳定但颜色明显的情况。

[显示参数]
实时视频:
  打开/关闭 MJPEG 实时画面。

清晰度:
  实时视频是压缩 MJPEG 传输。
  流畅: 低带宽低延迟；标准: 默认；高清/超清: 更清楚但延迟和带宽更高。

ROI:
  是否在画面上显示蓝色 ROI 框、像素原点和 x/y 方向。

箱子:
  是否显示最后一次检测得到的箱子上表面外接框。
  不是每帧实时检测，需要先点一次“检测中线”。

中线:
  是否显示最后一次检测得到、手动校准后或手动选择后的中线。
  不是每帧实时检测，需要先点一次“检测中线”。

标定验证:
  拍摄标定板后，在图像上选择一个角点或标记点。
  后端会用该像素的深度、相机内参和手眼外参换算出 base_link 坐标。
  用方向键把探针尖端移到该点后，点击“计算误差”读取当前 TCP 并输出误差。

[机械臂参数]
步长 mm:
  X/Y/Z 点动每次移动的距离，单位 mm。

步频 ms:
  长按方向键时的速度换算参数。
  页面会按“步长 mm / 步频 ms”换算连续点动速度，并持续发送保活。
  默认 1 mm、80 ms，约等于 12.5 mm/s；确认方向正确后再调快。

速度 %:
  发送给 PiPER SDK 的速度百分比，数值越小越慢。

运动模式:
  moveL  TCP 尖端尽量按直线移动，适合到起点、到终点、切割和小范围微调。
  moveP  点到点运动，路径不保证直线，适合远距离回到保存点位或姿态调整。

允许运动:
  安全开关。不勾选时，回到点位、到起点、到终点、点动都是 dry-run。

EXECUTE:
  网页会在勾选“允许运动”时自动传给后端，不需要手动填写。

[按钮含义]
拍摄 ROI:
  拍一张 RGB-D 图并显示 ROI。

检测中线:
  用当前 ROI 和 mask mode 检测箱子框、中线，并转换成 TCP 目标坐标。
  检测完成后会在“线段起点/终点 px”里显示当前中线的两个像素端点。

校准线段 / 确认校准:
  点击“校准线段”后，在图像上先点起点、再点终点。
  两个点会先写入输入框，不会立即修改目标；检查无误后点击“确认校准”，后端才会重建整条中线并重新生成目标坐标。
  也可以直接修改四个像素输入框，然后点“确认校准”。

保存点位:
  按“点位名称”把当前 TCP 位姿保存成一个命名点位。
  点位文件保存在 config/saved_poses/。

移动到 XYZ:
  三个输入框默认显示当前 TCP 尖端坐标，单位 mm。
  你可以修改 X/Y/Z 中的任意值，系统保持当前 TCP 姿态，只移动到新的 X/Y/Z。

回到点位:
  移动到下拉框中选择的任意保存点位，默认 dry-run。

清除点位:
  删除下拉框当前选择的保存点位文件，不发送机械臂运动命令。

到起点 / 到终点:
  先移动到当前中线起点。
  勾选“启用分段”时，到终点会把 start/end 插值成多段逐段执行；关闭时一次性到终点。

拍摄标定板:
  停止实时视频并保留一张静态 RGB-D 快照，用于点击验证点。

选择验证点:
  在静态图像上点击一个标定板角点或标记点，生成 base_link 目标坐标。

计算误差:
  读取当前 TCP 尖端坐标，并与上一次选择的验证点坐标相减。

恢复 SDK 模式:
  把机械臂切回 SDK 可控模式，默认只检查状态。

一键执行:
  顺序执行：检测中线 -> 恢复 SDK 模式 -> 移动到保存点位 point_2 -> 到起点 -> 分段到终点。
  勾选“三线切割”时，切割路径会替换为：A->B -> B->C -> C->D -> D->E -> E->F。
  网页会分步执行；检测阶段短暂停止实时视频，检测完成后恢复视频再继续运动。
  默认 dry-run。真实运动需要勾选“允许运动”，网页会自动发送确认词。`;

function roi() {
  return [
    Number($("roiX").value),
    Number($("roiY").value),
    Number($("roiW").value),
    Number($("roiH").value),
  ];
}

function targetZMinMm() {
  const value = Number($("targetZMin").value || 99);
  if (!Number.isFinite(value)) {
    throw new Error("Z最小值必须是数字，单位 mm");
  }
  return value;
}

function segmentMm() {
  const value = Number($("segmentMm").value || 30);
  if (!Number.isFinite(value) || value <= 0) {
    throw new Error("分段长度必须是正数，单位 mm");
  }
  return value;
}

function useSegments() {
  return $("segmentToggle").checked;
}

function useThreeCut() {
  return $("threeCutToggle").checked;
}

function sideCutPx() {
  const value = Number($("sideCutPx").value || 90);
  if (!Number.isFinite(value) || value < 5) {
    throw new Error("端部线长度必须是正数，单位 px");
  }
  return value;
}

function motionPayload() {
  const execute = $("executeToggle").checked;
  return {
    execute,
    confirm: execute ? "EXECUTE" : "",
    speed_percent: Number($("speedPercent").value || 5),
    motion_mode: $("motionMode").value,
    target_z_min_mm: targetZMinMm(),
  };
}

function manualXyzPayload() {
  const x = Number($("targetX").value);
  const y = Number($("targetY").value);
  const z = Number($("targetZ").value);
  if (![x, y, z].every(Number.isFinite)) {
    throw new Error("请输入有效的 X/Y/Z，单位 mm");
  }
  return { x_mm: x, y_mm: y, z_mm: z };
}

function fillTargetInputsFromPose(pose) {
  const xyz = pose?.corrected_probe_tip?.xyz_mm;
  if (!Array.isArray(xyz) || xyz.length !== 3) return;
  $("targetX").value = Number(xyz[0]).toFixed(3);
  $("targetY").value = Number(xyz[1]).toFixed(3);
  $("targetZ").value = Number(xyz[2]).toFixed(3);
}

function fillLineInputsFromPixels(seamPixels) {
  const start = seamPixels?.start_px;
  const end = seamPixels?.end_px;
  if (!Array.isArray(start) || !Array.isArray(end) || start.length !== 2 || end.length !== 2) return;
  $("lineStartX").value = Number(start[0]).toFixed(3);
  $("lineStartY").value = Number(start[1]).toFixed(3);
  $("lineEndX").value = Number(end[0]).toFixed(3);
  $("lineEndY").value = Number(end[1]).toFixed(3);
}

function linePixelsPayload() {
  const start = [Number($("lineStartX").value), Number($("lineStartY").value)];
  const end = [Number($("lineEndX").value), Number($("lineEndY").value)];
  if (![...start, ...end].every(Number.isFinite)) {
    throw new Error("请输入有效的线段起点/终点像素坐标");
  }
  return { start_px: start, end_px: end };
}

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function setBusy(value, text = "ready") {
  state.busy = value;
  document.querySelectorAll("button").forEach((button) => {
    button.disabled = value;
  });
  $("statusText").textContent = text;
}

function showImage(url) {
  const image = $("cameraImage");
  image.src = `${url}?t=${Date.now()}`;
  image.style.display = "block";
  $("emptyImage").style.display = "none";
}

function isStreamEnabled() {
  return $("streamToggle").checked;
}

function streamQualityConfig() {
  const mode = $("streamQuality").value;
  if (mode === "smooth") return { quality: "42", fps: "12" };
  if (mode === "clear") return { quality: "78", fps: "8" };
  if (mode === "ultra") return { quality: "90", fps: "5" };
  return { quality: "58", fps: "10" };
}

function streamUrl() {
  const streamQuality = streamQualityConfig();
  const params = new URLSearchParams({
    roi: roi().join(","),
    show_roi: $("showRoiToggle").checked ? "1" : "0",
    show_box: $("showBoxToggle").checked ? "1" : "0",
    show_seam: $("showSeamToggle").checked ? "1" : "0",
    quality: streamQuality.quality,
    fps: streamQuality.fps,
    t: String(Date.now()),
  });
  return `/stream.mjpg?${params.toString()}`;
}

function startStream() {
  if (!isStreamEnabled()) return;
  const image = $("cameraImage");
  state.stoppingStream = false;
  image.src = streamUrl();
  image.dataset.stream = "1";
  image.style.display = "block";
  $("emptyImage").style.display = "none";
  $("statusText").textContent = "streaming";
}

function markStreamAlive() {
  state.streamRetryCount = 0;
}

function stopStream(showPlaceholder = false) {
  clearTimeout(state.streamTimer);
  state.stoppingStream = true;
  const image = $("cameraImage");
  image.dataset.stream = "0";
  image.src = "about:blank";
  if (showPlaceholder) {
    image.style.display = "none";
    $("emptyImage").style.display = "block";
  }
}

function restartStreamSoon(delayMs = 900) {
  clearTimeout(state.streamTimer);
  stopStream();
  $("statusText").textContent = "restarting stream";
  state.streamTimer = setTimeout(() => {
    startStream();
  }, delayMs);
}

function retryStreamSoon() {
  if (!isStreamEnabled() || state.busy || state.stoppingStream) return;
  if (state.streamRetryCount >= 8) {
    $("statusText").textContent = "stream retry failed";
    return;
  }
  state.streamRetryCount += 1;
  $("statusText").textContent = `stream retry ${state.streamRetryCount}`;
  clearTimeout(state.streamTimer);
  state.streamTimer = setTimeout(() => {
    startStream();
  }, 1000);
}

function postUsesCamera(path) {
  return path === "/api/capture" || path === "/api/detect" || path === "/api/run_cut_sequence";
}

function showLog(data) {
  if (Array.isArray(data.steps)) {
    $("logBox").textContent = data.steps.map((step, index) => {
      const result = step.result || {};
      return [
        `#${index + 1} ${step.name}: ${step.ok ? "OK" : "FAILED"}${step.skipped ? " (skipped)" : ""}`,
        step.message || "",
        result.command ? `$ ${result.command.join(" ")}` : "",
        result.returncode !== undefined ? `returncode=${result.returncode} duration=${result.duration_s}s` : "",
        result.output || step.error || "",
      ].filter(Boolean).join("\n");
    }).join("\n\n");
    return;
  }
  const result = data.result || {};
  $("logBox").textContent = [
    result.command ? `$ ${result.command.join(" ")}` : "",
    result.returncode !== undefined ? `returncode=${result.returncode} duration=${result.duration_s}s` : "",
    result.output || data.error || "",
  ].filter(Boolean).join("\n");
}

function showCoords(data) {
  if (data.target) {
    state.lastTarget = data.target;
    const block = data.target.probe_tip_contact_targets_base || {};
    if (data.target_z_min_mm !== undefined) {
      $("targetZMin").value = Number(data.target_z_min_mm).toFixed(3);
    }
    $("coordsBox").textContent = JSON.stringify({
      start_m: block.start_m,
      end_m: block.end_m,
      target_z_min: data.target.target_z_min,
      length_mm: data.target.seam_length_mm,
      target_json: data.target_json,
      seam_pixels: data.seam_pixels,
      box_pixels: data.box_pixels,
      cut_mode: data.cut_mode,
      cut_lines: data.cut_lines,
      cut_points: data.cut_points,
    }, null, 2);
    if (data.cut_points) updateCutPointSelect(data.cut_points);
    fillLineInputsFromPixels(data.seam_pixels);
    return;
  }
  if (data.validation_error) {
    if (data.pose) fillTargetInputsFromPose(data.pose);
    $("coordsBox").textContent = JSON.stringify({
      target_xyz_mm: data.validation_error.target_xyz_mm,
      actual_tcp_xyz_mm: data.validation_error.actual_tcp_xyz_mm,
      diff_actual_minus_target_mm: data.validation_error.diff_actual_minus_target_mm,
      norm_3d_mm: data.validation_error.norm_3d_mm,
      norm_xy_mm: data.validation_error.norm_xy_mm,
      z_error_mm: data.validation_error.z_error_mm,
      target_pixel_xy: data.validation_error.target_pixel_xy,
      target_json: data.validation_error.target_json,
    }, null, 2);
    return;
  }
  if (data.pose) {
    state.lastPose = data.pose;
    fillTargetInputsFromPose(data.pose);
    $("coordsBox").textContent = JSON.stringify({
      tcp_pose: data.pose.corrected_probe_tip,
      flange_pose: data.pose.raw_code_output_flange,
      joint_feedback: data.pose.joint_feedback,
      pose_json: data.pose_json,
    }, null, 2);
    return;
  }
  if (data.far_pose) {
    $("coordsBox").textContent = JSON.stringify({
      far_pose: data.far_pose.corrected_probe_tip,
      far_pose_json: data.far_pose_json,
    }, null, 2);
    return;
  }
  if (data.saved_pose) {
    fillTargetInputsFromPose(data.saved_pose);
    $("coordsBox").textContent = JSON.stringify({
      pose_name: data.pose_name,
      saved_pose: data.saved_pose.corrected_probe_tip,
      joint_feedback: data.saved_pose.joint_feedback,
      pose_json: data.pose_json,
    }, null, 2);
    return;
  }
  if (data.validation_target) {
    $("coordsBox").textContent = JSON.stringify({
      pixel_xy: data.validation_target.pixel_xy,
      base_xyz_mm: data.validation_target.base_xyz_mm,
      camera_xyz_m: data.validation_target.camera_xyz_m,
      depth: data.validation_target.depth,
      target_json: data.validation_target_json,
    }, null, 2);
    return;
  }
  if (data.target_xyz_mm) {
    $("coordsBox").textContent = JSON.stringify({
      jog_target_xyz_mm: data.target_xyz_mm,
      jog_target_rpy_deg: data.target_rpy_deg,
    }, null, 2);
    return;
  }
  if (data.cut_point) {
    $("coordsBox").textContent = JSON.stringify({
      cut_mode: data.cut_mode,
      point_label: data.point_label,
      cut_point: data.cut_point,
      cut_points: data.cut_points,
    }, null, 2);
    if (data.cut_points) updateCutPointSelect(data.cut_points);
    return;
  }
  if (data.segmented_line) {
    $("coordsBox").textContent = JSON.stringify({
      segmented_line: data.segmented_line,
      stopped_at: data.stopped_at,
    }, null, 2);
    return;
  }
  if (data.cut_lines) {
    $("coordsBox").textContent = JSON.stringify({
      cut_mode: data.cut_mode,
      cut_lines: data.cut_lines,
      cut_points: data.cut_points,
      stopped_at: data.stopped_at,
    }, null, 2);
    if (data.cut_points) updateCutPointSelect(data.cut_points);
  }
}

function updateSavedPoseSelect(savedPoses) {
  if (!Array.isArray(savedPoses)) return;
  state.savedPoses = savedPoses;
  const select = $("savedPoseSelect");
  const previous = select.value;
  select.innerHTML = "";
  if (savedPoses.length === 0) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "暂无保存点位";
    select.appendChild(option);
    return;
  }
  savedPoses.forEach((pose) => {
    const option = document.createElement("option");
    option.value = pose.name;
    const xyz = Array.isArray(pose.xyz_mm)
      ? pose.xyz_mm.map((value) => Number(value).toFixed(1)).join(", ")
      : "";
    option.textContent = xyz ? `${pose.name} (${xyz} mm)` : pose.name;
    select.appendChild(option);
  });
  if (savedPoses.some((pose) => pose.name === previous)) {
    select.value = previous;
  }
}

function updateCutPointSelect(cutPoints) {
  if (!Array.isArray(cutPoints)) return;
  state.cutPoints = cutPoints;
  const select = $("threeCutPointSelect");
  const previous = select.value;
  select.innerHTML = "";
  if (cutPoints.length === 0) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "先生成三线";
    select.appendChild(option);
    return;
  }
  cutPoints.forEach((point) => {
    const option = document.createElement("option");
    option.value = point.label;
    const xyz = Array.isArray(point.xyz_mm)
      ? point.xyz_mm.map((value) => Number(value).toFixed(1)).join(", ")
      : "";
    option.textContent = xyz ? `${point.label} (${xyz} mm)` : point.label;
    select.appendChild(option);
  });
  if (cutPoints.some((point) => point.label === previous)) {
    select.value = previous;
  }
}

function setManualMode(mode) {
  if (mode !== null && mode !== "validation" && mode !== "line_start" && mode !== "line_end") return;
  state.manualMode = mode;
  $("validationPointBtn").classList.toggle("active", mode === "validation");
  $("selectLineBtn").classList.toggle("active", mode === "line_start" || mode === "line_end");
  $("validationPointBtn").textContent = mode === "validation" ? "点击画面选点" : "选择验证点";
  $("selectLineBtn").textContent = mode === "line_start"
    ? "点击起点"
    : mode === "line_end"
      ? "点击终点"
      : "校准线段";
  if (mode === "line_start") {
    $("statusText").textContent = "click line start";
  } else if (mode === "line_end") {
    $("statusText").textContent = "click line end";
  } else if (mode === "validation") {
    $("statusText").textContent = "click validation point";
  } else {
    $("statusText").textContent = "ready";
  }
  document.querySelector(".image-stage").classList.toggle("calibrating", mode !== null);
}

function imagePixelFromClick(event) {
  const image = $("cameraImage");
  const rect = image.getBoundingClientRect();
  const naturalWidth = image.naturalWidth || 1280;
  const naturalHeight = image.naturalHeight || 720;
  const scale = Math.min(rect.width / naturalWidth, rect.height / naturalHeight);
  const renderedWidth = naturalWidth * scale;
  const renderedHeight = naturalHeight * scale;
  const left = rect.left + (rect.width - renderedWidth) * 0.5;
  const top = rect.top + (rect.height - renderedHeight) * 0.5;
  const x = (event.clientX - left) / scale;
  const y = (event.clientY - top) / scale;
  if (x < 0 || y < 0 || x >= naturalWidth || y >= naturalHeight) {
    return null;
  }
  return [Number(x.toFixed(3)), Number(y.toFixed(3))];
}

async function post(path, payload, label) {
  const shouldRestartStream = isStreamEnabled() && postUsesCamera(path);
  if (shouldRestartStream) {
    stopStream();
    await new Promise((resolve) => setTimeout(resolve, 700));
  }
  setBusy(true, label);
  try {
    const response = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload || {}),
    });
    const data = await response.json();
    if (!shouldRestartStream && data.image_url) showImage(data.image_url);
    if (!shouldRestartStream && data.overlay_url) showImage(data.overlay_url);
    if (data.saved_poses) updateSavedPoseSelect(data.saved_poses);
    showCoords(data);
    showLog(data);
    $("statusText").textContent = data.ok ? "done" : "failed";
    return data;
  } catch (error) {
    $("statusText").textContent = "failed";
    $("logBox").textContent = String(error);
    return { ok: false, error: String(error) };
  } finally {
    setBusy(false, $("statusText").textContent);
    if (shouldRestartStream) {
      restartStreamSoon(1200);
    }
  }
}

async function runSequenceStep(steps, name, path, payload, label) {
  const data = await post(path, payload, label);
  steps.push({ name, ...data });
  if (!data.ok) {
    showLog({ steps });
    $("statusText").textContent = `failed at ${name}`;
    return false;
  }
  return true;
}

async function runOneClickSequence() {
  const steps = [];
  const motion = motionPayload();

  if (!await runSequenceStep(steps, "detect_seam", "/api/detect", {
    roi: roi(),
    mask_mode: $("maskMode").value,
    target_z_min_mm: targetZMinMm(),
    use_last_snapshot: false,
  }, "detecting...")) return;

  if (useThreeCut()) {
    if (!await runSequenceStep(steps, "build_three_cut_lines", "/api/build_three_cut_lines", {
      roi: roi(),
      target_z_min_mm: targetZMinMm(),
      side_cut_px: sideCutPx(),
    }, "building three lines...")) return;
  }

  if (isStreamEnabled()) {
    restartStreamSoon(100);
    await sleep(700);
  }

  if (!await runSequenceStep(steps, "restore_sdk_control_mode", "/api/restore", {
    execute: motion.execute,
    confirm: motion.execute ? "RESTORE_CONTROL_MODE" : "",
  }, "restore...")) return;

  if (!await runSequenceStep(steps, "move_point_2_pose", "/api/move_named_pose", {
    pose_name: "point_2",
    ...motion,
  }, "move point_2...")) return;

  if (useThreeCut()) {
    if (!await runSequenceStep(steps, "move_three_cut_lines", "/api/move_three_cut_lines", {
      segment_mm: segmentMm(),
      use_segments: useSegments(),
      ...motion,
    }, "move three lines...")) return;
  } else {
    if (!await runSequenceStep(steps, "move_seam_start", "/api/move_point", {
      point_name: "start",
      ...motion,
    }, "move start...")) return;

    if (useSegments()) {
      if (!await runSequenceStep(steps, "move_seam_end_segments", "/api/move_line_segments", {
        segment_mm: segmentMm(),
        ...motion,
      }, "move segments...")) return;
    } else if (!await runSequenceStep(steps, "move_seam_end", "/api/move_point", {
      point_name: "end",
      ...motion,
    }, "move end...")) {
      return;
    }
  }

  showLog({ steps });
  $("statusText").textContent = "sequence done";
}

async function buildThreeCutLinesWithAutoDetect() {
  const payload = {
    roi: roi(),
    target_z_min_mm: targetZMinMm(),
    side_cut_px: sideCutPx(),
  };
  let data = await post("/api/build_three_cut_lines", payload, "building three lines...");
  if (data.ok) {
    restartStreamSoon(900);
    return data;
  }
  if (!String(data.error || "").includes("run seam detection")) {
    return data;
  }
  const detected = await post("/api/detect", {
    roi: roi(),
    mask_mode: $("maskMode").value,
    target_z_min_mm: targetZMinMm(),
    use_last_snapshot: false,
  }, "detecting first...");
  if (!detected.ok) return detected;
  data = await post("/api/build_three_cut_lines", payload, "building three lines...");
  if (data.ok) restartStreamSoon(900);
  return data;
}

async function captureValidationSnapshot() {
  stopStream(true);
  setBusy(true, "capturing board...");
  try {
    const response = await fetch("/api/capture", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ roi: roi(), include_detection_overlay: false }),
    });
    const data = await response.json();
    if (data.image_url) showImage(data.image_url);
    showCoords(data);
    showLog(data);
    $("statusText").textContent = data.ok ? "board captured" : "failed";
    return data;
  } catch (error) {
    $("statusText").textContent = "failed";
    $("logBox").textContent = String(error);
    return { ok: false, error: String(error) };
  } finally {
    setBusy(false, $("statusText").textContent);
  }
}

async function postJog(payload) {
  state.jogRunning = true;
  $("statusText").textContent = "jogging...";
  try {
    const response = await fetch("/api/jog", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await response.json();
    showCoords(data);
    showLog(data);
    $("statusText").textContent = data.ok ? "jog done" : "jog failed";
    return data;
  } catch (error) {
    $("statusText").textContent = "jog failed";
    $("logBox").textContent = String(error);
    return { ok: false, error: String(error) };
  } finally {
    state.jogRunning = false;
  }
}

$("helpBtn").addEventListener("click", () => {
  $("logBox").textContent = PARAMETER_HELP;
});

$("runSequenceBtn").addEventListener("click", () => {
  runOneClickSequence().catch((error) => {
    $("statusText").textContent = "sequence failed";
    $("logBox").textContent = String(error);
  });
});

$("captureBtn").addEventListener("click", () => {
  post("/api/capture", { roi: roi() }, "capturing...");
});

$("detectBtn").addEventListener("click", () => {
  post("/api/detect", {
    roi: roi(),
    mask_mode: $("maskMode").value,
    target_z_min_mm: targetZMinMm(),
    use_last_snapshot: false,
  }, "detecting...");
});

$("buildThreeLinesBtn").addEventListener("click", () => {
  try {
    $("showSeamToggle").checked = true;
    buildThreeCutLinesWithAutoDetect();
  } catch (error) {
    $("statusText").textContent = "invalid three cut";
    $("logBox").textContent = String(error);
  }
});

$("moveThreeCutPointBtn").addEventListener("click", () => {
  const pointLabel = $("threeCutPointSelect").value;
  if (!pointLabel) {
    $("statusText").textContent = "select A-F";
    return;
  }
  post("/api/move_three_cut_point", {
    point_label: pointLabel,
    ...motionPayload(),
  }, `move ${pointLabel}...`);
});

$("moveThreeCutAllBtn").addEventListener("click", () => {
  const steps = [];
  const motion = motionPayload();
  runSequenceStep(steps, "move_point_2_pose", "/api/move_named_pose", {
    pose_name: "point_2",
    ...motion,
  }, "move point_2...")
    .then((ok) => {
      if (!ok) return false;
      return runSequenceStep(steps, "move_A_to_F", "/api/move_three_cut_lines", {
        segment_mm: segmentMm(),
        use_segments: useSegments(),
        ...motion,
      }, "move A-F...");
    })
    .then((ok) => {
      if (ok) {
        showLog({ steps });
        $("statusText").textContent = "A-F done";
      }
    })
    .catch((error) => {
      $("statusText").textContent = "A-F failed";
      $("logBox").textContent = String(error);
    });
});

$("selectLineBtn").addEventListener("click", () => {
  if (state.manualMode === "line_start" || state.manualMode === "line_end") {
    state.pendingLineStartPx = null;
    setManualMode(null);
    return;
  }
  state.pendingLineStartPx = null;
  $("showSeamToggle").checked = true;
  setManualMode("line_start");
});

$("applyLineBtn").addEventListener("click", () => {
  try {
    $("showSeamToggle").checked = true;
    post("/api/manual_seam_line", {
      roi: roi(),
      target_z_min_mm: targetZMinMm(),
      ...linePixelsPayload(),
    }, "applying line...").then(() => {
      restartStreamSoon(900);
    });
  } catch (error) {
    $("statusText").textContent = "invalid line";
    $("logBox").textContent = String(error);
  }
});

$("captureValidationBtn").addEventListener("click", () => {
  captureValidationSnapshot();
});

$("validationPointBtn").addEventListener("click", () => {
  setManualMode(state.manualMode === "validation" ? null : "validation");
});

$("validationErrorBtn").addEventListener("click", () => {
  post("/api/calibration_validation_error", {}, "checking error...");
});

$("cameraImage").addEventListener("click", (event) => {
  if (!state.manualMode || state.busy) return;
  const pointPx = imagePixelFromClick(event);
  if (!pointPx) {
    $("statusText").textContent = "click inside image";
    return;
  }
  const mode = state.manualMode;
  setManualMode(null);
  const payload = { roi: roi(), target_z_min_mm: targetZMinMm() };
  if (mode === "validation") {
    payload.point_px = pointPx;
    post("/api/calibration_target_from_pixel", payload, "converting point...");
    return;
  }
  $("showSeamToggle").checked = true;
  if (mode === "line_start") {
    state.pendingLineStartPx = pointPx;
    $("lineStartX").value = pointPx[0].toFixed(3);
    $("lineStartY").value = pointPx[1].toFixed(3);
    setManualMode("line_end");
    return;
  }
  if (mode === "line_end") {
    const start = state.pendingLineStartPx;
    if (!start) {
      $("statusText").textContent = "select line start first";
      setManualMode("line_start");
      return;
    }
    $("lineEndX").value = pointPx[0].toFixed(3);
    $("lineEndY").value = pointPx[1].toFixed(3);
    state.pendingLineStartPx = null;
    $("statusText").textContent = "line ready, confirm calibration";
    $("logBox").textContent = "已选择起点和终点。确认无误后点击“确认校准”，再重新生成目标坐标。";
    return;
  }
});

$("cameraImage").addEventListener("load", () => {
  if ($("cameraImage").dataset.stream === "1") {
    markStreamAlive();
  }
});

$("cameraImage").addEventListener("error", () => {
  const image = $("cameraImage");
  if (image.dataset.stream === "1") {
    retryStreamSoon();
  }
});

$("poseBtn").addEventListener("click", () => {
  post("/api/read_pose", {}, "reading TCP...");
});

$("moveXyzBtn").addEventListener("click", () => {
  try {
    post("/api/move_xyz", {
      ...manualXyzPayload(),
      ...motionPayload(),
    }, "move XYZ...");
  } catch (error) {
    $("statusText").textContent = "invalid XYZ";
    $("logBox").textContent = String(error);
  }
});

$("savePoseBtn").addEventListener("click", () => {
  post("/api/save_named_pose", {
    pose_name: $("poseName").value.trim(),
  }, "saving pose...");
});

$("movePoseBtn").addEventListener("click", () => {
  const poseName = $("savedPoseSelect").value;
  if (!poseName) {
    $("statusText").textContent = "select saved pose";
    return;
  }
  post("/api/move_named_pose", {
    pose_name: poseName,
    ...motionPayload(),
  }, "move saved pose...");
});

$("deletePoseBtn").addEventListener("click", () => {
  const poseName = $("savedPoseSelect").value;
  if (!poseName) {
    $("statusText").textContent = "select saved pose";
    return;
  }
  post("/api/delete_saved_pose", {
    pose_name: poseName,
  }, "deleting pose...");
});

$("moveStartBtn").addEventListener("click", () => {
  post("/api/move_point", { point_name: "start", ...motionPayload() }, "move start...");
});

$("moveEndBtn").addEventListener("click", () => {
  if (!useSegments()) {
    post("/api/move_point", { point_name: "end", ...motionPayload() }, "move end...");
    return;
  }
  try {
    post("/api/move_line_segments", {
      segment_mm: segmentMm(),
      ...motionPayload(),
    }, "move segments...");
  } catch (error) {
    $("statusText").textContent = "invalid segment";
    $("logBox").textContent = String(error);
  }
});

$("segmentToggle").addEventListener("change", () => {
  $("moveEndBtn").textContent = useSegments() ? "分段到终点" : "到终点";
});

$("restoreBtn").addEventListener("click", () => {
  const execute = $("executeToggle").checked;
  post("/api/restore", {
    execute,
    confirm: execute ? "RESTORE_CONTROL_MODE" : "",
  }, "restore...");
});

function jogPayload(button) {
  return {
    axis: button.dataset.axis,
    direction: Number(button.dataset.dir),
    step_mm: Number($("stepMm").value || 1),
    send_duration: 0.22,
    send_rate_hz: 100,
    ...motionPayload(),
  };
}

function jogRepeatDelayMs() {
  const value = Number($("jogIntervalMs").value || 80);
  return Math.max(50, Math.min(1000, value));
}

function continuousJogPayload(button) {
  const stepMm = Number($("stepMm").value || 1);
  const intervalMs = jogRepeatDelayMs();
  const speedMmS = Math.max(1, Math.min(80, stepMm * 1000 / intervalMs));
  return {
    axis: button.dataset.axis,
    direction: Number(button.dataset.dir),
    speed_mm_s: Number(speedMmS.toFixed(3)),
    ...motionPayload(),
  };
}

async function jogFetch(path, payload = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await response.json();
  showCoords(data);
  showLog(data);
  $("statusText").textContent = data.ok ? "jogging" : "jog failed";
  return data;
}

function startJogKeepalive() {
  clearInterval(state.jogKeepaliveTimer);
  state.jogKeepaliveTimer = setInterval(async () => {
    if (!state.jogHeld) return;
    try {
      const data = await jogFetch("/api/jog_keepalive");
      if (!data.ok) stopJog();
    } catch (error) {
      $("logBox").textContent = String(error);
      stopJog();
    }
  }, 120);
}

async function startContinuousJog(button) {
  if (state.jogRunning) return;
  state.jogRunning = true;
  $("statusText").textContent = "starting jog...";
  try {
    const data = await jogFetch("/api/jog_start", continuousJogPayload(button));
    if (!data.ok || data?.jog?.dry_run) {
      stopJog();
      return;
    }
    if (!state.jogHeld) {
      await jogFetch("/api/jog_stop");
      $("statusText").textContent = "jog stopped";
      return;
    }
    startJogKeepalive();
  } catch (error) {
    $("statusText").textContent = "jog failed";
    $("logBox").textContent = String(error);
    stopJog();
  } finally {
    state.jogRunning = false;
  }
}

async function stopJog() {
  const wasHeld = Boolean(state.jogHeld);
  state.jogHeld?.classList.remove("active");
  state.jogHeld = null;
  clearTimeout(state.jogTimer);
  state.jogTimer = null;
  clearInterval(state.jogKeepaliveTimer);
  state.jogKeepaliveTimer = null;
  if (!wasHeld) return;
  try {
    const data = await jogFetch("/api/jog_stop");
    $("statusText").textContent = data.ok ? "jog stopped" : "jog stop failed";
  } catch (error) {
    $("statusText").textContent = "jog stop failed";
    $("logBox").textContent = String(error);
  }
}

document.querySelectorAll(".jog-grid button").forEach((button) => {
  button.addEventListener("pointerdown", (event) => {
    event.preventDefault();
    if (state.jogHeld) stopJog();
    state.jogHeld = button;
    button.classList.add("active");
    button.setPointerCapture?.(event.pointerId);
    startContinuousJog(button);
  });
  button.addEventListener("pointerup", stopJog);
  button.addEventListener("pointerleave", stopJog);
  button.addEventListener("pointercancel", stopJog);
  button.addEventListener("contextmenu", (event) => event.preventDefault());
});

window.addEventListener("beforeunload", () => {
  stopJog();
});
window.addEventListener("blur", stopJog);
window.addEventListener("pointerup", stopJog);

["roiX", "roiY", "roiW", "roiH", "showRoiToggle", "showBoxToggle", "showSeamToggle", "streamQuality"].forEach((id) => {
  $(id).addEventListener("change", restartStreamSoon);
});

$("streamToggle").addEventListener("change", () => {
  if (isStreamEnabled()) {
    startStream();
  } else {
    stopStream(true);
    $("statusText").textContent = "stream stopped";
  }
});

fetch("/api/status")
  .then((response) => response.json())
  .then((data) => {
    if (data.state?.roi) {
      const [x, y, w, h] = data.state.roi;
      $("roiX").value = x;
      $("roiY").value = y;
      $("roiW").value = w;
      $("roiH").value = h;
    }
    if (data.state?.target_z_min_mm !== undefined) {
      $("targetZMin").value = Number(data.state.target_z_min_mm).toFixed(3);
    }
    updateSavedPoseSelect(data.state?.saved_poses || []);
    if (isStreamEnabled()) {
      startStream();
    } else if (data.state?.last_image_url) {
      showImage(data.state.last_image_url);
    } else {
      post("/api/capture", { roi: roi() }, "capturing...");
    }
    post("/api/read_pose", {}, "reading TCP...");
  })
  .catch(() => {});
