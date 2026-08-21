const $ = (id) => document.getElementById(id);

function createViewport() {
  return { zoom: 1, panX: 0, panY: 0, rotation: 0 };
}

const appState = {
  page: "mapping",
  lastSeq: 0,
  mapVersion: -1,
  planVersion: -1,
  status: null,
  scene: {
    map: null,
    scan: { count: 0, ranges_b64: "" },
    plan: { points: 0, points_xy: [] },
    robot_pose_map: null,
    goal_pose: null,
    initial_pose: null,
  },
  previewMap: null,
  previewMapName: "",
  previewLocations: [],
  previewSelectedLocation: "",
  previewAnnotationMode: false,
  previewAnnotationDraft: null,
  navMapName: "",
  navLocations: [],
  navLocationsFor: "",
  statusPanel: "overview",
  dockView: "events",
  dockAutoFollow: {
    events: true,
    mapping: true,
    navigation: true,
  },
  events: [],
  runtimeLogs: {
    mapping: { lines: [], updatedAt: null },
    navigation: { lines: [], updatedAt: null },
  },
  savedMaps: [],
  configs: [],
  configEditor: {
    name: "",
    path: "",
    content: "",
    impact: null,
  },
  stream: {
    source: null,
    mode: null,
    frames: 0,
    measuredHz: 0,
    latencyMs: null,
    lastFrameAt: null,
    sampleStartedAt: performance.now(),
  },
  navPlacementMode: null,
  navDestinationName: "",
  navDrag: null,
  viewportGesture: null,
  touchPointers: new Map(),
  touchGesture: null,
  viewports: {
    mappingCanvas: createViewport(),
    navigationCanvas: createViewport(),
    previewCanvas: createViewport(),
  },
};

const mapRasterCache = new Map();
let configSaveNoticeTimer = null;
let toastTimer = null;

function fmt(n, digits = 2) {
  if (n === null || n === undefined || Number.isNaN(Number(n))) return "--";
  return Number(n).toFixed(digits);
}

async function api(path, method = "GET", body = null, options = {}) {
  const { headers = {}, timeoutMs = 8000, ...rest } = options;
  const controller = new AbortController();
  const timeout = window.setTimeout(() => controller.abort(), timeoutMs);
  const opt = {
    method,
    headers: { "Content-Type": "application/json", ...headers },
    signal: controller.signal,
    ...rest,
  };
  if (body !== null && body !== undefined) opt.body = JSON.stringify(body);
  try {
    const res = await fetch(path, opt);
    if (!res.ok) {
      const err = new Error(await readApiError(res));
      err.status = res.status;
      setConnectionState("online");
      throw err;
    }
    setConnectionState("online");
    return res.json();
  } catch (err) {
    if (!err?.status) setConnectionState("offline");
    if (err?.name === "AbortError") throw new Error("请求超时，请检查 Finav 服务连接");
    throw err;
  } finally {
    window.clearTimeout(timeout);
  }
}

function setConnectionState(state) {
  const badge = $("connectionBadge");
  const label = $("connectionBadgeText");
  if (!badge || !label) return;
  badge.classList.remove("pending", "online", "offline");
  badge.classList.add(state);
  label.textContent = state === "online" ? "在线" : state === "offline" ? "连接中断" : "正在连接";
}

async function readApiError(res) {
  try {
    const data = await res.json();
    if (data && data.error) return String(data.error);
  } catch (_err) {
    // fall through to HTTP status text
  }
  return `${res.status} ${res.statusText}`;
}

function showToast(message, kind = "ok") {
  const node = $("toast");
  if (!node) return;
  if (toastTimer) window.clearTimeout(toastTimer);
  node.textContent = message;
  node.dataset.kind = kind;
  node.classList.remove("hidden");
  toastTimer = window.setTimeout(() => {
    node.classList.add("hidden");
    node.dataset.kind = "";
  }, 3200);
}

function reportActionError(err, fallback = "操作失败") {
  console.error(err);
  showToast(err?.message || fallback, "error");
}

function setText(id, text) {
  const el = $(id);
  if (el) el.textContent = text;
}

function setHidden(id, hidden) {
  const el = $(id);
  if (el) el.classList.toggle("hidden", hidden);
}

function setButtonBusy(button, busy, busyText = "处理中…") {
  if (!button) return;
  if (busy) {
    button.dataset.originalText = button.textContent;
    button.disabled = true;
    button.textContent = busyText;
    button.classList.add("busy");
  } else {
    button.textContent = button.dataset.originalText || button.textContent;
    delete button.dataset.originalText;
    button.classList.remove("busy");
  }
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function resizeCanvas(canvas) {
  if (!canvas) return false;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 1 || rect.height < 1) return false;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.round(rect.width * dpr);
  const height = Math.round(rect.height * dpr);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return true;
}

function getViewport(canvas) {
  return appState.viewports[canvas.id];
}

function resetViewport(canvasId) {
  appState.viewports[canvasId] = createViewport();
}

function getCanvasPointer(canvas, event) {
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 1 || rect.height < 1) return null;
  return {
    x: (event.clientX - rect.left) * (canvas.width / rect.width),
    y: (event.clientY - rect.top) * (canvas.height / rect.height),
  };
}

function getMapCacheKey(mapData, prefix) {
  if (!mapData) return `${prefix}-empty`;
  if (prefix === "live") return `${prefix}-${appState.mapVersion}`;
  return `${prefix}-${mapData.name || "preview"}-${mapData.width}-${mapData.height}`;
}

function buildMapRaster(mapData, prefix) {
  const cacheKey = getMapCacheKey(mapData, prefix);
  if (mapRasterCache.has(cacheKey)) return mapRasterCache.get(cacheKey);

  const offscreen = document.createElement("canvas");
  offscreen.width = mapData.width;
  offscreen.height = mapData.height;
  const ctx = offscreen.getContext("2d");
  const image = ctx.createImageData(mapData.width, mapData.height);

  let occupancy = mapData.data;
  if (!occupancy && mapData.encoding === "int8-base64" && mapData.data_b64) {
    const binary = window.atob(mapData.data_b64);
    const decoded = new Int8Array(binary.length);
    for (let index = 0; index < binary.length; index += 1) {
      const value = binary.charCodeAt(index);
      decoded[index] = value > 127 ? value - 256 : value;
    }
    occupancy = decoded;
  }
  occupancy = occupancy || [];

  for (let y = 0; y < mapData.height; y += 1) {
    for (let x = 0; x < mapData.width; x += 1) {
      const src = y * mapData.width + x;
      const dstY = prefix === "preview" ? y : mapData.height - 1 - y;
      const dstX = x;
      const dst = (dstY * mapData.width + dstX) * 4;
      const value = occupancy[src] ?? -1;
      let color;
      if (value < 0) color = [207, 215, 207];
      else if (value >= 65) color = [52, 70, 61];
      else color = [246, 246, 239];
      image.data[dst] = color[0];
      image.data[dst + 1] = color[1];
      image.data[dst + 2] = color[2];
      image.data[dst + 3] = 255;
    }
  }

  ctx.putImageData(image, 0, 0);
  mapRasterCache.set(cacheKey, offscreen);
  return offscreen;
}

function buildView(canvas, mapData) {
  const padding = 40 * (window.devicePixelRatio || 1);
  const worldWidth = mapData.width * mapData.resolution;
  const worldHeight = mapData.height * mapData.resolution;
  const scale = Math.min(
    (canvas.width - padding * 2) / Math.max(worldWidth, 0.001),
    (canvas.height - padding * 2) / Math.max(worldHeight, 0.001),
  );
  const drawWidth = worldWidth * scale;
  const drawHeight = worldHeight * scale;
  const offsetX = (canvas.width - drawWidth) / 2;
  const offsetY = (canvas.height - drawHeight) / 2;

  return {
    offsetX,
    offsetY,
    scale,
    drawWidth,
    drawHeight,
    minX: mapData.origin.x,
    minY: mapData.origin.y,
    maxX: mapData.origin.x + worldWidth,
    maxY: mapData.origin.y + worldHeight,
  };
}

function worldToBaseScreen(view, x, y) {
  return {
    x: view.offsetX + (x - view.minX) * view.scale,
    y: view.offsetY + view.drawHeight - (y - view.minY) * view.scale,
  };
}

function applyCameraToPoint(canvas, point) {
  const camera = getViewport(canvas);
  const centerX = canvas.width / 2;
  const centerY = canvas.height / 2;
  const dx = (point.x - centerX) * camera.zoom;
  const dy = (point.y - centerY) * camera.zoom;
  const cos = Math.cos(camera.rotation);
  const sin = Math.sin(camera.rotation);
  return {
    x: centerX + camera.panX + dx * cos - dy * sin,
    y: centerY + camera.panY + dx * sin + dy * cos,
  };
}

function invertCameraPoint(canvas, point) {
  const camera = getViewport(canvas);
  const centerX = canvas.width / 2;
  const centerY = canvas.height / 2;
  const tx = point.x - centerX - camera.panX;
  const ty = point.y - centerY - camera.panY;
  const cos = Math.cos(camera.rotation);
  const sin = Math.sin(camera.rotation);
  return {
    x: centerX + (tx * cos + ty * sin) / camera.zoom,
    y: centerY + (-tx * sin + ty * cos) / camera.zoom,
  };
}

function applyCameraToContext(ctx, canvas) {
  const camera = getViewport(canvas);
  const centerX = canvas.width / 2;
  const centerY = canvas.height / 2;
  ctx.translate(camera.panX, camera.panY);
  ctx.translate(centerX, centerY);
  ctx.rotate(camera.rotation);
  ctx.scale(camera.zoom, camera.zoom);
  ctx.translate(-centerX, -centerY);
}

function worldToScreen(view, canvas, x, y) {
  return applyCameraToPoint(canvas, worldToBaseScreen(view, x, y));
}

function clientToWorld(canvas, clientX, clientY) {
  const view = canvas._view;
  if (!view) return null;

  const rect = canvas.getBoundingClientRect();
  if (rect.width < 1 || rect.height < 1) return null;
  const screenPoint = {
    x: (clientX - rect.left) * (canvas.width / rect.width),
    y: (clientY - rect.top) * (canvas.height / rect.height),
  };
  const basePoint = invertCameraPoint(canvas, screenPoint);
  const worldX = view.minX + (basePoint.x - view.offsetX) / view.scale;
  const worldY = view.minY + (view.drawHeight - (basePoint.y - view.offsetY)) / view.scale;

  if (worldX < view.minX || worldX > view.maxX || worldY < view.minY || worldY > view.maxY) {
    return null;
  }
  return { x: worldX, y: worldY };
}

function drawArrow(ctx, view, canvas, pose, color, label) {
  if (!pose) return;
  const center = worldToScreen(view, canvas, pose.x, pose.y);
  const length = 18 * (window.devicePixelRatio || 1);
  const yaw = (pose.yaw_deg || 0) * Math.PI / 180;
  const camera = getViewport(canvas);
  const cosRot = Math.cos(camera.rotation);
  const sinRot = Math.sin(camera.rotation);
  const rotateVector = (dx, dy) => ({
    x: dx * cosRot - dy * sinRot,
    y: dx * sinRot + dy * cosRot,
  });
  const forward = rotateVector(Math.cos(yaw) * length, -Math.sin(yaw) * length);
  const tip = {
    x: center.x + forward.x,
    y: center.y + forward.y,
  };

  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 3 * (window.devicePixelRatio || 1);
  ctx.beginPath();
  ctx.moveTo(center.x, center.y);
  ctx.lineTo(tip.x, tip.y);
  ctx.stroke();

  const wing = 7 * (window.devicePixelRatio || 1);
  const leftWing = rotateVector(
    -Math.cos(yaw - Math.PI / 6) * wing,
    Math.sin(yaw - Math.PI / 6) * wing,
  );
  const rightWing = rotateVector(
    -Math.cos(yaw + Math.PI / 6) * wing,
    Math.sin(yaw + Math.PI / 6) * wing,
  );
  ctx.beginPath();
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(tip.x + leftWing.x, tip.y + leftWing.y);
  ctx.lineTo(tip.x + rightWing.x, tip.y + rightWing.y);
  ctx.closePath();
  ctx.fill();

  ctx.beginPath();
  ctx.arc(center.x, center.y, 5 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
  ctx.fill();

  if (label) {
    ctx.font = `${12 * (window.devicePixelRatio || 1)}px "Source Han Sans SC", sans-serif`;
    ctx.fillText(label, center.x + 10, center.y - 10);
  }
  ctx.restore();
}

function scanWorldPoints(scan) {
  if (!scan) return [];
  if (scan._decodedAt === scan.updated_at && scan._worldPoints) return scan._worldPoints;
  if (scan.encoding !== "uint16-mm-base64" || !scan.ranges_b64 || !scan.pose_map) return [];

  const binary = window.atob(scan.ranges_b64);
  const count = Math.floor(binary.length / 2);
  const pose = scan.pose_map;
  const poseYaw = (pose.yaw_deg || 0) * Math.PI / 180;
  const cosYaw = Math.cos(poseYaw);
  const sinYaw = Math.sin(poseYaw);
  const points = [];
  for (let index = 0; index < count; index += 1) {
    const offset = index * 2;
    const millimeters = binary.charCodeAt(offset) | (binary.charCodeAt(offset + 1) << 8);
    if (!millimeters) continue;
    const distance = millimeters / 1000;
    const angle = Number(scan.angle_min || 0) + index * Number(scan.angle_increment || 0);
    const localX = distance * Math.cos(angle);
    const localY = distance * Math.sin(angle);
    points.push([
      pose.x + localX * cosYaw - localY * sinYaw,
      pose.y + localX * sinYaw + localY * cosYaw,
    ]);
  }
  scan._decodedAt = scan.updated_at;
  scan._worldPoints = points;
  return points;
}

function drawRobotFootprint(ctx, view, canvas, pose, footprint = {}) {
  if (!pose) return;
  const yaw = (pose.yaw_deg || 0) * Math.PI / 180;
  const cos = Math.cos(yaw);
  const sin = Math.sin(yaw);
  const front = Number(footprint.front_m ?? 0.84);
  const rear = Number(footprint.rear_m ?? 0.25);
  const left = Number(footprint.left_m ?? 0.45);
  const right = Number(footprint.right_m ?? 0.45);
  const margin = Number(footprint.margin_m ?? 0.03);
  const project = (x, y) => worldToScreen(
    view,
    canvas,
    pose.x + x * cos - y * sin,
    pose.y + x * sin + y * cos,
  );
  const polygon = (extra) => [
    project(front + extra, left + extra),
    project(front + extra, -(right + extra)),
    project(-(rear + extra), -(right + extra)),
    project(-(rear + extra), left + extra),
  ];
  const trace = (points) => {
    ctx.beginPath();
    points.forEach((point, index) => index ? ctx.lineTo(point.x, point.y) : ctx.moveTo(point.x, point.y));
    ctx.closePath();
  };

  ctx.save();
  const body = polygon(0);
  trace(body);
  ctx.fillStyle = "rgba(196, 106, 43, 0.34)";
  ctx.strokeStyle = "#a84f1c";
  ctx.lineWidth = 2.2 * (window.devicePixelRatio || 1);
  ctx.fill();
  ctx.stroke();

  if (margin > 0) {
    trace(polygon(margin));
    ctx.setLineDash([5 * (window.devicePixelRatio || 1), 4 * (window.devicePixelRatio || 1)]);
    ctx.strokeStyle = "rgba(183, 58, 49, 0.75)";
    ctx.lineWidth = 1.2 * (window.devicePixelRatio || 1);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  const center = project(0, 0);
  const nose = project(front, 0);
  ctx.beginPath();
  ctx.moveTo(center.x, center.y);
  ctx.lineTo(nose.x, nose.y);
  ctx.strokeStyle = "#7f3210";
  ctx.lineWidth = 2 * (window.devicePixelRatio || 1);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(center.x, center.y, 3.5 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
  ctx.fillStyle = "#7f3210";
  ctx.fill();
  ctx.restore();
}

function drawScene(canvas, mapData, scene, options = {}) {
  if (!resizeCanvas(canvas)) return;
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  if (!mapData) {
    canvas._view = null;
    return;
  }

  const view = buildView(canvas, mapData);
  canvas._view = view;
  const raster = buildMapRaster(mapData, options.prefix || "live");

  ctx.save();
  applyCameraToContext(ctx, canvas);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(raster, view.offsetX, view.offsetY, view.drawWidth, view.drawHeight);
  ctx.strokeStyle = "rgba(21, 35, 27, 0.2)";
  ctx.lineWidth = 1;
  ctx.strokeRect(view.offsetX, view.offsetY, view.drawWidth, view.drawHeight);
  ctx.restore();

  if (options.showPlan && scene?.plan?.points_xy?.length) {
    ctx.save();
    ctx.strokeStyle = "rgba(37, 93, 77, 0.88)";
    ctx.lineWidth = 4 * (window.devicePixelRatio || 1);
    ctx.beginPath();
    scene.plan.points_xy.forEach(([x, y], index) => {
      const pt = worldToScreen(view, canvas, x, y);
      if (index === 0) ctx.moveTo(pt.x, pt.y);
      else ctx.lineTo(pt.x, pt.y);
    });
    ctx.stroke();
    ctx.restore();
  }

  const scanPoints = scanWorldPoints(scene?.scan);
  if (scanPoints.length) {
    ctx.save();
    ctx.fillStyle = "rgba(45, 155, 178, 0.75)";
    const radius = 1.8 * (window.devicePixelRatio || 1);
    ctx.beginPath();
    scanPoints.forEach(([x, y]) => {
      const pt = worldToScreen(view, canvas, x, y);
      ctx.moveTo(pt.x + radius, pt.y);
      ctx.arc(pt.x, pt.y, radius, 0, Math.PI * 2);
    });
    ctx.fill();
    ctx.restore();
  }

  if (scene?.initial_pose && options.showTargets) {
    drawArrow(ctx, view, canvas, scene.initial_pose, "#7a4f1b", "初始");
  }
  if (scene?.goal_pose && options.showTargets) {
    drawArrow(ctx, view, canvas, scene.goal_pose, "#bd5d38", "目标");
  }
  if (scene?.robot_pose_map) {
    drawRobotFootprint(ctx, view, canvas, scene.robot_pose_map, scene.robot_footprint);
  }

  if (options.locations?.length) {
    options.locations.forEach((loc) => {
      const color = loc.name === options.selectedLocation ? "#ff9f1c" : "#d83b2d";
      drawArrow(ctx, view, canvas, loc, color, loc.name);
    });
  }

  if (options.dragPose) {
    drawArrow(ctx, view, canvas, options.dragPose, "rgba(28, 57, 48, 0.82)", options.dragPose.label);
  }

  if (options.annotationDraft) {
    drawArrow(ctx, view, canvas, options.annotationDraft, "#1d6fc2", "新地点");
  }
}

function renderStatus(status) {
  if (!status) return;
  const ros = status.ros || {};
  const robot = status.robot || {};
  const tf = ros.tf_hz || {};
  const hz = ros.topic_hz || {};
  const plan = robot.plan || {};
  const control = status.control || {};
  if (typeof renderManualControlStatus === "function") renderManualControlStatus(control);
  const runtime = status.runtime || {};
  const lastSeen = ros.last_seen || {};
  const joystickState = control.joystick_online ? (control.joystick_active ? "正在接管" : "在线") : "离线";
  const age = (key) => Number(lastSeen[key]);
  const fresh = (key, limit = 1.5) => Number.isFinite(age(key)) && age(key) <= limit;
  const ageText = (key) => Number.isFinite(age(key)) ? `${fmt(age(key), age(key) < 10 ? 1 : 0)} 秒前` : "未收到";
  const setDataState = (id, ok, idle = false) => {
    const node = $(id);
    if (!node) return;
    node.textContent = idle ? "待命" : ok ? "正常" : "异常";
    node.dataset.state = idle ? "idle" : ok ? "ok" : "error";
  };
  const runtimeLabel = (item) => item?.stopping ? "停止中" : item?.running ? "运行中" : "未运行";
  const mapReceived = Number.isFinite(age("map"));
  const mappingReady = !!ros.connected && fresh("scan") && fresh("odom") && fresh("tf_odom_base_link");
  const navigationReady = mappingReady && mapReceived && fresh("tf_map_odom", 3.0);
  const activeRuntime = runtime.mapping?.running ? "建图运行中" : runtime.navigation?.running ? "导航运行中" : "无建图/导航任务";
  const overallOk = !!ros.connected && (fresh("scan") || (!runtime.mapping?.running && !runtime.navigation?.running));

  setConnectionState(ros.connected ? "online" : "offline");

  setText("rosConnected", ros.connected ? "在线" : "离线");
  setText("tfMapOdom", `${fmt(tf.map_odom)} Hz`);
  setText("tfOdomBase", `${fmt(tf.odom_base_link)} Hz`);
  setText("hzOdom", `${fmt(hz.odom)} Hz`);
  setText("hzPlan", `${fmt(hz.plan)} Hz`);
  setText("hzScan", `${fmt(hz.scan)} Hz`);
  setText("hzMap", "Event");
  setText("ageMap", ageText("map"));
  setText("ageScan", ageText("scan"));
  setText("ageOdom", ageText("odom"));
  setText("ageTfMap", ageText("tf_map_odom"));
  setText("ageTfOdom", ageText("tf_odom_base_link"));
  setText("agePlan", ageText("plan"));
  setDataState("stateMap", mapReceived, !runtime.mapping?.running && !runtime.navigation?.running);
  setDataState("stateScan", fresh("scan"), !runtime.mapping?.running && !runtime.navigation?.running);
  setDataState("stateOdom", fresh("odom"), !runtime.mapping?.running && !runtime.navigation?.running);
  setDataState("stateTfMap", fresh("tf_map_odom", 3.0), !runtime.mapping?.running && !runtime.navigation?.running);
  setDataState("stateTfOdom", fresh("tf_odom_base_link"), !runtime.mapping?.running && !runtime.navigation?.running);
  setDataState("statePlan", fresh("plan", 3.0), !runtime.navigation?.running || Number(plan.points || 0) === 0);
  setText("joystickState", joystickState);
  setText("runtimeMapping", runtimeLabel(runtime.mapping));
  setText("runtimeNavigation", runtimeLabel(runtime.navigation));
  setText("runtimeRelocate", runtimeLabel(runtime.relocate));
  setText("overallHealth", overallOk ? "运行正常" : "需要检查");
  setText("dockHealthSummary", `Scan ${fmt(hz.scan, 0)} Hz · Odom ${fmt(hz.odom, 0)} Hz · TF ${fresh("tf_odom_base_link") ? "正常" : "异常"}`);
  setText("dockHealthLabel", overallOk ? "系统正常" : "需要检查");
  setText("statusCenterSummary", `${activeRuntime}；${mappingReady ? "建图条件正常" : "建图条件待检查"}，${navigationReady ? "导航条件正常" : "导航条件待检查"}。`);
  setText("overviewScanHz", `${fmt(hz.scan)} Hz`);
  setText("overviewOdomHz", `${fmt(hz.odom)} Hz`);
  setText("overviewTfState", fresh("tf_odom_base_link") ? "正常" : "异常");
  setText("mappingSensorScan", fresh("scan") ? "正常" : "异常");
  setText("mappingSensorOdom", fresh("odom") ? "正常" : "异常");
  setText("mappingSensorTf", fresh("tf_odom_base_link") ? "正常" : "异常");
  [
    ["mappingSensorScan", fresh("scan")],
    ["mappingSensorOdom", fresh("odom")],
    ["mappingSensorTf", fresh("tf_odom_base_link")],
  ].forEach(([id, ok]) => {
    const node = $(id);
    if (node) node.dataset.state = ok ? "ok" : "error";
  });
  if (runtime.mapping?.running) {
    setText("mappingElapsed", `已运行 ${formatClockDuration(Date.now() / 1000 - Number(runtime.mapping.started_at || Date.now() / 1000))}`);
  }
  setText("navTaskGoal", appState.navDestinationName || (robot.goal_pose ? "地图目标" : "等待目标"));
  setText("navTaskPlan", Number(plan.length_m) > 0 ? `${fmt(plan.length_m, 1)} m` : "规划中");
  setText("navTaskState", robot.goal_pose || Number(plan.points || 0) > 0 ? "导航中" : "等待目标");

  const healthDot = $("dockHealthDot");
  if (healthDot) healthDot.className = `health-dot ${overallOk ? "ok" : "warn"}`;
  const drawerHealthDot = $("drawerHealthDot");
  if (drawerHealthDot) drawerHealthDot.className = `health-dot ${overallOk ? "ok" : "warn"}`;
  renderRuntimeControls();
}

function formatClockDuration(seconds) {
  const value = Math.max(0, Math.floor(Number(seconds) || 0));
  const hours = Math.floor(value / 3600);
  const minutes = Math.floor(value % 3600 / 60);
  const secs = value % 60;
  return hours > 0
    ? `${String(hours).padStart(2, "0")}:${String(minutes).padStart(2, "0")}:${String(secs).padStart(2, "0")}`
    : `${String(minutes).padStart(2, "0")}:${String(secs).padStart(2, "0")}`;
}

function formatExtra(extra) {
  if (!extra || !Object.keys(extra).length) return "";
  const pairs = Object.entries(extra).map(([key, value]) => `${key}=${value}`);
  return ` (${pairs.join(", ")})`;
}

function escapeHtml(text) {
  return String(text)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

function formatTs(ts) {
  if (!ts) return "--";
  const d = new Date(Number(ts) * 1000);
  const hh = String(d.getHours()).padStart(2, "0");
  const mm = String(d.getMinutes()).padStart(2, "0");
  const ss = String(d.getSeconds()).padStart(2, "0");
  return `${hh}:${mm}:${ss}`;
}

function normalizeLevel(level) {
  return String(level || "info").toLowerCase();
}

function deriveLogLevel(line) {
  const text = String(line || "");
  if (/\berror\b|\bfailed\b|\bfatal\b/i.test(text)) return "err";
  if (/\bwarn\b|warning/i.test(text)) return "warn";
  if (/\binfo\b|启动|started|finished|cleanup/i.test(text)) return "info";
  return "raw";
}

function stripAnsi(text) {
  return String(text || "")
    .replace(/\x1b\[[0-9;?]*[ -/]*[@-~]/gi, "")
    .replace(/[\u0000-\u0008\u000b-\u001f\u007f-\u009f]/g, "");
}

function normalizeLogLine(text) {
  return stripAnsi(text).replace(/\r/g, "");
}

function isNearBottom(node, threshold = 28) {
  return node.scrollHeight - node.scrollTop - node.clientHeight <= threshold;
}
