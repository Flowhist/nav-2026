const $ = (id) => document.getElementById(id);

function createViewport() {
  return { zoom: 1, panX: 0, panY: 0, rotation: 0 };
}

const appState = {
  page: "mapping",
  lastSeq: 0,
  mapVersion: -1,
  status: null,
  scene: {
    map: null,
    scan: { points: [] },
    plan: { points: 0, points_xy: [] },
    robot_pose_map: null,
    goal_pose: null,
    initial_pose: null,
  },
  previewMap: null,
  previewMapName: "",
  navMapName: "",
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
  },
  teleop: {
    keyboardEnabled: false,
    currentCommand: "stop",
    speedStages: [0.1, 0.2, 0.4, 0.6],
    angularSpeed: 0.5,
    stageIndex: 1,
  },
  navPlacementMode: null,
  navDrag: null,
  viewportGesture: null,
  viewports: {
    mappingCanvas: createViewport(),
    navigationCanvas: createViewport(),
    previewCanvas: createViewport(),
  },
};

const mapRasterCache = new Map();
const stageMeta = [
  { label: "1档", desc: "慢速", speed: 0.10 },
  { label: "2档", desc: "常用", speed: 0.20 },
  { label: "3档", desc: "较快", speed: 0.40 },
  { label: "4档", desc: "高速", speed: 0.60 },
];
let configSaveNoticeTimer = null;

function fmt(n, digits = 2) {
  if (n === null || n === undefined || Number.isNaN(Number(n))) return "--";
  return Number(n).toFixed(digits);
}

async function api(path, method = "GET", body = null, options = {}) {
  const { headers = {}, ...rest } = options;
  const opt = {
    method,
    headers: { "Content-Type": "application/json", ...headers },
    ...rest,
  };
  if (body !== null && body !== undefined) opt.body = JSON.stringify(body);
  const res = await fetch(path, opt);
  if (!res.ok) throw new Error(`${res.status} ${res.statusText}`);
  return res.json();
}

function setText(id, text) {
  const el = $(id);
  if (el) el.textContent = text;
}

function setHidden(id, hidden) {
  const el = $(id);
  if (el) el.classList.toggle("hidden", hidden);
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function resizeCanvas(canvas) {
  if (!canvas) return false;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 1 || rect.height < 1) return false;
  const dpr = window.devicePixelRatio || 1;
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

  for (let y = 0; y < mapData.height; y += 1) {
    for (let x = 0; x < mapData.width; x += 1) {
      const src = y * mapData.width + x;
      const dstY = mapData.height - 1 - y;
      const dstX = mapData.width - 1 - x;
      const dst = (dstY * mapData.width + dstX) * 4;
      const value = mapData.data[src];
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
    x: view.offsetX + (view.maxX - x) * view.scale,
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
  const worldX = view.maxX - (basePoint.x - view.offsetX) / view.scale;
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
  const tip = {
    x: center.x + Math.cos(yaw) * length,
    y: center.y - Math.sin(yaw) * length,
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
  ctx.beginPath();
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(
    tip.x - Math.cos(yaw - Math.PI / 6) * wing,
    tip.y + Math.sin(yaw - Math.PI / 6) * wing,
  );
  ctx.lineTo(
    tip.x - Math.cos(yaw + Math.PI / 6) * wing,
    tip.y + Math.sin(yaw + Math.PI / 6) * wing,
  );
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

  if (scene?.scan?.points?.length) {
    ctx.save();
    ctx.fillStyle = "rgba(45, 155, 178, 0.75)";
    const radius = 1.8 * (window.devicePixelRatio || 1);
    scene.scan.points.forEach(([x, y]) => {
      const pt = worldToScreen(view, canvas, x, y);
      ctx.beginPath();
      ctx.arc(pt.x, pt.y, radius, 0, Math.PI * 2);
      ctx.fill();
    });
    ctx.restore();
  }

  if (scene?.initial_pose && options.showTargets) {
    drawArrow(ctx, view, canvas, scene.initial_pose, "#7a4f1b", "初始");
  }
  if (scene?.goal_pose && options.showTargets) {
    drawArrow(ctx, view, canvas, scene.goal_pose, "#bd5d38", "目标");
  }
  if (scene?.robot_pose_map) {
    drawArrow(ctx, view, canvas, scene.robot_pose_map, "#c46a2b");
  }

  if (options.dragPose) {
    drawArrow(ctx, view, canvas, options.dragPose, "rgba(28, 57, 48, 0.82)", options.dragPose.label);
  }
}

function renderStatus(status) {
  if (!status) return;
  const ros = status.ros || {};
  const robot = status.robot || {};
  const tf = ros.tf_hz || {};
  const hz = ros.topic_hz || {};
  const poseMap = robot.pose_map || {};
  const poseOdom = robot.pose_odom || {};
  const vel = robot.velocity || {};
  const plan = robot.plan || {};
  const teleop = status.teleop || {};
  const joystickState = teleop.joystick_online ? (teleop.joystick_active ? "激活中" : "关闭") : "离线";

  setText("rosConnected", ros.connected ? "在线" : "离线");
  setText("tfMapOdom", `${fmt(tf.map_odom)} Hz`);
  setText("tfOdomBase", `${fmt(tf.odom_base_link)} Hz`);
  setText("hzOdom", `${fmt(hz.odom)} Hz`);
  setText("hzPlan", `${fmt(hz.plan)} Hz`);
  setText("hzScan", `${fmt(hz.scan)} Hz`);
  setText("poseMap", `x=${fmt(poseMap.x, 3)}, y=${fmt(poseMap.y, 3)}, yaw=${fmt(poseMap.yaw_deg, 1)}°`);
  setText("poseOdom", `x=${fmt(poseOdom.x, 3)}, y=${fmt(poseOdom.y, 3)}, yaw=${fmt(poseOdom.yaw_deg, 1)}°`);
  setText("velOdom", `vx=${fmt(vel.vx, 3)} m/s, wz=${fmt(vel.wz, 3)} rad/s`);
  setText("planPts", String(plan.points ?? "--"));
  setText("planLen", `${fmt(plan.length_m, 2)} m`);
  setText("teleopState", teleop.active ? "发送中" : appState.teleop.keyboardEnabled ? "待命中" : "离线");
  setText("joystickState", joystickState);
  renderRuntimeControls();
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
