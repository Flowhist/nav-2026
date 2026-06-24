function syncTeleopStage() {
  const idx = appState.teleop.stageIndex;
  const meta = stageMeta[idx] || { label: `${idx + 1}档`, desc: "", speed: appState.teleop.speedStages[idx] || 0 };
  $("speedStageLabel").textContent = `${meta.label} · ${meta.speed.toFixed(2)} m/s · ${appState.teleop.angularSpeed.toFixed(2)} rad/s`;
  document.querySelectorAll("#stagePanel .stage-option").forEach((btn) => {
    const stageIndex = Number(btn.dataset.stage);
    const stage = stageMeta[stageIndex] || { label: `${stageIndex + 1}档`, desc: "", speed: appState.teleop.speedStages[stageIndex] || 0 };
    btn.classList.toggle("active", stageIndex === idx);
    const descNode = btn.querySelector("span");
    if (descNode) {
      descNode.textContent = `${stage.desc} · ${stage.speed.toFixed(2)} m/s · ${appState.teleop.angularSpeed.toFixed(2)} rad/s`;
    }
  });
}

function syncTeleopButtons() {
  const current = appState.teleop.currentCommand || "stop";
  document.querySelectorAll(".teleop-btn").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.cmd === current);
  });
}

function syncTeleopAvailability() {
  const joystickActive = !!appState.status?.teleop?.joystick_active;
  const enabled = !!appState.teleop.keyboardEnabled && !joystickActive;
  const panel = $("teleopPanel");
  const blocker = $("teleopBlocker");

  panel?.classList.toggle("teleop-locked", joystickActive);
  blocker?.classList.toggle("hidden", !joystickActive);
  if ($("teleopToggle")) $("teleopToggle").disabled = joystickActive;
  if ($("speedStageToggle")) $("speedStageToggle").disabled = joystickActive;
  document.querySelectorAll("#stagePanel .stage-option").forEach((btn) => {
    btn.disabled = joystickActive;
  });
  document.querySelectorAll(".teleop-btn").forEach((btn) => {
    btn.disabled = !enabled;
  });
  if (joystickActive) {
    toggleStageMenu(false);
    stopTeleop(false);
  }
}

async function applyTeleopCommand(kind, options = {}) {
  const { force = false } = options;
  const command = kind || "stop";
  if (!force && appState.teleop.currentCommand === command) return;

  appState.teleop.currentCommand = command;
  syncTeleopButtons();

  if (command === "stop") {
    await api("/api/teleop/stop", "POST", {});
    return;
  }

  const [lin, ang] = resolveTeleopCommand(command);
  await api("/api/teleop/state", "POST", {
    command,
    linear_x: lin,
    angular_z: ang,
  });
}

function stopTeleop(sendStop = true) {
  appState.teleop.currentCommand = "stop";
  syncTeleopButtons();
  if (sendStop) api("/api/teleop/stop", "POST", {}).catch(console.error);
}

function sendTeleopStopBeacon() {
  if (!navigator.sendBeacon) return;
  const body = new Blob(["{}"], { type: "application/json" });
  navigator.sendBeacon("/api/teleop/stop", body);
}

function resolveTeleopCommand(kind) {
  const linear = appState.teleop.speedStages[appState.teleop.stageIndex] || 0;
  const angular = appState.teleop.angularSpeed || 0;
  const mapping = {
    forward: [linear, 0],
    backward: [-linear, 0],
    left: [0, angular],
    right: [0, -angular],
    stop: [0, 0],
  };
  return mapping[kind] || [0, 0];
}

async function loadTeleopConfig() {
  try {
    const data = await api("/api/configs/base_control.yaml");
    const content = String(data.content || "");
    const angMatch = content.match(/^\s*keyboard_angular_speed\s*:\s*([0-9]+(?:\.[0-9]+)?)/m);
    if (angMatch) {
      const value = Number(angMatch[1]);
      if (Number.isFinite(value) && value > 0) {
        appState.teleop.angularSpeed = value;
      }
    }

    const speedMatch = content.match(/^\s*keyboard_linear_speeds\s*:\s*\[([^\]]+)\]/m);
    if (speedMatch) {
      const speeds = speedMatch[1]
        .split(",")
        .map((item) => Number(item.trim()))
        .filter((item) => Number.isFinite(item) && item > 0);
      if (speeds.length) {
        appState.teleop.speedStages = speeds;
        appState.teleop.stageIndex = Math.min(appState.teleop.stageIndex, speeds.length - 1);
      }
    }
  } catch (err) {
    console.error(err);
  }
  syncTeleopStage();
}

function setKeyboardTeleop(enabled, options = {}) {
  const { sendStop = true } = options;
  if (enabled && appState.status?.teleop?.joystick_active) {
    enabled = false;
  }
  appState.teleop.keyboardEnabled = enabled;
  $("teleopToggle").classList.toggle("active", enabled);
  $("teleopToggle").textContent = enabled ? "关闭" : "开启";
  $("teleopHint").textContent = enabled
    ? "开启状态\n输入一次持续运动，W/A/S/D 移动，Space 急停，J/K 调整前进档位"
    : "关闭状态\n开启后可用 W/A/S/D、Space、J/K 控制";
  if (!enabled) stopTeleop(sendStop);
  syncTeleopAvailability();
  if (!appState.status?.teleop?.joystick_active) renderStatus(appState.status);
}

function handleKeyboardTeleop(event) {
  if (!appState.teleop.keyboardEnabled || appState.status?.teleop?.joystick_active) return;
  if (event.target && ["INPUT", "TEXTAREA"].includes(event.target.tagName)) return;

  const key = event.key.toLowerCase();
  if (key === "w") {
    event.preventDefault();
    applyTeleopCommand("forward").catch(console.error);
  } else if (key === "s") {
    event.preventDefault();
    applyTeleopCommand("backward").catch(console.error);
  } else if (key === "a") {
    event.preventDefault();
    applyTeleopCommand("left").catch(console.error);
  } else if (key === "d") {
    event.preventDefault();
    applyTeleopCommand("right").catch(console.error);
  } else if (key === " ") {
    event.preventDefault();
    applyTeleopCommand("stop", { force: true }).catch(console.error);
  } else if (key === "j") {
    event.preventDefault();
    appState.teleop.stageIndex = clamp(appState.teleop.stageIndex - 1, 0, appState.teleop.speedStages.length - 1);
    syncTeleopStage();
    if (["forward", "backward"].includes(appState.teleop.currentCommand)) {
      applyTeleopCommand(appState.teleop.currentCommand, { force: true }).catch(console.error);
    }
  } else if (key === "k") {
    event.preventDefault();
    appState.teleop.stageIndex = clamp(appState.teleop.stageIndex + 1, 0, appState.teleop.speedStages.length - 1);
    syncTeleopStage();
    if (["forward", "backward"].includes(appState.teleop.currentCommand)) {
      applyTeleopCommand(appState.teleop.currentCommand, { force: true }).catch(console.error);
    }
  }
}

function setNavPlacementMode(mode) {
  appState.navPlacementMode = mode;
  $("btnArmInit").classList.toggle("active", mode === "initial");
  $("btnArmGoal").classList.toggle("active", mode === "goal");
  $("btnMappingManualRelocate").classList.toggle(
    "active",
    appState.page === "mapping" && mode === "initial",
  );
  updateSceneHints();
}

function startViewportGesture(canvas, event, mode) {
  appState.viewportGesture = {
    canvasId: canvas.id,
    pointerId: event.pointerId,
    mode,
    lastX: event.clientX,
    lastY: event.clientY,
  };
  if (canvas.setPointerCapture) canvas.setPointerCapture(event.pointerId);
}

function updateViewportGesture(event) {
  const gesture = appState.viewportGesture;
  if (!gesture || gesture.pointerId !== event.pointerId) return;

  const canvas = $(gesture.canvasId);
  const rect = canvas.getBoundingClientRect();
  const scaleX = rect.width > 0 ? canvas.width / rect.width : 1;
  const scaleY = rect.height > 0 ? canvas.height / rect.height : 1;
  const dx = (event.clientX - gesture.lastX) * scaleX;
  const dy = (event.clientY - gesture.lastY) * scaleY;
  const viewport = getViewport(canvas);

  if (gesture.mode === "pan") {
    viewport.panX += dx;
    viewport.panY += dy;
  } else if (gesture.mode === "rotate") {
    viewport.rotation += dx * 0.0022;
  }

  gesture.lastX = event.clientX;
  gesture.lastY = event.clientY;
  renderCanvasById(canvas.id);
}

function endViewportGesture(event) {
  const gesture = appState.viewportGesture;
  if (!gesture || gesture.pointerId !== event.pointerId) return;
  const canvas = $(gesture.canvasId);
  if (canvas.releasePointerCapture) {
    try {
      canvas.releasePointerCapture(event.pointerId);
    } catch (_err) {
      // ignore
    }
  }
  appState.viewportGesture = null;
}

function startNavDrag(canvas, event) {
  const world = clientToWorld(canvas, event.clientX, event.clientY);
  if (!world) return false;
  appState.navDrag = {
    canvasId: canvas.id,
    pointerId: event.pointerId,
    start: world,
    current: world,
    yawDeg: 0,
  };
  if (canvas.setPointerCapture) canvas.setPointerCapture(event.pointerId);
  renderLiveCanvases();
  return true;
}

function updateNavDrag(event) {
  if (!appState.navDrag || appState.navDrag.pointerId !== event.pointerId) return;
  const canvas = $(appState.navDrag.canvasId);
  const world = clientToWorld(canvas, event.clientX, event.clientY);
  if (!world) return;
  appState.navDrag.current = world;
  appState.navDrag.yawDeg = Math.atan2(world.y - appState.navDrag.start.y, world.x - appState.navDrag.start.x) * 180 / Math.PI;
  renderLiveCanvases();
}

async function finishNavDrag(event) {
  if (!appState.navDrag || appState.navDrag.pointerId !== event.pointerId) return;
  const drag = appState.navDrag;
  const canvas = $(drag.canvasId);
  const world = clientToWorld(canvas, event.clientX, event.clientY) || drag.current || drag.start;
  const dx = world.x - drag.start.x;
  const dy = world.y - drag.start.y;
  const yawDeg = Math.hypot(dx, dy) < 0.03 ? 0 : Math.atan2(dy, dx) * 180 / Math.PI;
  const body = { x: drag.start.x, y: drag.start.y, yaw_deg: yawDeg };
  const endpoint = appState.navPlacementMode === "initial" ? "/api/nav/initialpose" : "/api/nav/goal";
  if (
    endpoint === "/api/nav/initialpose"
    && appState.page === "mapping"
    && getRuntime("mapping").launch_args?.map_file
  ) {
    body.resume_mapping = true;
  }

  appState.navDrag = null;
  if (canvas.releasePointerCapture) {
    try {
      canvas.releasePointerCapture(event.pointerId);
    } catch (_err) {
      // ignore
    }
  }

  try {
    await api(endpoint, "POST", body);
  } catch (err) {
    console.error(err);
  } finally {
    setNavPlacementMode(null);
    renderLiveCanvases();
  }
}

function findPreviewLocationAt(canvas, event) {
  if (!appState.previewMap || !canvas._view) return "";
  const pointer = getCanvasPointer(canvas, event);
  if (!pointer) return "";

  let bestName = "";
  let bestDist = Infinity;
  appState.previewLocations.forEach((loc) => {
    const pt = worldToScreen(canvas._view, canvas, loc.x, loc.y);
    const dist = Math.hypot(pt.x - pointer.x, pt.y - pointer.y);
    if (dist < bestDist) {
      bestDist = dist;
      bestName = loc.name;
    }
  });
  return bestDist <= 18 * (window.devicePixelRatio || 1) ? bestName : "";
}

function selectPreviewLocation(name) {
  appState.previewSelectedLocation = name || "";
  renderPreviewLocationList();
  renderPreviewCanvas();
}

function startPreviewAnnotationDrag(canvas, event) {
  const world = clientToWorld(canvas, event.clientX, event.clientY);
  if (!world) return false;
  appState.previewAnnotationDraft = {
    canvasId: canvas.id,
    pointerId: event.pointerId,
    start: world,
    current: world,
    x: world.x,
    y: world.y,
    yaw_deg: 0,
  };
  if (canvas.setPointerCapture) canvas.setPointerCapture(event.pointerId);
  renderPreviewCanvas();
  return true;
}

function updatePreviewAnnotationDrag(event) {
  const draft = appState.previewAnnotationDraft;
  if (!draft || draft.pointerId !== event.pointerId) return;
  const canvas = $(draft.canvasId);
  const world = clientToWorld(canvas, event.clientX, event.clientY);
  if (!world) return;
  draft.current = world;
  draft.yaw_deg = Math.atan2(world.y - draft.start.y, world.x - draft.start.x) * 180 / Math.PI;
  renderPreviewCanvas();
}

function showAnnotationDialog(options) {
  return new Promise((resolve) => {
    const dialog = $("annotationDialog");
    const title = $("annotationDialogTitle");
    const meta = $("annotationDialogMeta");
    const nameField = $("annotationNameLabel");
    const input = $("annotationNameInput");
    const message = $("annotationDialogMessage");
    const cancel = $("annotationDialogCancel");
    const confirm = $("annotationDialogConfirm");
    const isNameMode = options.mode === "name";

    title.textContent = options.title || "地点标注";
    meta.textContent = options.meta || "";
    message.textContent = options.message || "";
    message.classList.remove("error");
    nameField.classList.toggle("hidden", !isNameMode);
    input.value = options.initialValue || "";
    confirm.textContent = options.confirmText || "确定";
    confirm.className = options.danger ? "danger" : "primary";

    const cleanup = () => {
      dialog.classList.add("hidden");
      dialog.setAttribute("aria-hidden", "true");
      confirm.removeEventListener("click", onConfirm);
      cancel.removeEventListener("click", onCancel);
      dialog.removeEventListener("keydown", onKeydown);
    };
    const close = (value) => {
      cleanup();
      resolve(value);
    };
    const onCancel = () => close(isNameMode ? null : false);
    const onConfirm = () => {
      if (!isNameMode) {
        close(true);
        return;
      }
      const value = input.value.trim();
      const error = options.validate ? options.validate(value) : "";
      if (error) {
        message.textContent = error;
        message.classList.add("error");
        input.focus();
        return;
      }
      close(value);
    };
    const onKeydown = (event) => {
      if (event.key === "Escape") {
        event.preventDefault();
        onCancel();
      } else if (event.key === "Enter") {
        event.preventDefault();
        onConfirm();
      }
    };

    confirm.addEventListener("click", onConfirm);
    cancel.addEventListener("click", onCancel);
    dialog.addEventListener("keydown", onKeydown);
    dialog.classList.remove("hidden");
    dialog.setAttribute("aria-hidden", "false");
    window.setTimeout(() => (isNameMode ? input : confirm).focus(), 0);
  });
}

function showAnnotationNameDialog({ x, y, yawDeg }) {
  return showAnnotationDialog({
    mode: "name",
    title: "添加地点",
    meta: `坐标 (${x.toFixed(2)}, ${y.toFixed(2)}) · 朝向 ${yawDeg.toFixed(1)}°`,
    message: "输入后会自动保存到当前地图的 locations 文件。",
    confirmText: "添加",
    validate: (value) => {
      if (!value) return "地点名称不能为空。";
      if (value.includes(":")) return "地点名称不能包含冒号。";
      return "";
    },
  });
}

function showMapSaveDialog(initialValue) {
  return showAnnotationDialog({
    mode: "name",
    title: "保存地图",
    meta: "保存栅格地图和可继续建图的 posegraph 数据",
    initialValue: initialValue || "manual_map",
    message: "输入地图名称；使用已有名称会覆盖对应地图文件。",
    confirmText: "保存",
    validate: (value) => {
      if (!value) return "地图名称不能为空。";
      if (!/^[A-Za-z0-9_-]+$/.test(value)) {
        return "地图名称只能包含字母、数字、下划线和连字符。";
      }
      return "";
    },
  });
}

function showAnnotationConfirmDialog({ title, message, confirmText = "确定", danger = false }) {
  return showAnnotationDialog({
    mode: "confirm",
    title,
    message,
    confirmText,
    danger,
  });
}

async function finishPreviewAnnotationDrag(event) {
  const draft = appState.previewAnnotationDraft;
  if (!draft || draft.pointerId !== event.pointerId) return;
  const canvas = $(draft.canvasId);
  const world = clientToWorld(canvas, event.clientX, event.clientY) || draft.current || draft.start;
  const dx = world.x - draft.start.x;
  const dy = world.y - draft.start.y;
  const yawDeg = Math.hypot(dx, dy) < 0.03 ? 0 : Math.atan2(dy, dx) * 180 / Math.PI;

  if (canvas.releasePointerCapture) {
    try {
      canvas.releasePointerCapture(event.pointerId);
    } catch (_err) {
      // ignore
    }
  }
  appState.previewAnnotationDraft = null;

  const cleanName = await showAnnotationNameDialog({
    x: draft.start.x,
    y: draft.start.y,
    yawDeg,
  });
  if (!cleanName) {
    renderPreviewCanvas();
    return;
  }
  const existing = appState.previewLocations.findIndex((loc) => loc.name === cleanName);
  if (existing >= 0) {
    const overwrite = await showAnnotationConfirmDialog({
      title: "覆盖地点",
      message: `地点“${cleanName}”已存在，是否用新的坐标和朝向覆盖？`,
      confirmText: "覆盖",
    });
    if (!overwrite) {
      renderPreviewCanvas();
      return;
    }
  }

  const loc = {
    name: cleanName,
    x: Number(draft.start.x.toFixed(3)),
    y: Number(draft.start.y.toFixed(3)),
    yaw_deg: Number(yawDeg.toFixed(1)),
  };
  if (existing >= 0) appState.previewLocations.splice(existing, 1, loc);
  else appState.previewLocations.push(loc);
  appState.previewSelectedLocation = cleanName;
  appState.previewAnnotationMode = false;
  renderPreviewLocationList();
  renderPreviewCanvas();
  savePreviewLocations();
}

function setPreviewAnnotationMode(enabled) {
  appState.previewAnnotationMode = !!enabled && !!appState.previewMapName;
  appState.previewAnnotationDraft = null;
  if (appState.previewAnnotationMode) appState.previewSelectedLocation = "";
  renderPreviewLocationList();
  renderPreviewCanvas();
}

async function deletePreviewSelectedLocation() {
  const name = appState.previewSelectedLocation;
  if (!name) return;
  const confirmed = await showAnnotationConfirmDialog({
    title: "删除地点",
    message: `确认删除地点“${name}”吗？删除后会自动保存。`,
    confirmText: "删除",
    danger: true,
  });
  if (!confirmed) return;
  appState.previewLocations = appState.previewLocations.filter((loc) => loc.name !== name);
  appState.previewSelectedLocation = "";
  renderPreviewLocationList();
  renderPreviewCanvas();
  savePreviewLocations();
}

function bindCanvasInteractions(canvasId, options = {}) {
  const canvas = $(canvasId);
  canvas.addEventListener("contextmenu", (event) => event.preventDefault());

  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    const pointer = getCanvasPointer(canvas, event);
    if (!pointer) return;

    const viewport = getViewport(canvas);
    const anchor = invertCameraPoint(canvas, pointer);
    const factor = event.deltaY < 0 ? 1.12 : 1 / 1.12;
    const nextZoom = clamp(viewport.zoom * factor, 0.45, 10);
    if (nextZoom === viewport.zoom) return;
    viewport.zoom = nextZoom;
    const movedAnchor = applyCameraToPoint(canvas, anchor);
    viewport.panX += pointer.x - movedAnchor.x;
    viewport.panY += pointer.y - movedAnchor.y;
    renderCanvasById(canvas.id);
  }, { passive: false });

  canvas.addEventListener("pointerdown", (event) => {
    if (event.button === 2) {
      startViewportGesture(canvas, event, "rotate");
      return;
    }

    if (event.button !== 0) return;

    if (options.allowPlacement && appState.navPlacementMode && appState.scene.map) {
      if (startNavDrag(canvas, event)) return;
    }
    if (options.allowPreviewAnnotation && appState.previewMap) {
      if (appState.previewAnnotationMode && startPreviewAnnotationDrag(canvas, event)) return;
      const selected = findPreviewLocationAt(canvas, event);
      if (selected) {
        selectPreviewLocation(selected);
        return;
      }
    }

    startViewportGesture(canvas, event, "pan");
  });
}

function toggleHelp(helpId) {
  document.querySelectorAll(".map-help").forEach((panel) => {
    panel.classList.toggle("hidden", panel.id !== helpId || !panel.classList.contains("hidden"));
  });
}

function toggleStageMenu(force) {
  const panel = $("stagePanel");
  const shouldOpen = typeof force === "boolean" ? force : !panel.classList.contains("open");
  panel.classList.toggle("open", shouldOpen);
  $("speedStageToggle").setAttribute("aria-expanded", String(shouldOpen));
}

function toggleNavMapMenu(force) {
  const panel = $("navMapPanel");
  const shouldOpen = typeof force === "boolean" ? force : !panel.classList.contains("open");
  panel.classList.toggle("open", shouldOpen);
  $("navMapToggle").setAttribute("aria-expanded", String(shouldOpen));
}

function toggleMappingMapMenu(force) {
  const panel = $("mappingMapPanel");
  const shouldOpen = typeof force === "boolean" ? force : !panel.classList.contains("open");
  panel.classList.toggle("open", shouldOpen);
  $("mappingMapToggle").setAttribute("aria-expanded", String(shouldOpen));
}

function confirmPageSwitchStop(mode, page) {
  const label = mode === "mapping" ? "建图" : "导航";
  const target = { mapping: "建图", navigation: "导航", preview: "地图预览", configs: "配置文件" }[page] || page;
  return showAnnotationConfirmDialog({
    title: `结束${label}会话`,
    message: `切换到“${target}”会先结束当前${label}会话，是否继续？`,
    confirmText: "继续切换",
    danger: true,
  });
}

async function setPage(page) {
  if (page !== appState.page) {
    if (appState.page === "mapping" && page !== "mapping" && getRuntime("mapping").running) {
      const confirmed = await confirmPageSwitchStop("mapping", page);
      if (!confirmed) return;
      await stopRuntime("mapping", { showModal: true });
    }
    if (appState.page === "navigation" && page !== "navigation" && getRuntime("navigation").running) {
      const confirmed = await confirmPageSwitchStop("navigation", page);
      if (!confirmed) return;
      await stopRuntime("navigation", { showModal: true });
    }
  }

  appState.page = page;
  document.querySelectorAll(".tab").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.page === page);
  });
  document.querySelectorAll(".page").forEach((node) => {
    node.classList.toggle("active", node.id === `page-${page}`);
  });
  $("statusDock").classList.toggle("hidden", page === "preview" || page === "configs");
  if (page === "preview") loadSavedMaps().catch(console.error);
  if (page === "navigation" && !appState.savedMaps.length) loadSavedMaps().catch(console.error);
  if (page === "configs") {
    showConfigOverview();
    loadConfigs().catch(console.error);
  }
  renderCurrentPageCanvases();
}

function bind() {
  document.querySelectorAll(".tab").forEach((btn) => {
    btn.addEventListener("click", () => setPage(btn.dataset.page).catch(console.error));
  });

  document.querySelectorAll(".hint-toggle").forEach((btn) => {
    btn.addEventListener("click", () => toggleHelp(btn.dataset.help));
  });

  $("speedStageToggle").addEventListener("click", () => toggleStageMenu());
  $("navMapToggle").addEventListener("click", () => {
    loadSavedMaps().catch(console.error);
    toggleNavMapMenu();
  });
  $("mappingMapToggle").addEventListener("click", () => {
    loadSavedMaps().catch(console.error);
    toggleMappingMapMenu();
  });
  document.querySelectorAll(".stage-option").forEach((btn) => {
    btn.addEventListener("click", () => {
      appState.teleop.stageIndex = Number(btn.dataset.stage);
      syncTeleopStage();
      toggleStageMenu(false);
      if (["forward", "backward"].includes(appState.teleop.currentCommand)) {
        applyTeleopCommand(appState.teleop.currentCommand, { force: true }).catch(console.error);
      }
    });
  });

  document.addEventListener("click", (event) => {
    if (event.target.closest(".help-wrap")) return;
    document.querySelectorAll(".map-help").forEach((panel) => panel.classList.add("hidden"));
    if (!event.target.closest("#stagePanel")) toggleStageMenu(false);
    if (!event.target.closest("#navMapPanel")) toggleNavMapMenu(false);
    if (!event.target.closest("#mappingMapPanel")) toggleMappingMapMenu(false);
  });

  $("teleopToggle").addEventListener("click", () => setKeyboardTeleop(!appState.teleop.keyboardEnabled));
  $("btnStartMapping").addEventListener("click", () => startRuntime("mapping").catch(console.error));
  $("btnStartContinueMapping").addEventListener("click", () => {
    startRuntime("mapping", { mapFile: appState.mappingMapName }).catch(console.error);
  });
  $("btnStopMapping").addEventListener("click", () => stopRuntime("mapping", { showModal: true }).catch(console.error));
  $("btnStartNavigation").addEventListener("click", () => startRuntime("navigation").catch(console.error));
  $("btnStopNavigation").addEventListener("click", () => stopRuntime("navigation", { showModal: true }).catch(console.error));
  $("btnSaveMap").addEventListener("click", async () => {
    try {
      const mapping = getRuntime("mapping");
      const name = await showMapSaveDialog(mapping.launch_args?.map_file || "manual_map");
      if (!name) return;
      await api("/api/map/save", "POST", { name });
      window.setTimeout(() => loadSavedMaps().catch(console.error), 1800);
      showToast("已请求保存地图", "ok");
    } catch (err) {
      reportActionError(err, "地图保存失败");
    }
  });

  document.querySelectorAll(".teleop-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      if (!appState.teleop.keyboardEnabled || appState.status?.teleop?.joystick_active) return;
      applyTeleopCommand(btn.dataset.cmd, { force: btn.dataset.cmd === "stop" }).catch(console.error);
    });
  });

  $("btnArmInit").addEventListener("click", () => setNavPlacementMode(appState.navPlacementMode === "initial" ? null : "initial"));
  $("btnArmGoal").addEventListener("click", () => setNavPlacementMode(appState.navPlacementMode === "goal" ? null : "goal"));
  $("btnRelocate").addEventListener("click", async () => {
    setNavPlacementMode(null);
    appState.navDrag = null;
    try {
      const body = appState.navMapName ? { map_file: appState.navMapName } : {};
      const data = await api("/api/nav/relocate", "POST", body);
      if (data.runtime) {
        appState.status = { ...(appState.status || {}), runtime: data.runtime };
        renderRuntimeControls();
      }
    } catch (err) {
      reportActionError(err, "重定位失败");
    }
  });
  $("btnMappingAutoRelocate").addEventListener("click", async () => {
    setNavPlacementMode(null);
    appState.navDrag = null;
    try {
      const data = await api("/api/nav/relocate", "POST", {
        map_file: appState.mappingMapName,
        resume_mapping: true,
      });
      if (data.runtime) {
        appState.status = { ...(appState.status || {}), runtime: data.runtime };
        renderRuntimeControls();
      }
    } catch (err) {
      reportActionError(err, "自动重定位失败");
    }
  });
  $("btnMappingManualRelocate").addEventListener("click", () => {
    setNavPlacementMode(appState.navPlacementMode === "initial" ? null : "initial");
  });
  $("btnCancelNav").addEventListener("click", async () => {
    try {
      setNavPlacementMode(null);
      appState.navDrag = null;
      await api("/api/nav/cancel", "POST", {});
      renderLiveCanvases();
    } catch (err) {
      reportActionError(err, "导航取消失败");
    }
  });

  const sendNavLocation = async () => {
    const input = $("navLocationInput");
    const name = (input.value || "").trim();
    if (!name) return;
    try {
      await api("/api/nav/location", "POST", { name });
    } catch (err) {
      reportActionError(err, "地点导航失败");
    }
  };
  $("btnNavLocation").addEventListener("click", () => sendNavLocation());
  $("navLocationInput").addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      sendNavLocation();
    }
  });
  $("navLocationInput").addEventListener("input", () => renderRuntimeControls());

  $("statusToggle").addEventListener("click", () => {
    const dock = $("statusDock");
    const collapsed = dock.classList.toggle("collapsed");
    $("statusToggle").setAttribute("aria-expanded", String(!collapsed));
    $("dockToggleText").textContent = collapsed ? "展开" : "收起";
  });

  document.querySelectorAll(".dock-tab").forEach((btn) => {
    btn.addEventListener("click", () => setDockView(btn.dataset.logView));
  });
  $("btnClearDockPane").addEventListener("click", clearDockPane);
  $("dockStream").addEventListener("scroll", (event) => {
    appState.dockAutoFollow[appState.dockView] = isNearBottom(event.currentTarget);
  });
  $("btnRefreshMaps").addEventListener("click", () => loadSavedMaps().catch(console.error));
  $("btnDeletePreviewMap").addEventListener("click", () => deleteSelectedPreviewMap().catch(console.error));
  $("btnAddPreviewLocation").addEventListener("click", () => setPreviewAnnotationMode(true));
  $("btnCancelPreviewLocation").addEventListener("click", () => setPreviewAnnotationMode(false));
  $("btnDeletePreviewLocation").addEventListener("click", () => deletePreviewSelectedLocation().catch(console.error));
  $("btnRefreshConfigs").addEventListener("click", () => loadConfigs().catch(console.error));
  $("configBackBtn").addEventListener("click", showConfigOverview);
  $("configSaveBtn").addEventListener("click", () => saveConfigEditor().catch(console.error));
  $("configEditor").addEventListener("input", () => {
    renderConfigHighlight();
    setConfigSaveNotice("");
  });
  $("configEditor").addEventListener("scroll", syncConfigEditorScroll);

  bindCanvasInteractions("mappingCanvas", { allowPlacement: true });
  bindCanvasInteractions("navigationCanvas", { allowPlacement: true });
  bindCanvasInteractions("previewCanvas", { allowPreviewAnnotation: true });

  window.addEventListener("pointermove", (event) => {
    updateViewportGesture(event);
    updateNavDrag(event);
    updatePreviewAnnotationDrag(event);
  });

  window.addEventListener("pointerup", (event) => {
    endViewportGesture(event);
    finishNavDrag(event).catch(console.error);
    finishPreviewAnnotationDrag(event).catch(console.error);
  });

  window.addEventListener("pointercancel", (event) => {
    endViewportGesture(event);
    finishNavDrag(event).catch(console.error);
    finishPreviewAnnotationDrag(event).catch(console.error);
  });

  window.addEventListener("keydown", handleKeyboardTeleop);
  window.addEventListener("resize", () => {
    renderCurrentPageCanvases();
  });
  window.addEventListener("blur", () => {
    if (appState.teleop.keyboardEnabled) stopTeleop(true);
  });
  window.addEventListener("pagehide", () => {
    if (appState.teleop.keyboardEnabled) sendTeleopStopBeacon();
  });

  syncTeleopStage();
  syncTeleopButtons();
  setKeyboardTeleop(false);
  syncTeleopAvailability();
  updateSceneHints();
  renderDockStream();
  renderRuntimeControls();
  loadTeleopConfig().catch(console.error);
  loadSavedMaps().catch(console.error);
  setPage("mapping").catch(console.error);
}

bind();
pollStatus();
pollEvents();
pollDockLogs();
pollScene();
