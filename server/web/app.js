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
  const enabled = !!appState.teleop.keyboardEnabled;
  document.querySelectorAll(".teleop-btn").forEach((btn) => {
    btn.disabled = !enabled;
  });
}

async function sendCmd(lin, ang, timeoutMs = 320) {
  await api("/api/teleop/cmd_vel", "POST", { linear_x: lin, angular_z: ang, timeout_ms: timeoutMs });
}

function stopTeleopLoop(sendStop = true) {
  if (appState.teleop.timer) {
    window.clearInterval(appState.teleop.timer);
    appState.teleop.timer = null;
  }
  appState.teleop.currentCommand = "stop";
  syncTeleopButtons();
  if (sendStop) api("/api/teleop/stop", "POST", {}).catch(console.error);
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

function startTeleopLoop(kind) {
  const [lin, ang] = resolveTeleopCommand(kind);
  stopTeleopLoop(false);
  appState.teleop.currentCommand = kind;
  syncTeleopButtons();
  sendCmd(lin, ang).catch(console.error);
  if (kind !== "stop") {
    appState.teleop.timer = window.setInterval(() => {
      sendCmd(lin, ang).catch(console.error);
    }, 140);
  } else {
    api("/api/teleop/stop", "POST", {}).catch(console.error);
  }
}

function setKeyboardTeleop(enabled) {
  appState.teleop.keyboardEnabled = enabled;
  $("teleopToggle").classList.toggle("active", enabled);
  $("teleopToggle").textContent = enabled ? "关闭" : "开启";
  $("teleopHint").textContent = enabled
    ? "开启状态\n输入一次持续运动，W/A/S/D 移动，Space 急停，J/K 调整前进档位"
    : "关闭状态\n开启后可用 W/A/S/D、Space、J/K 控制";
  if (!enabled) stopTeleopLoop(true);
  syncTeleopAvailability();
  renderStatus(appState.status);
}

function handleKeyboardTeleop(event) {
  if (!appState.teleop.keyboardEnabled) return;
  if (event.target && ["INPUT", "TEXTAREA"].includes(event.target.tagName)) return;

  const key = event.key.toLowerCase();
  if (key === "w") {
    event.preventDefault();
    startTeleopLoop("forward");
  } else if (key === "s") {
    event.preventDefault();
    
    startTeleopLoop("backward");
  } else if (key === "a") {
    event.preventDefault();
    startTeleopLoop("left");
  } else if (key === "d") {
    event.preventDefault();
    startTeleopLoop("right");
  } else if (key === " ") {
    event.preventDefault();
    startTeleopLoop("stop");
  } else if (key === "j") {
    event.preventDefault();
    appState.teleop.stageIndex = clamp(appState.teleop.stageIndex - 1, 0, appState.teleop.speedStages.length - 1);
    syncTeleopStage();
    if (["forward", "backward"].includes(appState.teleop.currentCommand)) startTeleopLoop(appState.teleop.currentCommand);
  } else if (key === "k") {
    event.preventDefault();
    appState.teleop.stageIndex = clamp(appState.teleop.stageIndex + 1, 0, appState.teleop.speedStages.length - 1);
    syncTeleopStage();
    if (["forward", "backward"].includes(appState.teleop.currentCommand)) startTeleopLoop(appState.teleop.currentCommand);
  }
}

function setNavPlacementMode(mode) {
  appState.navPlacementMode = mode;
  $("btnArmInit").classList.toggle("active", mode === "initial");
  $("btnArmGoal").classList.toggle("active", mode === "goal");
  $("navModeBadge").textContent = mode === "initial" ? "等待设置初始位姿" : mode === "goal" ? "等待设置目的地" : "当前未设置";
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

async function setPage(page) {
  if (page !== appState.page) {
    if (appState.page === "mapping" && page !== "mapping" && getRuntime("mapping").running) {
      await stopRuntime("mapping", { showModal: true });
    }
    if (appState.page === "navigation" && page !== "navigation" && getRuntime("navigation").running) {
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
  renderLiveCanvases();
  renderPreviewCanvas();
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
  document.querySelectorAll(".stage-option").forEach((btn) => {
    btn.addEventListener("click", () => {
      appState.teleop.stageIndex = Number(btn.dataset.stage);
      syncTeleopStage();
      toggleStageMenu(false);
      if (["forward", "backward"].includes(appState.teleop.currentCommand)) startTeleopLoop(appState.teleop.currentCommand);
    });
  });

  document.addEventListener("click", (event) => {
    if (event.target.closest(".help-wrap")) return;
    document.querySelectorAll(".map-help").forEach((panel) => panel.classList.add("hidden"));
    if (!event.target.closest("#stagePanel")) toggleStageMenu(false);
    if (!event.target.closest("#navMapPanel")) toggleNavMapMenu(false);
  });

  $("teleopToggle").addEventListener("click", () => setKeyboardTeleop(!appState.teleop.keyboardEnabled));
  $("btnStartMapping").addEventListener("click", () => startRuntime("mapping").catch(console.error));
  $("btnStopMapping").addEventListener("click", () => stopRuntime("mapping", { showModal: true }).catch(console.error));
  $("btnStartNavigation").addEventListener("click", () => startRuntime("navigation").catch(console.error));
  $("btnStopNavigation").addEventListener("click", () => stopRuntime("navigation", { showModal: true }).catch(console.error));
  $("btnSaveMap").addEventListener("click", async () => {
    const name = ($("mapName").value || "manual_map").trim();
    await api("/api/map/save", "POST", { name });
    window.setTimeout(() => loadSavedMaps().catch(console.error), 1800);
  });

  document.querySelectorAll(".teleop-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      if (!appState.teleop.keyboardEnabled) return;
      startTeleopLoop(btn.dataset.cmd);
    });
  });

  $("btnArmInit").addEventListener("click", () => setNavPlacementMode(appState.navPlacementMode === "initial" ? null : "initial"));
  $("btnArmGoal").addEventListener("click", () => setNavPlacementMode(appState.navPlacementMode === "goal" ? null : "goal"));
  $("btnCancelNav").addEventListener("click", async () => {
    setNavPlacementMode(null);
    appState.navDrag = null;
    await api("/api/nav/cancel", "POST", {});
    renderLiveCanvases();
  });

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
  $("btnRefreshConfigs").addEventListener("click", () => loadConfigs().catch(console.error));
  $("configBackBtn").addEventListener("click", showConfigOverview);
  $("configSaveBtn").addEventListener("click", () => saveConfigEditor().catch(console.error));
  $("configEditor").addEventListener("input", () => {
    renderConfigHighlight();
    setConfigSaveNotice("");
  });
  $("configEditor").addEventListener("scroll", syncConfigEditorScroll);

  bindCanvasInteractions("mappingCanvas");
  bindCanvasInteractions("navigationCanvas", { allowPlacement: true });
  bindCanvasInteractions("previewCanvas");

  window.addEventListener("pointermove", (event) => {
    updateViewportGesture(event);
    updateNavDrag(event);
  });

  window.addEventListener("pointerup", (event) => {
    endViewportGesture(event);
    finishNavDrag(event).catch(console.error);
  });

  window.addEventListener("pointercancel", (event) => {
    endViewportGesture(event);
    finishNavDrag(event).catch(console.error);
  });

  window.addEventListener("keydown", handleKeyboardTeleop);
  window.addEventListener("resize", () => {
    renderLiveCanvases();
    renderPreviewCanvas();
  });
  window.addEventListener("blur", () => {
    if (appState.teleop.keyboardEnabled) startTeleopLoop("stop");
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
