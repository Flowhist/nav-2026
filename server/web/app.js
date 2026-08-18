function setNavPlacementMode(mode) {
  appState.navPlacementMode = mode;
  $("btnArmInit").classList.toggle("active", mode === "initial");
  $("btnArmGoal").classList.toggle("active", mode === "goal");
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
    showToast(appState.navPlacementMode === "initial" ? "初始位姿已发送" : "导航目标已发送", "ok");
  } catch (err) {
    reportActionError(err, "位姿指令发送失败");
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

function toggleNavSelect(panelId, force) {
  const panel = $(panelId);
  if (!panel) return;
  const shouldOpen = typeof force === "boolean" ? force : !panel.classList.contains("open");
  if (shouldOpen) {
    document.querySelectorAll(".nav-select").forEach((item) => {
      const open = item === panel;
      item.classList.toggle("open", open);
      $(item.dataset.control)?.setAttribute("aria-expanded", String(open));
    });
  } else {
    panel.classList.remove("open");
    $(panel.dataset.control)?.setAttribute("aria-expanded", "false");
  }
}

function confirmPageSwitchStop(mode, page) {
  const label = mode === "mapping" ? "建图" : "导航";
  const target = { mapping: "建图", navigation: "导航", preview: "地图预览", configs: "配置文件" }[page] || page;
  return showAnnotationConfirmDialog({
    title: `结束${label}会话`,
    message: mode === "mapping"
      ? `切换到“${target}”会结束当前建图，且不会自动保存地图。确认已经保存并继续？`
      : `切换到“${target}”会先结束当前导航会话，是否继续？`,
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
  toggleNavSelect("navMapPanel", false);
  toggleNavSelect("navLocationPanel", false);
  document.querySelectorAll(".tab").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.page === page);
  });
  document.querySelectorAll(".page").forEach((node) => {
    node.classList.toggle("active", node.id === `page-${page}`);
  });
  $("statusDock").classList.toggle("hidden", page === "preview" || page === "configs");
  if (!isLivePage()) setStatusExpanded(false);
  if (page === "preview") loadSavedMaps().catch(console.error);
  if (page === "navigation" && !appState.savedMaps.length) loadSavedMaps().catch(console.error);
  if (page === "configs") {
    showConfigOverview();
    loadConfigs().catch(console.error);
    loadSystemSupervisor().catch(console.error);
  }
  if (isLivePage() && $("statusDock").classList.contains("collapsed")) startSceneStream();
  else stopSceneStream();
  renderCurrentPageCanvases();
}

function setStatusExpanded(expanded) {
  const dock = $("statusDock");
  dock.classList.toggle("collapsed", !expanded);
  $("statusToggle").setAttribute("aria-expanded", String(expanded));
  $("dockToggleText").textContent = expanded ? "收起" : "展开";
  if (expanded) {
    stopSceneStream();
    if (appState.dockView !== "events") loadRuntimeLog(appState.dockView).catch(console.error);
  } else if (isLivePage()) {
    startSceneStream();
  }
}

function bind() {
  document.querySelectorAll(".tab").forEach((btn) => {
    btn.addEventListener("click", () => setPage(btn.dataset.page).catch(console.error));
  });

  document.querySelectorAll(".hint-toggle").forEach((btn) => {
    btn.addEventListener("click", () => toggleHelp(btn.dataset.help));
  });

  $("navMapToggle").addEventListener("click", () => {
    if (!appState.savedMaps.length) loadSavedMaps().catch(console.error);
    toggleNavSelect("navMapPanel");
  });
  document.addEventListener("click", (event) => {
    if (event.target.closest(".help-wrap")) return;
    document.querySelectorAll(".map-help").forEach((panel) => panel.classList.add("hidden"));
    if (!event.target.closest("#navMapPanel")) toggleNavSelect("navMapPanel", false);
    if (!event.target.closest("#navLocationPanel")) toggleNavSelect("navLocationPanel", false);
  });

  $("btnStartMapping").addEventListener("click", () => startRuntime("mapping").catch(console.error));
  $("btnStopMapping").addEventListener("click", async () => {
    const confirmed = await showAnnotationConfirmDialog({
      title: "结束建图",
      message: "结束建图不会自动保存地图。确认已经保存所需地图并结束当前会话？",
      confirmText: "结束建图",
      danger: true,
    });
    if (confirmed) await stopRuntime("mapping", { showModal: true });
  });
  $("btnStartNavigation").addEventListener("click", () => startRuntime("navigation").catch(console.error));
  $("btnStopNavigation").addEventListener("click", async () => {
    const confirmed = await showAnnotationConfirmDialog({
      title: "结束导航",
      message: "将停止当前导航链路并清理目标与路径状态，是否继续？",
      confirmText: "结束导航",
      danger: true,
    });
    if (confirmed) await stopRuntime("navigation", { showModal: true });
  });
  $("btnSaveMap").addEventListener("click", async () => {
    const button = $("btnSaveMap");
    try {
      const mapping = getRuntime("mapping");
      const name = await showMapSaveDialog(mapping.launch_args?.map_file || "manual_map");
      if (!name) return;
      setButtonBusy(button, true, "保存中…");
      await api("/api/map/save", "POST", { name });
      window.setTimeout(() => loadSavedMaps().catch(console.error), 1800);
      showToast("已请求保存地图", "ok");
    } catch (err) {
      reportActionError(err, "地图保存失败");
    } finally {
      setButtonBusy(button, false);
      renderRuntimeControls();
    }
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
      showToast("自动重定位已开始", "ok");
    } catch (err) {
      reportActionError(err, "重定位失败");
    }
  });
  $("btnCancelNav").addEventListener("click", async () => {
    try {
      setNavPlacementMode(null);
      appState.navDrag = null;
      await api("/api/nav/cancel", "POST", {});
      renderLiveCanvases();
      showToast("已发送停止导航指令", "ok");
    } catch (err) {
      reportActionError(err, "导航取消失败");
    }
  });

  const sendNavLocation = async () => {
    const input = $("navLocationInput");
    const name = (input.value || "").trim();
    if (!name) return;
    toggleNavSelect("navLocationPanel", false);
    try {
      await api("/api/nav/location", "POST", { name });
      showToast(`已发送地点“${name}”`, "ok");
    } catch (err) {
      reportActionError(err, "地点导航失败");
    }
  };
  $("btnNavLocation").addEventListener("click", () => sendNavLocation());
  const navLocationInput = $("navLocationInput");
  const showAllNavLocations = () => {
    renderNavLocationList();
    toggleNavSelect("navLocationPanel", true);
  };
  navLocationInput.addEventListener("focus", showAllNavLocations);
  navLocationInput.addEventListener("click", showAllNavLocations);
  navLocationInput.addEventListener("keydown", (event) => {
    if (event.key === "Enter") {
      event.preventDefault();
      sendNavLocation();
    } else if (event.key === "Escape") {
      event.preventDefault();
      toggleNavSelect("navLocationPanel", false);
    } else if (event.key === "ArrowDown") {
      event.preventDefault();
      showAllNavLocations();
    }
  });
  navLocationInput.addEventListener("input", () => {
    renderNavLocationList();
    renderRuntimeControls();
    renderLiveCanvases();
  });

  $("statusToggle").addEventListener("click", () => setStatusExpanded($("statusDock").classList.contains("collapsed")));
  $("statusClose").addEventListener("click", () => setStatusExpanded(false));

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
  $("btnRestartFinav").addEventListener("click", (event) => requestFinavPowerAction("restart", event.currentTarget));
  $("btnShutdownFinav").addEventListener("click", (event) => requestFinavPowerAction("shutdown", event.currentTarget));

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

  window.addEventListener("resize", () => {
    renderCurrentPageCanvases();
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) stopSceneStream();
    else if (isLivePage() && $("statusDock").classList.contains("collapsed")) startSceneStream();
  });
  window.addEventListener("pagehide", stopSceneStream);

  updateSceneHints();
  renderDockStream();
  renderRuntimeControls();
  loadSavedMaps().catch(console.error);
  setPage("mapping").catch(console.error);
}

bind();
pollStatus();
pollEvents();
pollDockLogs();
