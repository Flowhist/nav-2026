function setNavPlacementMode(mode) {
  appState.navPlacementMode = mode;
  $("btnArmInit").classList.toggle("active", mode === "initial");
  $("btnMappingInitialPose").classList.toggle("active", mode === "initial" && appState.page === "mapping");
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

function updateViewportDisplay(canvasId) {
  const viewport = appState.viewports[canvasId];
  const toolbar = document.querySelector(`.canvas-toolbar[data-canvas="${canvasId}"]`);
  if (!viewport || !toolbar) return;
  const value = toolbar.querySelector(".zoom-value");
  if (value) value.textContent = `${Math.round(viewport.zoom * 100)}%`;
}

function changeViewport(canvasId, action) {
  const canvas = $(canvasId);
  const viewport = getViewport(canvas);
  if (!canvas || !viewport) return;
  if (action === "fit") {
    resetViewport(canvasId);
  } else if (action === "reset-rotation") {
    viewport.rotation = 0;
  } else {
    const factor = action === "zoom-in" ? 1.2 : 1 / 1.2;
    viewport.zoom = clamp(viewport.zoom * factor, 0.45, 10);
  }
  renderCanvasById(canvasId);
}

function trackTouchPointer(canvas, event) {
  if (event.pointerType !== "touch") return false;
  appState.touchPointers.set(event.pointerId, { canvasId: canvas.id, x: event.clientX, y: event.clientY });
  const points = [...appState.touchPointers.entries()].filter(([, point]) => point.canvasId === canvas.id);
  if (points.length > 1 && appState.navDrag) return true;
  if (points.length !== 2) return false;
  const [[idA, a], [idB, b]] = points;
  const viewport = getViewport(canvas);
  appState.viewportGesture = null;
  appState.touchGesture = {
    canvasId: canvas.id,
    pointerIds: [idA, idB],
    startDistance: Math.max(1, Math.hypot(b.x - a.x, b.y - a.y)),
    startCenterX: (a.x + b.x) / 2,
    startCenterY: (a.y + b.y) / 2,
    startZoom: viewport.zoom,
    startPanX: viewport.panX,
    startPanY: viewport.panY,
  };
  event.preventDefault();
  return true;
}

function updateTouchPointer(event) {
  const point = appState.touchPointers.get(event.pointerId);
  if (!point) return false;
  point.x = event.clientX;
  point.y = event.clientY;
  const gesture = appState.touchGesture;
  if (!gesture || !gesture.pointerIds.includes(event.pointerId)) return false;
  const a = appState.touchPointers.get(gesture.pointerIds[0]);
  const b = appState.touchPointers.get(gesture.pointerIds[1]);
  if (!a || !b) return false;
  const canvas = $(gesture.canvasId);
  const rect = canvas.getBoundingClientRect();
  const scaleX = rect.width > 0 ? canvas.width / rect.width : 1;
  const scaleY = rect.height > 0 ? canvas.height / rect.height : 1;
  const distance = Math.max(1, Math.hypot(b.x - a.x, b.y - a.y));
  const viewport = getViewport(canvas);
  viewport.zoom = clamp(gesture.startZoom * distance / gesture.startDistance, 0.45, 10);
  viewport.panX = gesture.startPanX + (((a.x + b.x) / 2) - gesture.startCenterX) * scaleX;
  viewport.panY = gesture.startPanY + (((a.y + b.y) / 2) - gesture.startCenterY) * scaleY;
  renderCanvasById(canvas.id);
  event.preventDefault();
  return true;
}

function endTouchPointer(event) {
  const gesture = appState.touchGesture;
  const wasPinching = !!gesture?.pointerIds.includes(event.pointerId);
  appState.touchPointers.delete(event.pointerId);
  if (wasPinching) appState.touchGesture = null;
  return wasPinching;
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
  const endpoint = appState.navPlacementMode === "initial"
    ? (appState.page === "mapping" ? "/api/mapping/initialpose" : "/api/nav/initialpose") : "/api/nav/goal";

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
    if (appState.navPlacementMode === "goal") appState.navDestinationName = "地图目标";
    showToast(appState.navPlacementMode === "initial" ? "初始位姿已发送" : "导航目标已发送", "ok");
  } catch (err) {
    reportActionError(err, "位姿指令发送失败");
  } finally {
    setNavPlacementMode(null);
    renderLiveCanvases();
  }
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
      } else if (event.key === "Enter" && !options.danger) {
        event.preventDefault();
        onConfirm();
      }
    };

    confirm.addEventListener("click", onConfirm);
    cancel.addEventListener("click", onCancel);
    dialog.addEventListener("keydown", onKeydown);
    dialog.classList.remove("hidden");
    dialog.setAttribute("aria-hidden", "false");
    window.setTimeout(() => (isNameMode ? input : options.danger ? cancel : confirm).focus(), 0);
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
    const editorHandlesPointer = canvasId === "previewCanvas"
      && appState.previewWorkspaceReady
      && event.pointerType !== "touch"
      && event.button === 0;
    if (editorHandlesPointer) return;
    if (trackTouchPointer(canvas, event)) return;
    if (event.button === 2) {
      startViewportGesture(canvas, event, "rotate");
      return;
    }

    if (event.button !== 0) return;
    if (canvasId === "previewCanvas" && appState.previewWorkspaceReady) return;

    if (options.allowPlacement && appState.navPlacementMode && appState.scene.map) {
      if (startNavDrag(canvas, event)) return;
    }
    startViewportGesture(canvas, event, "pan");
  });
}

function setStatusPanel(panel) {
  appState.statusPanel = panel;
  document.querySelectorAll(".status-tab").forEach((button) => {
    button.classList.toggle("active", button.dataset.statusPanel === panel);
    button.setAttribute("aria-selected", String(button.dataset.statusPanel === panel));
  });
  document.querySelectorAll(".status-panel").forEach((node) => node.classList.toggle("active", node.id === `statusPanel${panel[0].toUpperCase()}${panel.slice(1)}`));
  if (panel === "logs" && appState.dockView !== "events") loadRuntimeLog(appState.dockView).catch(console.error);
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
  if (appState.page === "preview" && page !== "preview" && appState.previewEditActive) {
    if (await window.finavMapEditor?.exit() === false) return;
  }
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
  $("navSpeedPanel").open = false;
  manualControlOnPageChange(page);
  toggleNavSelect("navMapPanel", false);
  toggleNavSelect("mappingMapPanel", false);
  toggleNavSelect("navLocationPanel", false);
  toggleNavSelect("navRoutePanel", false);
  toggleNavSelect("previewMapPanel", false);
  document.querySelectorAll(".tab").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.page === page);
  });
  document.querySelectorAll(".page").forEach((node) => {
    node.classList.toggle("active", node.id === `page-${page}`);
  });
  if (!isLivePage()) setStatusExpanded(false);
  if (page === "preview") loadSavedMaps().catch(console.error);
  if (page === "navigation" && !appState.savedMaps.length) loadSavedMaps().catch(console.error);
  if (page === "navigation") loadNavigationSpeed();
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
  $("dockToggleText").textContent = expanded ? "收起" : "状态中心 ›";
  if (expanded) {
    stopSceneStream();
    if (appState.dockView !== "events") loadRuntimeLog(appState.dockView).catch(console.error);
  } else if (isLivePage()) {
    startSceneStream();
  }
}

function bind() {
  bindManualControl();
  document.querySelectorAll(".tab").forEach((btn) => {
    btn.addEventListener("click", () => setPage(btn.dataset.page).catch(console.error));
  });

  document.querySelectorAll(".hint-toggle").forEach((btn) => {
    btn.addEventListener("click", () => toggleHelp(btn.dataset.help));
  });

  ["nav", "mapping"].forEach((prefix) => {
    const panel = $(`${prefix}MapPanel`);
    const control = $(`${prefix}MapToggle`);
    const menu = panel.querySelector(".nav-select-menu");
    const list = panel.querySelector(".nav-select-options");
    function positionMenu() {
      const rect = control.getBoundingClientRect();
      const boundary = panel.closest(".inspector").getBoundingClientRect();
      const below = Math.min(innerHeight - 12, boundary.bottom) - rect.bottom - 8;
      const above = rect.top - Math.max(12, boundary.top) - 8;
      const upward = below < Math.min(list.scrollHeight + 14, 254) && above > below;
      menu.style.position = "fixed";
      menu.style.width = `${Math.min(rect.width, innerWidth - 24)}px`;
      menu.style.left = `${Math.max(12, Math.min(rect.left, innerWidth - rect.width - 12))}px`;
      menu.style.right = "auto";
      menu.style.top = upward ? "auto" : `${rect.bottom + 6}px`;
      menu.style.bottom = upward ? `${innerHeight - rect.top + 6}px` : "auto";
      list.style.maxHeight = `${Math.max(40, Math.min(240, (upward ? above : below) - 14))}px`;
    }
    control.addEventListener("click", () => {
      if (!appState.savedMaps.length) loadSavedMaps().catch(console.error);
      toggleNavSelect(panel.id);
      if (panel.classList.contains("open")) positionMenu();
    });
    panel.addEventListener("keydown", (event) => {
      if (control.disabled) return;
      if (["ArrowDown", "ArrowUp", "Home", "End"].includes(event.key)) {
        event.preventDefault();
        event.stopPropagation();
        const options = Array.from(panel.querySelectorAll('[role="option"]'));
        const index = options.indexOf(document.activeElement);
        const open = panel.classList.contains("open");
        toggleNavSelect(panel.id, true);
        positionMenu();
        const next = event.key === "Home" ? 0 : event.key === "End" ? options.length - 1
          : !open || index < 0 ? Math.max(0, options.findIndex(item => item.getAttribute("aria-selected") === "true"))
          : (index + (event.key === "ArrowDown" ? 1 : -1) + options.length) % options.length;
        options[next]?.focus();
      } else if (event.key === "Escape" || event.key === "Tab") {
        toggleNavSelect(panel.id, false);
        if (event.key === "Escape") {
          event.preventDefault();
          event.stopPropagation();
          control.focus();
        }
      }
    });
    document.addEventListener("scroll", event => {
      if (panel.classList.contains("open") && !menu.contains(event.target)) positionMenu();
    }, true);
    window.addEventListener("resize", () => toggleNavSelect(panel.id, false));
  });
  $("previewMapToggle").addEventListener("click", () => {
    if (!appState.savedMaps.length) loadSavedMaps().catch(console.error);
    toggleNavSelect("previewMapPanel");
  });
  $("systemMenuToggle").addEventListener("click", (event) => {
    event.stopPropagation();
    const menu = $("systemMenu");
    const opening = menu.classList.contains("hidden");
    menu.classList.toggle("hidden", !opening);
    $("systemMenuToggle").setAttribute("aria-expanded", String(opening));
    if (opening) loadSystemSupervisor().catch(console.error);
  });
  document.addEventListener("click", (event) => {
    if (!event.target.closest(".help-wrap")) document.querySelectorAll(".map-help").forEach((panel) => panel.classList.add("hidden"));
    if (!event.target.closest("#navMapPanel")) toggleNavSelect("navMapPanel", false);
    if (!event.target.closest("#mappingMapPanel")) toggleNavSelect("mappingMapPanel", false);
    if (!event.target.closest("#navLocationPanel")) toggleNavSelect("navLocationPanel", false);
    if (!event.target.closest("#navRoutePanel")) toggleNavSelect("navRoutePanel", false);
    if (!event.target.closest("#previewMapPanel")) toggleNavSelect("previewMapPanel", false);
    if (!event.target.closest(".system-menu-wrap")) {
      $("systemMenu").classList.add("hidden");
      $("systemMenuToggle").setAttribute("aria-expanded", "false");
    }
  });

  $("btnStartMapping").addEventListener("click", () => startRuntime("mapping").catch(console.error));
  $("btnContinueMapping").addEventListener("click", () => {
    if (!appState.mappingMapName) return;
    startRuntime("mapping", appState.mappingMapName).catch(console.error);
  });
  $("btnMappingInitialPose").addEventListener("click", () => setNavPlacementMode(appState.navPlacementMode === "initial" ? null : "initial"));
  $("btnMappingRelocate").addEventListener("click", () => runMappingAction("relocate"));
  $("btnMappingResume").addEventListener("click", () => runMappingAction("resume"));
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
      title: "停止导航",
      message: "将停止当前导航链路并清理目标与路径状态，是否继续？",
      confirmText: "停止导航",
      danger: true,
    });
    if (confirmed) await stopRuntime("navigation", { showModal: true });
  });
  $("btnSaveMap").addEventListener("click", async () => {
    const button = $("btnSaveMap");
    try {
      const mapping = getRuntime("mapping");
      const name = await showMapSaveDialog(mapping.launch_args?.map_file ? `${mapping.launch_args.map_file}_extended` : "manual_map");
      if (!name) return;
      setButtonBusy(button, true, "保存中…");
      const result = await api("/api/map/save", "POST", { name });
      appState.mapSaveJob = result.job.id;
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
      appState.navDestinationName = "";
      if (appState.status?.robot) {
        appState.status.robot.goal_pose = null;
        appState.status.robot.plan = { points: 0, length_m: 0 };
      }
      renderLiveCanvases();
      renderRuntimeControls();
      showToast("已取消当前任务", "ok");
    } catch (err) {
      reportActionError(err, "导航取消失败");
    }
  });

  $("navRouteSelect").addEventListener("click", () => toggleNavSelect("navRoutePanel"));
  $("navRoutePanel").addEventListener("keydown", event => {
    if (event.key === "Escape") {
      event.preventDefault();
      toggleNavSelect("navRoutePanel", false);
      $("navRouteSelect").focus();
    } else if (["ArrowDown", "ArrowUp", "Home", "End"].includes(event.key)) {
      event.preventDefault();
      toggleNavSelect("navRoutePanel", true);
      const options = [...$("navRouteOptions").querySelectorAll("button")];
      const current = options.indexOf(document.activeElement);
      const index = event.key === "Home" ? 0 : event.key === "End" ? options.length - 1
        : event.key === "ArrowDown" ? (current + 1) % options.length : (current <= 0 ? options.length : current) - 1;
      options[index]?.focus();
    }
  });
  try {
    const saved = JSON.parse(localStorage.getItem("finav.navLayers") || "{}");
    Object.keys(appState.navLayers).forEach(key => {
      if (typeof saved?.[key] === "boolean") appState.navLayers[key] = saved[key];
    });
  } catch (_) { /* Storage may be unavailable in private browser sessions. */ }
  document.querySelectorAll("[data-nav-layer]").forEach(input => {
    input.checked = appState.navLayers[input.dataset.navLayer];
    input.addEventListener("change", () => {
      appState.navLayers[input.dataset.navLayer] = input.checked;
      try { localStorage.setItem("finav.navLayers", JSON.stringify(appState.navLayers)); } catch (_) {}
      renderLiveCanvases();
    });
  });
  document.querySelectorAll(".nav-map-popover").forEach(popover => {
    popover.addEventListener("toggle", () => {
      if (popover.open) document.querySelectorAll(".nav-map-popover").forEach(other => {
        if (other !== popover) other.open = false;
      });
    });
  });
  document.addEventListener("pointerdown", event => {
    if (!event.target.closest(".nav-map-popover")) {
      document.querySelectorAll(".nav-map-popover").forEach(popover => { popover.open = false; });
    }
  });
  document.addEventListener("keydown", event => {
    if (event.key === "Escape") document.querySelectorAll(".nav-map-popover").forEach(popover => {
      if (popover.open) { popover.open = false; popover.querySelector("summary").focus(); }
    });
  });
  for (const [id, action] of [["btnStartRoute", "start"], ["btnPauseRoute", "pause"], ["btnResumeRoute", "resume"]]) {
    $(id).addEventListener("click", async () => {
      $(id).disabled = true;
      try {
        await api("/api/nav/route", "POST", {
          action, map_name: getActiveNavigationMap(appState.navMapName), route_id: $("navRouteSelect").value,
        });
        if (action === "start") appState.navDestinationName = "";
      } catch (err) {
        reportActionError(err, "路线指令失败");
      } finally {
        renderRuntimeControls();
      }
    });
  }
  setInterval(() => {
    if (appState.page === "navigation" && getRuntime("navigation").running) loadNavMapObjects().catch(console.error);
  }, 5000);

  const sendNavLocation = async () => {
    const input = $("navLocationInput");
    const name = (input.value || "").trim();
    if (!name) return;
    toggleNavSelect("navLocationPanel", false);
    try {
      await api("/api/nav/location", "POST", { name });
      appState.navDestinationName = name;
      renderRuntimeControls();
      showToast(`已发送目的地“${name}”`, "ok");
    } catch (err) {
      reportActionError(err, "目的地导航请求失败");
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
  document.querySelectorAll(".status-tab").forEach((button) => {
    button.addEventListener("click", () => setStatusPanel(button.dataset.statusPanel));
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
  $("btnRefreshConfigs").addEventListener("click", () => loadConfigs().catch(console.error));
  $("configBackBtn").addEventListener("click", showConfigOverview);
  $("configSaveBtn").addEventListener("click", () => saveConfigEditor().catch(console.error));
  $("configEditor").addEventListener("input", () => {
    renderConfigHighlight();
    setConfigSaveNotice($("configEditor").value === appState.configEditor.content ? "" : "● 未保存", "dirty");
  });
  $("configEditor").addEventListener("scroll", syncConfigEditorScroll);
  $("btnRestartBaseControl").addEventListener("click", (event) => restartBaseControl(event.currentTarget));
  $("btnRestartFinav").addEventListener("click", (event) => requestFinavPowerAction("restart", event.currentTarget));
  $("btnShutdownFinav").addEventListener("click", (event) => requestFinavPowerAction("shutdown", event.currentTarget));

  bindCanvasInteractions("mappingCanvas", { allowPlacement: true });
  bindCanvasInteractions("navigationCanvas", { allowPlacement: true });
  bindCanvasInteractions("previewCanvas");

  document.querySelectorAll(".canvas-toolbar").forEach((toolbar) => {
    toolbar.addEventListener("click", (event) => {
      const button = event.target.closest("[data-view-action]");
      if (button) changeViewport(toolbar.dataset.canvas, button.dataset.viewAction);
    });
  });

  window.addEventListener("pointermove", (event) => {
    if (updateTouchPointer(event)) return;
    updateViewportGesture(event);
    updateNavDrag(event);
  });

  window.addEventListener("pointerup", (event) => {
    if (endTouchPointer(event)) return;
    endViewportGesture(event);
    finishNavDrag(event).catch(console.error);
  });

  window.addEventListener("pointercancel", (event) => {
    if (endTouchPointer(event)) return;
    endViewportGesture(event);
    finishNavDrag(event).catch(console.error);
  });

  window.addEventListener("resize", () => {
    renderCurrentPageCanvases();
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      stopManualControl("页面已隐藏");
      stopSceneStream();
    }
    else if (isLivePage() && $("statusDock").classList.contains("collapsed")) startSceneStream();
  });
  window.addEventListener("pagehide", () => {
    emergencyStopManualControl();
    stopSceneStream();
  });

  updateSceneHints();
  $("eventLevel").addEventListener("change", (event) => {
    appState.eventLevel = event.target.value;
    renderDockStream();
  });
  $("eventLimit").addEventListener("change", (event) => {
    appState.eventLimit = Number(event.target.value);
    if (appState.events.length > appState.eventLimit) appState.eventTruncated = true;
    appState.events = appState.events.slice(-appState.eventLimit);
    renderDockStream();
  });
  renderDockStream();
  renderRuntimeControls();
  loadSavedMaps().catch(console.error);
  loadSystemSupervisor().catch(console.error);
  setStatusPanel("overview");
  setPage("mapping").catch(console.error);
}

bind();
enhanceSelects();
pollStatus();
pollEvents();
pollDockLogs();
