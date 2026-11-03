function stopSceneStream() {
  if (appState.stream.renderFrame !== null) cancelAnimationFrame(appState.stream.renderFrame);
  appState.stream.finishRender?.();
  appState.stream.finishRender = null;
  appState.stream.renderFrame = null;
  if (appState.stream.source) appState.stream.source.close();
  appState.stream.source = null;
  setConnectionState(null, "stream");
}

function startSceneStream() {
  stopSceneStream();
  if (!isLivePage() || document.hidden) return;

  const mode = appState.page;
  appState.stream.frames = 0;
  appState.stream.sampleStartedAt = performance.now();

  const source = realtime.subscribe("scene", {
    hz: mode === "mapping" ? 10 : 15, mode,
    onstate: (state) => {
      if (appState.stream.source === source) setConnectionState(state, "stream");
    },
    ondata: (payload) => {
      if (appState.stream.source !== source) return;
      applyManualControlStatus(payload.control);
      appState.mapVersion = payload.map_version ?? appState.mapVersion;
      appState.planningMapVersion = payload.planning_map_version ?? appState.planningMapVersion;
      appState.planVersion = payload.plan_version ?? appState.planVersion;
      if (payload.map_changed) {
        appState.scene.map = payload.map;
        [...mapRasterCache.keys()].filter((key) => key.startsWith("live-")).forEach((key) => mapRasterCache.delete(key));
      }
      if (payload.planning_map_changed) {
        appState.scene.planning_map = payload.planning_map;
        [...mapRasterCache.keys()].filter((key) => key.startsWith("planning-")).forEach((key) => mapRasterCache.delete(key));
      }
      if (payload.scan) appState.scene.scan = payload.scan;
      if (payload.plan) appState.scene.plan = payload.plan;
      if ("robot_pose_map" in payload) appState.scene.robot_pose_map = payload.robot_pose_map;
      if ("goal_pose" in payload) appState.scene.goal_pose = payload.goal_pose;
      if ("initial_pose" in payload) appState.scene.initial_pose = payload.initial_pose;
      if (payload.robot_footprint) appState.scene.robot_footprint = payload.robot_footprint;

      const now = performance.now();
      appState.stream.frames += 1;
      const elapsed = now - appState.stream.sampleStartedAt;
      if (elapsed >= 1000) {
        appState.stream.measuredHz = appState.stream.frames * 1000 / elapsed;
        appState.stream.frames = 0;
        appState.stream.sampleStartedAt = now;
      }
      // Rendering completes before acknowledging, bounding browser work as well as network queues.
      return new Promise((resolve, reject) => {
        appState.stream.finishRender = resolve;
        appState.stream.renderFrame = requestAnimationFrame(() => {
          appState.stream.renderFrame = null;
          appState.stream.finishRender = null;
          try {
            if (appState.stream.source === source) { renderLiveCanvases(); updateSceneHints(); }
            resolve();
          } catch (error) { reject(error); }
        });
      });
    },
  });
  appState.stream.source = source;
}

function isLivePage() {
  return appState.page === "mapping" || appState.page === "navigation";
}

function resetLiveScene() {
  appState.mapVersion = -1;
  appState.planningMapVersion = -1;
  appState.planVersion = -1;
  appState.scene = {
    map: null,
    planning_map: null,
    scan: { count: 0, ranges: new Uint16Array() },
    plan: { points: 0, points_xy: [] },
    robot_pose_map: null,
    goal_pose: null,
    initial_pose: null,
  };
  appState.navDrag = null;
}

function updateSceneHints() {
  const map = appState.scene.map;
  const hasMap = !!map;
  $("mappingHint").textContent = appState.page === "mapping" && appState.navPlacementMode === "initial"
    ? "拖拽设置机器人位置与朝向"
    : hasMap
      ? `Scan ${fmt(appState.status?.ros?.topic_hz?.scan, 0)} Hz · Map ${map.width}×${map.height} · Stream ${fmt(appState.stream.measuredHz, 1)} Hz`
      : "等待 /map 数据…";

  if (appState.navPlacementMode === "initial") {
    $("navigationHint").textContent = "拖拽设置机器人位置与朝向";
  } else if (appState.navPlacementMode === "goal") {
    $("navigationHint").textContent = "拖拽设置目标点与朝向";
  } else {
    $("navigationHint").textContent = hasMap
      ? `Scan ${fmt(appState.status?.ros?.topic_hz?.scan, 0)} Hz · Plan ${appState.scene.plan?.points || 0} pts · Stream ${fmt(appState.stream.measuredHz, 1)} Hz`
      : "等待 /map 数据…";
  }

  if (window.finavMapEditor?.ready) window.finavMapEditor.updateMapStatus();
  else $("previewHint").textContent = "请选择地图";
}

function renderLiveCanvases() {
  const dragPose = appState.navDrag ? {
    x: appState.navDrag.start.x,
    y: appState.navDrag.start.y,
    yaw_deg: appState.navDrag.yawDeg,
    label: appState.navPlacementMode === "initial" ? "初始" : "目标",
  } : null;
  const activeMap = getActiveNavigationMap(appState.navMapName);
  const locations = activeMap && appState.navLocationsFor === activeMap
    ? appState.navLocations
    : [];

  if (appState.page === "mapping") {
    drawScene($("mappingCanvas"), appState.scene.map, appState.scene, {
      prefix: "live",
      showTargets: false,
      dragPose,
    });
  }
  if (appState.page === "navigation") {
    const layers = appState.navLayers;
    const objectsReady = activeMap && appState.navObjectsFor === activeMap;
    const task = appState.status?.robot?.navigation || {};
    setText("navInflationState", appState.scene.planning_map ? "" : "待就绪");
    drawScene($("navigationCanvas"), appState.scene.map, appState.scene, {
      prefix: "live",
      showPlan: layers.plan,
      showPlanningMap: layers.inflation,
      showScan: layers.scan,
      showTargets: true,
      locations: layers.locations ? locations : [],
      keepouts: layers.keepouts && objectsReady ? appState.navKeepouts : [],
      routes: layers.routes && objectsReady ? appState.navRoutes : [],
      selectedRoute: ["PLANNING", "FOLLOWING", "PAUSED"].includes(task.stage) ? task.route_id : $("navRouteSelect").value,
      selectedLocation: $("navLocationInput")?.value.trim() || "",
      dragPose,
    });
  }
}

function renderPreviewCanvas() {
  if (appState.page !== "preview") return;
  if (appState.previewWorkspaceReady && window.finavMapEditor) {
    window.finavMapEditor.draw();
    return;
  }
  drawScene($("previewCanvas"), appState.previewMap, null, {
    prefix: "preview",
  });
  updateSceneHints();
}

function renderCanvasById(canvasId) {
  if (canvasId === "previewCanvas") renderPreviewCanvas();
  else renderLiveCanvases();
  if (typeof updateViewportDisplay === "function") updateViewportDisplay(canvasId);
}

function renderCurrentPageCanvases() {
  if (appState.page === "preview") renderPreviewCanvas();
  else if (isLivePage()) renderLiveCanvases();
}

function createInfoButton(className, title, detail) {
  const btn = document.createElement("button");
  btn.className = className;
  const strong = document.createElement("strong");
  const span = document.createElement("span");
  strong.textContent = title;
  span.textContent = detail;
  btn.append(strong, span);
  return btn;
}

function renderPreviewMapPicker() {
  const box = $("previewMapOptions");
  box.innerHTML = "";
  if (!appState.savedMaps.length) {
    $("previewMapLabel").textContent = "未发现地图";
    box.innerHTML = '<div class="subtle">请先保存至少一张地图</div>';
  } else {
    $("previewMapLabel").textContent = appState.previewMapName || "选择地图";
    appState.savedMaps.forEach((item) => {
      const btn = createInfoButton(
        "nav-select-option" + (item.name === appState.previewMapName ? " active" : ""),
        item.name,
        `${item.width} × ${item.height} · ${fmt(item.resolution, 3)} m/px`,
      );
      btn.type = "button";
      btn.setAttribute("role", "option");
      btn.setAttribute("aria-selected", String(item.name === appState.previewMapName));
      btn.addEventListener("click", () => {
        toggleNavSelect("previewMapPanel", false);
        if (item.name !== appState.previewMapName) loadPreviewMap(item.name).catch(console.error);
      });
      box.appendChild(btn);
    });
  }
  $("previewMapToggle").disabled = appState.previewEditActive || !appState.savedMaps.length;
  $("btnDeletePreviewMap").disabled = !appState.previewMapName;
  $("btnEditorExport").disabled = !appState.previewWorkspaceReady;
}

function renderNavMapPicker(prefix = "nav") {
  const mapping = prefix === "mapping";
  const maps = mapping ? appState.savedMaps.filter(item => item.continuable) : appState.savedMaps;
  const stateKey = mapping ? "mappingMapName" : "navMapName";
  const button = $(mapping ? "btnContinueMapping" : "btnStartNavigation");
  const runtime = getRuntime(mapping ? "mapping" : "navigation");
  const box = $(`${prefix}MapOptions`);
  if (!box) return;
  box.innerHTML = "";

  if (!maps.length) {
    appState[stateKey] = "";
    $(`${prefix}MapLabel`).textContent = "未发现地图";
    box.textContent = mapping ? "暂无可续建地图" : "请先保存至少一张地图";
    button.disabled = true;
    $(`${prefix}MapToggle`).disabled = true;
    toggleNavSelect(`${prefix}MapPanel`, false);
    return;
  }

  const previousMap = appState[stateKey];
  if (!previousMap || !maps.some((item) => item.name === previousMap)) {
    appState[stateKey] = maps[0].name;
    if (!mapping && previousMap && previousMap !== appState[stateKey]) $("navLocationInput").value = "";
  }

  $(`${prefix}MapLabel`).textContent = appState[stateKey];
  button.disabled = runtime.running || runtime.stopping;
  $(`${prefix}MapToggle`).disabled = button.disabled;

  maps.forEach((item) => {
    const btn = createInfoButton(
      "nav-select-option" + (item.name === appState[stateKey] ? " active" : ""),
      item.name,
      `${item.width} × ${item.height} · ${fmt(item.resolution, 3)} m/px`,
    );
    btn.type = "button";
    btn.setAttribute("role", "option");
    btn.setAttribute("aria-selected", String(item.name === appState[stateKey]));
    btn.addEventListener("click", () => {
      if (!mapping && appState.navMapName !== item.name) {
        $("navLocationInput").value = "";
        appState.navLocations = [];
        appState.navLocationsFor = "";
      }
      appState[stateKey] = item.name;
      renderNavMapPicker(prefix);
      toggleNavSelect(`${prefix}MapPanel`, false);
      $(`${prefix}MapToggle`).focus();
      if (!mapping) loadNavLocations().catch(console.error);
      renderRuntimeControls();
    });
    box.appendChild(btn);
  });
}

async function loadNavLocations(force = false) {
  loadNavMapObjects().catch(console.error);
  const map = appState.navMapName;
  if (!map) {
    appState.navLocations = [];
    appState.navLocationsFor = "";
    renderNavLocationList();
    return;
  }
  if (!force && appState.navLocationsFor === map) {
    renderNavLocationList();
    return;
  }
  try {
    const data = await api(`/api/maps/${encodeURIComponent(map)}/locations`);
    if (map !== appState.navMapName) return;
    appState.navLocations = Array.isArray(data.locations) ? data.locations : [];
    appState.navLocationsFor = map;
  } catch (err) {
    if (map !== appState.navMapName) return;
    console.error(err);
    appState.navLocations = [];
    appState.navLocationsFor = map;
  }
  renderNavLocationList();
  renderLiveCanvases();
}

async function loadNavMapObjects() {
  const map = getActiveNavigationMap(appState.navMapName);
  if (map && appState.navObjectsLoadingMap === map) return;
  const request = ++appState.navObjectsRequest;
  if (map !== appState.navObjectsFor) {
    appState.navRoutes = [];
    appState.navKeepouts = [];
    appState.navObjectsFor = "";
    renderNavRouteList([]);
    renderNavLocationList();
  }
  if (!map) {
    appState.navRoutes = [];
    renderNavRouteList([]);
    renderNavLocationList();
    return;
  }
  appState.navObjectsLoadingMap = map;
  try {
    const data = await api(`/api/maps/${encodeURIComponent(map)}/editor`);
    if (request !== appState.navObjectsRequest || map !== getActiveNavigationMap(appState.navMapName)) return;
    appState.navRoutes = data.document?.routes || [];
    appState.navKeepouts = data.document?.keepouts || [];
    appState.navObjectsFor = map;
    const available = data.read_only ? [] : appState.navRoutes.filter(route => !route.closed);
    renderNavRouteList(available);
    renderNavLocationList();
    renderRuntimeControls();
    renderLiveCanvases();
  } finally {
    if (appState.navObjectsLoadingMap === map) appState.navObjectsLoadingMap = "";
  }
}

function renderNavRouteList(routes) {
  const control = $("navRouteSelect");
  const selected = routes.find(route => route.id === control.value);
  control.value = selected?.id || "";
  setText("navRouteLabel", selected?.name || (routes.length ? "选择路线" : "暂无开放路线"));
  const list = $("navRouteOptions");
  list.replaceChildren();
  if (!routes.length) {
    const empty = document.createElement("div");
    empty.className = "empty-state compact";
    empty.textContent = "暂无开放路线";
    list.appendChild(empty);
  }
  routes.forEach(route => {
    const button = createInfoButton(
      "nav-select-option" + (route.id === control.value ? " active" : ""),
      route.name, route.direction === "reverse" ? "反向" : "正向",
    );
    button.type = "button";
    button.setAttribute("role", "option");
    button.setAttribute("aria-selected", String(route.id === control.value));
    button.addEventListener("click", () => {
      control.value = route.id;
      renderNavRouteList(routes);
      toggleNavSelect("navRoutePanel", false);
      control.focus();
      renderRuntimeControls();
      renderLiveCanvases();
    });
    list.appendChild(button);
  });
}

function renderNavLocationList() {
  const list = $("navLocationOptions");
  const input = $("navLocationInput");
  const map = getActiveNavigationMap(appState.navMapName);
  const routes = appState.navObjectsFor === map
    ? appState.navRoutes.filter(route => !route.closed && route.end_name?.trim()) : [];
  const locations = appState.navLocationsFor === map ? appState.navLocations : [];
  const destinations = [
    ...routes.map(route => ({name: route.end_name, detail: `巡线 · ${route.name}`})),
    ...locations.map(loc => ({name: loc.name, detail: `地点 · x ${fmt(loc.x, 2)} · y ${fmt(loc.y, 2)}`})),
  ];
  if (list) {
    list.innerHTML = "";
    if (!destinations.length) {
      list.innerHTML = `<div class="empty-state compact"><strong>暂无目的地</strong></div>`;
    } else {
      const selected = input.value.trim();
      destinations.forEach((destination) => {
        const btn = createInfoButton(
          "nav-select-option" + (destination.name === selected ? " active" : ""),
          destination.name,
          destination.detail,
        );
        btn.type = "button";
        btn.setAttribute("role", "option");
        btn.setAttribute("aria-selected", String(destination.name === selected));
        btn.addEventListener("click", () => {
          input.value = destination.name;
          toggleNavSelect("navLocationPanel", false);
          renderRuntimeControls();
          renderLiveCanvases();
        });
        list.appendChild(btn);
      });
    }
  }
  const hint = $("navLocationHint");
  if (hint) {
    if (!appState.navMapName) {
      hint.textContent = "请先选择导航地图。";
    } else if (!destinations.length) {
      hint.textContent = "当前地图暂无目的地";
    } else {
      hint.textContent = `${routes.length} 个路线终点 · ${locations.length} 个地点`;
    }
  }
}

function formatFileSize(bytes) {
  if (!Number.isFinite(bytes)) return "--";
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / 1024 / 1024).toFixed(1)} MB`;
}

function getRuntime(mode) {
  return appState.status?.runtime?.[mode] || { running: false, stopping: false };
}

function getActiveNavigationMap(fallback = "") {
  const navigation = getRuntime("navigation");
  return navigation.running && !navigation.stopping
    ? String(navigation.launch_args?.map_file || fallback).trim()
    : "";
}

function applyRunState(id, runtime, activeText) {
  const node = $(id);
  const readiness = runtime.readiness?.state;
  const text = ({ starting: "启动中", not_ready: "未就绪", failed: "启动失败" })[readiness]
    || (runtime.stopping ? "结束中" : runtime.running ? activeText : "未启动");
  node.textContent = text;
  node.title = (runtime.readiness?.missing || []).join(", ");
  node.classList.remove("idle", "running", "stopping");
  node.classList.add(runtime.stopping ? "stopping" : runtime.running ? "running" : "idle");
}

function renderRuntimeControls() {
  const mapping = getRuntime("mapping");
  const navigation = getRuntime("navigation");
  const activeMap = getActiveNavigationMap();
  if (
    activeMap
    && activeMap !== appState.navMapName
    && appState.savedMaps.some((item) => item.name === activeMap)
  ) {
    appState.navMapName = activeMap;
    appState.navLocations = [];
    appState.navLocationsFor = "";
    $("navLocationInput").value = "";
    renderNavMapPicker();
    loadNavLocations().catch(console.error);
  }
  const mappingBusy = mapping.running || mapping.stopping;
  if (activeMap && appState.navObjectsFor !== activeMap && appState.navObjectsLoadingMap !== activeMap) {
    loadNavMapObjects().catch(console.error);
  }
  const navigationBusy = navigation.running || navigation.stopping;
  const navCommandsEnabled = navigation.running && !navigation.stopping;
  applyRunState("mappingRunState", mapping, "正在建图");
  applyRunState("navigationRunState", navigation, "正在运行");

  setHidden("mappingIdleActions", mappingBusy);
  setHidden("mappingActiveActions", !mappingBusy);
  setHidden("navIdleSession", navigationBusy);
  setHidden("navActiveSession", !navigationBusy);
  setHidden("navCommandSections", !navCommandsEnabled);
  setText("navActiveMap", activeMap || appState.navMapName || "--");
  const plan = appState.status?.robot?.plan || {};
  const navTask = appState.status?.robot?.navigation || {};
  const routeActive = navTask.task_type === "route" && ["PLANNING", "FOLLOWING", "PAUSED"].includes(navTask.stage);
  $("btnStartRoute").disabled = !navCommandsEnabled || !$("navRouteSelect").value || routeActive;
  $("navRouteSelect").disabled = !navCommandsEnabled || routeActive;
  if ($("navRouteSelect").disabled) toggleNavSelect("navRoutePanel", false);
  setHidden("btnPauseRoute", !routeActive || !["FOLLOWING", "PLANNING"].includes(navTask.stage));
  setHidden("btnResumeRoute", !routeActive || navTask.stage !== "PAUSED");
  $("btnPauseRoute").disabled = !navCommandsEnabled;
  $("btnResumeRoute").disabled = !navCommandsEnabled;
  const hasTask = navCommandsEnabled && (
    Number(navTask.task_id || 0) > 0
    || !!appState.status?.robot?.goal_pose
    || Number(plan.points || 0) > 0
    || !!appState.navDestinationName
  );
  setHidden("navCurrentTask", !navCommandsEnabled);

  $("btnStartMapping").disabled = mappingBusy;
  $("mappingMapToggle").disabled = mappingBusy || !appState.savedMaps.some(item => item.continuable);
  $("btnContinueMapping").disabled = mappingBusy || !appState.mappingMapName || !appState.savedMaps.length;
  if (mappingBusy) toggleNavSelect("mappingMapPanel", false);
  $("btnStopMapping").disabled = !mappingBusy;
  $("btnSaveMap").disabled = !mapping.running || mapping.stopping || !!appState.mapSaveJob || appState.status?.map_save?.state === "running";
  const continuing = mappingBusy && !!mapping.launch_args?.map_file;
  const continuation = appState.status?.continued_mapping || {};
  const freshContinuation = Date.now() / 1000 - (continuation.updated_at || 0) < 3
    && (continuation.updated_at || 0) >= (mapping.started_at || 0);
  const mappingStage = freshContinuation ? continuation.stage : "loading";
  const mappingLocalizing = continuing && mappingStage !== "mapping";
  const relocating = getRuntime("relocate").running || getRuntime("relocate").stopping;
  setHidden("mappingLocalizationActions", !mappingLocalizing);
  setText("mappingLocalizationState", relocating ? "正在重定位" : ({loading: "正在加载旧地图", localizing: "定位中", ready: "匹配已更新"}[mappingStage] || "定位中"));
  $("btnMappingInitialPose").disabled = mapping.stopping || relocating || !["localizing", "ready"].includes(mappingStage);
  $("btnMappingRelocate").disabled = $("btnMappingInitialPose").disabled;
  $("btnMappingResume").disabled = mapping.stopping || relocating || !["localizing", "ready"].includes(mappingStage);
  if (mappingLocalizing) {
    $("btnSaveMap").disabled = true;
    setText("mappingRunState", mapping.stopping ? "停止中" : "定位中");
  }
  $("btnStartNavigation").disabled = navigationBusy || !appState.savedMaps.length;
  $("btnStopNavigation").disabled = !navigationBusy;
  $("navMapToggle").disabled = navigationBusy || !appState.savedMaps.length;
  $("btnArmInit").disabled = !navCommandsEnabled;
  $("btnArmGoal").disabled = !navCommandsEnabled;
  $("btnCancelNav").disabled = !hasTask || ["REACHED", "FAILED", "CANCELED", "IDLE"].includes(navTask.stage);

  const relocate = getRuntime("relocate");
  const relocateBusy = relocate.running || relocate.stopping;
  const btnRelocate = $("btnRelocate");
  if (btnRelocate) {
    btnRelocate.disabled = !navCommandsEnabled || relocateBusy;
    setText("navRelocateLabel", relocateBusy ? "重定位中…" : "自动重定位");
    btnRelocate.classList.toggle("active", relocateBusy);
  }
  if (!navCommandsEnabled && !(appState.page === "mapping" && mappingLocalizing && !$("btnMappingInitialPose").disabled) && appState.navPlacementMode && typeof setNavPlacementMode === "function") {
    setNavPlacementMode(null);
  }
  if (!navCommandsEnabled) appState.navDestinationName = "";

  const navLocationInput = $("navLocationInput");
  const btnNavLocation = $("btnNavLocation");
  if (navLocationInput) navLocationInput.disabled = !navCommandsEnabled;
  if (btnNavLocation) btnNavLocation.disabled = !navCommandsEnabled || !navLocationInput.value.trim();
  if (typeof toggleNavSelect === "function") {
    if (navigationBusy) toggleNavSelect("navMapPanel", false);
    if (!navCommandsEnabled) toggleNavSelect("navLocationPanel", false);
  }
}

function splitYamlComment(line) {
  let quote = null;
  let escaped = false;
  for (let i = 0; i < line.length; i += 1) {
    const ch = line[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (quote === "\"") {
      if (ch === "\\") escaped = true;
      else if (ch === "\"") quote = null;
      continue;
    }
    if (quote === "'") {
      if (ch === "'") quote = null;
      continue;
    }
    if (ch === "\"" || ch === "'") {
      quote = ch;
      continue;
    }
    if (ch === "#" && (i === 0 || /\s/.test(line[i - 1]))) {
      return [line.slice(0, i), line.slice(i)];
    }
  }
  return [line, ""];
}

function highlightYamlValue(text) {
  const tokenRe = /("(?:[^"\\]|\\.)*"|'(?:[^']|'{2})*'|\b(?:true|false|yes|no|null|on|off)\b|-?(?:0|[1-9]\d*)(?:\.\d+)?)/gi;
  let html = "";
  let last = 0;
  let match;
  while ((match = tokenRe.exec(text))) {
    html += escapeHtml(text.slice(last, match.index));
    const token = match[0];
    if (token.startsWith("\"") || token.startsWith("'")) {
      html += `<span class="yaml-string">${escapeHtml(token)}</span>`;
    } else if (/^-?(?:0|[1-9]\d*)(?:\.\d+)?$/.test(token)) {
      html += `<span class="yaml-number">${escapeHtml(token)}</span>`;
    } else {
      html += `<span class="yaml-boolean">${escapeHtml(token)}</span>`;
    }
    last = match.index + token.length;
  }
  html += escapeHtml(text.slice(last));
  return html;
}

function highlightYaml(text) {
  const lines = String(text || "").split("\n");
  const rendered = lines.map((line) => {
    const [body, comment] = splitYamlComment(line);
    let html = "";
    const keyMatch = body.match(/^(\s*-\s*)?([^:#\n][^:\n]*?)(\s*:)(.*)$/);
    if (keyMatch) {
      const [, prefix = "", key = "", colon = ":", rest = ""] = keyMatch;
      html = `${escapeHtml(prefix)}<span class="yaml-key">${escapeHtml(key)}</span><span class="yaml-punc">${escapeHtml(colon)}</span>${highlightYamlValue(rest)}`;
    } else {
      html = highlightYamlValue(body);
    }
    if (comment) html += `<span class="yaml-comment">${escapeHtml(comment)}</span>`;
    return html || " ";
  });
  return rendered.join("\n");
}

function syncConfigEditorScroll() {
  const editor = $("configEditor");
  const highlight = $("configHighlight");
  const lineNumbers = $("configLineNumbers");
  highlight.style.transform = `translate(${-editor.scrollLeft}px, ${-editor.scrollTop}px)`;
  if (lineNumbers) lineNumbers.style.transform = `translateY(${-editor.scrollTop}px)`;
}

function renderConfigLineNumbers() {
  const editor = $("configEditor");
  const lineNumbers = $("configLineNumbers");
  if (!lineNumbers) return;
  const count = Math.max(1, String(editor.value || "").split("\n").length);
  lineNumbers.textContent = Array.from({ length: count }, (_item, index) => String(index + 1)).join("\n");
}

function renderConfigHighlight() {
  const editor = $("configEditor");
  const highlight = $("configHighlight");
  highlight.innerHTML = highlightYaml(editor.value);
  renderConfigLineNumbers();
  syncConfigEditorScroll();
}

function setConfigSaveNotice(text = "", kind = "ok") {
  const node = $("configSaveNotice");
  if (configSaveNoticeTimer) {
    window.clearTimeout(configSaveNoticeTimer);
    configSaveNoticeTimer = null;
  }
  if (!text) {
    node.textContent = "";
    node.classList.add("hidden");
    node.dataset.kind = "";
    return;
  }
  node.textContent = text;
  node.dataset.kind = kind;
  node.classList.remove("hidden");
  if (kind === "dirty" || kind === "restart") return;
  configSaveNoticeTimer = window.setTimeout(() => {
    node.classList.add("hidden");
    node.dataset.kind = "";
  }, 2200);
}

function renderConfigCards() {
  const box = $("configCards");
  box.innerHTML = "";
  if (!appState.configs.length) {
    box.innerHTML = `<div class="empty-state"><strong>尚无配置文件</strong><span>config/ 中可编辑的 YAML 文件会显示在这里。</span></div>`;
    return;
  }

  appState.configs.forEach((file) => {
    const card = document.createElement("button");
    const name = document.createElement("strong");
    const size = document.createElement("span");
    card.type = "button";
    card.className = "config-card";
    name.textContent = file.name;
    size.className = "config-size";
    size.textContent = formatFileSize(file.size);
    card.append(name, size);
    card.addEventListener("click", () => openConfigEditor(file.name));
    box.appendChild(card);
  });
}

async function loadConfigs() {
  try {
    const data = await api("/api/configs");
    appState.configs = data.files || [];
    renderConfigCards();
  } catch (err) {
    reportActionError(err, "配置列表加载失败");
  }
}

let avoidanceSwitchBusy = false;

function renderAvoidanceSwitch() {
  const control = appState.status?.control || {};
  const fresh = connectionState.realtime === "online" && appState.status?.ros?.connected &&
    Date.now() / 1000 - (control.avoidance_updated_at || 0) < 2.5;
  const toggle = $("avoidanceEnabled");
  toggle.checked = control.avoidance_enabled === true;
  toggle.disabled = avoidanceSwitchBusy || !fresh;
  $("avoidanceSwitchState").textContent = avoidanceSwitchBusy ? "切换中" :
    !fresh ? "状态未知" : toggle.checked ? "已开启" : "已关闭";
}

async function toggleAvoidance(event) {
  const enabled = event.target.checked;
  avoidanceSwitchBusy = true;
  renderAvoidanceSwitch();
  try {
    await api("/api/avoidance/control", "POST", {enabled}, {timeoutMs: 30000});
    showToast(enabled ? "避障已开启" : "避障已关闭", "ok");
  } catch (error) {
    showToast(error.message, "error");
  } finally {
    avoidanceSwitchBusy = false;
    renderAvoidanceSwitch();
  }
}

async function loadSystemSupervisor() {
  try {
    const data = await api("/api/system");
    const state = $("systemSupervisorState");
    state.textContent = data.managed
      ? (data.backend === "systemd" ? "Finav 服务运行管理已连接" : `start_finav.sh 正在监督当前服务 · PID ${data.pid}`)
      : "服务管理不可用，关闭和重启不可用";
    state.dataset.state = data.managed ? "ok" : "error";
    $("btnShutdownFinav").disabled = !data.managed;
    $("btnRestartFinav").disabled = !data.managed;
  } catch (err) {
    const state = $("systemSupervisorState");
    state.textContent = "监督进程状态读取失败";
    state.dataset.state = "error";
    $("btnShutdownFinav").disabled = true;
    $("btnRestartFinav").disabled = true;
  }
}

async function requestFinavPowerAction(action, button) {
  const restarting = action === "restart";
  const confirmed = await showAnnotationConfirmDialog({
    title: restarting ? "重启 Finav" : "关闭 Finav",
    message: restarting
      ? "将停止当前建图、导航及底盘控制，随后重新启动底盘和 Web。导航任务不会自动恢复，页面会短暂断开，是否继续？"
      : "将停止当前建图、导航、底盘、手柄、路由和 Web 进程。Jetson 系统不会关机，是否继续？",
    confirmText: restarting ? "确认重启" : "确认关闭",
    danger: true,
  });
  if (!confirmed) return;

  setButtonBusy(button, true, restarting ? "正在重启…" : "正在关闭…");
  showBlockingModal(true, restarting ? "正在重启 Finav，等待服务恢复…" : "正在关闭 Finav 服务…");
  try {
    await api(`/api/system/${action}`, "POST", {}, { timeoutMs: 4000 });
    if (!restarting) {
      setText("blockingModalText", "Finav 已收到关闭请求，本页面即将断开。");
      return;
    }
    stopSceneStream();
    await waitForFinavRecovery();
    showBlockingModal(false);
    setButtonBusy(button, false);
    showToast("Finav 已完成重启", "ok");
    loadSystemSupervisor().catch(console.error);
    if (isLivePage()) startSceneStream();
  } catch (err) {
    if (restarting && !err?.status) {
      try {
        await waitForFinavRecovery();
        showBlockingModal(false);
        setButtonBusy(button, false);
        showToast("Finav 已恢复连接", "ok");
        loadSystemSupervisor().catch(console.error);
        return;
      } catch (_recoveryError) {
        // Report the original action error below.
      }
    }
    showBlockingModal(false);
    setButtonBusy(button, false);
    reportActionError(err, restarting ? "Finav 重启失败" : "Finav 关闭失败");
  }
}

async function waitForFinavRecovery(timeoutMs = 30000) {
  const deadline = Date.now() + timeoutMs;
  await new Promise((resolve) => window.setTimeout(resolve, 1200));
  while (Date.now() < deadline) {
    try {
      await api(`/api/health?t=${Date.now()}`, "GET", null, { timeoutMs: 1500 });
      return;
    } catch (_err) {
      await new Promise((resolve) => window.setTimeout(resolve, 800));
    }
  }
  throw new Error("30 秒内未检测到 Finav 服务恢复");
}

function showConfigOverview() {
  $("configOverview").classList.remove("hidden");
  $("configEditorView").classList.add("hidden");
  setConfigSaveNotice("");
}

function showConfigEditor() {
  $("configOverview").classList.add("hidden");
  $("configEditorView").classList.remove("hidden");
}

async function openConfigEditor(name) {
  try {
    const data = await api(`/api/configs/${encodeURIComponent(name)}`);
    appState.configEditor = {
      name: data.name || name,
      path: data.path || `config/${name}`,
      content: data.content || "",
      revision: data.revision,
      impact: data.impact || null,
    };
    $("configEditorTitle").textContent = data.name || name;
    $("configEditorMeta").textContent = data.path || `config/${name}`;
    $("configEditor").value = data.content || "";
    renderConfigHighlight();
    setConfigSaveNotice("");
    showConfigEditor();
  } catch (err) {
    reportActionError(err, "配置打开失败");
  }
}

async function saveConfigEditor() {
  if (!appState.configEditor.name) return;
  const content = $("configEditor").value;
  try {
    const data = await api(`/api/configs/${encodeURIComponent(appState.configEditor.name)}`, "POST", {
      content, expected_revision: appState.configEditor.revision,
    });
    appState.configEditor.revision = data.revision;
    appState.configEditor.content = content;
    appState.configEditor.impact = data.impact || appState.configEditor.impact;
    await loadConfigs();
    const impact = appState.configEditor.impact;
    const needsBaseRestart = impact?.restart_targets?.some((target) => target.mode === "base_drive");
    const notice = needsBaseRestart ? "需重启底盘控制后生效" : impact?.message;
    setConfigSaveNotice(notice ? `已保存；${notice}` : "保存成功", notice ? "restart" : "ok");
    showToast("配置已保存", "ok");
  } catch (err) {
    reportActionError(err, "配置保存失败");
    setConfigSaveNotice("保存失败", "error");
  }
}

async function restartBaseControl(button) {
  const confirmed = await showAnnotationConfirmDialog({
    title: "重启底盘控制",
    message: "将停止底盘、复位 PCAN USB 适配器后重新启动，并停止当前建图或导航任务。请确认车辆安全、手柄回中。任务不会自动恢复，Web 服务保持运行，是否继续？",
    confirmText: "确认重启",
    danger: true,
  });
  if (!confirmed) return;
  const original = button.textContent;
  button.disabled = true;
  button.textContent = "重启中…";
  try {
    const data = await api("/api/runtime/base_drive/restart", "POST", {}, { timeoutMs: 45000 });
    if (data.runtime) {
      appState.status = { ...(appState.status || {}), runtime: data.runtime };
      renderRuntimeControls();
    }
    showToast("底盘控制重启请求已发送", "ok");
  } catch (err) {
    reportActionError(err, "重启失败");
  } finally {
    button.disabled = false;
    button.textContent = original;
  }
}

async function runMappingAction(action, body = {}) {
  try {
    setNavPlacementMode(null);
    const data = await withBlockingModal(() => api(`/api/mapping/${action}`, "POST", body, { timeoutMs: 15000 }),
      action === "resume" ? "正在切换建图模式…" : "正在启动重定位…");
    if (data?.runtime) appState.status = { ...(appState.status || {}), runtime: data.runtime };
    renderRuntimeControls();
    showToast(action === "resume" ? "已开始续建" : "重定位已启动", "ok");
  } catch (err) {
    reportActionError(err, "续建操作失败");
  }
}

async function startRuntime(mode, mapFile = "") {
  const button = mode === "mapping" ? $(mapFile ? "btnContinueMapping" : "btnStartMapping") : $("btnStartNavigation");
  setButtonBusy(button, true, "启动中…");
  try {
    if (mode === "navigation" && !appState.savedMaps.length) {
      await loadSavedMaps();
      if (!appState.savedMaps.length) return;
    }
    let body = {};
    if (mode === "mapping" && mapFile) body = { map_file: mapFile };
    if (mode === "navigation" && appState.navMapName) {
      body = { map_file: appState.navMapName };
    }
    const data = await api(`/api/runtime/${mode}/start`, "POST", body, { timeoutMs: 45000 });
    if (data.runtime) {
      appState.status = { ...(appState.status || {}), runtime: data.runtime };
      renderRuntimeControls();
    }
    showToast(mode === "mapping" ? "建图启动请求已发送" : "导航启动请求已发送", "ok");
  } catch (err) {
    reportActionError(err, "运行启动失败");
  } finally {
    setButtonBusy(button, false);
    renderRuntimeControls();
  }
}

async function stopRuntime(mode, options = {}) {
  const run = async () => {
    try {
      const data = await api(`/api/runtime/${mode}/stop`, "POST", {}, { timeoutMs: 45000 });
      if (data.runtime) {
        appState.status = { ...(appState.status || {}), runtime: data.runtime };
        renderRuntimeControls();
      }
      resetLiveScene();
      renderLiveCanvases();
      updateSceneHints();
      showToast(mode === "mapping" ? "建图会话已结束" : "导航会话已结束", "ok");
    } catch (err) {
      reportActionError(err, "运行停止失败");
    }
  };
  if (options.showModal) await withBlockingModal(run, mode === "mapping" ? "正在结束建图进程…" : "正在结束导航进程…");
  else await run();
}

async function loadSavedMaps() {
  try {
    const data = await api("/api/maps");
    appState.savedMaps = data.maps || [];
    renderNavMapPicker("mapping");
    if (!appState.savedMaps.length) {
      appState.previewMapName = "";
      appState.previewMap = null;
      appState.previewWorkspaceReady = false;
      appState.navMapName = "";
      appState.navLocations = [];
      appState.navLocationsFor = "";
      $("navLocationInput").value = "";
      $("previewMeta").textContent = "尚无可用地图";
      window.finavMapEditor?.clear();
      renderPreviewMapPicker();
      renderNavMapPicker();
      renderNavLocationList();
      renderPreviewCanvas();
      return;
    }
    if (!appState.previewMapName || !appState.savedMaps.some((item) => item.name === appState.previewMapName)) {
      appState.previewMapName = appState.savedMaps[0].name;
    }
    renderPreviewMapPicker();
    renderNavMapPicker();
    loadNavLocations().catch(console.error);
    if (
      appState.page === "preview"
      && appState.previewMapName
      && !appState.previewEditActive
    ) await loadPreviewMap(appState.previewMapName);
  } catch (err) {
    reportActionError(err, "地图列表加载失败");
  }
}

async function deleteSelectedPreviewMap() {
  const name = appState.previewMapName;
  if (!name) return;
  const confirmed = await showAnnotationConfirmDialog({
    title: "删除地图",
    message: `确认删除地图“${name}”吗？该目录及其全部文件会被删除。`,
    confirmText: "删除",
    danger: true,
  });
  if (!confirmed) return;
  try {
    await window.finavMapEditor?.exit();
    await api(`/api/maps/${encodeURIComponent(name)}/delete`, "POST", {});
    if (appState.navMapName === name) appState.navMapName = "";
    if (appState.previewMapName === name) {
      appState.previewMapName = "";
      appState.previewMap = null;
    }
    await loadSavedMaps();
    showToast(`地图“${name}”已删除`, "ok");
  } catch (err) {
    reportActionError(err, "地图删除失败");
  }
}

async function loadPreviewMap(name) {
  if (window.finavRasterEditor?.active) return;
  try {
    const changed = window.finavMapEditor?.mapName !== name;
    appState.previewMap = await api(`/api/maps/${encodeURIComponent(name)}`);
    mapRasterCache.clear();
    appState.previewMapName = name;
    if (changed) resetViewport("previewCanvas");
    $("previewMeta").textContent = `${appState.previewMap.width} × ${appState.previewMap.height} · ${fmt(appState.previewMap.resolution, 3)} m/px`;
    await window.finavMapEditor?.load(name);
    renderPreviewMapPicker();
    renderPreviewCanvas();
  } catch (err) {
    window.finavMapEditor?.clear();
    renderPreviewMapPicker();
    renderPreviewCanvas();
    reportActionError(err, "地图预览加载失败");
  }
}
