function stopSceneStream() {
  if (appState.stream.source) appState.stream.source.close();
  appState.stream.source = null;
  appState.stream.mode = null;
}

function startSceneStream() {
  stopSceneStream();
  if (!isLivePage() || document.hidden) return;

  const mode = appState.page;
  const source = new EventSource(
    `/api/stream?mode=${encodeURIComponent(mode)}&map_version=${appState.mapVersion}&plan_version=${appState.planVersion}`,
  );
  appState.stream.source = source;
  appState.stream.mode = mode;
  appState.stream.frames = 0;
  appState.stream.sampleStartedAt = performance.now();

  source.onopen = () => setConnectionState("online");
  source.onmessage = (event) => {
    try {
      const payload = JSON.parse(event.data);
      appState.mapVersion = payload.map_version ?? appState.mapVersion;
      appState.planVersion = payload.plan_version ?? appState.planVersion;
      if (payload.map) {
        appState.scene.map = payload.map;
        [...mapRasterCache.keys()].filter((key) => key.startsWith("live-")).forEach((key) => mapRasterCache.delete(key));
      }
      if (payload.scan) appState.scene.scan = payload.scan;
      if (payload.plan) appState.scene.plan = payload.plan;
      if ("robot_pose_map" in payload) appState.scene.robot_pose_map = payload.robot_pose_map;
      if ("goal_pose" in payload) appState.scene.goal_pose = payload.goal_pose;
      if ("initial_pose" in payload) appState.scene.initial_pose = payload.initial_pose;
      if (payload.robot_footprint) appState.scene.robot_footprint = payload.robot_footprint;

      const now = performance.now();
      appState.stream.frames += 1;
      appState.stream.lastFrameAt = Date.now();
      if (payload.stream?.server_sent_at) {
        appState.stream.latencyMs = Math.max(0, Date.now() - payload.stream.server_sent_at * 1000);
      }
      const elapsed = now - appState.stream.sampleStartedAt;
      if (elapsed >= 1000) {
        appState.stream.measuredHz = appState.stream.frames * 1000 / elapsed;
        appState.stream.frames = 0;
        appState.stream.sampleStartedAt = now;
      }
      renderLiveCanvases();
      updateSceneHints();
    } catch (err) {
      console.error(err);
    }
  };
  source.onerror = () => {
    if (appState.stream.source === source) setConnectionState("offline");
  };
}

function isLivePage() {
  return appState.page === "mapping" || appState.page === "navigation";
}

function resetLiveScene() {
  appState.mapVersion = -1;
  appState.planVersion = -1;
  appState.scene = {
    map: null,
    scan: { count: 0, ranges_b64: "" },
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
  const cameraHint = "滚轮缩放｜左键平移｜右键旋转";
  $("mappingHint").textContent = appState.page === "mapping" && appState.navPlacementMode === "initial"
    ? "手动重定位中：在地图上拖拽设置车体位置和朝向。"
    : hasMap
      ? `${cameraHint}｜地图 ${map.width} × ${map.height}｜雷达点 ${appState.scene.scan?.count || 0}｜画面 ${fmt(appState.stream.measuredHz, 1)} Hz`
      : "等待 `/map` 数据...";

  if (appState.navPlacementMode === "initial") {
    $("navigationHint").textContent = "手动重定位中：左键拖拽方向，右键仍可旋转视图。";
  } else if (appState.navPlacementMode === "goal") {
    $("navigationHint").textContent = "手动设置目的地中：左键拖拽方向，右键仍可旋转视图。";
  } else {
    $("navigationHint").textContent = hasMap
      ? `${cameraHint}｜路径点 ${appState.scene.plan?.points || 0}｜雷达点 ${appState.scene.scan?.count || 0}｜画面 ${fmt(appState.stream.measuredHz, 1)} Hz`
      : "等待 `/map` 数据...";
  }

  if (appState.previewAnnotationMode) {
    $("previewHint").textContent = "添加地点中：左键拖拽方向，松开后输入名称；右键仍可旋转视图。";
  } else {
    $("previewHint").textContent = appState.previewMap
      ? `${cameraHint}｜标注 ${appState.previewLocations.length} 个｜${appState.previewMap.width} × ${appState.previewMap.height}`
      : "请选择左侧地图";
  }
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
    drawScene($("navigationCanvas"), appState.scene.map, appState.scene, {
      prefix: "live",
      showPlan: true,
      showTargets: true,
      locations,
      selectedLocation: $("navLocationInput")?.value.trim() || "",
      dragPose,
    });
  }
}

function renderPreviewCanvas() {
  drawScene($("previewCanvas"), appState.previewMap, null, {
    prefix: "preview",
    locations: appState.previewLocations,
    selectedLocation: appState.previewSelectedLocation,
    annotationDraft: appState.previewAnnotationDraft,
  });
  updateSceneHints();
}

function renderCanvasById(canvasId) {
  if (canvasId === "previewCanvas") renderPreviewCanvas();
  else renderLiveCanvases();
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

function renderMapList() {
  const box = $("mapList");
  box.innerHTML = "";
  if (!appState.savedMaps.length) {
    box.innerHTML = `<div class="subtle">未发现可预览地图</div>`;
    $("btnDeletePreviewMap").disabled = true;
    return;
  }

  appState.savedMaps.forEach((item) => {
    const btn = createInfoButton(
      "map-item" + (item.name === appState.previewMapName ? " active" : ""),
      item.name,
      `${item.width} × ${item.height} · ${fmt(item.resolution, 3)} m/px`,
    );
    btn.addEventListener("click", () => loadPreviewMap(item.name));
    box.appendChild(btn);
  });
  $("btnDeletePreviewMap").disabled = !appState.previewMapName;
}

function renderNavMapPicker() {
  const box = $("navMapOptions");
  if (!box) return;
  box.innerHTML = "";

  if (!appState.savedMaps.length) {
    $("navMapLabel").textContent = "未发现地图";
    box.innerHTML = `<div class="subtle">请先保存至少一张地图</div>`;
    $("btnStartNavigation").disabled = true;
    return;
  }

  const previousMap = appState.navMapName;
  if (!previousMap || !appState.savedMaps.some((item) => item.name === previousMap)) {
    appState.navMapName = appState.savedMaps[0].name;
    if (previousMap && previousMap !== appState.navMapName) $("navLocationInput").value = "";
  }

  $("navMapLabel").textContent = appState.navMapName;
  $("btnStartNavigation").disabled = getRuntime("navigation").running || getRuntime("navigation").stopping || !appState.savedMaps.length;

  appState.savedMaps.forEach((item) => {
    const btn = createInfoButton(
      "nav-select-option" + (item.name === appState.navMapName ? " active" : ""),
      item.name,
      `${item.width} × ${item.height} · ${fmt(item.resolution, 3)} m/px`,
    );
    btn.type = "button";
    btn.setAttribute("role", "option");
    btn.setAttribute("aria-selected", String(item.name === appState.navMapName));
    btn.addEventListener("click", () => {
      if (appState.navMapName !== item.name) {
        $("navLocationInput").value = "";
        appState.navLocations = [];
        appState.navLocationsFor = "";
      }
      appState.navMapName = item.name;
      renderNavMapPicker();
      toggleNavSelect("navMapPanel", false);
      loadNavLocations().catch(console.error);
      renderRuntimeControls();
    });
    box.appendChild(btn);
  });
}

async function loadNavLocations(force = false) {
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
    appState.navLocations = Array.isArray(data.locations) ? data.locations : [];
    appState.navLocationsFor = map;
  } catch (err) {
    console.error(err);
    appState.navLocations = [];
    appState.navLocationsFor = map;
  }
  renderNavLocationList();
  renderLiveCanvases();
}

function renderNavLocationList() {
  const list = $("navLocationOptions");
  const input = $("navLocationInput");
  if (list) {
    list.innerHTML = "";
    if (!appState.navLocations.length) {
      list.innerHTML = `<div class="subtle">当前地图没有可选地点</div>`;
    } else {
      const selected = input.value.trim();
      appState.navLocations.forEach((loc) => {
        const btn = createInfoButton(
          "nav-select-option" + (loc.name === selected ? " active" : ""),
          loc.name,
          `x=${fmt(loc.x, 2)} · y=${fmt(loc.y, 2)} · yaw=${fmt(loc.yaw_deg, 1)}°`,
        );
        btn.type = "button";
        btn.setAttribute("role", "option");
        btn.setAttribute("aria-selected", String(loc.name === selected));
        btn.addEventListener("click", () => {
          input.value = loc.name;
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
    } else if (!appState.navLocations.length) {
      hint.textContent = `地图“${appState.navMapName}”未标注地点，可直接输入地名尝试下发。`;
    } else {
      const names = appState.navLocations.map((loc) => loc.name).join("、");
      hint.textContent = `可用地点：${names}`;
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
  const text = runtime.stopping ? "结束中" : runtime.running ? activeText : "未启动";
  node.textContent = text;
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
  const navigationBusy = navigation.running || navigation.stopping;
  const navCommandsEnabled = navigation.running && !navigation.stopping;
  applyRunState("mappingRunState", mapping, "运行中");
  applyRunState("navigationRunState", navigation, "运行中");

  $("btnStartMapping").disabled = mappingBusy;
  $("btnStopMapping").disabled = !mappingBusy;
  $("btnSaveMap").disabled = !mapping.running || mapping.stopping;
  $("btnStartNavigation").disabled = navigationBusy || !appState.savedMaps.length;
  $("btnStopNavigation").disabled = !navigationBusy;
  $("navMapToggle").disabled = navigationBusy || !appState.savedMaps.length;
  $("btnArmInit").disabled = !navCommandsEnabled;
  $("btnArmGoal").disabled = !navCommandsEnabled;
  $("btnCancelNav").disabled = !navCommandsEnabled;

  const relocate = getRuntime("relocate");
  const relocateBusy = relocate.running || relocate.stopping;
  const btnRelocate = $("btnRelocate");
  if (btnRelocate) {
    btnRelocate.disabled = !navCommandsEnabled || relocateBusy;
    btnRelocate.textContent = relocateBusy ? "自动重定位中…" : "自动重定位";
    btnRelocate.classList.toggle("active", relocateBusy);
  }
  if (!navCommandsEnabled && appState.navPlacementMode && typeof setNavPlacementMode === "function") {
    setNavPlacementMode(null);
  }

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

function validateConfigYaml(text) {
  const stack = [];
  const pairs = { "]": "[", "}": "{" };
  const lines = String(text || "").split("\n");
  for (let i = 0; i < lines.length; i += 1) {
    const line = lines[i];
    if (/^\t+/.test(line)) return `YAML 第 ${i + 1} 行使用了 Tab 缩进，请改为空格。`;
    const body = splitYamlComment(line)[0];
    for (const ch of body) {
      if (ch === "[" || ch === "{") stack.push({ ch, line: i + 1 });
      if (ch === "]" || ch === "}") {
        const prev = stack.pop();
        if (!prev || prev.ch !== pairs[ch]) return `YAML 第 ${i + 1} 行括号不匹配。`;
      }
    }
  }
  if (stack.length) return `YAML 第 ${stack[stack.length - 1].line} 行括号未闭合。`;
  return "";
}

function renderConfigRestartActions() {
  const panel = $("configApplyPanel");
  const actions = $("configRestartActions");
  const targetsByMode = new Map();
  appState.configs.forEach((file) => {
    const targets = file.impact && Array.isArray(file.impact.restart_targets) ? file.impact.restart_targets : [];
    targets.forEach((target) => {
      if (target && target.mode && !targetsByMode.has(target.mode)) {
        targetsByMode.set(target.mode, target);
      }
    });
  });
  actions.innerHTML = "";
  const targets = Array.from(targetsByMode.values());
  if (!targets.length) {
    panel.classList.add("hidden");
    return;
  }

  panel.classList.remove("hidden");
  targets.forEach((target) => {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "danger";
    btn.dataset.restartMode = target.mode;
    btn.textContent = `重启${target.label || target.mode}`;
    btn.addEventListener("click", () => restartRuntimeTarget(target.mode, btn).catch(console.error));
    actions.appendChild(btn);
  });
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
  configSaveNoticeTimer = window.setTimeout(() => {
    node.classList.add("hidden");
    node.dataset.kind = "";
  }, 2200);
}

function setConfigRestartNotice(text = "", kind = "ok") {
  const node = $("configRestartNotice");
  if (!node) return;
  node.textContent = text;
  node.dataset.kind = kind;
  node.classList.toggle("hidden", !text);
}

function renderConfigCards() {
  const box = $("configCards");
  box.innerHTML = "";
  if (!appState.configs.length) {
    box.innerHTML = `<div class="subtle">未发现可编辑的配置文件</div>`;
    return;
  }

  appState.configs.forEach((file) => {
    const card = createInfoButton("config-card", file.name, file.path);
    const size = document.createElement("span");
    size.textContent = formatFileSize(file.size);
    card.appendChild(size);
    card.addEventListener("click", () => openConfigEditor(file.name));
    box.appendChild(card);
  });
}

async function loadConfigs() {
  try {
    const data = await api("/api/configs");
    appState.configs = data.files || [];
    renderConfigRestartActions();
    renderConfigCards();
  } catch (err) {
    reportActionError(err, "配置列表加载失败");
  }
}

async function loadSystemSupervisor() {
  try {
    const data = await api("/api/system");
    const state = $("systemSupervisorState");
    state.textContent = data.managed
      ? `start_finav.sh 正在监督当前服务 · PID ${data.pid}`
      : "未检测到 start_finav.sh 监督进程，关闭和重启不可用";
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
      ? "将停止当前建图、导航、底盘、手柄、路由和 Web 进程，随后重新启动整套 start_finav。页面会短暂断开，是否继续？"
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
  const validationError = validateConfigYaml(content);
  if (validationError) {
    setConfigSaveNotice("校验失败", "error");
    showToast(validationError, "error");
    return;
  }
  try {
    const data = await api(`/api/configs/${encodeURIComponent(appState.configEditor.name)}`, "POST", { content });
    appState.configEditor.content = content;
    appState.configEditor.impact = data.impact || appState.configEditor.impact;
    await loadConfigs();
    setConfigSaveNotice("保存成功", "ok");
    showToast("配置已保存", "ok");
  } catch (err) {
    reportActionError(err, "配置保存失败");
    setConfigSaveNotice("保存失败", "error");
  }
}

async function restartRuntimeTarget(mode, button) {
  if (!mode) return;
  const original = button.textContent;
  button.disabled = true;
  button.textContent = "重启中…";
  try {
    const data = await api(`/api/runtime/${encodeURIComponent(mode)}/restart`, "POST", {});
    if (data.runtime) {
      appState.status = { ...(appState.status || {}), runtime: data.runtime };
      renderRuntimeControls();
    }
    setConfigRestartNotice("已请求重启", "ok");
    showToast("底盘控制重启请求已发送", "ok");
  } catch (err) {
    reportActionError(err, "重启失败");
    setConfigRestartNotice("重启失败", "error");
  } finally {
    button.disabled = false;
    button.textContent = original;
  }
}

async function startRuntime(mode) {
  const button = mode === "mapping" ? $("btnStartMapping") : $("btnStartNavigation");
  setButtonBusy(button, true, "启动中…");
  try {
    if (mode === "navigation" && !appState.savedMaps.length) {
      await loadSavedMaps();
      if (!appState.savedMaps.length) return;
    }
    let body = {};
    if (mode === "navigation" && appState.navMapName) {
      body = { map_file: appState.navMapName };
    }
    const data = await api(`/api/runtime/${mode}/start`, "POST", body);
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
      const data = await api(`/api/runtime/${mode}/stop`, "POST", {});
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
    if (!appState.savedMaps.length) {
      appState.previewMapName = "";
      appState.previewMap = null;
      appState.previewLocations = [];
      appState.previewSelectedLocation = "";
      appState.previewAnnotationMode = false;
      appState.previewAnnotationDraft = null;
      appState.navMapName = "";
      appState.navLocations = [];
      appState.navLocationsFor = "";
      $("navLocationInput").value = "";
      $("previewTitle").textContent = "地图预览";
      $("previewMeta").textContent = "从 `maps/` 目录读取 `.pgm` 与 `.yaml`";
      renderMapList();
      renderPreviewLocationList();
      renderNavMapPicker();
      renderNavLocationList();
      renderPreviewCanvas();
      return;
    }
    if (!appState.previewMapName || !appState.savedMaps.some((item) => item.name === appState.previewMapName)) {
      appState.previewMapName = appState.savedMaps[0].name;
    }
    renderMapList();
    renderNavMapPicker();
    loadNavLocations().catch(console.error);
    if (appState.previewMapName) {
      await loadPreviewMap(appState.previewMapName, false);
    }
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
    await api(`/api/maps/${encodeURIComponent(name)}/delete`, "POST", {});
    if (appState.navMapName === name) appState.navMapName = "";
    if (appState.previewMapName === name) {
      appState.previewMapName = "";
      appState.previewMap = null;
      appState.previewLocations = [];
      appState.previewSelectedLocation = "";
      appState.previewAnnotationMode = false;
      appState.previewAnnotationDraft = null;
    }
    await loadSavedMaps();
    showToast(`地图“${name}”已删除`, "ok");
  } catch (err) {
    reportActionError(err, "地图删除失败");
  }
}

async function loadPreviewMap(name, rerenderList = true) {
  try {
    appState.previewMap = await api(`/api/maps/${encodeURIComponent(name)}`);
    appState.previewMapName = name;
    appState.previewSelectedLocation = "";
    appState.previewAnnotationMode = false;
    appState.previewAnnotationDraft = null;
    await loadPreviewLocations(name);
    resetViewport("previewCanvas");
    $("previewTitle").textContent = `地图预览 · ${name}`;
    $("previewMeta").textContent = `尺寸 ${appState.previewMap.width} × ${appState.previewMap.height}，分辨率 ${fmt(appState.previewMap.resolution, 3)} m/px`;
    if (rerenderList) renderMapList();
    else $("btnDeletePreviewMap").disabled = false;
    renderPreviewLocationList();
    renderPreviewCanvas();
  } catch (err) {
    reportActionError(err, "地图预览加载失败");
  }
}

async function loadPreviewLocations(name) {
  try {
    const data = await api(`/api/maps/${encodeURIComponent(name)}/locations`);
    appState.previewLocations = Array.isArray(data.locations) ? data.locations : [];
  } catch (err) {
    console.error(err);
    appState.previewLocations = [];
  }
}

function renderPreviewLocationList() {
  const box = $("previewLocationList");
  if (!box) return;
  box.innerHTML = "";
  if (!appState.previewMapName) {
    box.innerHTML = `<div class="subtle">请选择地图后标注</div>`;
  } else if (!appState.previewLocations.length) {
    box.innerHTML = `<div class="subtle">暂无地点标注</div>`;
  } else {
    appState.previewLocations.forEach((loc) => {
      const btn = document.createElement("button");
      btn.className = "location-item" + (loc.name === appState.previewSelectedLocation ? " active" : "");
      btn.type = "button";
      btn.innerHTML = `<strong>${escapeHtml(loc.name)}</strong><span>x=${fmt(loc.x, 3)}, y=${fmt(loc.y, 3)}, yaw=${fmt(loc.yaw_deg, 1)}°</span>`;
      btn.addEventListener("click", () => {
        appState.previewSelectedLocation = loc.name;
        renderPreviewLocationList();
        renderPreviewCanvas();
      });
      box.appendChild(btn);
    });
  }

  setText(
    "previewLocationMeta",
    appState.previewMapName
      ? `${appState.previewLocations.length} 个地点`
      : "未选择地图",
  );
  $("btnAddPreviewLocation").disabled = !appState.previewMapName;
  $("btnAddPreviewLocation").classList.toggle("active", appState.previewAnnotationMode);
  $("btnCancelPreviewLocation").disabled = !appState.previewAnnotationMode;
  $("btnDeletePreviewLocation").disabled = !appState.previewSelectedLocation;
}

async function savePreviewLocations() {
  if (!appState.previewMapName) return;
  try {
    const data = await api(`/api/maps/${encodeURIComponent(appState.previewMapName)}/locations`, "POST", {
      locations: appState.previewLocations,
    });
    appState.previewLocations = Array.isArray(data.locations) ? data.locations : appState.previewLocations;
    if (appState.navMapName === appState.previewMapName) {
      appState.navLocationsFor = "";
      loadNavLocations(true).catch(console.error);
    }
    showToast("地点标注已保存", "ok");
  } catch (err) {
    reportActionError(err, "地点保存失败");
  } finally {
    renderPreviewLocationList();
    renderPreviewCanvas();
  }
}
