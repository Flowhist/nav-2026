async function pollScene() {
  try {
    const payload = await api(`/api/scene?map_version=${appState.mapVersion}`);
    appState.mapVersion = payload.map_version ?? appState.mapVersion;
    if (payload.map) appState.scene.map = payload.map;
    if ("scan" in payload) appState.scene.scan = payload.scan;
    if ("plan" in payload) appState.scene.plan = payload.plan;
    if ("robot_pose_map" in payload) appState.scene.robot_pose_map = payload.robot_pose_map;
    if ("goal_pose" in payload) appState.scene.goal_pose = payload.goal_pose;
    if ("initial_pose" in payload) appState.scene.initial_pose = payload.initial_pose;
    renderLiveCanvases();
    updateSceneHints();
  } catch (err) {
    console.error(err);
  } finally {
    window.setTimeout(pollScene, 350);
  }
}

function resetLiveScene() {
  appState.mapVersion = -1;
  appState.scene = {
    map: null,
    scan: { points: [] },
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
  $("mappingHint").textContent = hasMap
    ? `${cameraHint}｜地图 ${map.width} × ${map.height}｜雷达点 ${appState.scene.scan?.points?.length || 0}`
    : "等待 `/map` 数据...";

  if (appState.navPlacementMode === "initial") {
    $("navigationHint").textContent = "设置初始位姿中：左键拖拽方向，右键仍可旋转视图。";
  } else if (appState.navPlacementMode === "goal") {
    $("navigationHint").textContent = "设置目的地中：左键拖拽方向，右键仍可旋转视图。";
  } else {
    $("navigationHint").textContent = hasMap
      ? `${cameraHint}｜路径点 ${appState.scene.plan?.points || 0}｜雷达点 ${appState.scene.scan?.points?.length || 0}`
      : "等待 `/map` 数据...";
  }

  $("previewHint").textContent = appState.previewMap
    ? `${cameraHint}｜${appState.previewMap.width} × ${appState.previewMap.height}｜分辨率 ${fmt(appState.previewMap.resolution, 3)} m`
    : "请选择左侧地图";
}

function renderLiveCanvases() {
  const dragPose = appState.navDrag ? {
    x: appState.navDrag.start.x,
    y: appState.navDrag.start.y,
    yaw_deg: appState.navDrag.yawDeg,
    label: appState.navPlacementMode === "initial" ? "初始" : "目标",
  } : null;

  drawScene($("mappingCanvas"), appState.scene.map, appState.scene, { prefix: "live", showTargets: false });
  drawScene($("navigationCanvas"), appState.scene.map, appState.scene, {
    prefix: "live",
    showPlan: true,
    showTargets: true,
    dragPose,
  });
}

function renderPreviewCanvas() {
  drawScene($("previewCanvas"), appState.previewMap, null, { prefix: "preview" });
  updateSceneHints();
}

function renderCanvasById(canvasId) {
  if (canvasId === "previewCanvas") renderPreviewCanvas();
  else renderLiveCanvases();
}

function renderMapList() {
  const box = $("mapList");
  box.innerHTML = "";
  if (!appState.savedMaps.length) {
    box.innerHTML = `<div class="subtle">未发现可预览地图</div>`;
    return;
  }

  appState.savedMaps.forEach((item) => {
    const btn = document.createElement("button");
    btn.className = "map-item" + (item.name === appState.previewMapName ? " active" : "");
    btn.innerHTML = `<strong>${item.name}</strong><span>${item.width} × ${item.height} · ${fmt(item.resolution, 3)} m/px</span>`;
    btn.addEventListener("click", () => loadPreviewMap(item.name));
    box.appendChild(btn);
  });
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

  if (!appState.navMapName || !appState.savedMaps.some((item) => item.name === appState.navMapName)) {
    appState.navMapName = appState.savedMaps[0].name;
  }

  $("navMapLabel").textContent = appState.navMapName;
  $("btnStartNavigation").disabled = getRuntime("navigation").running || getRuntime("navigation").stopping || !appState.savedMaps.length;

  appState.savedMaps.forEach((item) => {
    const btn = document.createElement("button");
    btn.className = "stage-option map-option" + (item.name === appState.navMapName ? " active" : "");
    btn.type = "button";
    btn.innerHTML = `<strong>${item.name}</strong><span>${item.width} × ${item.height} · ${fmt(item.resolution, 3)} m/px</span>`;
    btn.addEventListener("click", () => {
      appState.navMapName = item.name;
      renderNavMapPicker();
      toggleNavMapMenu(false);
    });
    box.appendChild(btn);
  });
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
  applyRunState("mappingRunState", mapping, "运行中");
  applyRunState("navigationRunState", navigation, "运行中");

  $("btnStartMapping").disabled = mapping.running || mapping.stopping;
  $("btnStopMapping").disabled = !mapping.running && !mapping.stopping;
  $("btnStartNavigation").disabled = navigation.running || navigation.stopping || !appState.savedMaps.length;
  $("btnStopNavigation").disabled = !navigation.running && !navigation.stopping;
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
  highlight.style.transform = `translate(${-editor.scrollLeft}px, ${-editor.scrollTop}px)`;
}

function renderConfigHighlight() {
  const editor = $("configEditor");
  const highlight = $("configHighlight");
  highlight.innerHTML = highlightYaml(editor.value);
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
  configSaveNoticeTimer = window.setTimeout(() => {
    node.classList.add("hidden");
    node.dataset.kind = "";
  }, 2200);
}

function renderConfigCards() {
  const box = $("configCards");
  box.innerHTML = "";
  if (!appState.configs.length) {
    box.innerHTML = `<div class="subtle">未发现可编辑的配置文件</div>`;
    return;
  }

  appState.configs.forEach((file) => {
    const card = document.createElement("button");
    card.className = "config-card";
    card.innerHTML = `<strong>${file.name}</strong><span>${file.path}</span><span>${formatFileSize(file.size)}</span>`;
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
    console.error(err);
  }
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
    };
    $("configEditorTitle").textContent = data.name || name;
    $("configEditorMeta").textContent = data.path || `config/${name}`;
    $("configEditor").value = data.content || "";
    renderConfigHighlight();
    setConfigSaveNotice("");
    showConfigEditor();
  } catch (err) {
    console.error(err);
  }
}

async function saveConfigEditor() {
  if (!appState.configEditor.name) return;
  const content = $("configEditor").value;
  try {
    await api(`/api/configs/${encodeURIComponent(appState.configEditor.name)}`, "POST", { content });
    appState.configEditor.content = content;
    await loadConfigs();
    if (appState.configEditor.name === "base_control.yaml" && typeof loadTeleopConfig === "function") {
      await loadTeleopConfig();
    }
    setConfigSaveNotice("保存成功", "ok");
  } catch (err) {
    console.error(err);
    setConfigSaveNotice("保存失败", "error");
  }
}

async function startRuntime(mode) {
  try {
    if (mode === "navigation" && !appState.savedMaps.length) {
      await loadSavedMaps();
      if (!appState.savedMaps.length) return;
    }
    const body = mode === "navigation" && appState.navMapName ? { map_file: appState.navMapName } : {};
    const data = await api(`/api/runtime/${mode}/start`, "POST", body);
    if (data.runtime) {
      appState.status = { ...(appState.status || {}), runtime: data.runtime };
      renderRuntimeControls();
    }
  } catch (err) {
    console.error(err);
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
    } catch (err) {
      console.error(err);
    }
  };
  if (options.showModal) await withBlockingModal(run);
  else await run();
}

async function loadSavedMaps() {
  try {
    const data = await api("/api/maps");
    appState.savedMaps = data.maps || [];
    if (!appState.previewMapName && appState.savedMaps.length) {
      appState.previewMapName = appState.savedMaps[0].name;
    }
    renderMapList();
    renderNavMapPicker();
    if (appState.previewMapName) {
      await loadPreviewMap(appState.previewMapName, false);
    }
  } catch (err) {
    console.error(err);
  }
}

async function loadPreviewMap(name, rerenderList = true) {
  try {
    appState.previewMap = await api(`/api/maps/${encodeURIComponent(name)}`);
    appState.previewMapName = name;
    resetViewport("previewCanvas");
    $("previewTitle").textContent = `地图预览 · ${name}`;
    $("previewMeta").textContent = `尺寸 ${appState.previewMap.width} × ${appState.previewMap.height}，分辨率 ${fmt(appState.previewMap.resolution, 3)} m/px`;
    if (rerenderList) renderMapList();
    renderPreviewCanvas();
  } catch (err) {
    console.error(err);
  }
}
