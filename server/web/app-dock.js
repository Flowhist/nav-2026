function renderDockStream() {
  const box = $("dockStream");
  const meta = $("dockStreamMeta");
  const title = $("dockLogTitle");
  const clearBtn = $("btnClearDockPane");
  const view = appState.dockView;
  document.querySelector(".dock-event-filters").hidden = view !== "events";
  const shouldFollow = appState.dockAutoFollow[view] ?? true;
  const prevScrollTop = box.scrollTop;
  const prevScrollHeight = box.scrollHeight;

  document.querySelectorAll(".dock-tab").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.logView === view);
  });

  if (view === "events") {
    const events = appState.events.filter((evt) => appState.eventLevel === "all" || normalizeLevel(evt.level) === appState.eventLevel);
    title.textContent = "事件";
    meta.textContent = `${events.length} 条事件${appState.eventTruncated ? " · 较早事件已截断" : ""}`;
    clearBtn.textContent = "清空显示";
    box.className = "dock-stream events";
    box.innerHTML = "";
    if (!events.length) {
      box.innerHTML = `<div class="event"><span class="lvl">INFO</span><div class="msg">当前没有事件</div></div>`;
      return;
    }
    events.forEach((evt) => {
      const row = document.createElement("div");
      const lvl = normalizeLevel(evt.level);
      row.className = "event";
      row.dataset.level = lvl;
      row.innerHTML = `<span class="lvl">${escapeHtml(String(evt.level || "info").toUpperCase())}</span><div class="msg">[${formatTs(evt.ts)}] ${escapeHtml(evt.message || "")}${escapeHtml(formatExtra(evt.extra))}</div>`;
      box.appendChild(row);
    });
    if (shouldFollow) box.scrollTop = box.scrollHeight;
    else box.scrollTop = prevScrollTop + (box.scrollHeight - prevScrollHeight);
    return;
  }

  const log = appState.runtimeLogs[view] || { lines: [], updatedAt: null };
  title.textContent = view === "mapping" ? "建图日志" : "导航日志";
  meta.textContent = `${view === "mapping" ? "mapping.log" : "navigation.log"} · ${log.updatedAt ? `最近更新 ${formatTs(log.updatedAt)}` : "暂无日志"}`;
  clearBtn.textContent = "清空日志";
  box.className = "dock-stream";
  box.innerHTML = "";
  if (!log.lines.length) {
    box.innerHTML = `<div class="log-line raw"><span class="lvl">LOG</span><div class="msg">当前没有日志内容</div></div>`;
    return;
  }
  log.lines.forEach((line) => {
    const level = deriveLogLevel(line);
    const row = document.createElement("div");
    row.className = `log-line ${level}`;
    row.innerHTML = `<span class="lvl">${level === "raw" ? "LOG" : level.toUpperCase()}</span><div class="msg">${escapeHtml(normalizeLogLine(line))}</div>`;
    box.appendChild(row);
  });
  if (shouldFollow) box.scrollTop = box.scrollHeight;
  else box.scrollTop = prevScrollTop + (box.scrollHeight - prevScrollHeight);
}

function addEvents(events) {
  if (!events?.length) return;
  events.forEach((evt) => {
    appState.events.push(evt);
    appState.lastSeq = Math.max(appState.lastSeq, evt.seq || 0);
  });
  if (appState.events.length > appState.eventLimit) {
    appState.events = appState.events.slice(-appState.eventLimit);
    appState.eventTruncated = true;
  }
  if (appState.dockView === "events") renderDockStream();
}

function clearDockPane() {
  if (appState.dockView === "events") {
    appState.events = [];
    renderDockStream();
    return;
  }
  const mode = appState.dockView;
  appState.runtimeLogs[mode] = { lines: [], updatedAt: null };
  renderDockStream();
  api(`/api/runtime/logs/${mode}/clear`, "POST", {}).catch(console.error);
}

async function loadRuntimeLog(mode) {
  try {
    const version = appState.runtimeLogs[mode]?.version || "";
    const data = await api(`/api/runtime/logs/${mode}?tail=320&version=${encodeURIComponent(version)}`);
    if (data.unchanged) return;
    appState.runtimeLogs[mode] = {
      lines: data.lines || [],
      updatedAt: data.updated_at || null,
      version: data.version,
    };
    if (appState.dockView === mode) renderDockStream();
  } catch (err) {
    console.error(err);
  }
}

function setDockView(view) {
  appState.dockView = view;
  renderDockStream();
  if (view === "mapping" || view === "navigation") {
    loadRuntimeLog(view).catch(console.error);
  }
}

function showBlockingModal(visible, message = "正在处理，请稍候…") {
  setText("blockingModalText", message);
  setHidden("blockingModal", !visible);
}

async function withBlockingModal(task, message) {
  showBlockingModal(true, message);
  try {
    return await task();
  } finally {
    showBlockingModal(false);
  }
}

async function pollStatus() {
  try {
    const status = await api("/api/status");
    if (appState.serverSession && appState.serverSession !== status.session_id) {
      resetLiveScene();
      if (isLivePage() && $("statusDock").classList.contains("collapsed")) startSceneStream();
      if (appState.mapSaveJob) showToast("服务已重启，请检查地图保存结果", "error");
      appState.mapSaveJob = null;
    }
    appState.serverSession = status.session_id;
    const save = status.map_save;
    if (save?.id === appState.mapSaveJob && save.state !== "running") {
      appState.mapSaveJob = null;
      if (save.state === "succeeded") {
        showToast(`地图“${save.name}”已保存`, "ok");
        loadSavedMaps().catch(console.error);
      } else showToast(save.error || "地图保存失败", "error");
    }
    const control = appState.status?.control;
    appState.status = { ...status, control };
    applyManualControlStatus(status.control);
    renderStatus(appState.status);
  } catch (err) {
    console.error(err);
  } finally {
    window.setTimeout(pollStatus, document.hidden ? 3000 : 1000);
  }
}

async function pollEvents() {
  try {
    if (appState.statusPanel === "logs" && !$("statusDock").classList.contains("collapsed")) {
      const data = await api(`/api/history?since=${appState.lastSeq}&limit=200&session_id=${encodeURIComponent(appState.eventSession || "")}`);
      if (data.reset) {
        appState.events = [];
        appState.lastSeq = 0;
        appState.eventTruncated = false;
      }
      appState.eventSession = data.session_id;
      appState.eventTruncated ||= data.truncated;
      addEvents(data.events || []);
      appState.lastSeq = data.latest_seq;
      if (data.reset && !data.events?.length) renderDockStream();
    }
  } catch (err) {
    console.error(err);
  } finally {
    window.setTimeout(pollEvents, document.hidden ? 4000 : 1500);
  }
}

async function pollDockLogs() {
  try {
    if (
      appState.statusPanel === "logs"
      && !$("statusDock").classList.contains("collapsed")
      && (appState.dockView === "mapping" || appState.dockView === "navigation")
    ) {
      await loadRuntimeLog(appState.dockView);
    }
  } finally {
    window.setTimeout(pollDockLogs, document.hidden ? 5000 : 2000);
  }
}
