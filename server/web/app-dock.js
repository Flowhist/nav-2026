function renderDockStream() {
  const box = $("dockStream");
  const meta = $("dockStreamMeta");
  const title = $("dockLogTitle");
  const clearBtn = $("btnClearDockPane");
  const view = appState.dockView;
  const shouldFollow = appState.dockAutoFollow[view] ?? true;
  const prevScrollTop = box.scrollTop;
  const prevScrollHeight = box.scrollHeight;

  document.querySelectorAll(".dock-tab").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.logView === view);
  });

  if (view === "events") {
    title.textContent = "事件";
    meta.textContent = `显示后端事件流，共 ${appState.events.length} 条`;
    clearBtn.textContent = "清空显示";
    box.className = "dock-stream events";
    box.innerHTML = "";
    if (!appState.events.length) {
      box.innerHTML = `<div class="event"><span class="lvl">INFO</span><div class="msg">当前没有事件</div></div>`;
      return;
    }
    appState.events.forEach((evt) => {
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
  if (appState.events.length > 240) {
    appState.events = appState.events.slice(-240);
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
    const data = await api(`/api/runtime/logs/${mode}?tail=320`);
    appState.runtimeLogs[mode] = {
      lines: data.lines || [],
      updatedAt: data.updated_at || null,
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

function showBlockingModal(visible) {
  setHidden("blockingModal", !visible);
}

async function withBlockingModal(task) {
  showBlockingModal(true);
  try {
    return await task();
  } finally {
    showBlockingModal(false);
  }
}

async function pollStatus() {
  try {
    appState.status = await api("/api/status");
    renderStatus(appState.status);
  } catch (err) {
    console.error(err);
  } finally {
    window.setTimeout(pollStatus, 500);
  }
}

async function pollEvents() {
  try {
    const data = await api(`/api/history?since=${appState.lastSeq}&limit=200`);
    addEvents(data.events || []);
  } catch (err) {
    console.error(err);
  } finally {
    window.setTimeout(pollEvents, 1000);
  }
}

async function pollDockLogs() {
  try {
    if (appState.dockView === "mapping" || appState.dockView === "navigation") {
      await loadRuntimeLog(appState.dockView);
    }
  } finally {
    window.setTimeout(pollDockLogs, 1500);
  }
}

