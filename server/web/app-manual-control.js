const manualControl = {
  open: false,
  enabled: false,
  pointerId: null,
  keys: new Set(),
  linear: 0,
  angular: 0,
  pending: null,
  sending: false,
  sendTimer: null,
  lastSentAt: 0,
  heartbeat: null,
  lastErrorAt: 0,
};

function manualControlNodes() {
  const root = $("manualDrive");
  return {
    root,
    panel: $("manualDrivePanel"),
    trigger: root?.querySelector(".manual-drive-trigger"),
    close: root?.querySelector(".manual-drive-close"),
    state: root?.querySelector(".manual-drive-state"),
    gear: root?.querySelector(".manual-gear strong"),
    stick: root?.querySelector(".manual-stick"),
    knob: root?.querySelector(".manual-stick-knob"),
    enable: root?.querySelector(".manual-enable"),
  };
}

function manualControlAvailability() {
  const control = appState.status?.control || {};
  if (control.manual_locked) return { ready: false, label: "手柄抢停，等待控制回中" };
  if (control.joystick_active) return { ready: false, label: "物理摇杆正在接管" };
  if (!control.gear_online) return { ready: false, label: "等待手柄档位" };
  if (!appState.status?.ros?.connected) return { ready: false, label: "ROS 未连接" };
  return { ready: true, label: manualControl.enabled ? "控制已启用" : "尚未启用" };
}

function renderManualControlStatus(control = appState.status?.control || {}) {
  const nodes = manualControlNodes();
  if (!nodes.root) return;
  const availability = manualControlAvailability();
  nodes.gear.textContent = control.gear_online ? `${control.gear} 档 · 跟随手柄` : "不可用";
  nodes.state.textContent = availability.label;
  nodes.enable.disabled = !availability.ready && !manualControl.enabled;
  nodes.enable.textContent = manualControl.enabled ? "停用控制" : "启用控制";
  nodes.enable.classList.toggle("active", manualControl.enabled);
  nodes.root.classList.toggle("active", manualControl.enabled);
  nodes.root.classList.toggle("locked", !!control.manual_locked || !!control.joystick_active);

  if (manualControl.enabled && (!availability.ready || control.joystick_active)) {
    stopManualControl(availability.label, true);
  }
}

function setManualPanelOpen(open) {
  const nodes = manualControlNodes();
  manualControl.open = !!open;
  nodes.root.classList.toggle("open", manualControl.open);
  nodes.panel.classList.toggle("hidden", !manualControl.open);
  nodes.trigger.setAttribute("aria-expanded", String(manualControl.open));
  nodes.root.closest(".canvas-shell")?.classList.toggle("manual-open", manualControl.open);
  if (!manualControl.open) stopManualControl("尚未启用");
  renderManualControlStatus();
}

function updateManualKnob() {
  const { stick, knob } = manualControlNodes();
  if (!stick || !knob) return;
  const travel = Math.max(0, stick.clientWidth / 2 - knob.clientWidth / 2 - 5);
  knob.style.transform = `translate3d(${-manualControl.angular * travel}px, ${-manualControl.linear * travel}px, 0)`;
}

function queueManualCommand(linear, angular) {
  manualControl.pending = { linear, angular };
  if (manualControl.sending) return;
  const isStop = Math.hypot(linear, angular) <= 1e-6;
  if (isStop && manualControl.sendTimer) {
    window.clearTimeout(manualControl.sendTimer);
    manualControl.sendTimer = null;
  }
  if (manualControl.sendTimer) return;
  const delay = isStop ? 0 : Math.max(0, 50 - (performance.now() - manualControl.lastSentAt));
  manualControl.sendTimer = window.setTimeout(() => {
    manualControl.sendTimer = null;
    sendPendingManualCommand();
  }, delay);
}

async function sendPendingManualCommand() {
  const command = manualControl.pending;
  if (!command) return;
  manualControl.pending = null;
  manualControl.sending = true;
  manualControl.lastSentAt = performance.now();
  try {
    await api("/api/control/manual", "POST", command, { timeoutMs: 1200 });
  } catch (err) {
    if (Math.hypot(command.linear, command.angular) > 1e-6) {
      const now = Date.now();
      if (now - manualControl.lastErrorAt > 2500) {
        manualControl.lastErrorAt = now;
        reportActionError(err, "手动控制发送失败");
      }
      stopManualControl(err?.message || "控制已中断", true);
    }
  } finally {
    manualControl.sending = false;
    if (manualControl.pending) queueManualCommand(manualControl.pending.linear, manualControl.pending.angular);
  }
}

function applyManualVector(linear, angular) {
  const magnitude = Math.hypot(linear, angular);
  const scale = magnitude > 1 ? 1 / magnitude : 1;
  manualControl.linear = linear * scale;
  manualControl.angular = angular * scale;
  updateManualKnob();
  queueManualCommand(manualControl.linear, manualControl.angular);
}

function resetManualInput(sendStop = true) {
  manualControl.keys.clear();
  manualControl.pointerId = null;
  manualControl.linear = 0;
  manualControl.angular = 0;
  manualControlNodes().root?.classList.remove("dragging");
  updateManualKnob();
  if (sendStop) queueManualCommand(0, 0);
}

function setManualControlEnabled(enabled) {
  if (enabled && !manualControlAvailability().ready) {
    renderManualControlStatus();
    return;
  }
  manualControl.enabled = !!enabled;
  resetManualInput(true);
  if (manualControl.heartbeat) window.clearInterval(manualControl.heartbeat);
  manualControl.heartbeat = manualControl.enabled
    ? window.setInterval(() => {
        if (Math.hypot(manualControl.linear, manualControl.angular) > 1e-6) {
          queueManualCommand(manualControl.linear, manualControl.angular);
        }
      }, 100)
    : null;
  renderManualControlStatus();
}

function stopManualControl(reason = "尚未启用", keepPanel = false) {
  manualControl.enabled = false;
  if (manualControl.heartbeat) window.clearInterval(manualControl.heartbeat);
  manualControl.heartbeat = null;
  resetManualInput(true);
  const nodes = manualControlNodes();
  if (nodes.state) nodes.state.textContent = reason;
  if (!keepPanel && !manualControl.open) nodes.root?.closest(".canvas-shell")?.classList.remove("manual-open");
  renderManualControlStatus();
}

function emergencyStopManualControl() {
  manualControl.enabled = false;
  if (manualControl.sendTimer) window.clearTimeout(manualControl.sendTimer);
  manualControl.sendTimer = null;
  manualControl.pending = null;
  resetManualInput(false);
  const body = JSON.stringify({ linear: 0, angular: 0 });
  if (navigator.sendBeacon) {
    navigator.sendBeacon("/api/control/manual", new Blob([body], { type: "application/json" }));
  } else {
    fetch("/api/control/manual", { method: "POST", headers: { "Content-Type": "application/json" }, body, keepalive: true }).catch(() => {});
  }
}

function manualVectorFromKeys() {
  const has = (...keys) => keys.some((key) => manualControl.keys.has(key));
  const linear = Number(has("w", "arrowup")) - Number(has("s", "arrowdown"));
  const angular = Number(has("a", "arrowleft")) - Number(has("d", "arrowright"));
  applyManualVector(linear, angular);
}

function updateManualPointer(event) {
  if (!manualControl.enabled || event.pointerId !== manualControl.pointerId) return;
  const { stick } = manualControlNodes();
  const rect = stick.getBoundingClientRect();
  const radius = Math.max(1, rect.width / 2 - 5);
  const dx = event.clientX - (rect.left + rect.width / 2);
  const dy = event.clientY - (rect.top + rect.height / 2);
  applyManualVector(-dy / radius, -dx / radius);
  event.preventDefault();
}

function isEditableTarget(target) {
  return !!target?.closest?.("input, textarea, select, [contenteditable='true']");
}

function manualControlOnPageChange(page) {
  const root = $("manualDrive");
  const target = page === "mapping" ? $("mappingCanvas") : page === "navigation" ? $("navigationCanvas") : null;
  setManualPanelOpen(false);
  if (root && target) target.closest(".canvas-shell")?.appendChild(root);
  if (root) root.classList.toggle("hidden", !target);
}

function bindManualControl() {
  const nodes = manualControlNodes();
  nodes.trigger.addEventListener("click", () => setManualPanelOpen(true));
  nodes.close.addEventListener("click", () => setManualPanelOpen(false));
  nodes.enable.addEventListener("click", () => setManualControlEnabled(!manualControl.enabled));

  nodes.stick.addEventListener("pointerdown", (event) => {
    if (!manualControl.enabled || manualControl.pointerId !== null) return;
    manualControl.keys.clear();
    manualControl.pointerId = event.pointerId;
    nodes.root.classList.add("dragging");
    nodes.stick.setPointerCapture?.(event.pointerId);
    updateManualPointer(event);
    event.stopPropagation();
  });
  nodes.stick.addEventListener("pointermove", updateManualPointer);
  const releasePointer = (event) => {
    if (event.pointerId !== manualControl.pointerId) return;
    resetManualInput(true);
    event.stopPropagation();
  };
  nodes.stick.addEventListener("pointerup", releasePointer);
  nodes.stick.addEventListener("pointercancel", releasePointer);
  nodes.stick.addEventListener("lostpointercapture", releasePointer);

  window.addEventListener("keydown", (event) => {
    const key = event.key.toLowerCase();
    if (!manualControl.enabled || manualControl.pointerId !== null || isEditableTarget(event.target)) return;
    if (!["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(key)) return;
    event.preventDefault();
    manualControl.keys.add(key);
    manualVectorFromKeys();
  });
  window.addEventListener("keyup", (event) => {
    const key = event.key.toLowerCase();
    if (!manualControl.keys.has(key)) return;
    event.preventDefault();
    manualControl.keys.delete(key);
    manualVectorFromKeys();
  });
  window.addEventListener("blur", () => stopManualControl("窗口失去焦点"));
  renderManualControlStatus();
}
