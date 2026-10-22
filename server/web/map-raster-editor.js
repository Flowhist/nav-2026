"use strict";

// Sparse edits and event handlers exist only for the duration of a brush session.
window.finavRasterEditor = {
  active: false,
  async enter() {
    const editor = window.finavMapEditor;
    if (this.active || !editor?.ready || !editor.active || editor.readOnly) return;
    await editor.exit();
    this.active = true;
    appState.previewEditActive = true;
    this.busy = false;
    this.map = editor.map;
    this.canvas = editor.canvas;
    this.changes = new Map();
    this.history = [];
    this.value = 100;
    this.frame = null;
    this.pointer = null;
    this.stroke = null;
    this.controller = new AbortController();
    const options = { capture: true, signal: this.controller.signal };
    for (const type of ["pointerdown", "pointermove", "pointerup", "pointercancel", "lostpointercapture"]) {
      this.canvas.addEventListener(type, event => this.pointerEvent(event), options);
    }
    window.addEventListener("beforeunload", event => {
      if (this.changes.size) { event.preventDefault(); event.returnValue = ""; }
    }, { signal: this.controller.signal });
    window.addEventListener("blur", () => this.finishStroke(), { signal: this.controller.signal });
    document.addEventListener("keydown", event => {
      if (/INPUT|TEXTAREA|SELECT/.test(event.target.tagName)) return;
      if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z") {
        event.preventDefault(); this.undo();
      }
    }, { signal: this.controller.signal });
    this.canvas.style.cursor = "crosshair";
    this.controls(true);
    toggleNavSelect("previewMapPanel", false);
    $("editorMoreMenu").classList.add("hidden");
    this.setColor(100);
    this.update();
  },
  controls(active) {
    $("previewLayout").classList.toggle("raster-editing", active);
    $("btnMapViewMode").classList.toggle("active", !active);
    $("btnMapViewMode").setAttribute("aria-pressed", String(!active));
    $("btnMapEditMode").classList.toggle("active", active);
    $("btnMapEditMode").setAttribute("aria-pressed", String(active));
    $("editorTools").classList.toggle("hidden", !active);
    $("rasterSection").classList.toggle("hidden", !active);
    document.querySelectorAll("[data-editor-tool]").forEach(button => {
      button.disabled = active;
      if (active) button.classList.remove("active");
    });
    $("rasterTools").classList.toggle("hidden", !active);
    $("btnRasterMode").classList.toggle("active", active);
    $("btnRasterMode").setAttribute("aria-pressed", String(active));
    for (const id of ["btnMapEditMode", "previewMapToggle", "btnRefreshMaps", "btnDeletePreviewMap", "btnEditorExport"]) {
      $(id).disabled = active;
    }
  },
  setColor(value) {
    this.value = value;
    document.querySelectorAll("[data-raster-value]").forEach(button => {
      button.setAttribute("aria-pressed", String(Number(button.dataset.rasterValue) === value));
    });
    $("rasterColorLabel").textContent = {100: "占据", 0: "可通行", "-1": "未知"}[value];
  },
  point(event) {
    const point = getCanvasPointer(this.canvas, event);
    if (!point || !this.canvas._view) return null;
    const local = invertCameraPoint(this.canvas, point);
    const view = this.canvas._view;
    return { x: (local.x - view.offsetX) * this.map.width / view.drawWidth, y: (local.y - view.offsetY) * this.map.height / view.drawHeight };
  },
  pointerEvent(event) {
    if (event.type === "pointerdown" && event.button !== 0) return;
    if (event.type !== "pointerdown" && event.pointerId !== this.pointer) return;
    event.stopImmediatePropagation();
    event.preventDefault();
    if (this.busy) return;
    if (event.type === "pointerdown") {
      if (this.pointer !== null) return;
      this.pointer = event.pointerId;
      this.stroke = new Map();
      this.last = this.point(event);
      this.canvas.setPointerCapture(event.pointerId);
      if (this.last) this.paint(this.last);
    } else if (event.type === "pointermove") {
      const next = this.point(event);
      if (!next) return;
      const last = this.last || next;
      const steps = Math.ceil(Math.hypot(next.x - last.x, next.y - last.y));
      // Bound work even if a pointer is captured far outside the canvas.
      if (steps <= 4096) for (let i = 1; i <= steps; i++) {
        this.paint({x: last.x + (next.x - last.x) * i / steps, y: last.y + (next.y - last.y) * i / steps});
      }
      this.last = next;
    } else this.finishStroke();
  },
  paint(point) {
    const size = Number($("rasterRadius").value);
    const { width, height } = this.map;
    const left = Math.floor(point.x) - Math.floor((size - 1) / 2);
    const top = Math.floor(point.y) - Math.floor((size - 1) / 2);
    const context = buildMapRaster(this.map, "preview").getContext("2d");
    context.fillStyle = this.color(this.value);
    for (let y = Math.max(0, top); y < Math.min(height, top + size); y++) {
      for (let x = Math.max(0, left); x < Math.min(width, left + size); x++) {
        const index = y * width + x;
        if (this.map.data[index] === this.value) continue;
        if (!this.changes.has(index) && this.changes.size >= 100000) continue;
        if (!this.stroke.has(index)) this.stroke.set(index, this.map.data[index]);
        if (!this.changes.has(index)) this.changes.set(index, this.map.data[index]);
        this.map.data[index] = this.value;
        context.fillRect(x, y, 1, 1);
      }
    }
    this.drawSoon();
  },
  color(value) { return value < 0 ? "rgb(207,215,207)" : value >= 65 ? "rgb(52,70,61)" : "rgb(246,246,239)"; },
  drawSoon() {
    if (this.frame !== null) return;
    this.frame = requestAnimationFrame(() => {
      this.frame = null;
      window.finavMapEditor.draw();
    });
  },
  finishStroke() {
    const pointer = this.pointer;
    this.pointer = null;
    if (pointer !== null && this.canvas.hasPointerCapture(pointer)) this.canvas.releasePointerCapture(pointer);
    if (this.stroke?.size) {
      this.history.push(this.stroke);
      while (this.history.length > 20 || (this.history.length > 1 && this.history.reduce((n, stroke) => n + stroke.size, 0) > 100000)) this.history.shift();
    }
    this.stroke = null;
    this.last = null;
    this.update();
  },
  undo() {
    if (this.busy || this.pointer !== null) return;
    const stroke = this.history.pop();
    if (!stroke) return;
    const context = buildMapRaster(this.map, "preview").getContext("2d");
    stroke.forEach((value, index) => {
      this.map.data[index] = value;
      context.fillStyle = this.color(value);
      context.fillRect(index % this.map.width, Math.floor(index / this.map.width), 1, 1);
    });
    this.update();
    this.drawSoon();
  },
  update() {
    $("btnRasterUndo").disabled = this.busy || !this.history.length;
    $("btnRasterSave").disabled = this.busy || !this.changes.size;
    $("btnRasterCancel").disabled = this.busy;
    $("rasterStatus").textContent = this.busy ? "正在保存…" : this.changes.size >= 100000 ? "已达单次编辑上限，请保存" : this.changes.size ? "有未保存修改" : "未修改";
  },
  async save() {
    if (this.busy || !this.changes.size) return;
    this.finishStroke();
    this.busy = true;
    this.update();
    try {
      const cells = Array.from(this.changes.keys()).filter(index => this.map.data[index] !== this.changes.get(index)).map(index => [index, this.map.data[index]]);
      if (!cells.length) { this.close(true); return; }
      await api(`/api/maps/${encodeURIComponent(this.map.name)}/raster`, "PUT", {
        source_sha256: this.map.source_sha256,
        cells,
      });
      const name = this.map.name;
      this.close(false);
      await loadPreviewMap(name);
      await window.finavMapEditor.enter();
      showToast("地图栅格已保存");
    } catch (error) {
      this.busy = false;
      this.update();
      reportActionError(error, "地图栅格保存失败");
    }
  },
  async cancel() {
    if (!this.active) return true;
    if (this.busy) return false;
    if (this.cancelPending) return this.cancelPending;
    this.cancelPending = (async () => {
      if (this.changes.size && !await showAnnotationConfirmDialog({ title: "放弃栅格修改", message: "未保存的笔画将被丢弃。", confirmText: "放弃修改", danger: true })) return false;
      this.close(true);
      return true;
    })();
    try { return await this.cancelPending; }
    finally { this.cancelPending = null; }
  },
  close(restore) {
    this.finishStroke();
    this.controller.abort();
    if (this.frame !== null) cancelAnimationFrame(this.frame);
    if (restore) this.changes.forEach((value, index) => { this.map.data[index] = value; });
    mapRasterCache.delete(getMapCacheKey(this.map, "preview"));
    this.active = false;
    appState.previewEditActive = false;
    this.changes = this.history = this.map = this.controller = null;
    this.controls(false);
    window.finavMapEditor.applyMode();
    if (restore) window.finavMapEditor.enter();
    window.finavMapEditor.draw();
  },
};

$("btnRasterMode").addEventListener("click", () => window.finavRasterEditor.enter().catch(error => reportActionError(error, "无法启用画笔")));
$("btnRasterUndo").addEventListener("click", () => window.finavRasterEditor.undo());
$("btnRasterSave").addEventListener("click", () => window.finavRasterEditor.save());
$("btnRasterCancel").addEventListener("click", () => window.finavRasterEditor.cancel());
$("rasterRadius").addEventListener("input", event => { $("rasterRadiusValue").textContent = `${event.target.value} × ${event.target.value} 格`; });
document.querySelectorAll("[data-raster-value]").forEach(button => button.addEventListener("click", () => window.finavRasterEditor.setColor(Number(button.dataset.rasterValue))));
