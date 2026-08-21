"use strict";

const MAP_EDITOR_HINTS = {
  select: "选择对象并拖动编辑；拖动空白区域平移地图",
  location: "点击地点位置并拖动设置朝向",
  keepout: "从一个角拖到对角创建禁行区",
  route: "依次点击添加关键点；双击或 Enter 完成路线",
};

class FinavMapEditor {
  constructor(canvas) {
    this.canvas = canvas;
    this.active = false;
    this.mapName = "";
    this.map = null;
    this.document = null;
    this.revision = 0;
    this.selection = null;
    this.tool = "select";
    this.undoStack = [];
    this.redoStack = [];
    this.saveQueue = Promise.resolve();
    this.pendingSaves = 0;
    this.lastPersisted = null;
    this.routePreviews = new Map();
    this.hiddenObjects = new Set();
    this.routeDraft = [];
    this.hoverWorld = null;
    this.drag = null;
    this.readOnly = false;
    this.bind();
  }

  clone(value = this.document) {
    return value ? JSON.parse(JSON.stringify(value)) : value;
  }

  bind() {
    document.querySelectorAll("[data-editor-tool]").forEach((button) => {
      button.addEventListener("click", () => this.setTool(button.dataset.editorTool));
    });
    $("btnEditPreviewMap").addEventListener("click", () => this.enter());
    $("btnFinishMapEdit").addEventListener("click", () => this.exit());
    $("btnEditorUndo").addEventListener("click", () => this.undo());
    $("btnEditorRedo").addEventListener("click", () => this.redo());
    $("btnToggleObjectSearch").addEventListener("click", () => {
      const input = $("editorObjectSearch");
      input.classList.toggle("hidden");
      if (!input.classList.contains("hidden")) input.focus();
    });
    $("editorObjectSearch").addEventListener("input", () => this.renderTree());
    $("btnEditorMore").addEventListener("click", (event) => {
      event.stopPropagation();
      const menu = $("editorMoreMenu");
      menu.classList.toggle("hidden");
      $("btnEditorMore").setAttribute("aria-expanded", String(!menu.classList.contains("hidden")));
    });
    $("btnEditorExport").addEventListener("click", () => this.openExportDialog());
    $("btnCancelEditorExport").addEventListener("click", () => this.closeExportDialog());
    $("editorExportForm").addEventListener("submit", (event) => this.exportMap(event));
    document.addEventListener("click", (event) => {
      if (!event.target.closest(".editor-more-wrap")) {
        $("editorMoreMenu").classList.add("hidden");
        $("btnEditorMore").setAttribute("aria-expanded", "false");
      }
    });

    this.canvas.addEventListener("pointerdown", (event) => this.pointerDown(event));
    this.canvas.addEventListener("pointermove", (event) => this.pointerMove(event));
    this.canvas.addEventListener("pointerup", (event) => this.pointerUp(event));
    this.canvas.addEventListener("pointercancel", () => this.cancelDrag());
    this.canvas.addEventListener("dblclick", (event) => {
      if (this.active && this.tool === "route") {
        event.preventDefault();
        this.finishRoute();
      }
    });
    document.addEventListener("keydown", (event) => this.keyDown(event));
  }

  async enter(initialTool = "select") {
    if (!appState.previewMapName || this.active) return;
    try {
      const payload = await api(`/api/maps/${encodeURIComponent(appState.previewMapName)}/editor`);
      this.mapName = appState.previewMapName;
      this.map = {
        ...appState.previewMap,
        source_sha256: payload.document.map.source_sha256,
        occupancy: appState.previewMap.data,
      };
      this.document = payload.document;
      this.revision = Number(payload.document.revision || 0);
      this.lastPersisted = this.clone();
      this.readOnly = Boolean(payload.read_only);
      this.undoStack = [];
      this.redoStack = [];
      this.selection = null;
      this.routePreviews.clear();
      this.hiddenObjects.clear();
      this.routeDraft = [];
      this.active = true;
      appState.previewEditActive = true;
      this.applyMode();
      this.setTool(initialTool);
      this.renderAll();
      if (payload.source_changed) showToast("地图栅格已变化，路线预览需要重新生成", "error");
      if (this.readOnly) showToast(payload.read_only_reason || "当前编辑文档为只读状态", "error");
    } catch (error) {
      reportActionError(error, "地图编辑器加载失败");
    }
  }

  async exit() {
    await this.saveQueue;
    this.active = false;
    appState.previewEditActive = false;
    this.drag = null;
    this.routeDraft = [];
    this.selection = null;
    this.applyMode();
    await loadPreviewMap(this.mapName, false);
    if (appState.navMapName === this.mapName) {
      appState.navLocationsFor = "";
      loadNavLocations(true).catch(console.error);
    }
  }

  applyMode() {
    $("previewLayout").classList.toggle("editing", this.active);
    $("previewBrowseSidebar").classList.toggle("hidden", this.active);
    $("previewEditorSidebar").classList.toggle("hidden", !this.active);
    $("previewBrowseActions").classList.toggle("hidden", this.active);
    $("previewEditActions").classList.toggle("hidden", !this.active);
    $("editorPropertyInspector").classList.toggle("hidden", !this.active);
    $("editorCoordinates").classList.toggle("hidden", !this.active);
    if (this.active) {
      $("previewTitle").textContent = `地图 · ${this.mapName}`;
      $("previewMeta").textContent = "编辑地点、禁行区和巡航路线";
    }
  }

  setTool(tool) {
    this.tool = tool;
    this.routeDraft = [];
    this.drag = null;
    document.querySelectorAll("[data-editor-tool]").forEach((button) => {
      button.classList.toggle("active", button.dataset.editorTool === tool);
    });
    this.canvas.style.cursor = tool === "select" ? "default" : "crosshair";
    $("previewHint").textContent = MAP_EDITOR_HINTS[tool];
    this.draw();
  }

  setSelection(type = null, id = null, childId = null) {
    this.selection = type && id ? { type, id, childId } : null;
    this.renderTree();
    this.renderProperties();
    this.draw();
  }

  selectedObject() {
    if (!this.selection || !this.document) return null;
    const key = { location: "locations", keepout: "keepouts", route: "routes" }[this.selection.type];
    return this.document[key]?.find((item) => item.id === this.selection.id) || null;
  }

  objectBySelection(document, selection = this.selection) {
    if (!selection) return null;
    const key = { location: "locations", keepout: "keepouts", route: "routes" }[selection.type];
    return document[key]?.find((item) => item.id === selection.id) || null;
  }

  async commit(label, mutator) {
    if (!this.document || this.readOnly) {
      if (this.readOnly) showToast("当前编辑文档为只读状态", "error");
      return;
    }
    const before = this.clone();
    mutator(this.document);
    if (JSON.stringify(before) === JSON.stringify(this.document)) return;
    this.undoStack.push({ label, document: before });
    if (this.undoStack.length > 100) this.undoStack.shift();
    this.redoStack = [];
    this.routePreviews.clear();
    this.renderAll();
    this.queueSave(this.clone(), before);
  }

  queueSave(documentToSave, rollback) {
    this.pendingSaves += 1;
    this.setSaveState("保存中…", "saving");
    this.saveQueue = this.saveQueue.then(async () => {
      const payload = await api(
        `/api/maps/${encodeURIComponent(this.mapName)}/editor`,
        "PUT",
        { expected_revision: this.revision, document: documentToSave },
      );
      this.revision = payload.document.revision;
      this.lastPersisted = this.clone(payload.document);
      if (this.sameContent(this.document, documentToSave)) this.document = payload.document;
      else this.document.revision = payload.document.revision;
    }).catch((error) => {
      this.document = this.lastPersisted ? this.clone(this.lastPersisted) : rollback;
      this.revision = Number(this.document?.revision || this.revision);
      this.undoStack = [];
      this.redoStack = [];
      this.setSaveState("保存失败", "error");
      reportActionError(error, "地图编辑保存失败");
      this.renderAll();
    }).finally(() => {
      this.pendingSaves -= 1;
      if (this.pendingSaves === 0) this.setSaveState("已保存", "saved");
    });
    return this.saveQueue;
  }

  sameContent(left, right) {
    const clean = (value) => {
      const copy = this.clone(value);
      if (copy) copy.revision = 0;
      return JSON.stringify(copy);
    };
    return clean(left) === clean(right);
  }

  async undo() {
    await this.saveQueue;
    const entry = this.undoStack.pop();
    if (!entry || this.readOnly) return;
    const current = this.clone();
    this.redoStack.push({ label: entry.label, document: current });
    this.document = entry.document;
    this.document.revision = this.revision;
    this.routePreviews.clear();
    this.renderAll();
    this.queueSave(this.clone(), current);
  }

  async redo() {
    await this.saveQueue;
    const entry = this.redoStack.pop();
    if (!entry || this.readOnly) return;
    const current = this.clone();
    this.undoStack.push({ label: entry.label, document: current });
    this.document = entry.document;
    this.document.revision = this.revision;
    this.routePreviews.clear();
    this.renderAll();
    this.queueSave(this.clone(), current);
  }

  setSaveState(text, kind = "") {
    const node = $("editorSaveState");
    node.textContent = text;
    node.dataset.kind = kind;
  }

  renderAll() {
    if (!this.active) return;
    this.renderTree();
    this.renderProperties();
    $("btnEditorUndo").disabled = this.readOnly || !this.undoStack.length;
    $("btnEditorRedo").disabled = this.readOnly || !this.redoStack.length;
    this.draw();
  }

  renderTree() {
    if (!this.document) return;
    const tree = $("editorObjectTree");
    const query = $("editorObjectSearch").value.trim().toLowerCase();
    const groups = [
      ["location", "地点", this.document.locations],
      ["keepout", "禁行区", this.document.keepouts],
      ["route", "巡航路线", this.document.routes],
    ];
    const total = groups.reduce((sum, group) => sum + group[2].length, 0);
    $("editorObjectCount").textContent = `${total} 个对象`;
    tree.innerHTML = groups.map(([type, label, allItems]) => {
      const items = allItems.filter((item) => item.name.toLowerCase().includes(query));
      return `<section class="editor-object-group"><div class="editor-object-label"><span>${label}</span><span>${items.length}${query ? ` / ${allItems.length}` : ""}</span></div>${items.length ? items.map((item) => this.objectRow(type, item)).join("") : '<p class="editor-empty-group">暂无</p>'}</section>`;
    }).join("");
    tree.querySelectorAll("[data-editor-object]").forEach((button) => {
      button.addEventListener("click", () => {
        this.setTool("select");
        this.setSelection(button.dataset.type, button.dataset.id);
      });
    });
    tree.querySelectorAll("[data-editor-visibility]").forEach((button) => {
      button.addEventListener("click", () => {
        const id = button.dataset.editorVisibility;
        this.hiddenObjects.has(id) ? this.hiddenObjects.delete(id) : this.hiddenObjects.add(id);
        this.renderTree();
        this.draw();
      });
    });
    tree.querySelectorAll("[data-editor-waypoint]").forEach((button) => {
      button.addEventListener("click", () => this.setSelection("route", button.dataset.routeId, button.dataset.editorWaypoint));
    });
    tree.querySelectorAll("[data-waypoint-move]").forEach((button) => {
      button.addEventListener("click", () => this.moveWaypoint(button.dataset.routeId, Number(button.dataset.index), Number(button.dataset.waypointMove)));
    });
  }

  objectRow(type, item) {
    const selected = this.selection?.type === type && this.selection.id === item.id;
    const hidden = this.hiddenObjects.has(item.id);
    const icon = type === "location" ? "pin" : type === "keepout" ? "square" : "line";
    const waypoints = type === "route" && selected ? `<div class="editor-waypoints">${item.waypoints.map((point, index) => `<div class="editor-waypoint-row ${this.selection?.childId === point.id ? "active" : ""}"><button data-editor-waypoint="${escapeHtml(point.id)}" data-route-id="${escapeHtml(item.id)}" type="button"><span>${index + 1}</span><small>${this.round(point.x)}, ${this.round(point.y)}</small></button><button data-waypoint-move="-1" data-route-id="${escapeHtml(item.id)}" data-index="${index}" type="button" aria-label="上移" ${index === 0 ? "disabled" : ""}>‹</button><button data-waypoint-move="1" data-route-id="${escapeHtml(item.id)}" data-index="${index}" type="button" aria-label="下移" ${index === item.waypoints.length - 1 ? "disabled" : ""}>›</button></div>`).join("")}</div>` : "";
    return `<div class="editor-object-wrap"><div class="editor-object-row ${selected ? "active" : ""}"><button data-editor-object data-type="${type}" data-id="${escapeHtml(item.id)}" type="button"><i class="editor-object-mark ${icon}"></i><span>${escapeHtml(item.name)}</span>${type === "route" ? `<small>${item.waypoints.length} 点</small>` : ""}</button><button class="editor-visibility ${hidden ? "off" : ""}" data-editor-visibility="${escapeHtml(item.id)}" type="button" aria-label="${hidden ? "显示" : "隐藏"}${escapeHtml(item.name)}"><svg viewBox="0 0 24 24" aria-hidden="true"><path d="M2 12s3.5-6 10-6 10 6 10 6-3.5 6-10 6S2 12 2 12Z"/><circle cx="12" cy="12" r="3"/>${hidden ? '<path d="m4 4 16 16"/>' : ""}</svg></button></div>${waypoints}</div>`;
  }

  renderProperties() {
    const panel = $("editorPropertyPanel");
    const object = this.selectedObject();
    const typeLabel = this.selection ? { location: "地点", keepout: "禁行区", route: "巡航路线" }[this.selection.type] : "未选择";
    $("editorSelectionType").textContent = typeLabel;
    if (!object) {
      panel.innerHTML = '<p class="editor-no-selection">未选择对象<br><span>选择地图中的对象以查看属性。</span></p>';
      return;
    }
    let html = `<div class="editor-property-type">${typeLabel}</div>${this.textField("名称", "name", object.name)}`;
    if (this.selection.type === "location") {
      html += `<div class="editor-field-group"><span>位置</span><div class="editor-field-row">${this.numberField("X", "x", object.x)}${this.numberField("Y", "y", object.y)}</div></div>${this.numberField("朝向", "yaw_deg", object.yaw_deg, "°")}<div class="editor-property-actions"><button data-editor-action="copy-location" type="button">复制地点</button></div>`;
    } else if (this.selection.type === "keepout") {
      html += `<div class="editor-field-group"><span>位置</span><div class="editor-field-row">${this.numberField("X", "center.x", object.center.x)}${this.numberField("Y", "center.y", object.center.y)}</div></div><div class="editor-field-group"><span>尺寸</span><div class="editor-field-row">${this.numberField("长度", "width_m", object.width_m, "m", 'min="0.01"')}${this.numberField("宽度", "height_m", object.height_m, "m", 'min="0.01"')}</div></div>${this.numberField("旋转", "yaw_deg", object.yaw_deg, "°")}`;
    } else {
      const preview = this.routePreviews.get(object.id);
      const status = !preview ? "尚未预览" : preview.status === "invalid" ? `发现 ${preview.collisions?.length || 0} 处冲突` : preview.status === "valid_with_fallbacks" ? `有效，${preview.fallback_waypoint_ids.length} 个转角未平滑` : "有效";
      const statusKind = preview?.status === "invalid" ? "error" : preview?.status?.startsWith("valid") ? "valid" : "";
      html += `<label class="editor-field"><span>路线形式</span><select data-editor-path="closed"><option value="false" ${object.closed ? "" : "selected"}>开放路线</option><option value="true" ${object.closed ? "selected" : ""}>闭环路线</option></select></label><div class="editor-property-row"><span>关键点</span><strong>${object.waypoints.length}</strong></div>${this.numberField("安全净空", "settings.clearance", this.document.settings.safety_clearance_m, "m", 'min="0" step="0.05"')}<div class="editor-route-status ${statusKind}"><span>路径状态</span><strong>${escapeHtml(status)}</strong></div><div class="editor-property-actions"><button data-editor-action="smooth-route" type="button">预览平滑路线</button>${this.selection.childId ? '<button data-editor-action="delete-waypoint" type="button">删除当前关键点</button>' : ""}</div>`;
    }
    html += `<div class="editor-danger-zone"><button data-editor-action="delete-object" type="button">删除${typeLabel}</button></div>`;
    panel.innerHTML = html;
    this.bindProperties();
  }

  textField(label, path, value) {
    return `<label class="editor-field"><span>${label}</span><input data-editor-path="${path}" value="${escapeHtml(value)}" maxlength="80" /></label>`;
  }

  numberField(label, path, value, suffix = "", extra = "") {
    return `<label class="editor-field"><span>${label}</span><div class="editor-number"><input data-editor-path="${path}" type="number" step="0.01" value="${this.round(value)}" ${extra}/>${suffix ? `<small>${suffix}</small>` : ""}</div></label>`;
  }

  bindProperties() {
    $("editorPropertyPanel").querySelectorAll("[data-editor-path]").forEach((input) => {
      input.disabled = this.readOnly;
      input.addEventListener("change", () => this.changeProperty(input));
    });
    $("editorPropertyPanel").querySelectorAll("[data-editor-action]").forEach((button) => {
      button.disabled = this.readOnly;
      button.addEventListener("click", () => this.propertyAction(button.dataset.editorAction));
    });
  }

  changeProperty(input) {
    const selection = { ...this.selection };
    let value = input.value;
    if (input.type === "number") value = Number(value);
    if (input.tagName === "SELECT") value = value === "true";
    if (input.dataset.editorPath === "closed" && value && this.selectedObject()?.waypoints.length < 3) {
      showToast("闭环路线至少需要三个关键点", "error");
      this.renderProperties();
      return;
    }
    this.commit("修改属性", (document) => {
      if (input.dataset.editorPath === "settings.clearance") {
        document.settings.safety_clearance_m = Math.max(0, value);
        document.settings.safety_clearance_source = "editor_override";
        return;
      }
      const object = this.objectBySelection(document, selection);
      if (!object) return;
      this.setPath(object, input.dataset.editorPath, value);
    });
  }

  propertyAction(action) {
    if (action === "delete-object") this.deleteSelection();
    else if (action === "copy-location") this.copyLocation();
    else if (action === "smooth-route") this.smoothRoute();
    else if (action === "delete-waypoint") this.deleteWaypoint();
  }

  setPath(target, path, value) {
    const parts = path.split(".");
    const leaf = parts.pop();
    const parent = parts.reduce((current, part) => current[part], target);
    parent[leaf] = value;
  }

  async deleteSelection() {
    if (!this.selection) return;
    const selection = { ...this.selection };
    await this.commit("删除对象", (document) => {
      const key = { location: "locations", keepout: "keepouts", route: "routes" }[selection.type];
      document[key] = document[key].filter((item) => item.id !== selection.id);
    });
    this.setSelection();
  }

  async copyLocation() {
    const source = this.selectedObject();
    if (!source || this.selection.type !== "location") return;
    const id = crypto.randomUUID();
    await this.commit("复制地点", (document) => {
      const names = new Set(document.locations.map((item) => item.name));
      let name = `${source.name} 副本`;
      let index = 2;
      while (names.has(name)) name = `${source.name} 副本 ${index++}`;
      document.locations.push({ ...this.clone(source), id, name });
    });
    this.setSelection("location", id);
  }

  async deleteWaypoint() {
    const selection = { ...this.selection };
    const route = this.selectedObject();
    const minimum = route?.closed ? 3 : 2;
    if (!route || route.waypoints.length <= minimum) {
      showToast(`该路线至少需要保留 ${minimum} 个关键点`, "error");
      return;
    }
    await this.commit("删除路线点", (document) => {
      const target = document.routes.find((item) => item.id === selection.id);
      target.waypoints = target.waypoints.filter((item) => item.id !== selection.childId);
    });
    this.setSelection("route", selection.id);
  }

  moveWaypoint(routeId, index, delta) {
    this.commit("调整关键点顺序", (document) => {
      const route = document.routes.find((item) => item.id === routeId);
      const target = index + delta;
      if (!route || target < 0 || target >= route.waypoints.length) return;
      const [waypoint] = route.waypoints.splice(index, 1);
      route.waypoints.splice(target, 0, waypoint);
    });
  }

  async smoothRoute() {
    const route = this.selectedObject();
    if (!route || this.selection.type !== "route") return;
    await this.saveQueue;
    this.setSaveState("正在计算…", "saving");
    try {
      const payload = await api(`/api/maps/${encodeURIComponent(this.mapName)}/routes/${encodeURIComponent(route.id)}/smooth`, "POST", { expected_revision: this.revision });
      this.routePreviews.set(route.id, payload.preview);
      this.setSaveState("已保存", "saved");
      this.renderProperties();
      this.draw();
    } catch (error) {
      this.setSaveState("计算失败", "error");
      reportActionError(error, "路线平滑失败");
    }
  }

  openExportDialog() {
    $("editorMoreMenu").classList.add("hidden");
    $("editorExportName").value = `${this.mapName}_edited`;
    $("editorExportError").textContent = "";
    $("editorExportDialog").classList.remove("hidden");
    $("editorExportDialog").setAttribute("aria-hidden", "false");
    $("editorExportName").focus();
    $("editorExportName").select();
  }

  closeExportDialog() {
    $("editorExportDialog").classList.add("hidden");
    $("editorExportDialog").setAttribute("aria-hidden", "true");
  }

  async exportMap(event) {
    event.preventDefault();
    const target = $("editorExportName").value.trim();
    const button = $("btnConfirmEditorExport");
    button.disabled = true;
    try {
      await this.saveQueue;
      await api(`/api/maps/${encodeURIComponent(this.mapName)}/export`, "POST", { target_name: target });
      this.closeExportDialog();
      await loadSavedMaps();
      showToast(`已导出衍生地图“${target}”`, "ok");
    } catch (error) {
      $("editorExportError").textContent = error.message;
      $("editorExportError").classList.add("error");
    } finally {
      button.disabled = false;
    }
  }

  pointerDown(event) {
    if (!this.active || event.button !== 0 || !this.document || this.readOnly) return;
    const world = clientToWorld(this.canvas, event.clientX, event.clientY);
    if (this.tool === "location" && world) {
      this.drag = { kind: "create-location", start: world, current: world };
    } else if (this.tool === "keepout" && world) {
      this.drag = { kind: "create-keepout", start: world, current: world };
    } else if (this.tool === "route" && world) {
      this.routeDraft.push(world);
      this.draw();
      return;
    } else {
      const hit = this.hitTest(event);
      if (!hit) {
        this.setSelection();
        startViewportGesture(this.canvas, event, "pan");
        return;
      }
      this.setSelection(hit.type, hit.id, hit.childId);
      const object = this.selectedObject();
      this.drag = {
        kind: hit.handle || "move",
        type: hit.type,
        id: hit.id,
        childId: hit.childId,
        start: world,
        before: this.clone(),
        original: this.clone(object),
      };
    }
    if (this.drag && this.canvas.setPointerCapture) this.canvas.setPointerCapture(event.pointerId);
  }

  pointerMove(event) {
    if (!this.active) return;
    const world = clientToWorld(this.canvas, event.clientX, event.clientY);
    if (world) {
      this.hoverWorld = world;
      $("editorCoordinates").textContent = `x ${this.round(world.x, 2)} · y ${this.round(world.y, 2)} m`;
    }
    if (!this.drag) {
      if (this.tool === "route") this.draw();
      return;
    }
    if (this.drag.kind.startsWith("create-")) {
      if (world) this.drag.current = world;
      this.draw();
      return;
    }
    if (!world) return;
    const object = this.selectedObject();
    if (!object) return;
    const dx = world.x - this.drag.start.x;
    const dy = world.y - this.drag.start.y;
    if (this.drag.type === "location") {
      if (this.drag.kind === "rotate") object.yaw_deg = this.angle(object, world);
      else { object.x = this.drag.original.x + dx; object.y = this.drag.original.y + dy; }
    } else if (this.drag.type === "keepout") {
      if (this.drag.kind === "rotate") object.yaw_deg = this.angle(object.center, world) - 90;
      else if (this.drag.kind === "resize") {
        const local = this.rotatePoint(world.x - object.center.x, world.y - object.center.y, -object.yaw_deg);
        object.width_m = Math.max(this.map.resolution, Math.abs(local.x) * 2);
        object.height_m = Math.max(this.map.resolution, Math.abs(local.y) * 2);
      } else { object.center.x = this.drag.original.center.x + dx; object.center.y = this.drag.original.center.y + dy; }
    } else if (this.drag.type === "route") {
      const waypoint = object.waypoints.find((item) => item.id === this.drag.childId);
      if (waypoint) { waypoint.x = world.x; waypoint.y = world.y; }
      else object.waypoints.forEach((item, index) => { item.x = this.drag.original.waypoints[index].x + dx; item.y = this.drag.original.waypoints[index].y + dy; });
    }
    this.draw();
  }

  async pointerUp() {
    if (!this.active || !this.drag) return;
    const drag = this.drag;
    this.drag = null;
    if (drag.kind === "create-location") {
      if (!this.isPointValid(drag.start)) {
        showToast("地点必须位于满足安全净空的空闲区域", "error");
        this.draw();
        return;
      }
      const id = crypto.randomUUID();
      await this.commit("创建地点", (document) => document.locations.push({ id, name: this.nextName(document.locations, "地点"), x: drag.start.x, y: drag.start.y, yaw_deg: this.angle(drag.start, drag.current) }));
      this.setTool("select");
      this.setSelection("location", id);
      this.selectNameField();
    } else if (drag.kind === "create-keepout") {
      const zone = this.keepoutFromDrag(drag);
      const id = crypto.randomUUID();
      await this.commit("创建禁行区", (document) => document.keepouts.push({ id, name: this.nextName(document.keepouts, "禁行区"), ...zone }));
      this.setTool("select");
      this.setSelection("keepout", id);
      this.selectNameField();
    } else {
      const after = this.clone();
      this.document = drag.before;
      await this.commit("拖动编辑", (document) => Object.assign(document, after));
    }
    this.draw();
  }

  cancelDrag() {
    if (this.drag?.before) this.document = this.drag.before;
    this.drag = null;
    this.draw();
  }

  async finishRoute() {
    if (this.routeDraft.length < 2) {
      showToast("巡航路线至少需要两个关键点", "error");
      return;
    }
    const points = this.routeDraft.slice();
    this.routeDraft = [];
    const id = crypto.randomUUID();
    await this.commit("创建巡航路线", (document) => document.routes.push({ id, name: this.nextName(document.routes, "巡航路线"), closed: false, waypoints: points.map((point) => ({ id: crypto.randomUUID(), x: point.x, y: point.y })) }));
    this.setTool("select");
    this.setSelection("route", id);
    this.selectNameField();
  }

  keyDown(event) {
    if (!this.active) return;
    const editing = /INPUT|SELECT|TEXTAREA/.test(document.activeElement?.tagName);
    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z") {
      event.preventDefault();
      event.shiftKey ? this.redo() : this.undo();
    } else if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "y") {
      event.preventDefault();
      this.redo();
    } else if (!editing && event.key === "Delete") {
      this.deleteSelection();
    } else if (!editing && event.key === "Enter" && this.tool === "route") {
      this.finishRoute();
    } else if (event.key === "Escape") {
      this.routeDraft = [];
      this.cancelDrag();
      this.setTool("select");
    }
  }

  hitTest(event) {
    const pointer = getCanvasPointer(this.canvas, event);
    if (!pointer) return null;
    const selected = this.selectedObject();
    if (selected && this.selection.type === "location") {
      const handle = this.locationHandle(selected);
      if (this.distance(pointer, handle) < 13) return { type: "location", id: selected.id, handle: "rotate" };
    }
    if (selected && this.selection.type === "keepout") {
      const handles = this.keepoutHandles(selected);
      if (this.distance(pointer, handles.rotate) < 13) return { type: "keepout", id: selected.id, handle: "rotate" };
      if (this.distance(pointer, handles.resize) < 13) return { type: "keepout", id: selected.id, handle: "resize" };
    }
    for (const route of [...this.document.routes].reverse()) {
      if (this.hiddenObjects.has(route.id)) continue;
      for (const waypoint of [...route.waypoints].reverse()) {
        if (this.distance(pointer, this.screen(waypoint)) < 13) return { type: "route", id: route.id, childId: waypoint.id, handle: "waypoint" };
      }
      for (let index = 0; index < route.waypoints.length - (route.closed ? 0 : 1); index += 1) {
        if (this.screenSegmentDistance(pointer, this.screen(route.waypoints[index]), this.screen(route.waypoints[(index + 1) % route.waypoints.length])) < 9) return { type: "route", id: route.id };
      }
    }
    for (const location of [...this.document.locations].reverse()) {
      if (!this.hiddenObjects.has(location.id) && this.distance(pointer, this.screen(location)) < 14) return { type: "location", id: location.id };
    }
    const world = clientToWorld(this.canvas, event.clientX, event.clientY);
    for (const zone of [...this.document.keepouts].reverse()) {
      if (!this.hiddenObjects.has(zone.id) && world && this.insideKeepout(world, zone)) return { type: "keepout", id: zone.id };
    }
    return null;
  }

  draw() {
    if (!this.active || !this.map) return;
    drawScene(this.canvas, this.map, null, { prefix: `editor-${this.map.source_sha256}` });
    const ctx = this.canvas.getContext("2d");
    this.drawKeepouts(ctx);
    this.drawRoutes(ctx);
    this.drawLocations(ctx);
    this.drawDraft(ctx);
  }

  drawKeepouts(ctx) {
    this.document?.keepouts.forEach((zone) => {
      if (this.hiddenObjects.has(zone.id)) return;
      const selected = this.selection?.type === "keepout" && this.selection.id === zone.id;
      this.path(ctx, this.keepoutVertices(zone), true);
      ctx.fillStyle = "rgba(185,90,82,.08)";
      ctx.strokeStyle = selected ? "#234e41" : "#b95a52";
      ctx.lineWidth = selected ? 3 : 2;
      ctx.fill(); ctx.stroke();
      if (selected) {
        const handles = this.keepoutHandles(zone);
        this.handle(ctx, handles.resize);
        this.handle(ctx, handles.rotate);
      }
    });
  }

  drawRoutes(ctx) {
    this.document?.routes.forEach((route) => {
      if (this.hiddenObjects.has(route.id)) return;
      const selected = this.selection?.type === "route" && this.selection.id === route.id;
      const preview = this.routePreviews.get(route.id);
      if (preview?.points?.length) {
        this.path(ctx, preview.points, false);
        ctx.strokeStyle = preview.status === "invalid" ? "#b94a42" : "#2f8568";
        ctx.lineWidth = selected ? 4 : 3;
        ctx.setLineDash([]); ctx.stroke();
      }
      this.path(ctx, route.waypoints, route.closed);
      ctx.strokeStyle = selected ? "#234e41" : "#5e8d7d";
      ctx.lineWidth = 2;
      ctx.setLineDash(preview?.points?.length ? [7, 6] : []); ctx.stroke(); ctx.setLineDash([]);
      route.waypoints.forEach((point, index) => {
        const screen = this.screen(point);
        ctx.beginPath(); ctx.arc(screen.x, screen.y, selected ? 7 : 5, 0, Math.PI * 2);
        ctx.fillStyle = this.selection?.childId === point.id ? "#234e41" : "#fff";
        ctx.strokeStyle = "#2f8568"; ctx.lineWidth = 2; ctx.fill(); ctx.stroke();
        if (selected) { ctx.fillStyle = this.selection?.childId === point.id ? "#fff" : "#234e41"; ctx.font = "10px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(String(index + 1), screen.x, screen.y); }
      });
    });
  }

  drawLocations(ctx) {
    this.document?.locations.forEach((location) => {
      if (this.hiddenObjects.has(location.id)) return;
      const selected = this.selection?.type === "location" && this.selection.id === location.id;
      const center = this.screen(location);
      const length = selected ? 24 : 20;
      const yaw = location.yaw_deg * Math.PI / 180;
      const tip = this.screen({ x: location.x + Math.cos(yaw) * length / this.canvas._view.scale, y: location.y + Math.sin(yaw) * length / this.canvas._view.scale });
      ctx.beginPath(); ctx.moveTo(center.x, center.y); ctx.lineTo(tip.x, tip.y);
      ctx.strokeStyle = selected ? "#234e41" : "#b95a52"; ctx.lineWidth = selected ? 3 : 2; ctx.stroke();
      ctx.beginPath(); ctx.arc(center.x, center.y, selected ? 7 : 5, 0, Math.PI * 2);
      ctx.fillStyle = selected ? "#234e41" : "#c96c62"; ctx.fill();
      ctx.font = `${12 * Math.min(window.devicePixelRatio || 1, 2)}px sans-serif`; ctx.fillStyle = "#25312b"; ctx.textAlign = "left"; ctx.textBaseline = "bottom"; ctx.fillText(location.name, center.x + 9, center.y - 7);
      if (selected) this.handle(ctx, this.locationHandle(location));
    });
  }

  drawDraft(ctx) {
    if (this.routeDraft.length) {
      const points = this.hoverWorld ? [...this.routeDraft, this.hoverWorld] : this.routeDraft;
      this.path(ctx, points);
      ctx.strokeStyle = "#2f8568"; ctx.lineWidth = 2; ctx.setLineDash([6, 5]); ctx.stroke(); ctx.setLineDash([]);
      this.routeDraft.forEach((point) => this.handle(ctx, this.screen(point), 5));
    }
    if (this.drag?.kind === "create-location") {
      const start = this.screen(this.drag.start); const end = this.screen(this.drag.current);
      ctx.beginPath(); ctx.moveTo(start.x, start.y); ctx.lineTo(end.x, end.y); ctx.strokeStyle = "#b95a52"; ctx.lineWidth = 2; ctx.stroke(); this.handle(ctx, start);
    }
    if (this.drag?.kind === "create-keepout") {
      this.path(ctx, this.keepoutVertices(this.keepoutFromDrag(this.drag)), true);
      ctx.fillStyle = "rgba(185,90,82,.08)"; ctx.strokeStyle = "#b95a52"; ctx.fill(); ctx.stroke();
    }
  }

  path(ctx, points, close = false) {
    if (!points?.length) return;
    ctx.beginPath();
    points.forEach((point, index) => { const screen = this.screen(point); index ? ctx.lineTo(screen.x, screen.y) : ctx.moveTo(screen.x, screen.y); });
    if (close) ctx.closePath();
  }

  handle(ctx, point, radius = 6) {
    ctx.beginPath(); ctx.arc(point.x, point.y, radius, 0, Math.PI * 2); ctx.fillStyle = "#fff"; ctx.strokeStyle = "#234e41"; ctx.lineWidth = 2; ctx.fill(); ctx.stroke();
  }

  screen(point) { return worldToScreen(this.canvas._view, this.canvas, point.x, point.y); }
  distance(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
  angle(a, b) { return Math.atan2(b.y - a.y, b.x - a.x) * 180 / Math.PI; }
  round(value, digits = 3) { return Number(Number(value).toFixed(digits)); }
  rotatePoint(x, y, degrees) { const angle = degrees * Math.PI / 180; return { x: Math.cos(angle) * x - Math.sin(angle) * y, y: Math.sin(angle) * x + Math.cos(angle) * y }; }
  nextName(items, prefix) { const names = new Set(items.map((item) => item.name)); let index = items.length + 1; while (names.has(`${prefix} ${index}`)) index += 1; return `${prefix} ${index}`; }
  selectNameField() { window.setTimeout(() => $("editorPropertyPanel").querySelector('[data-editor-path="name"]')?.select(), 0); }

  screenSegmentDistance(point, start, end) {
    const dx = end.x - start.x; const dy = end.y - start.y; const length = dx * dx + dy * dy || 1;
    const t = Math.max(0, Math.min(1, ((point.x - start.x) * dx + (point.y - start.y) * dy) / length));
    return Math.hypot(point.x - start.x - t * dx, point.y - start.y - t * dy);
  }

  insideKeepout(point, zone) {
    const local = this.rotatePoint(point.x - zone.center.x, point.y - zone.center.y, -zone.yaw_deg);
    return Math.abs(local.x) <= zone.width_m / 2 && Math.abs(local.y) <= zone.height_m / 2;
  }

  keepoutVertices(zone) {
    const halfWidth = zone.width_m / 2; const halfHeight = zone.height_m / 2;
    return [[-halfWidth, -halfHeight], [halfWidth, -halfHeight], [halfWidth, halfHeight], [-halfWidth, halfHeight]].map(([x, y]) => { const point = this.rotatePoint(x, y, zone.yaw_deg); return { x: zone.center.x + point.x, y: zone.center.y + point.y }; });
  }

  keepoutHandles(zone) {
    const vertices = this.keepoutVertices(zone);
    const top = this.rotatePoint(0, zone.height_m / 2 + 24 / this.canvas._view.scale, zone.yaw_deg);
    return { resize: this.screen(vertices[2]), rotate: this.screen({ x: zone.center.x + top.x, y: zone.center.y + top.y }) };
  }

  locationHandle(location) {
    const distance = 30 / this.canvas._view.scale;
    const angle = location.yaw_deg * Math.PI / 180;
    return this.screen({ x: location.x + Math.cos(angle) * distance, y: location.y + Math.sin(angle) * distance });
  }

  keepoutFromDrag(drag) {
    return { center: { x: (drag.start.x + drag.current.x) / 2, y: (drag.start.y + drag.current.y) / 2 }, width_m: Math.max(this.map.resolution, Math.abs(drag.current.x - drag.start.x)), height_m: Math.max(this.map.resolution, Math.abs(drag.current.y - drag.start.y)), yaw_deg: 0 };
  }

  isPointValid(point) {
    const view = this.canvas._view;
    if (!view) return false;
    const yaw = Number(this.map.origin.yaw_deg || 0) * Math.PI / 180;
    const dx = point.x - this.map.origin.x;
    const dy = point.y - this.map.origin.y;
    const localX = Math.cos(yaw) * dx + Math.sin(yaw) * dy;
    const localY = -Math.sin(yaw) * dx + Math.cos(yaw) * dy;
    const px = localX / this.map.resolution;
    const py = this.map.height - 1 - localY / this.map.resolution;
    const ix = Math.floor(px); const iy = Math.ceil(py);
    if (ix <= 0 || iy <= 0 || ix >= this.map.width - 1 || iy >= this.map.height - 1) return false;
    const radius = Number(this.document.settings.safety_clearance_m || 0) / this.map.resolution;
    const reach = Math.ceil(radius);
    for (let row = Math.max(0, iy - reach); row <= Math.min(this.map.height - 1, iy + reach); row += 1) {
      for (let column = Math.max(0, ix - reach); column <= Math.min(this.map.width - 1, ix + reach); column += 1) {
        if (this.map.occupancy[row * this.map.width + column] !== 0 && Math.hypot(column - ix, row - iy) < Math.max(radius, 1e-6)) return false;
      }
    }
    return !this.document.keepouts.some((zone) => this.insideKeepout(point, { ...zone, width_m: zone.width_m + 2 * radius * this.map.resolution, height_m: zone.height_m + 2 * radius * this.map.resolution }));
  }
}

window.finavMapEditor = new FinavMapEditor($("previewCanvas"));
