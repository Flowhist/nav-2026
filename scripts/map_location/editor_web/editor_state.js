"use strict";

class EditorState extends EventTarget {
  constructor() {
    super();
    this.mapName = "";
    this.map = null;
    this.document = null;
    this.expected_revision = 0;
    this.undoStack = [];
    this.redoStack = [];
    this.saveQueue = Promise.resolve();
    this.pendingSaves = 0;
    this.saveEpoch = 0;
    this.lastSaveFailed = false;
    this.lastPersistedDocument = null;
    this.busy = false;
    this.exclusivePromise = Promise.resolve();
    this.releaseExclusive = null;
    this.readOnly = false;
    this.selection = null;
    this.routePreviews = new Map();
  }

  snapshot() {
    return JSON.parse(JSON.stringify(this.document));
  }

  emit(type = "change", detail = {}) {
    this.dispatchEvent(new CustomEvent(type, { detail }));
  }

  async request(path, options = {}) {
    const response = await fetch(path, {
      headers: { "Content-Type": "application/json" },
      ...options,
    });
    const payload = await response.json();
    if (!response.ok) {
      const error = new Error(payload.error?.message || `HTTP ${response.status}`);
      error.code = payload.error?.code;
      throw error;
    }
    return payload;
  }

  async load(mapName) {
    await this.exclusivePromise;
    await this.saveQueue;
    const payload = await this.request(`/api/maps/${encodeURIComponent(mapName)}/editor`);
    const sourceChanged = Boolean(payload.source_changed);
    const accepted = !sourceChanged || window.confirm(
      "检测到地图栅格已变化，现有平滑路线将失效。是否确认按新地图继续编辑？",
    );
    this.mapName = mapName;
    this.map = payload.map;
    this.document = payload.document;
    this.expected_revision = payload.document.revision;
    this.readOnly = Boolean(payload.read_only) || !accepted;
    this.readOnlyReason = payload.read_only_reason || (
      accepted ? "" : "尚未确认重新载入已变化的地图"
    );
    this.lastPersistedDocument = this.snapshot();
    this.undoStack = [];
    this.redoStack = [];
    this.selection = null;
    this.routePreviews.clear();
    this.emit("loaded");
  }

  setSelection(type, id, childId = null) {
    this.selection = id ? { type, id, childId } : null;
    this.emit("selection");
  }

  findSelected() {
    if (!this.selection || !this.document) return null;
    const key = { location: "locations", keepout: "keepouts", route: "routes" }[this.selection.type];
    return this.document[key]?.find(item => item.id === this.selection.id) || null;
  }

  clearRoutePreview(routeId = null) {
    if (routeId) this.routePreviews.delete(routeId);
    else this.routePreviews.clear();
  }

  sameContent(left, right) {
    const normalize = value => {
      const copy = JSON.parse(JSON.stringify(value));
      copy.revision = 0;
      return JSON.stringify(copy);
    };
    return normalize(left) === normalize(right);
  }

  async persist(documentToSave) {
    const requestRevision = this.expected_revision;
    const body = { expected_revision: requestRevision, document: documentToSave };
    const payload = await this.request(
      `/api/maps/${encodeURIComponent(this.mapName)}/editor`,
      { method: "PUT", body: JSON.stringify(body) },
    );
    this.expected_revision = payload.document.revision;
    this.lastPersistedDocument = JSON.parse(JSON.stringify(payload.document));
    this.lastSaveFailed = false;
    if (this.sameContent(this.document, documentToSave)) {
      this.document = payload.document;
    } else {
      this.document.revision = payload.document.revision;
    }
  }

  queuePersist(documentToSave, rollback) {
    const epoch = this.saveEpoch;
    this.pendingSaves += 1;
    this.emit("saving");
    let failed = false;
    this.saveQueue = this.saveQueue.then(() => {
      if (epoch !== this.saveEpoch) return;
      return this.persist(documentToSave);
    }).catch(error => {
      failed = true;
      this.lastSaveFailed = true;
      this.saveEpoch += 1;
      if (this.lastPersistedDocument) {
        this.document = JSON.parse(JSON.stringify(this.lastPersistedDocument));
        this.expected_revision = this.document.revision;
      } else if (rollback) {
        this.document = rollback;
      }
      this.undoStack = [];
      this.redoStack = [];
      this.emit("change");
      if (error.code === "revision_conflict") {
        this.emit("conflict", { error });
      }
      this.emit("saveerror", { error });
    }).finally(() => {
      this.pendingSaves -= 1;
      if (this.pendingSaves === 0 && !failed && !this.lastSaveFailed) this.emit("saved");
    });
    return this.saveQueue;
  }

  async commit(label, mutator) {
    await this.exclusivePromise;
    if (this.readOnly) {
      this.emit("readonly", { message: this.readOnlyReason });
      return;
    }
    if (!this.document) return;
    const before = this.snapshot();
    mutator(this.document);
    if (JSON.stringify(before) === JSON.stringify(this.document)) return;
    this.undoStack.push({ label, document: before });
    if (this.undoStack.length > 100) this.undoStack.shift();
    this.redoStack = [];
    this.emit("change");
    await this.queuePersist(this.snapshot(), before);
  }

  async undo() {
    await this.exclusivePromise;
    if (this.readOnly) return;
    const entry = this.undoStack.pop();
    if (!entry) return;
    const current = this.snapshot();
    this.redoStack.push({ label: entry.label, document: current });
    this.document = entry.document;
    this.document.revision = this.expected_revision;
    this.clearRoutePreview();
    this.emit("change");
    await this.queuePersist(this.snapshot(), current);
  }

  async redo() {
    await this.exclusivePromise;
    if (this.readOnly) return;
    const entry = this.redoStack.pop();
    if (!entry) return;
    const current = this.snapshot();
    this.undoStack.push({ label: entry.label, document: current });
    this.document = entry.document;
    this.document.revision = this.expected_revision;
    this.clearRoutePreview();
    this.emit("change");
    await this.queuePersist(this.snapshot(), current);
  }

  async smoothRoute(routeId) {
    if (this.busy || this.readOnly) return;
    this.busy = true;
    this.exclusivePromise = new Promise(resolve => { this.releaseExclusive = resolve; });
    this.emit("saving");
    try {
      await this.saveQueue;
      const payload = await this.request(
        `/api/maps/${encodeURIComponent(this.mapName)}/routes/${encodeURIComponent(routeId)}/smooth`,
        {
          method: "POST",
          body: JSON.stringify({ expected_revision: this.expected_revision }),
        },
      );
      this.routePreviews.set(routeId, payload.preview);
      this.emit("saved");
      this.emit("change");
    } catch (error) {
      if (error.code === "revision_conflict") {
        this.emit("conflict", { error });
      }
      this.emit("saveerror", { error });
    } finally {
      this.busy = false;
      this.releaseExclusive?.();
      this.releaseExclusive = null;
      this.exclusivePromise = Promise.resolve();
    }
  }

  async exportMap(targetName) {
    if (this.busy || this.readOnly) throw new Error("当前状态不可导出");
    this.busy = true;
    this.exclusivePromise = new Promise(resolve => { this.releaseExclusive = resolve; });
    try {
      await this.saveQueue;
      return await this.request(`/api/maps/${encodeURIComponent(this.mapName)}/export`, {
        method: "POST",
        body: JSON.stringify({ target_name: targetName }),
      });
    } finally {
      this.busy = false;
      this.releaseExclusive?.();
      this.releaseExclusive = null;
      this.exclusivePromise = Promise.resolve();
    }
  }
}

window.editorState = new EditorState();
