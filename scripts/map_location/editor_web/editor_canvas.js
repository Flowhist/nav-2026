"use strict";

const TOOL_HINTS = {
  select: "单击选择；拖拽对象或控制点。空白处拖拽平移，滚轮缩放。",
  location: "按下设置目标点，拖动确定朝向，松开完成。",
  keepout: "从一个角拖到对角拉出矩形；创建后再用旋转手柄调整角度。",
  route: "依次单击添加关键点；双击或 Enter 完成，Esc 取消。",
};

function nextObjectName(items, prefix) {
  const names = new Set(items.map(item => item.name));
  let index = items.length + 1;
  while (names.has(`${prefix} ${index}`)) index += 1;
  return `${prefix} ${index}`;
}

class MapEditorCanvas {
  constructor(canvas, state) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.state = state;
    this.tool = "select";
    this.scale = 1;
    this.offset = { x: 0, y: 0 };
    this.drag = null;
    this.hoverMap = null;
    this.spaceDown = false;
    this.routeDraft = [];
    this.hidden = new Set();
    this.background = document.createElement("canvas");
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas);
    this.bind();
    state.addEventListener("loaded", () => {
      this.routeDraft = [];
      this.drag = null;
      this.hidden.clear();
      this.buildBackground();
      requestAnimationFrame(() => this.fit());
    });
    state.addEventListener("change", () => this.draw());
    state.addEventListener("selection", () => this.draw());
  }

  bind() {
    this.canvas.addEventListener("pointerdown", event => this.pointerDown(event));
    this.canvas.addEventListener("pointermove", event => this.pointerMove(event));
    this.canvas.addEventListener("pointerup", event => this.pointerUp(event));
    this.canvas.addEventListener("pointercancel", () => this.cancelDrag());
    this.canvas.addEventListener("dblclick", event => this.doubleClick(event));
    this.canvas.addEventListener("wheel", event => this.wheel(event), { passive: false });
    this.canvas.addEventListener("contextmenu", event => event.preventDefault());
    window.addEventListener("keydown", event => {
      if (event.code === "Space" && !/INPUT|SELECT|TEXTAREA/.test(document.activeElement?.tagName)) {
        this.spaceDown = true;
        this.canvas.style.cursor = "grab";
        event.preventDefault();
      }
    });
    window.addEventListener("keyup", event => {
      if (event.code === "Space") {
        this.spaceDown = false;
        this.canvas.style.cursor = this.tool === "select" ? "default" : "crosshair";
      }
    });
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    this.canvas.width = Math.max(1, Math.round(rect.width * ratio));
    this.canvas.height = Math.max(1, Math.round(rect.height * ratio));
    this.ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.draw();
  }

  buildBackground() {
    const map = this.state.map;
    if (!map) return;
    this.background.width = map.width;
    this.background.height = map.height;
    const context = this.background.getContext("2d");
    const image = context.createImageData(map.width, map.height);
    for (let index = 0; index < map.pixels.length; index += 1) {
      const shade = map.pixels[index];
      const offset = index * 4;
      image.data[offset] = shade;
      image.data[offset + 1] = shade;
      image.data[offset + 2] = shade;
      image.data[offset + 3] = 255;
    }
    context.putImageData(image, 0, 0);
  }

  setTool(tool) {
    if (this.routeDraft.length) this.routeDraft = [];
    this.tool = tool;
    this.drag = null;
    this.canvas.style.cursor = tool === "select" ? "default" : "crosshair";
    document.getElementById("toolHint").textContent = TOOL_HINTS[tool];
    document.querySelectorAll("[data-tool]").forEach(button => {
      button.classList.toggle("active", button.dataset.tool === tool);
    });
    this.draw();
  }

  fit() {
    const map = this.state.map;
    if (!map) return;
    const rect = this.canvas.getBoundingClientRect();
    this.scale = Math.max(0.05, Math.min(
      (rect.width - 56) / map.width,
      (rect.height - 56) / map.height,
    ));
    this.offset.x = (rect.width - map.width * this.scale) / 2;
    this.offset.y = (rect.height - map.height * this.scale) / 2;
    this.updateZoom();
    this.draw();
  }

  zoomAt(factor, screen = null) {
    if (!this.state.map) return;
    const rect = this.canvas.getBoundingClientRect();
    const anchor = screen || { x: rect.width / 2, y: rect.height / 2 };
    const px = (anchor.x - this.offset.x) / this.scale;
    const py = (anchor.y - this.offset.y) / this.scale;
    this.scale = Math.min(80, Math.max(0.05, this.scale * factor));
    this.offset.x = anchor.x - px * this.scale;
    this.offset.y = anchor.y - py * this.scale;
    this.updateZoom();
    this.draw();
  }

  centerOnObject(type, object) {
    let point = object;
    if (type === "keepout") point = object.center;
    if (type === "route") {
      point = object.waypoints.reduce(
        (sum, item) => ({ x: sum.x + item.x, y: sum.y + item.y }),
        { x: 0, y: 0 },
      );
      point.x /= object.waypoints.length;
      point.y /= object.waypoints.length;
    }
    const pixel = this.mapToPixel(point);
    const rect = this.canvas.getBoundingClientRect();
    this.offset.x = rect.width / 2 - pixel.x * this.scale;
    this.offset.y = rect.height / 2 - pixel.y * this.scale;
    this.draw();
  }

  toggleVisibility(id) {
    this.hidden.has(id) ? this.hidden.delete(id) : this.hidden.add(id);
    this.draw();
  }

  updateZoom() {
    document.getElementById("zoomLabel").textContent = `${Math.round(this.scale * 100)}%`;
  }

  eventPoint(event) {
    const rect = this.canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  screenToPixel(screen) {
    return {
      x: (screen.x - this.offset.x) / this.scale,
      y: (screen.y - this.offset.y) / this.scale,
    };
  }

  pixelToScreen(pixel) {
    return {
      x: this.offset.x + pixel.x * this.scale,
      y: this.offset.y + pixel.y * this.scale,
    };
  }

  pixelToMap(pixel) {
    const map = this.state.map;
    const localX = pixel.x * map.resolution;
    const localY = (map.height - 1 - pixel.y) * map.resolution;
    const yaw = map.origin.yaw_deg * Math.PI / 180;
    return {
      x: map.origin.x + Math.cos(yaw) * localX - Math.sin(yaw) * localY,
      y: map.origin.y + Math.sin(yaw) * localX + Math.cos(yaw) * localY,
    };
  }

  mapToPixel(point) {
    const map = this.state.map;
    const yaw = map.origin.yaw_deg * Math.PI / 180;
    const dx = point.x - map.origin.x;
    const dy = point.y - map.origin.y;
    const localX = Math.cos(yaw) * dx + Math.sin(yaw) * dy;
    const localY = -Math.sin(yaw) * dx + Math.cos(yaw) * dy;
    return {
      x: localX / map.resolution,
      y: map.height - 1 - localY / map.resolution,
    };
  }

  mapToScreen(point) {
    return this.pixelToScreen(this.mapToPixel(point));
  }

  pointOnMap(screen) {
    const pixel = this.screenToPixel(screen);
    const map = this.state.map;
    if (!map || pixel.x < 0 || pixel.y < 0 || pixel.x >= map.width || pixel.y >= map.height) return null;
    return this.pixelToMap(pixel);
  }

  screenToMapUnclipped(screen) {
    return this.pixelToMap(this.screenToPixel(screen));
  }

  wheel(event) {
    event.preventDefault();
    this.zoomAt(event.deltaY < 0 ? 1.12 : 1 / 1.12, this.eventPoint(event));
  }

  pointerDown(event) {
    if (!this.state.document || event.button > 1) return;
    if (this.state.busy || this.state.readOnly) {
      window.showToast?.(
        this.state.readOnly ? this.state.readOnlyReason : "正在完成上一项操作，请稍候",
        true,
      );
      return;
    }
    const screen = this.eventPoint(event);
    const point = this.pointOnMap(screen);
    this.canvas.setPointerCapture(event.pointerId);
    if (event.button === 1 || event.altKey || this.spaceDown) {
      this.drag = { kind: "pan", screen, offset: { ...this.offset } };
      return;
    }
    if (this.tool === "location" && point) {
      this.drag = { kind: "create-location", start: point, current: point };
      return;
    }
    if (this.tool === "keepout" && point) {
      this.drag = { kind: "create-keepout", start: point, current: point };
      return;
    }
    if (this.tool === "route" && point) {
      this.routeDraft.push(point);
      this.draw();
      return;
    }
    const hit = this.hitTest(screen);
    if (hit) {
      this.state.setSelection(hit.type, hit.id, hit.childId || null);
      const object = this.state.findSelected();
      const dragStart = hit.handle ? this.screenToMapUnclipped(screen) : point;
      if (!dragStart) return;
      this.drag = {
        kind: hit.handle || "move",
        type: hit.type,
        id: hit.id,
        childId: hit.childId,
        start: dragStart,
        before: this.state.snapshot(),
        original: JSON.parse(JSON.stringify(object)),
      };
    } else {
      this.state.setSelection(null);
      this.drag = { kind: "pan", screen, offset: { ...this.offset } };
    }
  }

  pointerMove(event) {
    const screen = this.eventPoint(event);
    const point = this.pointOnMap(screen);
    if (point) {
      this.hoverMap = point;
      const pixel = this.mapToPixel(point);
      document.getElementById("coordinates").textContent =
        `px ${pixel.x.toFixed(1)}, ${pixel.y.toFixed(1)} · x ${point.x.toFixed(3)} · y ${point.y.toFixed(3)}`;
    }
    if (!this.drag) {
      this.draw();
      return;
    }
    if (this.drag.kind === "pan") {
      this.offset.x = this.drag.offset.x + screen.x - this.drag.screen.x;
      this.offset.y = this.drag.offset.y + screen.y - this.drag.screen.y;
      this.draw();
      return;
    }
    if (this.drag.kind.startsWith("create-")) {
      if (point) this.drag.current = point;
      this.draw();
      return;
    }
    const editPoint = (
      this.drag.kind === "rotate" || this.drag.kind === "resize"
        ? this.screenToMapUnclipped(screen)
        : point
    );
    if (!editPoint) return;
    const object = this.state.findSelected();
    if (!object) return;
    const dx = editPoint.x - this.drag.start.x;
    const dy = editPoint.y - this.drag.start.y;
    if (this.drag.type === "location") {
      if (this.drag.kind === "rotate") {
        object.yaw_deg = this.angleDeg(object, editPoint);
      } else {
        object.x = this.drag.original.x + dx;
        object.y = this.drag.original.y + dy;
      }
    } else if (this.drag.type === "keepout") {
      this.state.clearRoutePreview();
      if (this.drag.kind === "rotate") {
        const yaw = this.angleDeg(object.center, editPoint) - 90;
        object.yaw_deg = event.shiftKey ? Math.round(yaw / 15) * 15 : yaw;
      } else if (this.drag.kind === "resize") {
        const angle = -object.yaw_deg * Math.PI / 180;
        const relativeX = editPoint.x - object.center.x;
        const relativeY = editPoint.y - object.center.y;
        const localX = Math.cos(angle) * relativeX - Math.sin(angle) * relativeY;
        const localY = Math.sin(angle) * relativeX + Math.cos(angle) * relativeY;
        object.width_m = Math.max(this.state.map.resolution, 2 * Math.abs(localX));
        object.height_m = Math.max(this.state.map.resolution, 2 * Math.abs(localY));
      } else {
        object.center.x = this.drag.original.center.x + dx;
        object.center.y = this.drag.original.center.y + dy;
      }
    } else if (this.drag.type === "route") {
      this.state.clearRoutePreview(object.id);
      const waypoint = object.waypoints.find(item => item.id === this.drag.childId);
      if (waypoint) {
        waypoint.x = editPoint.x;
        waypoint.y = editPoint.y;
      } else {
        object.waypoints.forEach((item, index) => {
          item.x = this.drag.original.waypoints[index].x + dx;
          item.y = this.drag.original.waypoints[index].y + dy;
        });
      }
    }
    this.state.emit("change");
  }

  async pointerUp(event) {
    if (!this.drag) return;
    const drag = this.drag;
    this.drag = null;
    if (drag.kind === "create-location") {
      const start = drag.start;
      const current = drag.current;
      if (!this.isPointValid(start)) {
        window.showToast?.("目标点必须位于满足安全净空的空闲区域", true);
        this.draw();
        return;
      }
      const yaw = this.angleDeg(start, current);
      const id = crypto.randomUUID();
      await this.state.commit("创建目标点", document => {
        document.locations.push({
          id, name: nextObjectName(document.locations, "目标点"),
          x: start.x, y: start.y, yaw_deg: yaw,
        });
      });
      this.state.setSelection("location", id);
      this.setTool("select");
      document.querySelector("#propertyPanel [data-path='name']")?.select();
    } else if (drag.kind === "create-keepout") {
      const preview = this.keepoutFromDrag(drag);
      const id = crypto.randomUUID();
      this.state.clearRoutePreview();
      await this.state.commit("创建禁行区", document => {
        document.keepouts.push({
          id, name: nextObjectName(document.keepouts, "禁行区"),
          center: preview.center,
          width_m: preview.width_m,
          height_m: preview.height_m,
          yaw_deg: 0,
        });
      });
      this.state.setSelection("keepout", id);
      this.setTool("select");
      document.querySelector("#propertyPanel [data-path='name']")?.select();
    } else if (!["pan"].includes(drag.kind)) {
      const edited = this.state.findSelected();
      if (
        drag.type === "location"
        && edited
        && !this.isPointValid({ x: edited.x, y: edited.y })
      ) {
        this.state.document = drag.before;
        this.state.emit("change");
        window.showToast?.("目标点必须位于满足安全净空的空闲区域", true);
        return;
      }
      const after = this.state.snapshot();
      this.state.document = drag.before;
      await this.state.commit("拖拽编辑", document => Object.assign(document, after));
    }
    this.draw();
  }

  cancelDrag() {
    if (this.drag?.before) this.state.document = this.drag.before;
    this.drag = null;
    this.draw();
  }

  doubleClick(event) {
    if (this.tool === "route") {
      event.preventDefault();
      const last = this.routeDraft.at(-1);
      const previous = this.routeDraft.at(-2);
      const lastScreen = last ? this.mapToScreen(last) : null;
      const previousScreen = previous ? this.mapToScreen(previous) : null;
      if (
        lastScreen && previousScreen
        && Math.hypot(
          lastScreen.x - previousScreen.x,
          lastScreen.y - previousScreen.y,
        ) < 8
      ) {
        this.routeDraft.pop();
      }
      this.finishRoute();
      return;
    }
    if (this.tool !== "select") return;
    const point = this.pointOnMap(this.eventPoint(event));
    const selected = this.state.findSelected();
    if (!point || this.state.selection?.type !== "route" || !selected) return;
    let best = { index: -1, distance: Infinity };
    const count = selected.closed ? selected.waypoints.length : selected.waypoints.length - 1;
    for (let i = 0; i < count; i += 1) {
      const a = selected.waypoints[i];
      const b = selected.waypoints[(i + 1) % selected.waypoints.length];
      const distance = this.segmentDistance(point, a, b);
      if (distance < best.distance) best = { index: i, distance };
    }
    if (best.distance < 14 / this.scale * this.state.map.resolution) {
      this.state.commit("插入路线点", document => {
        const route = document.routes.find(item => item.id === selected.id);
        route.waypoints.splice(best.index + 1, 0, {
          id: crypto.randomUUID(), x: point.x, y: point.y,
        });
      });
      this.state.clearRoutePreview(selected.id);
    }
  }

  async finishRoute() {
    if (this.routeDraft.length < 2) {
      window.showToast?.("路线至少需要两个关键点", true);
      return;
    }
    const draft = this.routeDraft.slice();
    this.routeDraft = [];
    const id = crypto.randomUUID();
    await this.state.commit("创建巡航路线", document => {
      document.routes.push({
        id, name: nextObjectName(document.routes, "巡航路线"), closed: false,
        waypoints: draft.map(point => ({ id: crypto.randomUUID(), x: point.x, y: point.y })),
      });
    });
    this.state.setSelection("route", id);
    this.setTool("select");
    document.querySelector("#propertyPanel [data-path='name']")?.select();
  }

  angleDeg(origin, point) {
    return Math.atan2(point.y - origin.y, point.x - origin.x) * 180 / Math.PI;
  }

  keepoutFromDrag(drag) {
    return {
      center: {
        x: (drag.start.x + drag.current.x) / 2,
        y: (drag.start.y + drag.current.y) / 2,
      },
      width_m: Math.max(
        this.state.map.resolution,
        Math.abs(drag.current.x - drag.start.x),
      ),
      height_m: Math.max(
        this.state.map.resolution,
        Math.abs(drag.current.y - drag.start.y),
      ),
      yaw_deg: 0,
    };
  }

  segmentDistance(point, a, b) {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const length2 = dx * dx + dy * dy || 1;
    const t = Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2));
    return Math.hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy));
  }

  hitTest(screen) {
    const threshold = 11;
    const document = this.state.document;
    const selected = this.state.findSelected();
    if (selected && this.state.selection.type === "keepout") {
      const handles = this.keepoutHandles(selected);
      if (Math.hypot(screen.x - handles.rotate.x, screen.y - handles.rotate.y) < threshold)
        return { type: "keepout", id: selected.id, handle: "rotate" };
      if (Math.hypot(screen.x - handles.resize.x, screen.y - handles.resize.y) < threshold)
        return { type: "keepout", id: selected.id, handle: "resize" };
    }
    if (selected && this.state.selection.type === "location") {
      const rotate = this.locationRotateHandle(selected);
      if (Math.hypot(screen.x - rotate.x, screen.y - rotate.y) < threshold)
        return { type: "location", id: selected.id, handle: "rotate" };
    }
    for (const route of [...document.routes].reverse()) {
      if (this.hidden.has(route.id)) continue;
      for (const waypoint of [...route.waypoints].reverse()) {
        const p = this.mapToScreen(waypoint);
        if (Math.hypot(screen.x - p.x, screen.y - p.y) < threshold)
          return { type: "route", id: route.id, childId: waypoint.id, handle: "waypoint" };
      }
    }
    const mapPoint = this.pointOnMap(screen);
    if (mapPoint) {
      for (const route of [...document.routes].reverse()) {
        if (this.hidden.has(route.id)) continue;
        const segmentCount = route.closed ? route.waypoints.length : route.waypoints.length - 1;
        for (let index = 0; index < segmentCount; index += 1) {
          const distance = this.segmentDistance(
            mapPoint,
            route.waypoints[index],
            route.waypoints[(index + 1) % route.waypoints.length],
          );
          if (distance < 8 / this.scale * this.state.map.resolution)
            return { type: "route", id: route.id };
        }
      }
    }
    for (const location of [...document.locations].reverse()) {
      if (this.hidden.has(location.id)) continue;
      const p = this.mapToScreen(location);
      if (Math.hypot(screen.x - p.x, screen.y - p.y) < threshold)
        return { type: "location", id: location.id };
    }
    for (const keepout of [...document.keepouts].reverse()) {
      if (this.hidden.has(keepout.id)) continue;
      const p = this.pointOnMap(screen);
      if (p && this.insideKeepout(p, keepout)) return { type: "keepout", id: keepout.id };
    }
    return null;
  }

  insideKeepout(point, zone) {
    const angle = -zone.yaw_deg * Math.PI / 180;
    const dx = point.x - zone.center.x;
    const dy = point.y - zone.center.y;
    const x = Math.cos(angle) * dx - Math.sin(angle) * dy;
    const y = Math.sin(angle) * dx + Math.cos(angle) * dy;
    return (
      Math.abs(x) <= zone.width_m / 2
      && Math.abs(y) <= zone.height_m / 2
    );
  }

  isPointValid(point) {
    const map = this.state.map;
    const pixel = this.mapToPixel(point);
    const ix = Math.floor(pixel.x + 1e-9);
    const iy = Math.ceil(pixel.y - 1e-9);
    if (ix <= 0 || iy <= 0 || ix >= map.width - 1 || iy >= map.height - 1) return false;
    const clearance = Number(
      this.state.document?.settings?.safety_clearance_m || 0,
    );
    const radius = clearance / map.resolution;
    const reach = Math.ceil(radius);
    for (let row = Math.max(0, iy - reach); row <= Math.min(map.height - 1, iy + reach); row += 1) {
      for (let column = Math.max(0, ix - reach); column <= Math.min(map.width - 1, ix + reach); column += 1) {
        if (
          map.occupancy[row * map.width + column] !== 0
          && Math.hypot(column - ix, row - iy) < Math.max(radius, 1e-6)
        ) return false;
      }
    }
    for (const zone of this.state.document?.keepouts || []) {
      if (this.hidden.has(zone.id)) continue;
      const angle = -zone.yaw_deg * Math.PI / 180;
      const dx = point.x - zone.center.x;
      const dy = point.y - zone.center.y;
      const localX = Math.cos(angle) * dx - Math.sin(angle) * dy;
      const localY = Math.sin(angle) * dx + Math.cos(angle) * dy;
      const outsideX = Math.max(Math.abs(localX) - zone.width_m / 2, 0);
      const outsideY = Math.max(Math.abs(localY) - zone.height_m / 2, 0);
      if (Math.hypot(outsideX, outsideY) < Math.max(clearance, 1e-6)) return false;
    }
    return true;
  }

  keepoutVertices(zone) {
    const angle = zone.yaw_deg * Math.PI / 180;
    const halfWidth = zone.width_m / 2;
    const halfHeight = zone.height_m / 2;
    return [
      [-halfWidth, -halfHeight],
      [halfWidth, -halfHeight],
      [halfWidth, halfHeight],
      [-halfWidth, halfHeight],
    ].map(([x,y]) => ({
      x: zone.center.x + Math.cos(angle) * x - Math.sin(angle) * y,
      y: zone.center.y + Math.sin(angle) * x + Math.cos(angle) * y,
    }));
  }

  keepoutHandles(zone) {
    const vertices = this.keepoutVertices(zone);
    const resize = this.mapToScreen(vertices[2]);
    const angle = (zone.yaw_deg + 90) * Math.PI / 180;
    const rotateDistance = (
      zone.height_m / 2 + 20 / this.scale * this.state.map.resolution
    );
    const rotatePoint = {
      x: zone.center.x + Math.cos(angle) * rotateDistance,
      y: zone.center.y + Math.sin(angle) * rotateDistance,
    };
    return { resize, rotate: this.mapToScreen(rotatePoint) };
  }

  locationRotateHandle(location) {
    const length = 28 / this.scale * this.state.map.resolution;
    const angle = location.yaw_deg * Math.PI / 180;
    return this.mapToScreen({
      x: location.x + Math.cos(angle) * length,
      y: location.y + Math.sin(angle) * length,
    });
  }

  draw() {
    const ctx = this.ctx;
    const ratio = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();
    ctx.save();
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, rect.width, rect.height);
    if (!this.state.map) { ctx.restore(); return; }
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(
      this.background, this.offset.x, this.offset.y,
      this.state.map.width * this.scale, this.state.map.height * this.scale,
    );
    ctx.strokeStyle = "#536174";
    ctx.strokeRect(this.offset.x, this.offset.y, this.state.map.width * this.scale, this.state.map.height * this.scale);
    this.drawKeepouts(ctx);
    this.drawRoutes(ctx);
    this.drawLocations(ctx);
    this.drawDraft(ctx);
    ctx.restore();
  }

  path(ctx, points, close = false) {
    if (!points.length) return;
    const first = this.mapToScreen(points[0]);
    ctx.beginPath();
    ctx.moveTo(first.x, first.y);
    points.slice(1).forEach(point => {
      const p = this.mapToScreen(point);
      ctx.lineTo(p.x, p.y);
    });
    if (close) ctx.closePath();
  }

  drawKeepouts(ctx) {
    for (const zone of this.state.document?.keepouts || []) {
      const selected = this.state.selection?.type === "keepout" && this.state.selection.id === zone.id;
      this.path(ctx, this.keepoutVertices(zone), true);
      ctx.fillStyle = selected ? "rgba(251,113,133,.32)" : "rgba(251,113,133,.20)";
      ctx.strokeStyle = selected ? "#fda4af" : "#fb7185";
      ctx.lineWidth = selected ? 2.5 : 1.5;
      ctx.fill(); ctx.stroke();
      if (selected) {
        const handles = this.keepoutHandles(zone);
        this.handle(ctx, handles.resize, "#fb7185");
        this.handle(ctx, handles.rotate, "#fbbf24");
        const center = this.mapToScreen(zone.center);
        ctx.beginPath(); ctx.moveTo(center.x, center.y); ctx.lineTo(handles.rotate.x, handles.rotate.y);
        ctx.strokeStyle = "#fbbf24"; ctx.setLineDash([4,4]); ctx.stroke(); ctx.setLineDash([]);
      }
    }
  }

  drawRoutes(ctx) {
    for (const route of this.state.document?.routes || []) {
      if (this.hidden.has(route.id)) continue;
      const selected = this.state.selection?.type === "route" && this.state.selection.id === route.id;
      const smooth = this.state.routePreviews.get(route.id);
      if (smooth?.points?.length) {
        this.path(ctx, smooth.points, false);
        ctx.strokeStyle = smooth.status === "invalid" ? "#fb7185" : "#40d9b3";
        ctx.lineWidth = selected ? 4 : 3;
        ctx.stroke();
      }
      this.path(ctx, route.waypoints, route.closed);
      ctx.strokeStyle = selected ? "#93c5fd" : "#60a5fa";
      ctx.lineWidth = 1.5;
      ctx.setLineDash([7,5]); ctx.stroke(); ctx.setLineDash([]);
      route.waypoints.forEach((waypoint, index) => {
        const p = this.mapToScreen(waypoint);
        ctx.beginPath(); ctx.arc(p.x, p.y, selected ? 5.5 : 4, 0, Math.PI * 2);
        ctx.fillStyle = selected ? "#dbeafe" : "#60a5fa"; ctx.fill();
        if (selected) {
          ctx.fillStyle = "#0d1117"; ctx.font = "9px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
          ctx.fillText(String(index + 1), p.x, p.y);
        }
      });
    }
  }

  drawLocations(ctx) {
    for (const location of this.state.document?.locations || []) {
      if (this.hidden.has(location.id)) continue;
      const selected = this.state.selection?.type === "location" && this.state.selection.id === location.id;
      const valid = this.isPointValid(location);
      const p = this.mapToScreen(location);
      const angle = -(location.yaw_deg - this.state.map.origin.yaw_deg) * Math.PI / 180;
      ctx.save(); ctx.translate(p.x, p.y); ctx.rotate(angle);
      ctx.beginPath(); ctx.moveTo(11,0); ctx.lineTo(-7,-7); ctx.lineTo(-4,0); ctx.lineTo(-7,7); ctx.closePath();
      ctx.fillStyle = valid ? (selected ? "#bfdbfe" : "#60a5fa") : "#fb7185"; ctx.fill();
      ctx.strokeStyle = "#0b1119"; ctx.lineWidth = 1.5; ctx.stroke(); ctx.restore();
      if (selected) {
        const rotate = this.locationRotateHandle(location);
        ctx.beginPath(); ctx.moveTo(p.x,p.y); ctx.lineTo(rotate.x,rotate.y); ctx.strokeStyle="#fbbf24"; ctx.stroke();
        this.handle(ctx, rotate, "#fbbf24");
      }
    }
  }

  drawDraft(ctx) {
    if (this.routeDraft.length) {
      const points = this.hoverMap ? [...this.routeDraft, this.hoverMap] : this.routeDraft;
      this.path(ctx, points);
      ctx.strokeStyle = "#40d9b3"; ctx.lineWidth = 2; ctx.setLineDash([6,4]); ctx.stroke(); ctx.setLineDash([]);
      this.routeDraft.forEach(point => this.handle(ctx, this.mapToScreen(point), "#40d9b3", 4));
    }
    if (this.drag?.kind === "create-location") {
      const p = this.mapToScreen(this.drag.start);
      const q = this.mapToScreen(this.drag.current);
      const color = this.isPointValid(this.drag.start) ? "#60a5fa" : "#fb7185";
      ctx.beginPath(); ctx.moveTo(p.x,p.y); ctx.lineTo(q.x,q.y); ctx.strokeStyle=color; ctx.lineWidth=2; ctx.stroke();
      this.handle(ctx,p,color);
    }
    if (this.drag?.kind === "create-keepout") {
      const zone = this.keepoutFromDrag(this.drag);
      this.path(ctx, this.keepoutVertices(zone), true);
      ctx.fillStyle="rgba(251,113,133,.25)"; ctx.strokeStyle="#fb7185"; ctx.fill(); ctx.stroke();
    }
  }

  handle(ctx, point, color, radius = 6) {
    ctx.beginPath(); ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
    ctx.fillStyle = "#111720"; ctx.fill(); ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke();
  }
}

window.MapEditorCanvas = MapEditorCanvas;
