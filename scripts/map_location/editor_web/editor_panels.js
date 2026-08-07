"use strict";

const state = window.editorState;
const canvasEditor = new window.MapEditorCanvas(document.getElementById("mapCanvas"), state);

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, character => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#039;",
  })[character]);
}

function round(value, digits = 3) {
  return Number(Number(value).toFixed(digits));
}

function showToast(message, error = false) {
  const toast = document.createElement("div");
  toast.className = `toast${error ? " error" : ""}`;
  toast.textContent = message;
  document.getElementById("toastRegion").append(toast);
  setTimeout(() => toast.remove(), 4200);
}
window.showToast = showToast;

function setSaveState(text, className = "") {
  const element = document.getElementById("saveState");
  element.textContent = text;
  element.className = `save-state ${className}`;
}

function objectBySelection(document, selection) {
  if (!selection) return null;
  const key = { location: "locations", keepout: "keepouts", route: "routes" }[selection.type];
  return document[key]?.find(item => item.id === selection.id);
}

function renderTree() {
  const tree = document.getElementById("objectTree");
  if (!state.document) { tree.replaceChildren(); return; }
  const query = document.getElementById("objectSearch").value.trim().toLowerCase();
  const groups = [
    ["location", "目标点", state.document.locations],
    ["keepout", "禁行区", state.document.keepouts],
    ["route", "巡航路线", state.document.routes],
  ];
  tree.innerHTML = groups.map(([type, label, allItems]) => {
    const items = allItems.filter(item => item.name.toLowerCase().includes(query));
    return `
    <div class="tree-group">
      <div class="tree-label"><span>${label}</span><span>${items.length}${query ? ` / ${allItems.length}` : ""}</span></div>
      ${items.length ? items.map(item => `
        <div class="tree-object-row">
          <button class="tree-item ${state.selection?.type === type && state.selection?.id === item.id ? "selected" : ""}"
            data-object-type="${type}" data-object-id="${escapeHtml(item.id)}" title="双击定位">
            <span class="tree-dot ${type}"></span>
            <span>${escapeHtml(item.name)}</span>
            <span class="tree-meta">${type === "route" ? `${item.waypoints.length} 点` : ""}</span>
          </button>
          <button class="tree-visibility ${canvasEditor.hidden.has(item.id) ? "hidden" : ""}"
            data-visibility-id="${escapeHtml(item.id)}" title="显示/隐藏">◉</button>
        </div>
        ${type === "route" && state.selection?.id === item.id ? `
          <div class="waypoint-list">${item.waypoints.map((waypoint, index) => `
            <div class="waypoint-row ${state.selection.childId === waypoint.id ? "selected" : ""}">
              <button data-waypoint-id="${escapeHtml(waypoint.id)}" data-route-id="${escapeHtml(item.id)}">P${index + 1}</button>
              <span>${round(waypoint.x)}, ${round(waypoint.y)}</span>
              <button data-move-waypoint="${index}" data-route-id="${escapeHtml(item.id)}" data-delta="-1" ${index === 0 ? "disabled" : ""}>↑</button>
              <button data-move-waypoint="${index}" data-route-id="${escapeHtml(item.id)}" data-delta="1" ${index === item.waypoints.length - 1 ? "disabled" : ""}>↓</button>
            </div>`).join("")}</div>` : ""}`).join("") : `<div class="tree-item tree-meta">暂无</div>`}
    </div>`;
  }).join("");
  document.getElementById("objectCount").textContent = String(
    state.document.locations.length + state.document.keepouts.length + state.document.routes.length,
  );
  tree.querySelectorAll("[data-object-id]").forEach(button => button.addEventListener("click", () => {
    state.setSelection(button.dataset.objectType, button.dataset.objectId);
    canvasEditor.setTool("select");
  }));
  tree.querySelectorAll("[data-object-id]").forEach(button => button.addEventListener("dblclick", () => {
    const object = objectBySelection(state.document, {
      type: button.dataset.objectType,
      id: button.dataset.objectId,
    });
    if (object) canvasEditor.centerOnObject(button.dataset.objectType, object);
  }));
  tree.querySelectorAll("[data-visibility-id]").forEach(button => button.addEventListener("click", () => {
    canvasEditor.toggleVisibility(button.dataset.visibilityId);
    renderTree();
  }));
  tree.querySelectorAll("[data-waypoint-id]").forEach(button => button.addEventListener("click", () => {
    state.setSelection("route", button.dataset.routeId, button.dataset.waypointId);
    canvasEditor.setTool("select");
  }));
  tree.querySelectorAll("[data-move-waypoint]").forEach(button => button.addEventListener("click", () => {
    moveWaypoint(
      button.dataset.routeId,
      Number(button.dataset.moveWaypoint),
      Number(button.dataset.delta),
    );
  }));
}

function numberField(label, path, value, options = "") {
  return `<label class="field"><span>${label}</span><input type="number" step="0.01" value="${escapeHtml(round(value))}" data-path="${path}" ${options}></label>`;
}

function renderProperties() {
  const panel = document.getElementById("propertyPanel");
  const object = state.findSelected();
  document.getElementById("selectionType").textContent = object ?
    { location: "目标点", keepout: "禁行区", route: "巡航路线" }[state.selection.type] : "未选择";
  if (!object) {
    panel.innerHTML = `<div class="no-selection"><span>◇</span><p>从左侧列表或地图中选择对象</p></div>`;
    bindPropertyInputs();
    return;
  }
  const name = `<label class="field"><span>名称</span><input maxlength="80" value="${escapeHtml(object.name)}" data-path="name"></label>`;
  let content = name;
  if (state.selection.type === "location") {
    content += `<div class="field-row">${numberField("X（米）", "x", object.x)}${numberField("Y（米）", "y", object.y)}</div>
      ${numberField("朝向（度）", "yaw_deg", object.yaw_deg)}
      <div class="status-card">拖拽箭头移动，拖拽黄色控制点调整朝向。</div>
      <div class="property-actions"><button id="copyLocation">复制目标点</button></div>`;
  } else if (state.selection.type === "keepout") {
    content += `<div class="field-row">${numberField("中心 X", "center.x", object.center.x)}${numberField("中心 Y", "center.y", object.center.y)}</div>
      <div class="field-row">${numberField("长度（米）", "width_m", object.width_m, 'min="0.01"')}${numberField("宽度（米）", "height_m", object.height_m, 'min="0.01"')}</div>
      ${numberField("旋转（度）", "yaw_deg", object.yaw_deg)}
      <div class="status-card">红色区域会参与路线安全校验；导出时写入新地图栅格。</div>`;
  } else {
    const clearance = state.document.settings.safety_clearance_m;
    const clearanceSource = state.document.settings.safety_clearance_source;
    const smooth = state.routePreviews.get(object.id);
    const statusText = !smooth ? "尚未平滑" :
      smooth.status === "invalid" ? `无效：${smooth.collisions?.length || 0} 个碰撞线段` :
      smooth.status === "valid_with_fallbacks" ? `有效，${smooth.fallback_waypoint_ids.length} 个拐角保留折线` :
      "路径有效";
    content += `<label class="field"><span>路线形式</span><select data-path="closed">
        <option value="false" ${object.closed ? "" : "selected"}>开放路线</option>
        <option value="true" ${object.closed ? "selected" : ""}>闭环路线</option>
      </select></label>
      <div class="status-card ${smooth?.status?.startsWith("valid") ? "valid" : smooth?.status === "invalid" ? "invalid" : ""}">
        ${escapeHtml(statusText)} · ${object.waypoints.length} 个关键点
      </div>
      <div class="property-actions">
        <label class="field"><span>安全净空（米） · ${escapeHtml(clearanceSource)}</span>
          <input type="number" min="0" step="0.05" value="${escapeHtml(clearance)}" data-setting="clearance">
        </label>
        <button id="smoothRoute" class="primary">平滑效果预览</button>
        ${state.selection.childId ? '<button id="deleteWaypoint">删除当前关键点</button>' : ""}
      </div>`;
  }
  content += `<div class="property-actions"><button id="deleteObject" class="danger">删除此对象</button></div>`;
  panel.innerHTML = content;
  bindPropertyInputs();
  panel.querySelector("#copyLocation")?.addEventListener("click", () => copyLocation());
  panel.querySelector("#smoothRoute")?.addEventListener("click", () => state.smoothRoute(object.id));
  panel.querySelector("#deleteWaypoint")?.addEventListener("click", () => deleteWaypoint());
  panel.querySelector("#deleteObject")?.addEventListener("click", () => deleteSelection());
}

function setPath(target, path, value) {
  const parts = path.split(".");
  const leaf = parts.pop();
  const parent = parts.reduce((current, part) => current[part], target);
  parent[leaf] = value;
}

function bindPropertyInputs() {
  document.querySelectorAll("#propertyPanel [data-path]").forEach(input => {
    input.addEventListener("change", async () => {
      const selection = { ...state.selection };
      let value = input.value;
      if (input.type === "number") value = Number(value);
      if (input.tagName === "SELECT" && ["true", "false"].includes(value)) value = value === "true";
      const current = state.findSelected();
      if (input.dataset.path === "closed" && value && current?.waypoints.length < 3) {
        showToast("闭环路线至少需要三个关键点", true);
        renderProperties();
        return;
      }
      if (selection.type === "route" && input.dataset.path !== "name") {
        state.clearRoutePreview(selection.id);
      } else if (selection.type === "keepout" && input.dataset.path !== "name") {
        state.clearRoutePreview();
      }
      await state.commit("修改属性", document => {
        const object = objectBySelection(document, selection);
        if (!object) return;
        setPath(object, input.dataset.path, value);
      });
    });
  });
  document.querySelector("[data-setting='clearance']")?.addEventListener("change", event => {
    state.clearRoutePreview();
    state.commit("修改安全净空", document => {
      document.settings.safety_clearance_m = Math.max(0, Number(event.target.value));
      document.settings.safety_clearance_source = "editor_override";
    });
  });
  if (state.readOnly) {
    document.querySelectorAll("#propertyPanel input, #propertyPanel select, #propertyPanel button")
      .forEach(control => { control.disabled = true; });
  }
}

async function deleteSelection() {
  if (!state.selection) return;
  const selection = { ...state.selection };
  if (selection.type === "keepout") state.clearRoutePreview();
  if (selection.type === "route") state.clearRoutePreview(selection.id);
  await state.commit("删除对象", document => {
    const key = { location: "locations", keepout: "keepouts", route: "routes" }[selection.type];
    document[key] = document[key].filter(item => item.id !== selection.id);
  });
  state.setSelection(null);
}

async function deleteWaypoint() {
  const selection = { ...state.selection };
  const route = state.findSelected();
  const minimum = route?.closed ? 3 : 2;
  if (!route || route.waypoints.length <= minimum) {
    showToast(`该路线至少需要保留 ${minimum} 个关键点`, true);
    return;
  }
  state.clearRoutePreview(selection.id);
  await state.commit("删除路线点", document => {
    const target = document.routes.find(item => item.id === selection.id);
    target.waypoints = target.waypoints.filter(item => item.id !== selection.childId);
  });
  state.setSelection("route", selection.id);
}

async function moveWaypoint(routeId, index, delta) {
  state.clearRoutePreview(routeId);
  await state.commit("调整关键点顺序", document => {
    const route = document.routes.find(item => item.id === routeId);
    const target = index + delta;
    if (!route || target < 0 || target >= route.waypoints.length) return;
    const [waypoint] = route.waypoints.splice(index, 1);
    route.waypoints.splice(target, 0, waypoint);
  });
}

async function copyLocation() {
  const source = state.findSelected();
  if (!source || state.selection.type !== "location") return;
  const id = crypto.randomUUID();
  await state.commit("复制目标点", document => {
    const names = new Set(document.locations.map(item => item.name));
    let index = 2;
    let name = `${source.name} 副本`;
    while (names.has(name)) name = `${source.name} 副本 ${index++}`;
    document.locations.push({
      ...JSON.parse(JSON.stringify(source)),
      id,
      name,
    });
  });
  state.setSelection("location", id);
}

function updateHistory() {
  document.getElementById("undoButton").disabled = state.readOnly || !state.undoStack.length;
  document.getElementById("redoButton").disabled = state.readOnly || !state.redoStack.length;
  document.getElementById("exportButton").disabled = state.readOnly || !state.document;
}

async function loadMaps(preferred = null) {
  try {
    const payload = await state.request("/api/maps");
    const select = document.getElementById("mapSelect");
    select.innerHTML = payload.maps.map(map => `<option value="${escapeHtml(map.name)}">${escapeHtml(map.name)}</option>`).join("");
    document.getElementById("emptyState").hidden = payload.maps.length > 0;
    if (!payload.maps.length) { setSaveState("无地图", "error"); return; }
    const selected = payload.maps.some(map => map.name === preferred) ? preferred : payload.maps[0].name;
    select.value = selected;
    await state.load(selected);
  } catch (error) {
    setSaveState("载入失败", "error");
    showToast(error.message, true);
  }
}

document.querySelectorAll("[data-tool]").forEach(button =>
  button.addEventListener("click", () => canvasEditor.setTool(button.dataset.tool)));
document.getElementById("objectSearch").addEventListener("input", () => renderTree());
document.getElementById("undoButton").addEventListener("click", () => state.undo());
document.getElementById("redoButton").addEventListener("click", () => state.redo());
document.getElementById("mapSelect").addEventListener("change", async event => {
  const previous = state.mapName;
  try {
    await state.load(event.target.value);
  } catch (error) {
    event.target.value = previous;
    showToast(error.message, true);
  }
});
document.getElementById("zoomIn").addEventListener("click", () => canvasEditor.zoomAt(1.2));
document.getElementById("zoomOut").addEventListener("click", () => canvasEditor.zoomAt(1 / 1.2));
document.getElementById("fitMap").addEventListener("click", () => canvasEditor.fit());

const exportDialog = document.getElementById("exportDialog");
document.getElementById("exportButton").addEventListener("click", () => {
  document.getElementById("exportName").value = `${state.mapName}_edited`;
  exportDialog.showModal();
});
document.getElementById("exportForm").addEventListener("submit", async event => {
  if (event.submitter?.value === "cancel") return;
  event.preventDefault();
  const target = document.getElementById("exportName").value.trim();
  const button = document.getElementById("confirmExport");
  button.disabled = true;
  try {
    await state.exportMap(target);
    exportDialog.close();
    showToast(`已导出新地图：${target}`);
    await loadMaps(target);
  } catch (error) {
    showToast(error.message, true);
  } finally {
    button.disabled = false;
  }
});

document.addEventListener("keydown", event => {
  const editing = /INPUT|SELECT|TEXTAREA/.test(document.activeElement?.tagName);
  if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z") {
    event.preventDefault();
    event.shiftKey ? state.redo() : state.undo();
  } else if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "y") {
    event.preventDefault(); state.redo();
  } else if (!editing && event.key === "Delete") {
    deleteSelection();
  } else if (!editing && event.key === "Enter" && canvasEditor.tool === "route") {
    canvasEditor.finishRoute();
  } else if (event.key === "Escape") {
    canvasEditor.routeDraft = [];
    canvasEditor.cancelDrag();
    canvasEditor.setTool("select");
  }
});

state.addEventListener("loaded", () => {
  renderTree(); renderProperties(); updateHistory();
  if (state.readOnly) {
    setSaveState("只读模式", "error");
    showToast(state.readOnlyReason, true);
  } else {
    setSaveState("已保存", "saved");
  }
});
state.addEventListener("change", () => {
  renderTree(); renderProperties(); updateHistory();
});
state.addEventListener("selection", () => {
  renderTree(); renderProperties();
});
state.addEventListener("saving", () => setSaveState("正在保存…", "saving"));
state.addEventListener("saved", () => {
  setSaveState("已保存", "saved"); renderTree(); renderProperties(); updateHistory();
});
state.addEventListener("saveerror", event => {
  setSaveState("保存失败", "error");
  showToast(event.detail.error.message, true);
});
state.addEventListener("conflict", () => {
  showToast("文档已在另一窗口更新，正在重新载入", true);
  state.load(state.mapName).catch(error => showToast(error.message, true));
});
state.addEventListener("readonly", event => showToast(event.detail.message, true));

loadMaps(new URLSearchParams(location.search).get("map"));
