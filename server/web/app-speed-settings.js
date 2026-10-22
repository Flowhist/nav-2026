"use strict";

const navigationSpeed = { revision: null, busy: false, dirty: false };
const navigationSpeedFields = {
  linear_speed_mps: "navSpeedInput",
  angular_speed_degps: "navRotateSpeedInput",
  drive_angular_speed_degps: "navDriveTurnSpeedInput",
};

function closeNavigationSpeed(returnFocus = false) {
  $("navSpeedPanel").open = false;
  if (returnFocus) $("navSpeedPanel").querySelector("summary").focus();
}

$("btnCloseNavSpeed").addEventListener("click", () => closeNavigationSpeed(true));
$("navSpeedPanel").addEventListener("toggle", () => {
  if ($("navSpeedPanel").open) $("btnCloseNavSpeed").focus({ preventScroll: true });
});
document.addEventListener("keydown", event => {
  if (event.key === "Escape" && $("navSpeedPanel").open) {
    event.preventDefault();
    event.stopPropagation();
    closeNavigationSpeed(true);
  }
});
document.addEventListener("click", event => {
  if (!$("navSpeedPanel").contains(event.target)) closeNavigationSpeed();
});

async function loadNavigationSpeed() {
  if (navigationSpeed.busy || navigationSpeed.dirty) return;
  navigationSpeed.busy = true;
  setNavigationSpeedBusy(true);
  try {
    const settings = await api("/api/nav/speed");
    navigationSpeed.revision = settings.revision;
    Object.entries(navigationSpeedFields).forEach(([key, id]) => {
      $(id).value = settings[key];
      [$(id).min, $(id).max] = settings.limits[key];
    });
    $("navSpeedStatus").textContent = "";
  } catch (error) {
    $("navSpeedStatus").textContent = error.message;
  } finally {
    navigationSpeed.busy = false;
    setNavigationSpeedBusy(false);
  }
}

function setNavigationSpeedBusy(busy) {
  Object.values(navigationSpeedFields).forEach(id => { $(id).disabled = busy || !navigationSpeed.revision; });
  $("btnSaveNavSpeed").disabled = busy || !navigationSpeed.revision;
  $("btnReloadNavSpeed").disabled = busy;
}

Object.values(navigationSpeedFields).forEach(id => $(id).addEventListener("input", () => {
  navigationSpeed.dirty = true;
  $("navSpeedStatus").textContent = "未保存";
}));
$("btnReloadNavSpeed").addEventListener("click", () => {
  navigationSpeed.dirty = false;
  loadNavigationSpeed();
});
$("navSpeedForm").addEventListener("submit", async event => {
  event.preventDefault();
  if (navigationSpeed.busy || !navigationSpeed.revision) return;
  navigationSpeed.busy = true;
  setNavigationSpeedBusy(true);
  $("navSpeedStatus").textContent = "正在保存…";
  try {
    const result = await api("/api/nav/speed", "POST", {
      ...Object.fromEntries(Object.entries(navigationSpeedFields).map(([key, id]) => [key, Number($(id).value)])),
      expected_revision: navigationSpeed.revision,
    });
    navigationSpeed.revision = result.revision;
    navigationSpeed.dirty = false;
    $("navSpeedStatus").textContent = result.apply_error
      ? `已保存，当前导航未确认应用：${result.apply_error}`
      : result.applied ? "已保存，当前导航已应用" : "已保存，下次启动导航生效";
  } catch (error) {
    $("navSpeedStatus").textContent = error.message;
  } finally {
    navigationSpeed.busy = false;
    setNavigationSpeedBusy(false);
  }
});

function renderGearSlider() {
  const slider = $("manualGearSlider");
  slider.style.setProperty("--gear-fill", `${(Number(slider.value) - 1) * 25}%`);
  slider.setAttribute("aria-valuetext", `${slider.value} 档`);
  document.querySelectorAll(".manual-gear-ticks span").forEach((tick, index) => {
    tick.classList.toggle("active", index + 1 === Number(slider.value));
  });
}

$("manualGearSlider").addEventListener("pointerdown", () => { manualControl.gearEditing = true; });
$("manualGearSlider").addEventListener("input", () => {
  manualControl.gearEditing = true;
  renderGearSlider();
});
for (const eventName of ["pointerup", "pointercancel", "blur"]) {
  $("manualGearSlider").addEventListener(eventName, () => { manualControl.gearEditing = false; });
}
$("manualGearSlider").addEventListener("change", async () => {
    manualControl.gearEditing = false;
    if (manualControl.gearPending) return;
    const gear = Number($("manualGearSlider").value);
    if (gear === appState.status?.control?.gear) return;
    manualControl.gearPending = true;
    renderManualControlStatus();
    $("manualGearStatus").textContent = "正在同步档位…";
    try {
      await api("/api/control/gear", "POST", { gear });
      $("manualGearStatus").textContent = `${gear} 档已回读确认`;
    } catch (error) {
      $("manualGearStatus").textContent = error.message;
    } finally {
      manualControl.gearPending = false;
      renderManualControlStatus();
    }
});
