let selectSequence = 0;

// Keep native values and change handlers while sharing the map picker's menu UI.
function enhanceSelects(root = document) {
  root.querySelectorAll("select:not([hidden])").forEach((select) => {
    const id = `field-select-${++selectSequence}`;
    const panel = document.createElement("span");
    panel.id = id;
    panel.className = "nav-select field-select";
    panel.dataset.control = `${id}-control`;
    panel.innerHTML = `<button id="${id}-control" class="nav-select-control" type="button" aria-haspopup="listbox" aria-expanded="false" aria-controls="${id}-options"><span></span><span class="nav-select-chevron" aria-hidden="true"></span></button><span class="nav-select-menu"><span id="${id}-options" class="nav-select-options" role="listbox"></span></span>`;
    const control = panel.querySelector("button");
    const menu = panel.querySelector(".nav-select-menu");
    const list = panel.querySelector(".nav-select-options");
    const label = select.closest("label");
    const name = label?.querySelector("span")?.textContent || label?.firstChild?.textContent.trim() || "选择";
    control.setAttribute("aria-label", name);
    list.setAttribute("aria-label", name);
    select.before(panel);
    select.hidden = true;
    if (label) label.htmlFor = control.id;
    const options = Array.from(select.options, (option) => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "nav-select-option";
      button.setAttribute("role", "option");
      button.tabIndex = -1;
      button.textContent = option.textContent;
      button.disabled = option.disabled;
      button.addEventListener("click", () => {
        toggleNavSelect(id, false);
        control.focus();
        if (select.value === option.value) return;
        select.value = option.value;
        sync();
        select.dispatchEvent(new Event("change", { bubbles: true }));
        // Editor property changes rebuild the panel; restore keyboard focus.
        if (!control.isConnected && root.isConnected) {
          const replacement = Array.from(root.querySelectorAll("select")).find((item) => item.dataset.editorPath === select.dataset.editorPath);
          replacement?.previousElementSibling?.querySelector("button")?.focus();
        }
      });
      list.append(button);
      return button;
    });
    function sync() {
      control.firstElementChild.textContent = select.selectedOptions[0]?.textContent || "";
      control.disabled = select.disabled;
      options.forEach((button, index) => {
        button.classList.toggle("active", select.options[index].selected);
        button.setAttribute("aria-selected", String(select.options[index].selected));
      });
    }
    function open(index = select.selectedIndex) {
      toggleNavSelect(id, true);
      const rect = control.getBoundingClientRect();
      const width = Math.min(Math.max(rect.width, 160), window.innerWidth - 16);
      menu.style.width = `${width}px`;
      menu.style.left = `${Math.max(8, Math.min(rect.left, window.innerWidth - width - 8))}px`;
      const below = window.innerHeight - rect.bottom - 14;
      const above = rect.top - 14;
      const upwards = below < Math.min(menu.scrollHeight, 240) && above > below;
      list.style.maxHeight = `${Math.max(40, Math.min(240, upwards ? above : below) - 14)}px`;
      menu.style.top = upwards ? "auto" : `${rect.bottom + 6}px`;
      menu.style.bottom = upwards ? `${window.innerHeight - rect.top + 6}px` : "auto";
      (options[index]?.disabled ? options.find((item) => !item.disabled) : options[index])?.focus({ preventScroll: true });
    }
    control.addEventListener("click", () => {
      if (panel.classList.contains("open")) toggleNavSelect(id, false);
      else open();
    });
    panel.addEventListener("keydown", (event) => {
      const available = options.filter((item) => !item.disabled);
      const index = available.indexOf(document.activeElement);
      if (["ArrowDown", "ArrowUp", "Home", "End"].includes(event.key)) {
        event.preventDefault();
        event.stopPropagation();
        if (!panel.classList.contains("open")) open();
        else {
          const next = event.key === "Home" ? 0 : event.key === "End" ? available.length - 1 : (index + (event.key === "ArrowDown" ? 1 : -1) + available.length) % available.length;
          available[next]?.focus();
        }
      } else if (event.key === "Escape" || event.key === "Tab") {
        toggleNavSelect(id, false);
        control.focus();
        if (event.key === "Escape") {
          event.preventDefault();
          event.stopPropagation();
        }
      }
    });
    select.addEventListener("change", sync);
    sync();
  });
}

function closeFieldSelects(event) {
  document.querySelectorAll(".field-select.open").forEach((panel) => {
    if (!(event.target instanceof Node) || !panel.contains(event.target)) toggleNavSelect(panel.id, false);
  });
}
document.addEventListener("click", closeFieldSelects);
document.addEventListener("scroll", closeFieldSelects, true);
window.addEventListener("resize", closeFieldSelects);
