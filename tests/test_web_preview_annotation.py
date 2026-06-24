import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _run_node(source: str) -> str:
    proc = subprocess.run(
        ["node", "-e", source],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return proc.stdout.strip()


def test_preview_raster_keeps_source_pgm_y_order():
    app_core = (ROOT / "server" / "web" / "app-core.js").read_text(encoding="utf-8")
    script = (
        """
const context = {
  console,
  Map,
  Uint8ClampedArray,
  window: { devicePixelRatio: 1 },
  document: {
    createElement: () => ({
      width: 0,
      height: 0,
      getContext: () => ({
        createImageData: (width, height) => ({ width, height, data: new Uint8ClampedArray(width * height * 4) }),
        putImageData: function(image) { this.canvas._imageData = image; },
        canvas: null,
      }),
    }),
    getElementById: () => null,
  },
};
context.document.createElement = () => {
  const canvas = { width: 0, height: 0, _imageData: null };
  canvas.getContext = () => ({
    canvas,
    createImageData: (width, height) => ({ width, height, data: new Uint8ClampedArray(width * height * 4) }),
    putImageData: (image) => { canvas._imageData = image; },
  });
  return canvas;
};
const vm = require('vm');
"""
        + "vm.runInNewContext("
        + json.dumps(
            app_core
            + """
const raster = buildMapRaster({
  name: 'unit',
  width: 1,
  height: 2,
  resolution: 1,
  origin: { x: 0, y: 0, yaw_deg: 0 },
  data: [100, 0],
}, 'preview');
console.log(JSON.stringify(Array.from(raster._imageData.data)));
"""
        )
        + ", context);"
    )

    pixels = json.loads(_run_node(script))
    assert pixels[:4] == [52, 70, 61, 255]
    assert pixels[4:8] == [246, 246, 239, 255]


def test_preview_annotation_ui_auto_saves_without_save_button():
    index_html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    app_js = (ROOT / "server" / "web" / "app.js").read_text(encoding="utf-8")
    app_pages = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")

    assert "btnSavePreviewLocations" not in index_html
    assert "保存标注" not in index_html
    assert "未保存" not in app_pages
    assert "savePreviewLocations().catch" not in app_js
    assert "savePreviewLocations();" in app_js


def test_preview_annotation_uses_themed_modal_instead_of_native_dialogs():
    index_html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    app_js = (ROOT / "server" / "web" / "app.js").read_text(encoding="utf-8")

    for native_dialog in ("window.prompt", "window.alert", "window.confirm"):
        assert native_dialog not in app_js
    assert 'id="annotationDialog"' in index_html
    assert "showAnnotationNameDialog" in app_js
    assert "showAnnotationConfirmDialog" in app_js


def test_dynamic_cards_do_not_inject_names_with_inner_html():
    app_pages = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")

    assert "createInfoButton" in app_pages
    assert "btn.innerHTML = `<strong>${item.name}" not in app_pages
    assert "card.innerHTML = `<strong>${file.name}" not in app_pages


def test_scene_rendering_is_page_aware():
    app_pages = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")
    app_js = (ROOT / "server" / "web" / "app.js").read_text(encoding="utf-8")

    assert "getScenePollDelay" in app_pages
    assert 'appState.page === "mapping"' in app_pages
    assert 'appState.page === "navigation"' in app_pages
    set_page_body = app_js.split("async function setPage", 1)[1].split("function bind", 1)[0]
    assert "renderCurrentPageCanvases()" in set_page_body
    assert "renderLiveCanvases();" not in set_page_body


def test_user_actions_have_visible_error_toast():
    index_html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    app_core = (ROOT / "server" / "web" / "app-core.js").read_text(encoding="utf-8")
    app_pages = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")

    assert 'id="toast"' in index_html
    assert "function showToast" in app_core
    assert "function reportActionError" in app_core
    assert "await readApiError" in app_core
    assert "reportActionError(err" in app_pages


def test_console_visual_style_uses_compact_workbench_tokens():
    styles = (ROOT / "server" / "web" / "styles.css").read_text(encoding="utf-8")

    assert "--radius: 12px" in styles
    assert "--shadow: 0 10px 28px" in styles
    assert "h1 {\n  font-size: 28px;" in styles
    assert ".topbar {\n  position: sticky;" in styles
    assert "padding: 16px 24px 14px;" in styles
    assert "border-radius: 12px;" in styles


def test_running_session_page_switch_uses_themed_confirmation():
    app_js = (ROOT / "server" / "web" / "app.js").read_text(encoding="utf-8")
    set_page_body = app_js.split("async function setPage", 1)[1].split("function bind", 1)[0]
    helper_body = app_js.split("function confirmPageSwitchStop", 1)[1].split("async function setPage", 1)[0]

    assert "confirmPageSwitchStop" in app_js
    assert "showAnnotationConfirmDialog" in helper_body
    assert "await confirmPageSwitchStop" in set_page_body
    assert "return;" in set_page_body


def test_map_delete_uses_themed_confirmation_not_native_confirm():
    app_pages = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")

    assert "window.confirm" not in app_pages
    assert "showAnnotationConfirmDialog" in app_pages
    delete_body = app_pages.split("async function deleteSelectedPreviewMap", 1)[1].split("async function loadPreviewMap", 1)[0]
    assert "确认删除地图" in delete_body


def test_config_editor_has_line_numbers_and_client_validation():
    index_html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    app_pages = (ROOT / "server" / "web" / "app-pages.js").read_text(encoding="utf-8")
    styles = (ROOT / "server" / "web" / "styles.css").read_text(encoding="utf-8")

    assert 'id="configLineNumbers"' in index_html
    assert "renderConfigLineNumbers" in app_pages
    assert "validateConfigYaml" in app_pages
    assert "YAML 第" in app_pages
    assert ".line-numbers" in styles
