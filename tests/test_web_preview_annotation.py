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
