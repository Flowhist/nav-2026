from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_navigation_goal_button_uses_standard_button_style():
    html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")

    assert 'id="btnArmGoal" class="mode-tool"' in html
    assert 'id="btnArmInit" class="mode-tool"' in html
    assert 'id="btnArmGoal" class="primary wide"' not in html


def test_navigation_mode_badge_is_removed():
    html = (ROOT / "server" / "web" / "index.html").read_text(encoding="utf-8")
    app_js = (ROOT / "server" / "web" / "app.js").read_text(encoding="utf-8")

    assert "navModeBadge" not in html
    assert "navModeBadge" not in app_js
    assert "当前未设置" not in html
