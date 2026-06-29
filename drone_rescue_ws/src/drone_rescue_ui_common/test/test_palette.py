"""UiPalette is the single semantic-colour source, and the canonical
SEVERITY_COLOR table derives from it."""

import re

import pytest

from drone_rescue_ui_common.palette import DEFAULT_PALETTE, UiPalette
from drone_rescue_ui_common.constants import SEVERITY_COLOR

_HEX = re.compile(r'^#[0-9a-fA-F]{6}$')


def test_all_tokens_are_hex_strings():
    for name, value in vars(DEFAULT_PALETTE).items():
        assert _HEX.match(value), f'{name}={value!r} is not a #rrggbb hex'


def test_palette_is_frozen():
    with pytest.raises((AttributeError, Exception)):
        DEFAULT_PALETTE.error = '#000000'   # frozen dataclass


def test_severity_color_derives_from_palette():
    assert SEVERITY_COLOR[0] == DEFAULT_PALETTE.info
    assert SEVERITY_COLOR[1] == DEFAULT_PALETTE.warn
    assert SEVERITY_COLOR[2] == DEFAULT_PALETTE.error


def test_a_new_theme_is_a_new_instance():
    hi_contrast = UiPalette(error='#ff0000', warn='#ffaa00')
    assert hi_contrast.error == '#ff0000'
    assert DEFAULT_PALETTE.error == '#ef4444'   # default unchanged
