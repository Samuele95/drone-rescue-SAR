"""UiPalette: the operator UI's semantic colour vocabulary.

The look-and-feel world model as a first-class concern. Before this
module ~28 hex literals were scattered across ten widget files, with
three conflicting "error red" values and three conflicting "warn amber"
values, so the same severity rendered differently depending on which
panel produced it. This collapses them into one frozen value object of
*semantic tokens* (``ok`` / ``warn`` / ``error`` / ``info`` / surfaces
/ actions). Widgets reference ``DEFAULT_PALETTE.error`` instead of a
literal, so colour consistency is enforced by a single source rather
than convention, and re-theming (dark / light / high-contrast) becomes
a one-object swap instead of a ten-file edit.

Scope note: this palette covers *semantic status + surface + action*
colours only. Per-drone identity colours stay in ``constants.DRONE_COLORS``
(they are identity, not status), and component-specific one-offs (scene grid
lines, etc.) stay local; the palette must not bloat into every colour.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class UiPalette:
    """Frozen semantic colour tokens. One instantiation site
    (``DEFAULT_PALETTE``); a new theme is a new instance, not an edit."""

    # Semantic status
    ok: str = '#16a34a'      # confirmed / healthy / complete
    warn: str = '#f59e0b'    # degraded / timeout / pending
    error: str = '#ef4444'   # error / down / critical
    info: str = '#3b82f6'    # informational

    # Interactive accent: the single vivid accent for *interaction*:
    # selected tab, focused input, hover, active selection. Distinct
    # from ``info``: info is the SEVERITY blue (log INFO lines,
    # informational badges); accent is "you are here / this responds
    # to you". Keep them separate so a recolor of one never silently
    # re-reads the other.
    accent: str = '#2dd4bf'       # teal: selected / focused / hover
    accent_soft: str = '#134e48'  # selection fills, subtle highlights
    focus: str = '#5eead4'        # brightest tier: focus text/glow

    # Surfaces (cool navy-slate, deepened)
    bg_dark: str = '#0e1526'    # window / stage backgrounds
    bg_panel: str = '#151f33'   # panel headers, alternate rows
    text_muted: str = '#8ba0bd'  # secondary labels
    text_body: str = '#e6edf7'   # body text on dark
    # Surface tiers for the rail/stage/inspector layout. bg_deep was
    # previously a scene_view literal ('#0b1220'); bg_raised hosts
    # cards/inspector; stroke is the hairline border.
    bg_deep: str = '#090f1d'     # scene backgrounds, deepest layer
    bg_raised: str = '#1c2840'   # cards, inspector, raised panels
    stroke: str = '#2b3a55'      # hairline borders / dividers

    # Actions
    action_run: str = '#16a34a'
    action_stop: str = '#dc2626'
    action_export: str = '#7c3aed'


DEFAULT_PALETTE = UiPalette()
