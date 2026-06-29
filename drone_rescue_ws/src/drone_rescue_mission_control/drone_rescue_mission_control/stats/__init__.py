"""Mission Control statistical-inference primitives.

Extracted from ``analytics.py`` so the figure-rendering layer no longer
ships a numerical statistics library inline. Pure numpy / math, no
matplotlib, no run-dict shape dependency. Future additions (Mann-Whitney
U, paired-t, etc.) register here under sibling modules.
"""

from .welch import T_TEST_MIN_N, t_test

__all__ = ['T_TEST_MIN_N', 't_test']
