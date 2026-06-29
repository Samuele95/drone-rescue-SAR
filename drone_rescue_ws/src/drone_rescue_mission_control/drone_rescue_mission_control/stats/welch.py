"""Welch's two-sample t-test: pure numpy/math, no scipy.

Extracted from ``analytics.py`` so the statistical-inference subpackage
doesn't ship inline inside the figure-rendering module. Same numerics as
the legacy implementation (Welch 1947; Numerical Recipes 3e section 6.4
Lentz CF for the regularized incomplete beta).

The default ``min_n=3`` reflects that with n=2 per group the
Welch-Satterthwaite df is 2 and the test has effectively no power for
detectable effect sizes: a p-value computed at that sample size is
misleading rather than informative. Set ``min_n=2`` to opt back in to
the old behaviour where the caller has reason.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np


T_TEST_MIN_N = 3


def t_test(
    a: List[float], b: List[float],
    min_n: int = T_TEST_MIN_N,
) -> Tuple[float, float]:
    """Welch's two-sample t-test on unequal-variance samples.

    Returns ``(t_statistic, two_sided_p_value)``. Pure numpy/math,
    no scipy dependency. Welch, Biometrika 34, 1947.

    Returns ``(0.0, 1.0)`` (the conventional "no evidence" pair) when
    either group has fewer than ``min_n`` non-None samples.
    """
    a_arr = np.array([x for x in a if x is not None], dtype=float)
    b_arr = np.array([x for x in b if x is not None], dtype=float)
    if len(a_arr) < max(2, min_n) or len(b_arr) < max(2, min_n):
        return 0.0, 1.0
    m_a, m_b = a_arr.mean(), b_arr.mean()
    v_a, v_b = a_arr.var(ddof=1), b_arr.var(ddof=1)
    n_a, n_b = len(a_arr), len(b_arr)
    se = math.sqrt(v_a / n_a + v_b / n_b)
    if se == 0.0:
        return 0.0, 1.0
    t = (m_a - m_b) / se
    num = (v_a / n_a + v_b / n_b) ** 2
    den = (v_a ** 2) / (n_a ** 2 * (n_a - 1)) + (v_b ** 2) / (n_b ** 2 * (n_b - 1))
    df = num / den if den > 0 else (n_a + n_b - 2)
    p = _student_t_two_sided_p(abs(t), df)
    return float(t), float(p)


def _student_t_two_sided_p(t_abs: float, df: float) -> float:
    """Two-sided p-value for Student-t. Closed-form via the
    regularized incomplete beta function I_x(a, b) with
    x = df/(df + t²), a = df/2, b = 1/2. p = I_x(a, b)."""
    if t_abs == 0:
        return 1.0
    if df <= 0:
        return 1.0
    x = df / (df + t_abs * t_abs)
    return _regularized_incomplete_beta(x, df / 2.0, 0.5)


def _regularized_incomplete_beta(x: float, a: float, b: float) -> float:
    """Numerically-stable I_x(a, b) via the Lentz continued-fraction
    for the incomplete beta function. Adapted from Numerical Recipes
    3e section 6.4."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    lbeta = (math.lgamma(a) + math.lgamma(b) - math.lgamma(a + b))
    bt = math.exp(a * math.log(x) + b * math.log(1.0 - x) - lbeta)
    if x < (a + 1.0) / (a + b + 2.0):
        return bt * _betacf(x, a, b) / a
    return 1.0 - bt * _betacf(1.0 - x, b, a) / b


def _betacf(x: float, a: float, b: float, max_iter: int = 200,
            eps: float = 3e-7) -> float:
    """Lentz CF for incomplete beta. Internal helper for the t-test."""
    qab = a + b
    qap = a + 1.0
    qam = a - 1.0
    c = 1.0
    d = 1.0 - qab * x / qap
    if abs(d) < 1e-30:
        d = 1e-30
    d = 1.0 / d
    h = d
    for m in range(1, max_iter + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        if abs(d) < 1e-30:
            d = 1e-30
        c = 1.0 + aa / c
        if abs(c) < 1e-30:
            c = 1e-30
        d = 1.0 / d
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        if abs(d) < 1e-30:
            d = 1e-30
        c = 1.0 + aa / c
        if abs(c) < 1e-30:
            c = 1e-30
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < eps:
            break
    return h
