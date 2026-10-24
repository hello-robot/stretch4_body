#!/usr/bin/env python3
"""
Guarded contact line fit. The numbers are the stored BRI lift run on
stretch-se4-4032 (stretch_production_data_ii, 20260602132405): peak effort
per profile with the skew added, fitted then with the max profile pinned at
the clamp. No hardware.
"""
import pytest

from stretch4_body.core.contact_fit import ContactFitError, fit_contact_line

ACC = [0.2 * 57.2958, 0.3 * 57.2958, 0.5 * 57.2958, 1.0 * 57.2958]   # as that run sampled them
EFF_4032 = [1581.588, 1724.376, 1812.536, 4056.418]
CLAMP_6p7A = 6.7 * 526


def test_sample_at_the_clamp_is_dropped():
    a, b, kept = fit_contact_line(ACC, EFF_4032, CLAMP_6p7A)
    assert kept == [True, True, True, False]
    assert a == pytest.approx(12.6, abs=0.1)
    print(f'4032 lift: slope {a:.1f}, intercept {b:.0f} (stored fit 55.5, 703)')


def test_clean_runs_fit_as_before():
    eff = [1580.0, 1650.0, 1770.0, 1865.0]
    a, b, kept = fit_contact_line(ACC, eff, CLAMP_6p7A)
    assert all(kept)
    import numpy as np
    a0, b0 = np.polyfit(ACC, eff, 1)
    assert a == pytest.approx(a0) and b == pytest.approx(b0)


def test_negative_side_uses_the_magnitude():
    a, b, kept = fit_contact_line(ACC, [-200.0, -230.0, -260.0, -3500.0], CLAMP_6p7A)
    assert kept == [True, True, True, False]


def test_refuses_with_fewer_than_two_good_profiles():
    with pytest.raises(ContactFitError):
        fit_contact_line(ACC, [1500.0, 3400.0, 3500.0, 3524.0], CLAMP_6p7A)
