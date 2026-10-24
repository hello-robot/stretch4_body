"""
Fit of the stepper guarded contact line, effort = coeff_acc * a_des + coeff_intercept.

The calibration drives a joint at each motion profile and takes the peak
effort of each. A profile whose peak sits at the current clamp (iMax) did
not show how much effort that acceleration needs, only that it needed at
least the clamp, and one such point pulls the slope up several times: the
stored lift runs with a point at the clamp fit slopes of 55 to 58 against
7 to 13 without it. Those samples are dropped before the fit.
"""
import numpy as np


class ContactFitError(ValueError):
    pass


def fit_contact_line(acc, effort, clamp_ticks, margin=0.05):
    """
    acc: acceleration of each sample, in the units the firmware multiplies by
         (cmd.a_des, motor rad/s^2).
    effort: peak effort of each sample (ticks, skew already added). Signed.
    clamp_ticks: the effort clamp, iMax in ticks, positive.
    margin: samples within this fraction of the clamp are dropped.
    Returns (coeff_acc, coeff_intercept, kept), kept a list of bools.
    Raises ContactFitError if fewer than two samples are left.
    """
    acc = np.asarray(acc, dtype=float)
    effort = np.asarray(effort, dtype=float)
    kept = np.abs(effort) < (1.0 - margin) * abs(clamp_ticks)
    if kept.sum() < 2 or len(np.unique(acc[kept])) < 2:
        raise ContactFitError('fewer than two profiles below the current clamp; '
                              'lower the profiles or check the joint before calibrating')
    a, b = np.polyfit(acc[kept], effort[kept], 1)
    return float(a), float(b), kept.tolist()
