#!/usr/bin/env python3
"""
Stepper status: the MODE_VEL_TRAJ error-bound and velocity-watchdog diag bits,
and the AS5047 frame counts in StatusAux. No hardware: the unpack methods run
on byte strings laid out as the firmware sends them.
"""
import struct

import pytest

from stretch4_body.core.stepper import Stepper, StepperAux, StepperBase


def status_bytes(diag):
    # mode, effort_ticks, pos, vel, err, diag, timestamp, debug, guarded_event,
    # two deprecated fields, voltage, temperature (Status in Common.h)
    return struct.pack('<BfdffIQfIfHff', 3, 0.0, 1.0, 0.0, 0.0, diag, 0, 0.0, 0, 0.0, 0, 0.0, 0.0)


class FakeStepper:
    """Just what _unpack_status needs."""
    DIAG_MIN_VERSION = Stepper.DIAG_MIN_VERSION
    _firmware_version_tuple = StepperBase._firmware_version_tuple
    _diag_if_supported = StepperBase._diag_if_supported
    _unpack_status = StepperBase._unpack_status

    def __init__(self, fw):
        self.board_info = {'firmware_version': fw}
        self.status = {}
        self.timestamp = type('T', (), {'set': staticmethod(lambda x: x)})()
        for k in dir(Stepper):
            if k.startswith('DIAG_') and k != 'DIAG_MIN_VERSION':
                setattr(self, k, getattr(Stepper, k))

    def effort_ticks_to_current(self, e):
        return 0.0

    def current_to_effort_pct(self, c):
        return 0.0

    def get_voltage(self, v):
        return v

    def get_temperature(self, t):
        return t


def unpack(fw, diag):
    st = FakeStepper(fw)
    st._unpack_status(status_bytes(diag))
    return st.status


def test_bits_read_on_firmware_that_sends_them():
    both = Stepper.DIAG_VEL_TRAJ_AT_ERROR_BOUND | Stepper.DIAG_VEL_WATCHDOG_EXPIRED
    s = unpack('hello-stepper2.v0.1.11p8', both)
    assert s['vel_traj_at_error_bound'] is True
    assert s['vel_watchdog_expired'] is True
    s = unpack('hello-stepper2.v0.1.11p8', 0)
    assert s['vel_traj_at_error_bound'] is False
    assert s['vel_watchdog_expired'] is False


def test_unknown_on_older_firmware():
    s = unpack('hello-stepper2.v0.1.8p8', 0)
    assert s['vel_traj_at_error_bound'] is None
    assert s['vel_watchdog_expired'] is None
    s = unpack('hello-stepper2.v0.1.10p8', Stepper.DIAG_VEL_TRAJ_AT_ERROR_BOUND)
    assert s['vel_traj_at_error_bound'] is True
    assert s['vel_watchdog_expired'] is None
    assert unpack(None, 0)['vel_watchdog_expired'] is None
    assert unpack('garbage', 0)['vel_watchdog_expired'] is None


def test_existing_bits_unchanged():
    s = unpack('hello-stepper2.v0.1.11p8', Stepper.DIAG_IS_MOVING | Stepper.DIAG_IS_TRACE_ON)
    assert s['is_moving'] and s['trace_on'] and not s['runstop_on']


class FakeAux:
    _StepperAux__unpack_status_aux = StepperAux._StepperAux__unpack_status_aux

    def __init__(self):
        self.status_aux = {}


def test_aux_frame_counts_from_a_20_byte_reply():
    a = FakeAux()
    n = a._StepperAux__unpack_status_aux(struct.pack('<6H2I', 1, 2, 3, 4, 5, 0, 7, 9))
    assert n == 20
    assert a.status_aux['cmd_cnt_rpc'] == 1 and a.status_aux['sync_irq_overflow'] == 5
    assert a.status_aux['enc_parity_err_cnt'] == 7
    assert a.status_aux['enc_ef_cnt'] == 9


def test_aux_frame_counts_none_from_an_older_reply():
    a = FakeAux()
    a._StepperAux__unpack_status_aux(struct.pack('<6H', 1, 2, 3, 4, 5, 0))
    assert a.status_aux['cmd_cnt_rpc'] == 1
    assert a.status_aux['enc_parity_err_cnt'] is None
    assert a.status_aux['enc_ef_cnt'] is None
