#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common

SAFETY_HYUNDAI_CANFD_ADRV_INTERCEPTOR = 65534
CAR_BUS, TAP_BUS, ADRV_BUS = 0, 1, 2
TIMEOUT_US = 150000
# the intercepted set (gated independently on the comma's presence)
INTERCEPTED = (0x1A0, 0x12A, 0x1E0)
# a sample of traffic that must always pass untouched, both ways
PASSTHROUGH = (0xA0, 0xEA, 0x161, 0x9A0, 0x7D0, 0x50)


class TestHyundaiCanfdAdrvInterceptor(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.init_tests()
    self.assertEqual(0, self.safety.set_safety_hooks(SAFETY_HYUNDAI_CANFD_ADRV_INTERCEPTOR, 0))
    self.t = 0
    self.safety.set_timer(self.t)

  def _advance(self, us):
    self.t = (self.t + us) & 0xFFFFFFFF
    self.safety.set_timer(self.t)

  def _fwd(self, bus, addr):
    return self.safety.safety_fwd_hook(bus, addr)

  def _op(self, addr):
    # comma's copy on the car side -> notes presence, never forwarded to the ADRV
    return self._fwd(CAR_BUS, addr)

  def test_forwards_everything_without_comma(self):
    for addr in PASSTHROUGH:
      self.assertEqual(ADRV_BUS, self._fwd(CAR_BUS, addr), f"{addr=:#x}")
      self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, addr), f"{addr=:#x}")
    # intercepted msgs flow from the ADRV when the comma isn't sending them
    for addr in INTERCEPTED:
      self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, addr), f"{addr=:#x}")

  def test_tap_bus_never_forwarded(self):
    for addr in (*INTERCEPTED, *PASSTHROUGH):
      self.assertEqual(-1, self._fwd(TAP_BUS, addr))

  def test_each_intercept_is_independent(self):
    # activate only 0x12A; others still pass from the ADRV
    self._op(0x12A)
    self.assertEqual(-1, self._fwd(ADRV_BUS, 0x12A))
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, 0x1A0))
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, 0x1E0))
    # passthrough unaffected
    for addr in PASSTHROUGH:
      self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, addr))

  def test_comma_copy_never_reaches_adrv(self):
    for addr in INTERCEPTED:
      self.assertEqual(-1, self._op(addr))

  def test_block_while_comma_active(self):
    for addr in INTERCEPTED:
      self._op(addr)
      self.assertEqual(-1, self._fwd(ADRV_BUS, addr), f"{addr=:#x}")

  def test_timeout_restores_each(self):
    for addr in INTERCEPTED:
      self._op(addr)
    self._advance(TIMEOUT_US)
    for addr in INTERCEPTED:
      self.assertEqual(-1, self._fwd(ADRV_BUS, addr))
    self._advance(1)
    for addr in INTERCEPTED:
      self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, addr))

  def test_continuous_comma_keeps_blocking(self):
    for _ in range(100):
      for addr in INTERCEPTED:
        self._op(addr)
      self._advance(20000)  # 50 Hz
      for addr in INTERCEPTED:
        self.assertEqual(-1, self._fwd(ADRV_BUS, addr))

  def test_resumes_after_dropout(self):
    self._op(0x12A)
    self._advance(TIMEOUT_US + 1)
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, 0x12A))
    self._op(0x12A)
    self.assertEqual(-1, self._fwd(ADRV_BUS, 0x12A))

  def test_timer_wraparound(self):
    self.t = 0xFFFFFFFF - 50000
    self.safety.set_timer(self.t)
    self._op(0x1A0)
    self._advance(100000)  # wraps past zero, still within the timeout
    self.assertEqual(-1, self._fwd(ADRV_BUS, 0x1A0))
    self._advance(TIMEOUT_US)
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, 0x1A0))

  def test_no_blocking_at_boot(self):
    for addr in INTERCEPTED:
      self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, addr))

  def test_mode_change_resets_state(self):
    for addr in INTERCEPTED:
      self._op(addr)
    self.assertEqual(0, self.safety.set_safety_hooks(SAFETY_HYUNDAI_CANFD_ADRV_INTERCEPTOR, 0))
    for addr in INTERCEPTED:
      self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, addr))

  def test_no_tx_allowed(self):
    for bus in range(3):
      for addr in (*INTERCEPTED, *PASSTHROUGH):
        self.assertFalse(self.safety.safety_tx_hook(common.make_msg(bus, addr, 32)))


if __name__ == "__main__":
  unittest.main()
