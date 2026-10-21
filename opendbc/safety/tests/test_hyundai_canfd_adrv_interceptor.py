#!/usr/bin/env python3
import unittest

from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common

SAFETY_HYUNDAI_CANFD_ADRV_INTERCEPTOR = 65534
CAR_BUS, TAP_BUS, ADRV_BUS = 0, 1, 2
SCC_CONTROL = 0x1A0
TIMEOUT_US = 150000
# a sample of other E-CAN traffic that must always pass: wheel speeds, MDPS, LFA, LFA_ALT, CCNC HUD, ADRV status
OTHER_ADDRS = (0xA0, 0xEA, 0x12A, 0xCB, 0x161, 0x162, 0x1E0, 0x9A0, 0x7D0)


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

  def _op_scc(self):
    return self._fwd(CAR_BUS, SCC_CONTROL)

  def test_forwards_everything_without_openpilot(self):
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, SCC_CONTROL))
    for addr in OTHER_ADDRS:
      self.assertEqual(ADRV_BUS, self._fwd(CAR_BUS, addr), f"{addr=:#x}")
      self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, addr), f"{addr=:#x}")

  def test_tap_bus_never_forwarded(self):
    for addr in (SCC_CONTROL, *OTHER_ADDRS):
      self.assertEqual(-1, self._fwd(TAP_BUS, addr))
    self._op_scc()
    self.assertEqual(-1, self._fwd(TAP_BUS, SCC_CONTROL))

  def test_blocks_adrv_scc_while_openpilot_active(self):
    # openpilot's own SCC_CONTROL is never passed to the ADRV
    self.assertEqual(-1, self._op_scc())
    self.assertEqual(-1, self._fwd(ADRV_BUS, SCC_CONTROL))
    # everything else still flows both ways
    for addr in OTHER_ADDRS:
      self.assertEqual(ADRV_BUS, self._fwd(CAR_BUS, addr))
      self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, addr))

  def test_timeout_restores_adrv_scc(self):
    self._op_scc()
    self._advance(TIMEOUT_US)
    self.assertEqual(-1, self._fwd(ADRV_BUS, SCC_CONTROL))
    self._advance(1)
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, SCC_CONTROL))

  def test_continuous_openpilot_keeps_blocking(self):
    for _ in range(100):
      self._op_scc()
      self._advance(20000)  # 50 Hz
      self.assertEqual(-1, self._fwd(ADRV_BUS, SCC_CONTROL))

  def test_resumes_after_dropout(self):
    self._op_scc()
    self._advance(TIMEOUT_US + 1)
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, SCC_CONTROL))
    self._op_scc()
    self.assertEqual(-1, self._fwd(ADRV_BUS, SCC_CONTROL))

  def test_timer_wraparound(self):
    self.t = 0xFFFFFFFF - 50000
    self.safety.set_timer(self.t)
    self._op_scc()
    self._advance(100000)  # wraps past zero, still inside the timeout
    self.assertEqual(-1, self._fwd(ADRV_BUS, SCC_CONTROL))
    self._advance(TIMEOUT_US)
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, SCC_CONTROL))

  def test_no_blocking_at_boot(self):
    # a timer of 0 at boot must not look like a recent openpilot message
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, SCC_CONTROL))

  def test_mode_change_resets_state(self):
    self._op_scc()
    self.assertEqual(0, self.safety.set_safety_hooks(SAFETY_HYUNDAI_CANFD_ADRV_INTERCEPTOR, 0))
    self.assertEqual(CAR_BUS, self._fwd(ADRV_BUS, SCC_CONTROL))

  def test_no_tx_allowed(self):
    for bus in range(3):
      for addr in (SCC_CONTROL, *OTHER_ADDRS):
        self.assertFalse(self.safety.safety_tx_hook(common.make_msg(bus, addr, 32)))


if __name__ == "__main__":
  unittest.main()
