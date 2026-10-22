#!/usr/bin/env python3
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.hyundai import adrv_interceptor as ai

DBC = "hyundai_canfd_generated"


class FakeHud:
  def __init__(self, left=True, right=True):
    self.leftLaneVisible = left
    self.rightLaneVisible = right


def _lka_steering_cp():
  # Ioniq 5 gen5w is an LKA-steering CAN-FD car: ECAN is bus 1
  cp = structs.CarParams()
  cp.flags = HyundaiFlags.CANFD_LKA_STEER_MSG.value
  return cp


class TestAdrvInterceptor(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC)
    self.CAN = CanBus(_lka_steering_cp())

  def _parse(self, name, msg, signals):
    # round-trip the packed frame back to signal values on the bus it was sent on
    parser = CANParser(DBC, [(name, 0)], self.CAN.ECAN)
    parser.update([self.CAN.ECAN, [msg]])
    return {s: parser.vl[name][s] for s in signals}

  # ---- enable gate ----
  def test_enabled_flag(self):
    cp = structs.CarParams()
    for value in range(2 ** 12):
      cp_sp = structs.CarParamsSP()
      cp_sp.flags = value
      self.assertEqual(ai.AdrvInterceptor(cp, cp_sp).enabled, bool(value & HyundaiFlagsSP.ADRV_INTERCEPTOR))

  # ---- LFA (0x12A) ----
  def test_lfa_sends_on_ecan(self):
    msg = ai.create_lfa(self.packer, self.CAN, True, 100, 1, FakeHud())
    self.assertEqual(msg[0], ai.LFA_MSG)
    self.assertEqual(msg[2], self.CAN.ECAN)

  def test_lfa_carries_comma_steering(self):
    msg = ai.create_lfa(self.packer, self.CAN, True, 123, 1, FakeHud())
    vl = self._parse("LFA", msg, ["StrTqReqVal", "ActToiSta", "ToiFltSta"])
    self.assertEqual(round(vl["StrTqReqVal"]), 123)
    self.assertEqual(vl["ActToiSta"], 1)
    self.assertEqual(vl["ToiFltSta"], 0)

  def test_lfa_inactive_clears_toi(self):
    msg = ai.create_lfa(self.packer, self.CAN, False, 0, 0, FakeHud())
    vl = self._parse("LFA", msg, ["ActToiSta"])
    self.assertEqual(vl["ActToiSta"], 0)

  def test_lfa_cluster_lanes_follow_model(self):
    both = self._parse("LFA", ai.create_lfa(self.packer, self.CAN, True, 0, 1, FakeHud(True, True)),
                       ["LKA_LHLnWrnSta", "LKA_RHLnWrnSta", "LKA_RcgSta"])
    self.assertEqual(both["LKA_LHLnWrnSta"], ai.LANE_SHOWN)
    self.assertEqual(both["LKA_RHLnWrnSta"], ai.LANE_SHOWN)
    self.assertEqual(both["LKA_RcgSta"], ai.RCG_TRACKING)

    none = self._parse("LFA", ai.create_lfa(self.packer, self.CAN, True, 0, 1, FakeHud(False, False)),
                       ["LKA_LHLnWrnSta", "LKA_RHLnWrnSta", "LKA_RcgSta"])
    self.assertEqual(none["LKA_LHLnWrnSta"], ai.LANE_OFF)
    self.assertEqual(none["LKA_RHLnWrnSta"], ai.LANE_OFF)
    self.assertEqual(none["LKA_RcgSta"], ai.RCG_OFF)

  def test_lfa_preserves_adrv_warnings(self):
    adrv = {f: 1 for f in ai.ADRV_WARNING_FIELDS}
    msg = ai.create_lfa(self.packer, self.CAN, True, 50, 1, FakeHud(), adrv_lfa=adrv)
    vl = self._parse("LFA", msg, list(ai.ADRV_WARNING_FIELDS))
    for f in ai.ADRV_WARNING_FIELDS:
      self.assertEqual(vl[f], 1, f)

  def test_lfa_warnings_default_zero_without_capture(self):
    msg = ai.create_lfa(self.packer, self.CAN, True, 50, 1, FakeHud(), adrv_lfa=None)
    vl = self._parse("LFA", msg, list(ai.ADRV_WARNING_FIELDS))
    for f in ai.ADRV_WARNING_FIELDS:
      self.assertEqual(vl[f], 0, f)

  # ---- LFAHDA_CLUSTER (0x1E0) ----
  def test_lfahda_cluster(self):
    on = self._parse("LFAHDA_CLUSTER",
                     ai.create_lfahda_cluster(self.packer, self.CAN, True, 2), ["HDA_ICON", "LFA_ICON"])
    self.assertEqual(on["HDA_ICON"], 1)
    self.assertEqual(on["LFA_ICON"], 2)
    off = self._parse("LFAHDA_CLUSTER",
                      ai.create_lfahda_cluster(self.packer, self.CAN, False, 0), ["HDA_ICON"])
    self.assertEqual(off["HDA_ICON"], 0)

  # ---- capture latch ----
  def test_update_latches_last_seen(self):
    it = ai.AdrvInterceptor(structs.CarParams(), structs.CarParamsSP())
    self.assertIsNone(it.adrv_lfa)
    it.update({"FCA_LO_WrnSta": 1})
    self.assertEqual(it.adrv_lfa, {"FCA_LO_WrnSta": 1})
    it.update(None)  # a blocked/absent frame must not wipe the last good capture
    self.assertEqual(it.adrv_lfa, {"FCA_LO_WrnSta": 1})


class TestAdrvInterceptorIntegration(unittest.TestCase):
  """The flag flips the whole gen5w LKA-steering emission: comma 0x12A/0x1E0 on E-CAN,
  real lanes fed (no suppress), and the ADRV's 0x12A captured for its warning fields."""
  FP = "HYUNDAI_IONIQ_5"

  def _build(self, interceptor):
    from opendbc.car.hyundai.values import DBC
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.carcontroller import CarController
    from opendbc.car.hyundai.carstate import CarState
    CP = CarInterface.get_non_essential_params(self.FP)
    CP.flags |= HyundaiFlags.CANFD_LKA_STEER_MSG.value
    CP.openpilotLongitudinalControl = False
    CP_SP = structs.CarParamsSP()
    if interceptor:
      CP_SP.flags |= HyundaiFlagsSP.ADRV_INTERCEPTOR
    from opendbc.car import Bus
    return CarController(DBC[self.FP], CP, CP_SP), CarState(CP, CP_SP), CP, CP_SP, Bus

  def _emit(self, cc):
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.hudControl.leftLaneVisible = True
    CC.hudControl.rightLaneVisible = True

    class Shim:
      out = structs.CarState()
      is_metric = True
      adrv_lfa = {"FCA_LO_WrnSta": 1}
      main_cruise_enabled = False
      cruise_info: dict = {}
      buttons_counter = 0
      # CAM_0x2a4 copy the stock suppress path reads (BYTE3..BYTE23 except 7, + COUNTER)
      lfa_block_msg = {**{f"BYTE{i}": 0 for i in range(3, 24) if i != 7}, "COUNTER": 0}
      cruise_buttons = [0]
      main_buttons = [0]

    addrs = set()
    for f in range(10):
      cc.frame = f
      for addr, _dat, bus in cc.create_canfd_msgs(True, 100, 200, 0.0, False, CC.hudControl, Shim(), CC):
        addrs.add((addr, bus))
    return addrs

  def test_interceptor_emission(self):
    cc, _, _, _, _ = self._build(interceptor=True)
    addrs = self._emit(cc)
    ecan = cc.CAN.ECAN
    self.assertIn((ai.LFA_MSG, ecan), addrs)             # comma drives 0x12A on E-CAN
    self.assertIn((ai.LFAHDA_CLUSTER_MSG, ecan), addrs)  # comma drives the cluster icons
    self.assertFalse(any(a in (676, 866) for a, _ in addrs))  # no lane suppression (real lanes fed)

  def test_stock_emission_unchanged(self):
    cc, _, _, _, _ = self._build(interceptor=False)
    addrs = self._emit(cc)
    ecan = cc.CAN.ECAN
    self.assertNotIn((ai.LFA_MSG, ecan), addrs)  # stock: comma doesn't drive 0x12A on E-CAN (stock long off)
    self.assertTrue(any(a in (676, 866) for a, _ in addrs))  # stock still suppresses the camera lanes

  def test_parser_captures_adrv_lfa(self):
    _, cs, CP, CP_SP, Bus = self._build(interceptor=True)
    pt = cs.get_can_parsers(CP, CP_SP)[Bus.pt]
    self.assertIsNotNone(pt.vl["LFA"])  # LFA registered on the E-CAN parser


if __name__ == "__main__":
  unittest.main()
