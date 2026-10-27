import json
import os
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.adrv_failsafe import (AdrvInterceptorStatus, AdrvPresence, decode_status,
                                                          failsafe_watch_eligible)
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

S = 1_000_000_000
FIXTURE = os.path.join(os.path.dirname(__file__), "adrv_presence_fixture.json")


def vals(state, counter=0, flags=0, uptime=0, check=None, proto=1):
  d = [proto, state, counter, flags, uptime & 0xFF, (uptime >> 8) & 0xFF, (uptime >> 16) & 0xFF]
  return {"PROTOCOL": proto, "STATE": state, "FRAME_COUNTER": counter, "FLAGS": flags, "UPTIME": uptime,
          "FRAME_CHECK": ((sum(d) & 0xFF) ^ 0xA5) if check is None else check}


class TestDecode(unittest.TestCase):
  def test_states(self):
    for raw, name in enumerate(["starting", "engaged", "trippedTraffic", "trippedWatchdog", "canUpdate", "ignitionOff"]):
      self.assertEqual(name, decode_status(vals(raw, uptime=0x123456)))

  def test_rejects_bad_check_version_state(self):
    self.assertIsNone(decode_status(vals(1, check=0)))
    self.assertIsNone(decode_status(vals(1, proto=2)))
    self.assertIsNone(decode_status(vals(9)))

  def test_dbc_round_trip(self):
    packer = CANPacker("hyundai_canfd_generated")
    parser = CANParser("hyundai_canfd_generated", [("ADRV_INTERCEPTOR_STATUS", float("nan"))], 0)
    addr, dat, _ = packer.make_can_msg("ADRV_INTERCEPTOR_STATUS", 0, vals(2, counter=7, uptime=0xABCDEF))
    self.assertEqual(0x7F9, addr)
    parser.update([(1 * S, [(addr, dat, 0)])])
    self.assertEqual("trippedTraffic", decode_status(parser.vl["ADRV_INTERCEPTOR_STATUS"]))
    self.assertTrue(parser.can_valid)


class TestStatusTracker(unittest.TestCase):
  def test_none_until_a_valid_frame(self):
    t = AdrvInterceptorStatus()
    self.assertEqual("none", t.update(vals(1, check=0), 1 * S, 1 * S))
    self.assertEqual("engaged", t.update(vals(1, counter=1), 2 * S, 2 * S))

  def test_repeated_counter_is_ignored(self):
    t = AdrvInterceptorStatus()
    t.update(vals(1, counter=5), 1 * S, 1 * S)
    self.assertEqual("engaged", t.update(vals(2, counter=5), 2 * S, 2 * S))
    self.assertEqual("trippedTraffic", t.update(vals(2, counter=6), 3 * S, 3 * S))

  def test_stale_after_3_s(self):
    t = AdrvInterceptorStatus()
    t.update(vals(1, counter=1), 1 * S, 1 * S)
    self.assertEqual("engaged", t.update(vals(1, counter=1), 1 * S, 4 * S))
    self.assertEqual("none", t.update(vals(1, counter=1), 1 * S, 4 * S + 1))


class TestPresence(unittest.TestCase):
  def test_start_window(self):
    p = AdrvPresence()
    self.assertFalse(p.update(0, 10 * S))
    self.assertFalse(p.update(0, 15 * S - 1))
    self.assertTrue(p.update(0, 15 * S))

  def test_drop_after_seen(self):
    p = AdrvPresence()
    p.update(10 * S, 10 * S)
    self.assertFalse(p.update(10 * S, 11 * S - 1))
    self.assertTrue(p.update(10 * S, 11 * S))
    self.assertFalse(p.update(12 * S, 12 * S))

  def test_logged_drives(self):
    with open(FIXTURE) as f:
      data = json.load(f)
    for name, expect in (("bad", True), ("good", False)):
      p = AdrvPresence()
      results = [p.update(last, now) for now, last in data[name]]
      self.assertEqual(expect, results[-1], name)
      if not expect:
        self.assertFalse(any(results), name)


class TestEligibility(unittest.TestCase):
  def cp(self, flags, long=False, sp_flags=0):
    CP = structs.CarParams(carFingerprint=CAR.HYUNDAI_IONIQ_5, flags=int(flags), openpilotLongitudinalControl=long)
    return CP, structs.CarParamsSP(flags=int(sp_flags))

  def test_gen5w_lka_steer_stock_long(self):
    self.assertTrue(failsafe_watch_eligible(*self.cp(HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG)))

  def test_off_for_openpilot_long_without_interceptor(self):
    self.assertFalse(failsafe_watch_eligible(*self.cp(HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG, long=True)))

  def test_on_for_interceptor_long(self):
    self.assertTrue(failsafe_watch_eligible(*self.cp(HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG, long=True,
                                                     sp_flags=HyundaiFlagsSP.ADRV_INTERCEPTOR_LONG)))

  def test_off_for_other_layouts(self):
    for flags in (HyundaiFlags.CANFD, HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG | HyundaiFlags.CANFD_LKA_STEER_MSG_ALT,
                  HyundaiFlags.CANFD | HyundaiFlags.CANFD_LKA_STEER_MSG | HyundaiFlags.CANFD_CAMERA_SCC, 0):
      self.assertFalse(failsafe_watch_eligible(*self.cp(flags)), flags)


MS = 1_000_000


class TestCarStateWiring(unittest.TestCase):
  """The real HYUNDAI_IONIQ_5 CarInterface publishes the red panda's status (A-CAN) and ADRV presence (E-CAN)."""

  def _ci(self, op_long=False, interceptor_long=False):
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.sunnypilot.car.hyundai import adrv_interceptor
    CP = CarInterface.get_non_essential_params("HYUNDAI_IONIQ_5")
    # the gen5w Ioniq 5 (HDA2): ADRV-routed SCC, LKA steering message, standard buttons
    CP.flags |= HyundaiFlags.CANFD_LKA_STEER_MSG.value
    CP.flags &= ~(HyundaiFlags.CANFD_CAMERA_SCC.value | HyundaiFlags.CANFD_ALT_BUTTONS.value)
    CP.openpilotLongitudinalControl = op_long
    CP_SP = structs.CarParamsSP()
    if interceptor_long:
      adrv_interceptor.enable_interceptor_long(CP, CP_SP)
    return CarInterface(CP, CP_SP)

  def setUp(self):
    from opendbc.car import Bus
    from opendbc.car.hyundai.hyundaicanfd import CanBus
    from opendbc.car.hyundai.values import DBC
    self.Bus = Bus
    self.ci = self._ci()
    self.CAN = CanBus(self.ci.CP)
    self.packer = CANPacker(DBC["HYUNDAI_IONIQ_5"][Bus.pt])
    self.t = 1000 * MS
    self.n = 0

  def _frames(self, adrv=False, status=None, others=False):
    frames = []
    if adrv:
      frames.append(self.packer.make_can_msg("ADRV_0x160", self.CAN.ECAN, {"COUNTER": self.n % 256}))
    if status is not None:
      frames.append(self.packer.make_can_msg("ADRV_INTERCEPTOR_STATUS", self.CAN.ACAN, status))
    if others:
      # every message the car state reads, except the fail-safe's own, so canValid can only fail on those
      for cp in self.ci.can_parsers.values():
        for addr, st in cp.message_states.items():
          if addr in (0x160, 0x7F9) or st.ignore_alive:
            continue
          msg = cp.dbc.addr_to_msg[addr]
          counter = msg.sigs.get("COUNTER")
          values = {"COUNTER": self.n % (1 << counter.size)} if counter is not None else {}
          frames.append(self.packer.make_can_msg(msg.name, cp.bus, values))
    return frames

  def _step(self, dt=10 * MS, **kw):
    self.t += dt
    self.n += 1
    return self.ci.update([(self.t, self._frames(**kw))])

  def _run(self, seconds, **kw):
    for _ in range(round(seconds * 100)):
      ret, ret_sp = self._step(**kw)
    return ret, ret_sp

  def test_parsers(self):
    parsers = self.ci.can_parsers
    self.assertIn(self.Bus.adas, parsers)
    self.assertEqual(self.CAN.ACAN, parsers[self.Bus.adas].bus)
    self.assertTrue(parsers[self.Bus.adas].message_states[0x7F9].ignore_alive)
    self.assertTrue(parsers[self.Bus.pt].message_states[0x160].ignore_alive)

  def test_adrv_missing_then_present(self):
    _, ret_sp = self._run(5.1)
    self.assertTrue(ret_sp.adrvMissing)
    _, ret_sp = self._step(adrv=True)
    self.assertFalse(ret_sp.adrvMissing)

  def test_adrv_present_from_start(self):
    _, ret_sp = self._run(6, adrv=True)
    self.assertFalse(ret_sp.adrvMissing)

  def test_interceptor_status(self):
    _, ret_sp = self._step()
    self.assertEqual("none", ret_sp.adrvInterceptorState)
    _, ret_sp = self._step(status=vals(3, counter=1, uptime=42))
    self.assertEqual(structs.CarStateSP.AdrvInterceptorState.trippedWatchdog, ret_sp.adrvInterceptorState)
    _, ret_sp = self._run(3.1)
    self.assertEqual("none", ret_sp.adrvInterceptorState)

  def test_absent_failsafe_messages_keep_can_valid(self):
    self._step(others=True)  # the car state registers what it reads on its first update
    ret, ret_sp = self._run(11, others=True)  # past even a 1 Hz message's 10 s timeout
    self.assertTrue(ret_sp.adrvMissing)
    self.assertEqual("none", ret_sp.adrvInterceptorState)
    self.assertTrue(ret.canValid)
    self.assertFalse(ret.canTimeout)

  def test_off_for_openpilot_long_without_interceptor(self):
    self.ci = self._ci(op_long=True)
    self.assertNotIn(self.Bus.adas, self.ci.can_parsers)
    self.assertNotIn(0x160, self.ci.can_parsers[self.Bus.pt].message_states)
    _, ret_sp = self._run(6)
    self.assertFalse(ret_sp.adrvMissing)
    self.assertEqual("none", ret_sp.adrvInterceptorState)

  def test_on_for_interceptor_long(self):
    self.ci = self._ci(op_long=True, interceptor_long=True)
    self.assertIn(self.Bus.adas, self.ci.can_parsers)
    _, ret_sp = self._run(5.1)
    self.assertTrue(ret_sp.adrvMissing)
