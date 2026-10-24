#!/usr/bin/env python3
from opendbc.testing import parameterized_class
import unittest

from opendbc.car.hyundai.values import CarControllerParams, HyundaiSafetyFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]


class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  def _torque_driver_msg(self, torque):
    values = {"MDPS_StrTqSnsrVal": torque}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"StrTqReqVal": torque, "ActToiSta": steer_req}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 0.03125 for pos in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)

  def _acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.PT_BUS, values)

  def _main_cruise_button_msg(self, enabled):
    return self._button_msg(0, enabled)


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdBase):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)
    self.safety.init_tests()


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass


class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  SAFETY_PARAM: int

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      No button send allowed with alt buttons.
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # FIXME: the CANFD_ALT_BUTTONS cars are the only ones that use SCC_CONTROL to cancel, why can't we use buttons?
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False)))


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass


class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdBase):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringEVLowSpeedRate(TestHyundaiCanfdLKASteeringEV):
  """Ioniq 5: torque rate up scheduled on speed (HyundaiSafetyFlagsSP.CANFD_LOW_SPEED_RATE)"""

  # taken from the car side, so these tests also prove panda's table matches what openpilot sends
  MAX_RATE_UP_LOOKUP = CarControllerParams.CANFD_LOW_SPEED_RATE_LOOKUP
  MAX_RATE_UP = max(MAX_RATE_UP_LOOKUP[1])
  DYNAMIC_MAX_RATE_UP = True
  MAX_RT_DELTA = 150

  # the base _speed_msg takes raw wheel speed units (it only gates standstill); the schedule needs m/s
  WHEEL_SPEED_KPH_PER_RAW = 0.03125

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.CANFD_LOW_SPEED_RATE)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def tearDown(self):
    # the SP param is global state in libsafety; do not leak it into the next test class
    self.safety.set_current_safety_param_sp(0)

  def _raw_speed(self, speed):
    return round(speed * 3.6 / self.WHEEL_SPEED_KPH_PER_RAW)

  def _reset_speed_measurement(self, speed):
    for _ in range(common.MAX_SAMPLE_VALS):
      self._rx(self._speed_msg(self._raw_speed(speed)))

  def _get_max_rate_up(self, speed):
    # the wheel speed signal quantises the speed panda sees
    return super()._get_max_rate_up(self._raw_speed(speed) * self.WHEEL_SPEED_KPH_PER_RAW / 3.6)

  def test_rate_schedule_endpoints(self):
    """Doubled rate at low speed, stock rate of 2 from 35 mph up, and the flag is what enables it"""
    for speed, rate in ((0., 4), (5., 4), (9.9, 4), (16.6, 2), (30., 2)):
      self._reset_speed_measurement(speed)
      self.safety.set_controls_allowed(True)
      self._set_prev_torque(0)
      self.assertTrue(self._tx(self._torque_cmd_msg(rate)), speed)
      self._set_prev_torque(0)
      self.assertFalse(self._tx(self._torque_cmd_msg(rate + 1)), speed)

    self.safety.set_current_safety_param_sp(0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()
    self._reset_speed_measurement(5.)
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._torque_cmd_msg(3)), "without the flag the stock rate of 2 applies at every speed")
class TestHyundaiCanfdLKASteeringLaneFeedEV(TestHyundaiCanfdLKASteeringEV):
  # sunnypilot: the comma replaces the camera's lane geometry, so CAM_0x2A2/0x2A3 are blocked like CAM_0x2A4

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x2A2, 0], [0x2A3, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4, 0x2a2, 0x2a3)}
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4, 0x2a2, 0x2a3]}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.CANFD_LANE_FEED)
    super().setUp()

  def tearDown(self):
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.DEFAULT)

  def test_lane_feed_requires_flag(self):
    # without the flag the camera's lane geometry is forwarded and the comma can't send its own
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.DEFAULT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.safety.get_current_safety_param())
    for addr in (0x2A2, 0x2A3):
      self.assertEqual(0, self.safety.safety_fwd_hook(2, addr))
      self.assertFalse(self._tx(common.make_msg(0, addr, 32)))


# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEV(TestHyundaiCanfdBase):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)
class TestHyundaiCanfdLKASteeringLongAdrvInterceptorEV(TestHyundaiCanfdLKASteeringLongEV):
  """sunnypilot: op long through the red-panda ADRV interceptor. The ADRV stays alive for AEB/FCA: no
  0x730 knockout and no ADRV fakes (0x160 is its AEB path). Steering stays on LKAS, relayed by the ADRV."""

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x1a0, 1]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # the ADRV's 0x1A0 is legitimately on E-CAN
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  STEER_MSG = "LKAS"
  STEER_BUS = 0

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.CANFD_ADRV_INTERCEPTOR_LONG)
    super().setUp()
    self.t = 0
    self.safety.set_timer(self.t)

  def tearDown(self):
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.DEFAULT)

  def _advance(self, us):
    self.t += us
    self.safety.set_timer(self.t)

  def _adrv_scc(self):
    return self._rx(common.make_msg(1, 0x1a0, 32))

  def _send_scc(self):
    return self._tx(self._accel_msg(0))

  def test_tester_present_allowed(self, ecu_disable: bool = False):
    # the ADRV must not be knocked out: it provides AEB
    super().test_tester_present_allowed(ecu_disable=False)

  def test_no_adrv_fakes(self):
    for addr in (0x730, 0x160, 0x51, 0x1ea, 0x200, 0x345, 0x1da, 0x12a, 0x1e0):
      self.assertFalse(self._tx(common.make_msg(1, addr, 32)), f"{addr=:#x}")
      self.assertFalse(self._tx(common.make_msg(0, addr, 32)), f"{addr=:#x}")

  def test_disabled_ecu_alive(self):
    # the ADRV being alive is the point: its SCC_CONTROL alone is not a relay malfunction
    self._adrv_scc()
    self.assertFalse(self.safety.get_relay_malfunction())
    self.assertTrue(self._send_scc())

  def test_watchdog_ignores_adrv_scc_before_we_send(self):
    for _ in range(50):
      self._adrv_scc()
      self._advance(20000)
    self.assertTrue(self._send_scc())

  def test_watchdog_handover_grace(self):
    # the red panda blocks from the ADRV's next frame: one ADRV frame right after we start is fine
    self.assertTrue(self._send_scc())
    self._advance(20000)
    self._adrv_scc()
    self._advance(20000)
    self.assertTrue(self._send_scc())

  def test_watchdog_trips_when_interceptor_not_blocking(self):
    self.safety.set_controls_allowed(True)
    for _ in range(10):  # 200 ms of ours at 50 Hz
      self.assertTrue(self._send_scc())
      self._advance(20000)
    self._adrv_scc()  # the ADRV's still reaches E-CAN: two SCC sources
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._send_scc())
    # latched for the drive
    self._advance(10_000_000)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._send_scc())
    # steering is unaffected
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))

  def test_watchdog_allows_fallback_after_we_stop(self):
    for _ in range(10):
      self.assertTrue(self._send_scc())
      self._advance(20000)
    self._advance(150001)  # we stopped: the red panda times out and passes the ADRV's again
    for _ in range(10):
      self._adrv_scc()
      self._advance(20000)
    self.assertTrue(self._send_scc())  # new stretch of ours, new grace
    self._advance(20000)
    self._adrv_scc()
    self.assertTrue(self._send_scc())

  def test_watchdog_reset_on_init(self):
    for _ in range(10):
      self._send_scc()
      self._advance(20000)
    self._adrv_scc()
    self.assertFalse(self._send_scc())
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.safety.get_current_safety_param())
    self.assertTrue(self._send_scc())

  def test_expired_scc_watchdog_does_not_reappear_after_full_timer_period(self):
    for _ in range(10):
      self.assertTrue(self._send_scc())
      self._advance(20000)
    last_tx = self.t - 20000
    self._advance(150001)
    self._adrv_scc()  # observed stock fallback after expiry
    self.safety.set_timer(last_tx + 1)  # one full timer period after the last TX
    self._adrv_scc()
    self.assertTrue(self._send_scc())

  def test_scc_freshness_window_is_under_red_panda_timeout(self):
    # Ours expires at 120 ms, under the red panda's 150 ms: a send gap the red panda treats as a
    # stop (passing the ADRV's SCC_CONTROL again) must end our stretch too, not latch a fault.
    for gap, faults in ((120000, True), (120001, False)):
      self.setUp()
      for _ in range(10):
        self.assertTrue(self._send_scc())
        self._advance(20000)
      self._advance(gap - 20000)
      self._adrv_scc()
      self.assertEqual(not faults, self._send_scc(), gap)

  def test_scc_grace_does_not_reopen_after_full_timer_period(self):
    # A transmit stretch longer than one timer period must not get a second grace window.
    step = 110000
    target = step + (1 << 32) + 30000  # 30 ms into the wrapped grace window of the first TX
    elapsed = 0
    while elapsed + step <= target:
      elapsed += step
      self.safety.set_timer(elapsed & 0xFFFFFFFF)
      self.assertTrue(self._send_scc())
    self.safety.set_timer(target & 0xFFFFFFFF)
    self._adrv_scc()
    self.assertFalse(self._send_scc())

  def test_low_speed_rate_schedule_applies_to_lkas(self):
    # the Ioniq 5 runs the speed-scheduled torque rate (CANFD_LOW_SPEED_RATE); with the interceptor
    # it must limit LKAS, which is what steers: 4 counts/frame allowed at low speed, 2 without the schedule
    for sp, allowed in ((HyundaiSafetyFlagsSP.CANFD_ADRV_INTERCEPTOR_LONG | HyundaiSafetyFlagsSP.CANFD_LOW_SPEED_RATE, True),
                        (HyundaiSafetyFlagsSP.CANFD_ADRV_INTERCEPTOR_LONG, False)):
      self.safety.set_current_safety_param_sp(sp)
      self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.safety.get_current_safety_param())
      for _ in range(common.MAX_SAMPLE_VALS):
        self._rx(self._speed_msg(round(5.0 * 3.6 / 0.03125)))  # 5 m/s
      self.safety.set_controls_allowed(True)
      self._set_prev_torque(0)
      self.assertEqual(allowed, self._tx(self._torque_cmd_msg(4)), f"{sp=}")

  def test_flag_requires_lka_steering_long(self):
    # without op long, the flag doesn't allow sending SCC_CONTROL or change the TX list
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS)
    self.assertFalse(self._tx(self._accel_msg(0)))
    self.assertTrue(self._tx(common.make_msg(0, 0x2a4, 24)))


# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
class TestHyundaiCanfdLFASteeringLongBase(HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x160]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1a0, 0x160)}  # LFA, LFAHDA_CLUSTER, SCC_CONTROL, ADRV_0x160

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongBase":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  # no knockout
  def test_tester_present_allowed(self):
    pass


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      cls.safety = None
      raise unittest.SkipTest


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


if __name__ == "__main__":
  unittest.main()
