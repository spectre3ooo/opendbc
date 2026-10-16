#!/usr/bin/env python3
import unittest

import numpy as np

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py

# see VEHICLE_SPEED_FACTOR in declarations.h
VEHICLE_SPEED_FACTOR = 1000.0

# SteeringControlType
TORQUE_MOTOR_LIMITED = 0
TORQUE_DRIVER_LIMITED = 1

# The lookup handed to test_steer_torque_rate_up_limits() in tests/libsafety/safety.c is
#   {{BP_LOW, BP_HIGH, BP_HIGH}, {RATE_LOW, RATE_HIGH, RATE_HIGH}}
# which safety_interpolate() holds flat outside [BP_LOW, BP_HIGH].
BP_LOW, BP_HIGH = 5., 15.
RATE_LOW, RATE_HIGH = 12., 2.

# The ceiling sits strictly between the two ends of the schedule on purpose: below BP_LOW the
# ceiling is what binds, above BP_HIGH the schedule is what binds, and in between they cross.
MAX_RATE_UP = 8

# 12. and 14. sit strictly inside the ramp, more than the 1 m/s speed fudge away from a
# breakpoint, so a wrong fudge always changes the integer result
SPEEDS = (0., 2., 6., 9., 11., 12., 14., 16., 20., 35.)


class TestTorqueRateUpLimits(unittest.TestCase):
  """
  Speed scheduled torque rate up limits (TorqueSteeringLimits.dynamic_max_rate_up) have no
  production safety mode yet, so the generic mechanism in lateral.h is exercised here through a
  test only entry point rather than by wiring a no-op schedule into a shipped car.
  """

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.noOutput, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)

  def _set_speed(self, speed):
    raw = round(speed * VEHICLE_SPEED_FACTOR)
    self.safety.set_vehicle_speed(raw, raw)

  @staticmethod
  def _expected_rate_up(speed):
    # mirrors safety_interpolate() plus the 1 m/s speed fudge, then the ceiling
    rate = int(np.interp(speed - 1., [BP_LOW, BP_HIGH], [RATE_LOW, RATE_HIGH]))
    return min(rate, MAX_RATE_UP)

  def _violation(self, torque, control_type, prev_torque=0):
    self.safety.set_desired_torque_last(prev_torque)
    self.safety.set_rt_torque_last(prev_torque)
    self.safety.set_torque_meas(prev_torque, prev_torque)
    self.safety.set_torque_driver(0, 0)
    return self.safety.test_steer_torque_rate_up_limits(torque, 1, control_type, MAX_RATE_UP,
                                                        BP_LOW, BP_HIGH, RATE_LOW, RATE_HIGH)

  def test_rate_up_follows_the_speed_schedule(self):
    for control_type in (TORQUE_MOTOR_LIMITED, TORQUE_DRIVER_LIMITED):
      for speed in SPEEDS:
        self._set_speed(speed)
        rate_up = self._expected_rate_up(speed)
        for sign in (1, -1):
          msg = f"{control_type=} {speed=} {rate_up=} {sign=}"
          self.assertFalse(self._violation(sign * rate_up, control_type), msg)
          self.assertTrue(self._violation(sign * (rate_up + 1), control_type), msg)

  def test_schedule_never_exceeds_max_rate_up(self):
    # below BP_LOW the schedule asks for RATE_LOW, which is above MAX_RATE_UP. The scalar limit
    # must win at every speed, so a schedule can only ever tighten the rate, never loosen it.
    self.assertGreater(RATE_LOW, MAX_RATE_UP)
    for control_type in (TORQUE_MOTOR_LIMITED, TORQUE_DRIVER_LIMITED):
      for speed in SPEEDS:
        self._set_speed(speed)
        msg = f"{control_type=} {speed=}"
        self.assertLessEqual(self._expected_rate_up(speed), MAX_RATE_UP, msg)
        self.assertTrue(self._violation(MAX_RATE_UP + 1, control_type), msg)
        self.assertTrue(self._violation(-MAX_RATE_UP - 1, control_type), msg)

  def test_missing_speed_signal_holds_the_low_speed_point(self):
    # a mode that enables the schedule without calling UPDATE_VEHICLE_SPEED() sits at 0, which
    # is the most permissive end of the schedule, and is still bounded by max_rate_up
    for control_type in (TORQUE_MOTOR_LIMITED, TORQUE_DRIVER_LIMITED):
      self.assertEqual(self._expected_rate_up(0.), MAX_RATE_UP)
      self.assertFalse(self._violation(MAX_RATE_UP, control_type), control_type)
      self.assertTrue(self._violation(MAX_RATE_UP + 1, control_type), control_type)


if __name__ == "__main__":
  unittest.main()
