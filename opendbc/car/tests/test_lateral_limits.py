#!/usr/bin/env python3
import unittest
import importlib
import numpy as np
from opendbc.testing import parameterized_class

from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.car_helpers import interfaces
from opendbc.car.interfaces import get_torque_params
from opendbc.car.lateral import ISO_LATERAL_ACCEL
from opendbc.car.values import PLATFORMS

# ISO 11270 - allowed up jerk is strictly lower than recommended limits
MAX_LAT_JERK_UP = 2.5            # m/s^3
MAX_LAT_JERK_DOWN = 5.0          # m/s^3
MAX_LAT_JERK_UP_TOLERANCE = 0.5  # m/s^3

# jerk is measured over half a second
JERK_MEAS_T = 0.5


@parameterized_class('car_model', [(c,) for c in sorted(PLATFORMS)])
class TestLateralLimits(unittest.TestCase):
  car_model: str

  @classmethod
  def setUpClass(cls):
    if 'car_model' not in cls.__dict__:
      raise unittest.SkipTest('Base class')

    CarInterface = interfaces[cls.car_model]
    CP = CarInterface.get_non_essential_params(cls.car_model)
    _ = CarInterface.get_non_essential_params_sp(CP, cls.car_model)

    if cls.car_model == 'MOCK':
      raise unittest.SkipTest('Mock car')

    # TODO: test all platforms
    if CP.steerControlType != 'torque':
      raise unittest.SkipTest

    if CP.notCar:
      raise unittest.SkipTest

    CarControllerParams = importlib.import_module(f'opendbc.car.{CP.brand}.values').CarControllerParams
    cls.control_params = CarControllerParams(CP)
    cls.torque_params = get_torque_params()[cls.car_model]

  @staticmethod
  def calculate_0_5s_jerk(control_params, torque_params):
    steer_step = control_params.STEER_STEP
    max_lat_accel = torque_params['MAX_LAT_ACCEL_MEASURED']

    # Steer up/down delta per 10ms frame, in percentage of max torque
    steer_up_per_frame = control_params.STEER_DELTA_UP / control_params.STEER_MAX / steer_step
    steer_down_per_frame = control_params.STEER_DELTA_DOWN / control_params.STEER_MAX / steer_step

    # Lateral acceleration reached in 0.5 seconds, clipping to max torque
    accel_up_0_5_sec = min(steer_up_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * max_lat_accel
    accel_down_0_5_sec = min(steer_down_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * max_lat_accel

    # Convert to m/s^3
    return accel_up_0_5_sec / JERK_MEAS_T, accel_down_0_5_sec / JERK_MEAS_T

  def test_jerk_limits(self):
    up_jerk, down_jerk = self.calculate_0_5s_jerk(self.control_params, self.torque_params)
    assert up_jerk <= MAX_LAT_JERK_UP + MAX_LAT_JERK_UP_TOLERANCE
    assert down_jerk <= MAX_LAT_JERK_DOWN

  def test_scheduled_rate_jerk_limits(self):
    """A speed scheduled rate up limit, checked per speed against the lateral accel that full torque was
    measured to produce at that speed. Uses panda's side of the schedule (1 m/s speed fudge), the larger one."""
    lookup = getattr(self.control_params, 'STEER_DELTA_UP_LOOKUP', None)
    if lookup is None:
      raise unittest.SkipTest('no rate schedule')

    max_lat_accel_measured = self.torque_params['MAX_LAT_ACCEL_MEASURED']
    la_bp, la_v = self.control_params.CANFD_LOW_SPEED_RATE_MAX_LAT_ACCEL
    la_bp, la_v = [*la_bp, 55 * CV.MPH_TO_MS], [*la_v, max_lat_accel_measured]
    steer_step = self.control_params.STEER_STEP
    for v in np.arange(0., 40., 0.25):
      rate = min(int(np.interp(v - 1., *lookup)), max(lookup[1]))
      max_lat_accel = float(np.interp(v, la_bp, la_v))
      steer_up_per_frame = rate / self.control_params.STEER_MAX / steer_step
      up_jerk = min(steer_up_per_frame * JERK_MEAS_T / DT_CTRL, 1.0) * max_lat_accel / JERK_MEAS_T
      assert up_jerk <= MAX_LAT_JERK_UP, f'{up_jerk:.2f} m/s^3 at {v:.2f} m/s (rate {rate})'

  def test_max_lateral_accel(self):
    assert self.torque_params["MAX_LAT_ACCEL_MEASURED"] <= ISO_LATERAL_ACCEL
