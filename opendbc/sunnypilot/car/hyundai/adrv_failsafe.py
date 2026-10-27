"""Comma side of the red panda fail-safe.

The red panda (panda branch adrv-interceptor, board/adrv_interceptor/failsafe*.h) releases the harness
box to pass-through when it hangs or stops forwarding the ADRV, and reports its state on A-CAN in
ADRV_INTERCEPTOR_STATUS (0x7F9) once a second while the ignition is on. Independently, the comma checks
that the ADRV is on E-CAN at all (ADRV_0x160, which the red panda forwards in every mode), because a red
panda that hangs before its fail-safe acts can't report anything.
Spec: panda docs/superpowers/specs/2026-10-26-red-panda-failsafe-design.md
"""
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.adrv_interceptor import interceptor_long

STATUS_STATES = ("starting", "engaged", "trippedTraffic", "trippedWatchdog", "canUpdate", "ignitionOff")
STATUS_STALE_NS = 3_000_000_000
START_WINDOW_NS = 5_000_000_000
MISSING_NS = 1_000_000_000


def decode_status(vals) -> str | None:
  d = [int(vals["PROTOCOL"]), int(vals["STATE"]), int(vals["FRAME_COUNTER"]), int(vals["FLAGS"])]
  uptime = int(vals["UPTIME"])
  d += [uptime & 0xFF, (uptime >> 8) & 0xFF, (uptime >> 16) & 0xFF]
  if d[0] != 1 or ((sum(d) & 0xFF) ^ 0xA5) != int(vals["FRAME_CHECK"]) or d[1] >= len(STATUS_STATES):
    return None
  return STATUS_STATES[d[1]]


class AdrvInterceptorStatus:
  """The red panda's last valid status, or 'none' (not fitted, ignition off on its side, or silent)."""
  def __init__(self):
    self.state = "none"
    self._last_ts = 0
    self._valid_ts = 0
    self._last_counter = None

  def update(self, vals, ts_nanos, now_nanos) -> str:
    if ts_nanos > 0 and ts_nanos != self._last_ts:
      self._last_ts = ts_nanos
      st = decode_status(vals)
      counter = int(vals["FRAME_COUNTER"])
      if st is not None and counter != self._last_counter:
        self._last_counter = counter
        self.state = st
        self._valid_ts = ts_nanos
    if self._valid_ts == 0 or now_nanos - self._valid_ts > STATUS_STALE_NS:
      self.state = "none"
    return self.state


class AdrvPresence:
  """The ADRV on E-CAN: missing if ADRV_0x160 hasn't shown up within START_WINDOW_NS of the first update,
  or hasn't been seen for MISSING_NS."""
  def __init__(self):
    self.missing = False
    self._first = None

  def update(self, adrv_nanos, now_nanos) -> bool:
    self._first = now_nanos if self._first is None else self._first
    if adrv_nanos == 0:
      self.missing = now_nanos - self._first >= START_WINDOW_NS
    else:
      self.missing = now_nanos - adrv_nanos >= MISSING_NS
    return self.missing


def failsafe_watch_eligible(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  """gen5w LKA-steering layout with the ADRV meant to be alive (stock long, or interceptor long)."""
  layout = bool(CP.flags & HyundaiFlags.CANFD and CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and
                not CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT and not CP.flags & HyundaiFlags.CANFD_CAMERA_SCC)
  return layout and (not CP.openpilotLongitudinalControl or interceptor_long(CP, CP_SP))
