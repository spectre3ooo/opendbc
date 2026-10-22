"""Comma side of the Hyundai/Kia CAN-FD ADRV interceptor.

Pairs with the red-panda safety mode in
opendbc/safety/modes/hyundai_canfd_adrv_interceptor.h, which sits in-line at the ADAS
Driving ECU (ADRV) connector and *blocks* the ADRV's own copy of each message below while
the comma's copy is fresh (150 ms), failing safe to stock otherwise.

So for each intercepted address the comma must now produce the whole frame itself:

  0x12A LFA             steering to the rack + the cluster lane/status display + the
                        ADRV's safety-warning fields (which the comma does not compute)
  0x1E0 LFAHDA_CLUSTER  HDA / LFA cluster icons

Why this exists (the unified fix): sunnypilot suppresses the camera lane confidence
(CAM_0x2a4) so the ADRV's LFA won't fight the comma's steering. That same suppression makes
the ADRV believe the road is straight, which (a) flattens the cluster lane graphics and
(b) makes stock SCC lose the lead as an in-path target on curves (the curve surge). The
interceptor lets us instead feed the ADRV the comma's real lanes (handled on the camera
side, CAM_0x2a4) AND block the ADRV's steering output here, so the ADRV draws a correct
cluster / picks the right in-path target while the comma alone drives the rack.

SAFETY NOTE — 0x12A warning fields: 0x12A also carries forward-collision / blind-spot /
lane-keeping / pedestrian / evasive-steering *warning status* that the ADRV owns and the
comma cannot compute. Because the red panda blocks the ADRV's 0x12A whenever the comma's is
fresh, those fields have no live source while the comma is active; the best the comma can do
is carry the last value it saw (`adrv_lfa`). This is acceptable ONLY if those warnings are
also published on their own dedicated messages that the interceptor does NOT block (so the
cluster still shows them and AEB/FCA still actuate off those). THIS MUST BE VERIFIED ON-CAR
before relying on the interceptor for 0x12A. If they are 0x12A-only, the red panda must
re-address rather than block (a firmware change, not yet in mainline).
"""
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP

LFA_MSG = 0x12A
LFAHDA_CLUSTER_MSG = 0x1E0

# Fields on 0x12A that the ADRV owns as safety-warning status (not steering, not lane display).
# Preserved verbatim from the last ADRV frame the comma saw; see the SAFETY NOTE above.
ADRV_WARNING_FIELDS = (
  "BCA_Rear_WrnSta",     # blind-spot collision-avoidance rear warning
  "FCA_LO_WrnSta",       # forward-collision (leading object) warning
  "FCA_LS_WrnSta",       # forward-collision (lateral / cross) warning
  "Info_PedtrnDst",      # pedestrian distance indication
  "ELK_SysFlrSta",       # evasive lane-keeping system-failure status
  "ELK_SymbDisp",        # evasive lane-keeping symbol display
  "FCA_ESA_WrnSta",      # FCA evasive-steering-assist warning
  "FCA_ESA_CtrlSta",     # FCA evasive-steering-assist control status
  "LKA_OnOffEquip2Sta",  # LKA equipment status
)

# Cluster lane fields on 0x12A, from a stock-ADRV capture with real camera lanes (route 0000004c,
# 2026-10-22, comma passive). While stock LFA tracked the lanes, LKA_RcgSta was 3; it dropped to 0
# while crossing a line on a lane change. LKA_LHLnWrnSta/LKA_RHLnWrnSta stayed 0 the whole drive:
# they are lane-departure warnings, not "lane shown", so the comma leaves them at 0.
RCG_OFF = 0
RCG_TRACKING = 3      # lanes recognized, LFA tracking
LANE_NO_WARNING = 0


def cluster_lane_display(hud_control):
  """Cluster lane/recognition fields for 0x12A, driven by the comma's model."""
  return {
    "LKA_LHLnWrnSta": LANE_NO_WARNING,
    "LKA_RHLnWrnSta": LANE_NO_WARNING,
    "LKA_RcgSta": RCG_TRACKING if (hud_control.leftLaneVisible or hud_control.rightLaneVisible) else RCG_OFF,
    "LKA_SysWrn": 0,
  }


def create_lfa(packer, CAN, lat_active, apply_torque, lkas_icon, hud_control, adrv_lfa=None):
  """Build the comma's 0x12A (LFA) for the interceptor, on E-CAN.

  Carries the comma's steering (so the comma, not the ADRV, drives the rack — removing the
  2-frame drop/re-ramp), the cluster lane display from the comma's model, the LFA icon, and
  the ADRV's last-seen safety-warning fields.
  """
  values = {
    "LKA_OptUsmSta": 2,
    "LKA_UsmMod": 0,       # hide LKAS settings
    "LKA_SysIndReq": lkas_icon,
    "StrTqReqVal": apply_torque,
    "ActToiSta": 1 if lat_active else 0,
    "ToiFltSta": 0,
    "Damping_Gain": 100,
  }
  values.update(cluster_lane_display(hud_control))

  # preserve the ADRV's safety-warning fields (the comma doesn't compute them); see SAFETY NOTE
  if adrv_lfa is not None:
    for field in ADRV_WARNING_FIELDS:
      if field in adrv_lfa:
        values[field] = adrv_lfa[field]

  return packer.make_can_msg("LFA", CAN.ECAN, values)


def create_lfahda_cluster(packer, CAN, enabled, lfa_icon):
  """Build the comma's 0x1E0 (LFAHDA_CLUSTER) for the interceptor, on E-CAN."""
  values = {
    "HDA_ICON": 1 if enabled else 0,
    "LFA_ICON": lfa_icon,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)


class AdrvInterceptor:
  """Feature gate + captured ADRV state for the comma-side interceptor."""
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.adrv_lfa = None  # last ADRV 0x12A the comma saw (for the warning fields)

  @property
  def enabled(self):
    return bool(self.CP_SP.flags & HyundaiFlagsSP.ADRV_INTERCEPTOR)

  def update(self, adrv_lfa):
    """Latch the ADRV's own 0x12A while it is still visible (before/between comma frames)."""
    if adrv_lfa is not None:
      self.adrv_lfa = adrv_lfa


# *** longitudinal through the interceptor ***
# Stock openpilot long on LKA-steering cars knocks the ADAS Driving ECU out (0x730) and fakes its
# messages, which also removes AEB/FCA. Through the interceptor the ADRV stays alive: the red panda
# blocks only its SCC_CONTROL while the comma's is fresh. So the comma must not send the knockout,
# the ADRV fakes (0x160 is the ADRV's AEB path) or its own 0x12A/0x1E0 (the ADRV still sends those,
# relaying the comma's LKAS steering as with stock longitudinal).

def interceptor_long_eligible(CP: structs.CarParams) -> bool:
  return bool(CP.flags & HyundaiFlags.CANFD and CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and
              not CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT and CP.openpilotLongitudinalControl)


def enable_interceptor_long(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  """Opt in (from the sunnypilot side, before controls start). Returns whether it took effect."""
  if not interceptor_long_eligible(CP):
    return False
  CP_SP.flags |= HyundaiFlagsSP.ADRV_INTERCEPTOR_LONG.value
  CP_SP.safetyParam |= HyundaiSafetyFlagsSP.CANFD_ADRV_INTERCEPTOR_LONG
  return True


def interceptor_long(CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> bool:
  # getattr: CarInterface.deinit calls init without CP_SP
  return interceptor_long_eligible(CP) and bool(getattr(CP_SP, "flags", 0) & HyundaiFlagsSP.ADRV_INTERCEPTOR_LONG)
