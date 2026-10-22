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
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

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

# Cluster lane-display enum values on 0x12A. PROVISIONAL — these must be validated against a
# stock-ADRV 0x12A capture taken with real lane confidence (dashcam mode / interceptor lane
# feed). The suppression path zeroes the camera lanes, so every existing log shows these
# flatlined at 0 and cannot confirm the "lanes shown" values. Tunable here once captured.
LANE_OFF = 0
LANE_SHOWN = 2        # a lane line drawn on the cluster
RCG_OFF = 0
RCG_TRACKING = 2      # lanes recognized (HDA actively tracking)


def cluster_lane_display(hud_control):
  """Cluster lane/recognition fields for 0x12A, driven by the comma's model.

  Replaces the flatlined-at-0 fields the stock path produces under lane suppression, so the
  cluster can draw the lanes the comma actually sees. Enum values are PROVISIONAL (see above).
  """
  return {
    "LKA_LHLnWrnSta": LANE_SHOWN if hud_control.leftLaneVisible else LANE_OFF,
    "LKA_RHLnWrnSta": LANE_SHOWN if hud_control.rightLaneVisible else LANE_OFF,
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
