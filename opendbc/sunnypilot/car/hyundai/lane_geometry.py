"""Camera lane-geometry messages (CAM_0x2a2 / CAM_0x2a3) for HDA2 CAN-FD, built from the comma's model.

The front camera publishes its lane lines to the ADAS Driving ECU (ADRV) on its A-CAN at 20 Hz:

  0x2a2 CAM_0x2a2  ego lane lines (left/right)
  0x2a3 CAM_0x2a3  adjacent (outer) lane lines, same encoding with an ADJ_ prefix
  0x2a4 CAM_0x2a4  per-line confidence (LEFT/RIGHT_LANE_LINE), line type/colour, view range

Each line is a cubic y(x) = pos + hdg*x + crv/2*x^2 + dcrv/6*x^3 (x forward, y the same sign
convention as modelV2 laneLines: left lines negative). The 0x2a2/0x2a3 layout was reverse-engineered
from logs against FR_CMR_03_50ms (0x1b5, the camera's labelled copy of the same lines) and modelV2.
Re-packing real camera frames from these signals is byte-exact, CRC included. Position scale is
confirmed. Heading, curvature and curvature-rate scales are good to about +-10% and still
provisional (see the DBC comments).

This module only encodes. Getting the frames to the ADRV in place of the camera's needs the
interceptor on the camera's A-CAN path, which is not built yet.
"""
import numpy as np

CAM_LANE_EGO_MSG = 0x2A2
CAM_LANE_ADJ_MSG = 0x2A3

# fit range for the model lane cubic; past ~50 m the model lines get noisy and the camera's
# own fit matches them worse (measured on the Ioniq 5 logs)
FIT_MAX_X = 50.

# DBC scale and offset per field (raw = (value - offset) / scale, 16-bit unsigned)
FIELDS = {
  "POSITION": (0.01, -10.),
  "HEADING": (0.000976563, -0.356445),
  "CURVATURE": (2e-06, -0.065506),
  "CURVATURE_RATE": (2e-08, -0.00065422),
}
# the camera never sends above this raw value (0xFF87); it clamps rather than wrapping
RAW_MAX = 65415

# What the camera sends when it has no line, as physical values. Ego: its "zero" defaults
# (raw 1000/365/32753/32707). Adjacent: every field raw 0 (98% of 96k absent-line frames in the logs).
NO_LINE = {"POSITION": 0., "HEADING": 0., "CURVATURE": 0., "CURVATURE_RATE": -4 * 2e-08}
NO_ADJ_LINE = {k: offset for k, (_, offset) in FIELDS.items()}


def clamp(field, value):
  """Clamp to what the field can carry, like the camera: out-of-range values must not wrap
  around to the opposite extreme (e.g. a sharp curve read as a sharp curve the other way)."""
  scale, offset = FIELDS[field]
  return min(max(value, offset), offset + RAW_MAX * scale)


def fit_line(x, y, max_x=FIT_MAX_X):
  """Cubic fit of one model lane line -> (position m, heading rad, curvature 1/m, curvature rate 1/m^2)."""
  x = np.asarray(x, dtype=float)
  y = np.asarray(y, dtype=float)
  m = x < max_x
  c3, c2, c1, c0 = np.polyfit(x[m], y[m], 3)
  return {"POSITION": c0, "HEADING": c1, "CURVATURE": 2. * c2, "CURVATURE_RATE": 6. * c3}


def _side_values(prefix, side, line):
  return {f"{prefix}{side}_{k}": clamp(k, float(v)) for k, v in line.items()}


def lane_geometry_values(x, left_y, right_y, left_valid, right_valid, adjacent=False):
  """Signal dict for CAM_0x2a2 (or CAM_0x2a3 when adjacent=True) from two model lane lines."""
  prefix, default = ("ADJ_", NO_ADJ_LINE) if adjacent else ("", NO_LINE)
  values = {}
  values.update(_side_values(prefix, "LEFT", fit_line(x, left_y) if left_valid else default))
  values.update(_side_values(prefix, "RIGHT", fit_line(x, right_y) if right_valid else default))
  return values


def create_lane_geometry(packer, bus, x, lane_lines_y, lane_line_probs, counter, min_prob=0.5):
  """CAM_0x2a2 + CAM_0x2a3 from modelV2 laneLines (order: outer-left, left, right, outer-right).

  The packer fills in the HKG CAN-FD checksum.
  """
  valid = [p >= min_prob for p in lane_line_probs]
  ego = lane_geometry_values(x, lane_lines_y[1], lane_lines_y[2], valid[1], valid[2])
  adj = lane_geometry_values(x, lane_lines_y[0], lane_lines_y[3], valid[0], valid[3], adjacent=True)
  ego["COUNTER"] = counter % 256
  adj["COUNTER"] = counter % 256
  return [packer.make_can_msg("CAM_0x2a2", bus, ego), packer.make_can_msg("CAM_0x2a3", bus, adj)]
