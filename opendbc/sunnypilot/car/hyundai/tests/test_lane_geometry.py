#!/usr/bin/env python3
import unittest

import numpy as np

from opendbc.can import CANPacker, CANParser
from opendbc.sunnypilot.car.hyundai import lane_geometry as lg

DBC = "hyundai_canfd_generated"
BUS = 2

# real camera frames from the Ioniq 5 logs (bus 2): two lines tracked, then no lines
EGO_FRAMES = [
  "52e55e4f03ae04006601000069010000837f0000f2810000f97e000090900000",
  "14bc5f4f03ae04006601000069010000837f0000f2810000f97e000090900000",
  "befa604f03ae04006601000069010000837f0000f2810000f87e000090900000",
]
ADJ_FRAMES = [
  "0bc85d050224060061010000700100008f810000397e0000698400005c750000",
  "c1235e050224060061010000700100008f810000397e0000698400005c750000",
]
EGO_NO_LINES = "b5415ae803e803006d0100006d010000f17f0000f17f0000c37f0000c37f0000"

X = np.array([0., 0.1875, 0.75, 1.6875, 3., 4.6875, 6.75, 9.1875, 12., 15.1875, 18.75, 22.6875, 27., 31.6875,
              36.75, 42.1875, 48., 54.1875, 60.75])


def _parse(name, addr, frames):
  parser = CANParser(DBC, [(name, 0)], BUS)
  for i, hexdat in enumerate(frames):
    parser.update([(int((i + 1) * 5e7), [(addr, bytes.fromhex(hexdat), BUS)])])
  return dict(parser.vl[name])


class TestLaneGeometry(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC)

  def test_real_frames_repack_exact(self):
    # decode with the DBC, re-pack with the packer: byte-exact including the CRC
    for name, addr, frames in (("CAM_0x2a2", 0x2A2, EGO_FRAMES), ("CAM_0x2a3", 0x2A3, ADJ_FRAMES)):
      for n in range(1, len(frames) + 1):
        vals = _parse(name, addr, frames[:n])
        vals.pop("CHECKSUM")
        self.assertEqual(self.packer.make_can_msg(name, BUS, vals)[1].hex(), frames[n - 1], name)

  def test_real_frame_physical(self):
    vals = _parse("CAM_0x2a2", 0x2A2, EGO_FRAMES[:1])
    # matches the camera's own FR_CMR_03_50ms for this frame: left -1.553 m, right +2.013 m
    self.assertAlmostEqual(vals["LEFT_POSITION"], -1.53, places=6)
    self.assertAlmostEqual(vals["RIGHT_POSITION"], 1.98, places=6)

  def test_fit_recovers_cubic(self):
    pos, hdg, crv, dcrv = -1.7, 0.01, 1e-3, 2e-5
    y = pos + hdg * X + crv / 2 * X ** 2 + dcrv / 6 * X ** 3
    line = lg.fit_line(X, y)
    self.assertAlmostEqual(line["POSITION"], pos, places=6)
    self.assertAlmostEqual(line["HEADING"], hdg, places=6)
    self.assertAlmostEqual(line["CURVATURE"], crv, places=8)
    self.assertAlmostEqual(line["CURVATURE_RATE"], dcrv, places=10)

  def test_no_lines_matches_camera_default(self):
    y = np.zeros((4, len(X)))
    ego, _ = lg.create_lane_geometry(self.packer, BUS, X, y, [0., 0., 0., 0.], counter=0x5a)
    self.assertEqual(ego[0], lg.CAM_LANE_EGO_MSG)
    self.assertEqual(ego[1].hex(), EGO_NO_LINES)

  def test_model_lines_round_trip(self):
    ys = [-5.3 + 0.002 * X, -1.75 + 0.004 * X + 4e-4 * X ** 2, 1.85 + 0.004 * X + 4e-4 * X ** 2, 5.4 + 0.002 * X]
    msgs = lg.create_lane_geometry(self.packer, BUS, X, ys, [0.9, 0.9, 0.9, 0.9], counter=7)
    ego = _parse("CAM_0x2a2", 0x2A2, [msgs[0][1].hex()])
    adj = _parse("CAM_0x2a3", 0x2A3, [msgs[1][1].hex()])
    self.assertAlmostEqual(ego["LEFT_POSITION"], -1.75, delta=0.01)
    self.assertAlmostEqual(ego["RIGHT_POSITION"], 1.85, delta=0.01)
    self.assertAlmostEqual(ego["LEFT_CURVATURE"], 8e-4, delta=2e-6)
    self.assertAlmostEqual(adj["ADJ_LEFT_POSITION"], -5.3, delta=0.01)
    self.assertAlmostEqual(adj["ADJ_RIGHT_POSITION"], 5.4, delta=0.01)
    self.assertEqual(ego["COUNTER"], 7)

  def test_invalid_adjacent_line_uses_absent_default(self):
    ys = np.zeros((4, len(X)))
    msgs = lg.create_lane_geometry(self.packer, BUS, X, ys, [0.1, 0.9, 0.9, 0.1], counter=0)
    adj = msgs[1][1]
    self.assertEqual(adj[3:7], b"\x00\x00\x00\x00")  # raw 0 position = no adjacent line


if __name__ == "__main__":
  unittest.main()
