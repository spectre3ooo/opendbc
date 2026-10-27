"""Builds adrv_presence_fixture.json from two Ioniq 5 drives (2026-10-26):
00000053--1b9170c57e (red panda cut the ADRV off: no ADRV_0x160 on E-CAN) and 00000054--943a4409de
(red panda unplugged: normal). For segment 0 of each, every carState time (the car-state update clock)
with the time of the newest ADRV_0x160 seen on E-CAN (src 1) up to then, first 60 s, every 10th sample."""
import json
import sys

from opendbc.car.logreader import LogReader

LOGS = "/home/troy/sunnypilot-experiments/logs/devices/ioniq5"
ROUTES = {"bad": "00000053--1b9170c57e--0", "good": "00000054--943a4409de--0"}


def samples(path):
  last_160, out, t0 = 0, [], None
  for m in LogReader(path, sort_by_time=True):
    w = m.which()
    if w == "can":
      for c in m.can:
        if c.src == 1 and c.address == 0x160:
          last_160 = m.logMonoTime
    elif w == "carState":
      t0 = m.logMonoTime if t0 is None else t0
      if m.logMonoTime - t0 > 60e9:
        break
      out.append([m.logMonoTime, last_160])
  return out[::10]


if __name__ == "__main__":
  data = {k: samples(f"{LOGS}/{v}/rlog.zst") for k, v in ROUTES.items()}
  json.dump(data, open(sys.argv[1], "w"))
