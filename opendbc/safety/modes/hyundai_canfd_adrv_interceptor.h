#pragma once

#include "opendbc/safety/declarations.h"

// Standalone interceptor for the Hyundai/Kia CAN-FD ADAS Driving ECU (ADRV).
//
// Runs on a separate panda (e.g. red panda) wired in-line at the ADRV connector, with
// the E-CAN pair cut by the harness relay:
//   bus 0: E-CAN, car side (rack, cluster, the camera-position comma's injections, everything else)
//   bus 1: A-CAN tap (not forwarded)
//   bus 2: E-CAN, ADRV side (the ADAS Driving ECU, alone)
//
// Everything is forwarded between bus 0 and bus 2, EXCEPT the messages the comma takes over.
// For each intercepted address: while the comma's own copy of that message is seen on the car
// side (bus 0), the ADRV's copy (arriving from bus 2) is blocked so only the comma's reaches the
// car. When the comma stops sending a given message, the ADRV's copy flows again within the
// timeout, so the system fails safe to stock. The panda never transmits on its own.
//
// Intercepted set (each gated independently on the comma's presence):
//   0x1A0 SCC_CONTROL     - longitudinal (only active when the comma runs op long)
//   0x12A LFA             - steering to the rack (+ cluster/status fields)
//   0x1E0 LFAHDA_CLUSTER  - HDA/LFA cluster icons
// NOTE: 0x12A also carries FCA/BCA/ELK/pedestrian *warning* fields. Blocking it means the comma's
// 0x12A must carry those through (or they must be redundant on their own messages). This is
// verified on-car before relying on it; a future revision may re-address instead of block to
// preserve them (needs a forward-path frame rewrite, not yet in mainline).

#define HYUNDAI_CANFD_ADRV_INTERCEPTOR_CAR_BUS 0
#define HYUNDAI_CANFD_ADRV_INTERCEPTOR_ADRV_BUS 2

// the comma sends these at >= ~50 Hz, so this tolerates a few dropped frames
#define HYUNDAI_CANFD_ADRV_INTERCEPTOR_TIMEOUT_US 150000U

typedef struct {
  uint32_t addr;
  uint32_t op_last;   // last time the comma's copy was seen on the car side
  bool op_seen;
} AdrvInterceptMsg;

static AdrvInterceptMsg adrv_intercept_msgs[] = {
  {0x1A0U, 0U, false},  // SCC_CONTROL
  {0x12AU, 0U, false},  // LFA (steering)
  {0x1E0U, 0U, false},  // LFAHDA_CLUSTER
};
#define ADRV_INTERCEPT_MSG_CNT (sizeof(adrv_intercept_msgs) / sizeof(adrv_intercept_msgs[0]))

static safety_config hyundai_canfd_adrv_interceptor_init(uint16_t param) {
  SAFETY_UNUSED(param);
  for (unsigned int i = 0; i < ADRV_INTERCEPT_MSG_CNT; i++) {
    adrv_intercept_msgs[i].op_seen = false;
    adrv_intercept_msgs[i].op_last = 0U;
  }
  return (safety_config){NULL, 0, NULL, 0, false};  // NOLINT(readability/braces)
}

// GCOV_EXCL_START
// Unreachable by design (doesn't define any tx msgs)
static bool hyundai_canfd_adrv_interceptor_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return false;
}
// GCOV_EXCL_STOP

static bool hyundai_canfd_adrv_interceptor_fwd_hook(int bus_num, int addr) {
  const uint32_t ts = microsecond_timer_get();

  AdrvInterceptMsg *m = NULL;
  for (unsigned int i = 0; i < ADRV_INTERCEPT_MSG_CNT; i++) {
    if (adrv_intercept_msgs[i].addr == (uint32_t)addr) {
      m = &adrv_intercept_msgs[i];
      break;
    }
  }

  bool block = false;
  if (m != NULL) {
    if (bus_num == HYUNDAI_CANFD_ADRV_INTERCEPTOR_CAR_BUS) {
      // the comma's copy arrives on the car side: note its presence, and never pass it to the
      // ADRV (the ADAS ECU doesn't need the comma's version, and it would see its own back)
      m->op_seen = true;
      m->op_last = ts;
      block = true;
    } else if (bus_num == HYUNDAI_CANFD_ADRV_INTERCEPTOR_ADRV_BUS) {
      // block the ADRV's copy only while the comma is actively replacing this message
      const bool op_active = m->op_seen &&
        (safety_get_ts_elapsed(ts, m->op_last) <= HYUNDAI_CANFD_ADRV_INTERCEPTOR_TIMEOUT_US);
      block = op_active;
    } else {
      // other buses: no action
    }
  }
  return block;
}

const safety_hooks hyundai_canfd_adrv_interceptor_hooks = {
  .init = hyundai_canfd_adrv_interceptor_init,
  .rx = default_rx_hook,
  .tx = hyundai_canfd_adrv_interceptor_tx_hook,
  .fwd = hyundai_canfd_adrv_interceptor_fwd_hook,
};
