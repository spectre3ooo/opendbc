#pragma once

#include "opendbc/safety/declarations.h"

// Standalone interceptor for the Hyundai/Kia CAN-FD ADAS Driving ECU (ADRV).
//
// Runs on a separate panda (e.g. red panda) wired in-line at the ADRV connector, with
// the E-CAN pair cut by the harness relay:
//   bus 0: E-CAN, car side
//   bus 1: A-CAN tap (not forwarded)
//   bus 2: E-CAN, ADRV side
// Everything is forwarded between bus 0 and bus 2, except SCC_CONTROL: while openpilot's
// own SCC_CONTROL is seen on the car side, the ADRV's copy is blocked, so the car only
// acts on openpilot's. When openpilot stops sending, the ADRV's copy flows again.
// The panda never sends anything on its own in this mode.

#define HYUNDAI_CANFD_ADRV_INTERCEPTOR_CAR_BUS 0
#define HYUNDAI_CANFD_ADRV_INTERCEPTOR_ADRV_BUS 2
#define HYUNDAI_CANFD_SCC_CONTROL_ADDR 0x1A0

// openpilot sends SCC_CONTROL at 50 Hz, so this tolerates a few dropped frames
#define HYUNDAI_CANFD_ADRV_INTERCEPTOR_TIMEOUT_US 150000U

static bool adrv_interceptor_op_scc_seen = false;
static uint32_t adrv_interceptor_op_scc_last = 0U;

static safety_config hyundai_canfd_adrv_interceptor_init(uint16_t param) {
  SAFETY_UNUSED(param);
  adrv_interceptor_op_scc_seen = false;
  adrv_interceptor_op_scc_last = 0U;
  return (safety_config){NULL, 0, NULL, 0, false};  // NOLINT(readability/braces)
}

// GCOV_EXCL_START
// Unreachable by design (doesn't define any tx msgs)
static bool hyundai_canfd_adrv_interceptor_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return false;
}
// GCOV_EXCL_STOP

static bool hyundai_canfd_adrv_interceptor_op_scc_active(uint32_t ts) {
  return adrv_interceptor_op_scc_seen &&
         (safety_get_ts_elapsed(ts, adrv_interceptor_op_scc_last) <= HYUNDAI_CANFD_ADRV_INTERCEPTOR_TIMEOUT_US);
}

static bool hyundai_canfd_adrv_interceptor_fwd_hook(int bus_num, int addr) {
  const uint32_t ts = microsecond_timer_get();
  const bool is_scc = addr == HYUNDAI_CANFD_SCC_CONTROL_ADDR;

  // openpilot's SCC_CONTROL arrives on the car side: note it, and never pass it to the ADRV
  if (is_scc && (bus_num == HYUNDAI_CANFD_ADRV_INTERCEPTOR_CAR_BUS)) {
    adrv_interceptor_op_scc_seen = true;
    adrv_interceptor_op_scc_last = ts;
  }

  bool block = false;
  if (is_scc && hyundai_canfd_adrv_interceptor_op_scc_active(ts)) {
    block = true;
  }
  return block;
}

const safety_hooks hyundai_canfd_adrv_interceptor_hooks = {
  .init = hyundai_canfd_adrv_interceptor_init,
  .rx = default_rx_hook,
  .tx = hyundai_canfd_adrv_interceptor_tx_hook,
  .fwd = hyundai_canfd_adrv_interceptor_fwd_hook,
};
