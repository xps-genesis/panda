
const int HYUNDAI_COMMUNITY_MAX_ACCEL = 150;        // 1.5 m/s2
const int HYUNDAI_COMMUNITY_MIN_ACCEL = -300;       // -3.0 m/s2

const int HYUNDAI_COMMUNITY_ISO_MAX_ACCEL = 200;        // 2.0 m/s2
const int HYUNDAI_COMMUNITY_ISO_MIN_ACCEL = -350;       // -3.5 m/s2

bool hyundai_community_non_scc_car = false;
bool aeb_cmd_act = false;
int prev_desired_accel = 0;
int decel_not_ramping = 0;

const CanMsg HYUNDAI_COMMUNITY_TX_MSGS[] = {
  {832, 0, 8}, {832, 1, 8},    // LKAS11 Bus 0, 1
  {1265, 0, 4}, {1265, 1, 4},  // CLU11 Bus 0, 1
  {1157, 0, 4}                 // LFAHDA_MFC Bus 0
 };

const CanMsg HYUNDAI_COMMUNITY_NONSCC_TX_MSGS[] = {
  {832, 0, 8}, {832, 1, 8}, // LKAS11 Bus 0, 1
  {1265, 0, 4}, {1265, 1, 4}, {1265, 2, 4},// CLU11 Bus 0, 1, 2
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {1056, 0, 8}, //   SCC11,  Bus 0
  {1057, 0, 8}, //   SCC12,  Bus 0
  {1290, 0, 8}, //   SCC13,  Bus 0
  {905, 0, 8},  //   SCC14,  Bus 0
  {1186, 0, 8}, //  4a2SCC,  Bus 0
  {1155, 0, 8}, //   FCA12,  Bus 0
  {909, 0, 8},  //   FCA11,  Bus 0
  {2000, 0, 8}, // SCC_DIAG, Bus 0
  {593, 2, 8}   //  MDPS12,  Bus 2
 };

// TODO: missing checksum for wheel speeds message,worst failure case is
//       wheel speeds stuck at 0 and we don't disengage on brake press
AddrCheckStruct hyundai_community_rx_checks[] = {
  {.msg = {{902, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{916, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}}},
};
const int HYUNDAI_COMMUNITY_RX_CHECK_LEN = sizeof(hyundai_community_rx_checks) / sizeof(hyundai_community_rx_checks[0]);

// for non SCC hyundai vehicles
AddrCheckStruct hyundai_community_nonscc_rx_checks[] = {
  {.msg = {{902, 0, 8, .expected_timestep = 10000U}}},
  {.msg = {{916, 0, 8, .expected_timestep = 10000U}}},
};

const int HYUNDAI_COMMUNITY_NONSCC_RX_CHECK_LEN = sizeof(hyundai_community_nonscc_rx_checks) / sizeof(hyundai_community_nonscc_rx_checks[0]);

static uint8_t hyundai_community_get_counter(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t cnt;
  if (addr == 902) {
    cnt = ((GET_BYTE(to_push, 3) >> 6) << 2) | (GET_BYTE(to_push, 1) >> 6);
  } else if (addr == 916) {
    cnt = (GET_BYTE(to_push, 1) >> 5) & 0x7;
  } else if (addr == 1057) {
    cnt = GET_BYTE(to_push, 7) & 0xF;
  } else {
    cnt = 0;
  }
  return cnt;
}

static uint8_t hyundai_community_get_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum;
  if (addr == 916) {
    chksum = GET_BYTE(to_push, 6) & 0xF;
  } else if (addr == 1057) {
    chksum = GET_BYTE(to_push, 7) >> 4;
  } else {
    chksum = 0;
  }
  return chksum;
}

static uint8_t hyundai_community_compute_checksum(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);

  uint8_t chksum = 0;
  // same algorithm, but checksum is in a different place
  for (int i = 0; i < 8; i++) {
    uint8_t b = GET_BYTE(to_push, i);
    if (((addr == 916) && (i == 6)) || ((addr == 1057) && (i == 7))) {
      b &= (addr == 1057) ? 0x0FU : 0xF0U; // remove checksum
    }
    chksum += (b % 16U) + (b / 16U);
  }
  return (16U - (chksum %  16U)) % 16U;
}

static int hyundai_community_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  bool valid;
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);


  if (hyundai_community_non_scc_car) {
    valid = addr_safety_check(to_push, hyundai_community_nonscc_rx_checks, HYUNDAI_COMMUNITY_NONSCC_RX_CHECK_LEN,
                            hyundai_community_get_checksum, hyundai_community_compute_checksum,
                            hyundai_community_get_counter);
  }
  else {
    valid = addr_safety_check(to_push, hyundai_community_rx_checks, HYUNDAI_COMMUNITY_RX_CHECK_LEN,
                            hyundai_community_get_checksum, hyundai_community_compute_checksum,
                            hyundai_community_get_counter);
  }

  if (valid && (bus == 0)) {

    if (addr == 593) {
      int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ff) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if ((addr == 1057) && (!hyundai_community_non_scc_car)){
      // 2 bits: 13-14
      int cruise_engaged = (GET_BYTES_04(to_push) >> 13) & 0x3;
      if (cruise_engaged && !cruise_engaged_prev) {
        controls_allowed = 1;
      }
      if (!cruise_engaged) {
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // engage for non ACC car
    if ((addr == 1265) && hyundai_community_non_scc_car) {
      // first byte
      int cruise_engaged = (GET_BYTES_04(to_push) & 0x7);
      // enable on res+ or set- buttons press
      if (!controls_allowed && (cruise_engaged == 1 || cruise_engaged == 2)) {
        controls_allowed = 1;
      }
      // disable on cancel press
      if (cruise_engaged == 4) {
        controls_allowed = 0;
      }
    }

    // sample wheel speed, averaging opposite corners
    if (addr == 902) {
      int hyundai_speed = GET_BYTES_04(to_push) & 0x3FFF;  // FL
      hyundai_speed += (GET_BYTES_48(to_push) >> 16) & 0x3FFF;  // RL
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }

    if (addr == 916) {
      gas_pressed = ((GET_BYTE(to_push, 5) >> 5) & 0x3) == 1;
      brake_pressed = (GET_BYTE(to_push, 6) >> 7) != 0;
    }

    generic_rx_checks((addr == 832));
  }
    // monitor AEB active command to bypass panda accel safety, don't block AEB
  if ((addr == 1057) && (bus == 2) && (hyundai_community_non_scc_car)){
    aeb_cmd_act = (GET_BYTE(to_push, 6) & 0x40) != 0;
  }

  return valid;
}

static int hyundai_community_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if(hyundai_community_non_scc_car){
    if (!msg_allowed(to_send, HYUNDAI_COMMUNITY_NONSCC_TX_MSGS, sizeof(HYUNDAI_COMMUNITY_NONSCC_TX_MSGS)/sizeof(HYUNDAI_COMMUNITY_NONSCC_TX_MSGS[0]))) {
        tx = 0;
    }
  }
  else {
    if (!msg_allowed(to_send, HYUNDAI_COMMUNITY_TX_MSGS, sizeof(HYUNDAI_COMMUNITY_TX_MSGS)/sizeof(HYUNDAI_COMMUNITY_TX_MSGS[0]))) {
        tx = 0;
    }
  }


  if (relay_malfunction) {
    tx = 0;
  }

  // ACCEL: safety check

  if ((addr == 1057) && (bus == 0) && hyundai_community_non_scc_car && (!aeb_cmd_act) && vehicle_moving) {
    int desired_accel = (GET_BYTE(to_send, 3) | ((GET_BYTE(to_send, 4) & 0x7) << 8)) - 1024;
    prev_desired_accel = desired_accel;
    if (!controls_allowed) {
        if (((desired_accel < -10) && (prev_desired_accel >= desired_accel))||  //staying in braking or braking more
            ((desired_accel > 10) && (prev_desired_accel <= desired_accel)))     //staying in gas or accelerating more
        {
           decel_not_ramping +=1;
        }
        else
        {
           decel_not_ramping =0;
        }
        if (decel_not_ramping > 5) {  // allow 5 loops
            tx = 0;
        }
    }
    if (controls_allowed) {
      bool vio = (unsafe_mode & UNSAFE_RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX)?
          max_limit_check(desired_accel, HYUNDAI_COMMUNITY_ISO_MAX_ACCEL, HYUNDAI_COMMUNITY_ISO_MIN_ACCEL) :
          max_limit_check(desired_accel, HYUNDAI_COMMUNITY_MAX_ACCEL, HYUNDAI_COMMUNITY_MIN_ACCEL);
      if (vio) {
        tx = 0;
      }
    }
  }

  // LKA STEER: safety check
  if (addr == 832) {
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ff) - 1024;
    uint32_t ts = TIM2->CNT;
    bool violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  if ((addr == 1265) && !controls_allowed) {
    if ((GET_BYTES_04(to_send) & 0x7) != 4) {
      tx = 0;
    }
  }

  // 1 allows the message through
  return tx;
}

static int hyundai_community_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  // forward cam to ccan and viceversa, except lkas cmd
  if (!relay_malfunction) {
    if ((bus_num == 0) && (addr != 593)){
      bus_fwd = 2;
    }
    if ((bus_num == 2) && (addr != 832) && (addr != 1157)) {
      if ((addr != 1056) && (addr != 1057) && (addr != 905) && (addr != 1290)) {
        bus_fwd = 0;
      }
    }
  }
  return bus_fwd;
}

static void hyundai_community_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
  hyundai_community_non_scc_car = false;
}

static void hyundai_community_nonscc_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction_reset();
  hyundai_community_non_scc_car = true;
}

const safety_hooks hyundai_community_hooks = {
  .init = hyundai_community_init,
  .rx = hyundai_community_rx_hook,
  .tx = hyundai_community_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_community_fwd_hook,
  .addr_check = hyundai_community_rx_checks,
  .addr_check_len = sizeof(hyundai_community_rx_checks) / sizeof(hyundai_community_rx_checks[0]),
};

const safety_hooks hyundai_community_nonscc_hooks = {
  .init = hyundai_community_nonscc_init,
  .rx = hyundai_community_rx_hook,
  .tx = hyundai_community_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_community_fwd_hook,
  .addr_check = hyundai_community_nonscc_rx_checks,
  .addr_check_len = sizeof(hyundai_community_nonscc_rx_checks) / sizeof(hyundai_community_nonscc_rx_checks[0]),
};
