#include "include/sensing_task.hpp"

SensingTask::SensingTask() {}

SensingTask::~SensingTask() {}

void SensingTask::timer_200us_callback(void *arg) {
  SensingTask *instance = static_cast<SensingTask *>(arg);
  instance->timer_200us_callback_main();
}

void IRAM_ATTR SensingTask::timer_200us_callback_main() {
  if (!ready) {
    return;
  }
  const auto se = get_sensing_entity();
  const auto accl_l = (tgt_val->ego_in.v_l - vl_old) / dt;
  const auto accl_r = (tgt_val->ego_in.v_r - vr_old) / dt;

  const float tire = pt->suction_en ? param->tire2 : param->tire;
  const float tread = param->tire_tread;

  se->ego.v_l_old = se->ego.v_l;
  se->ego.v_r_old = se->ego.v_r;
  se->encoder.left_old = se->encoder.left;
  se->encoder.right_old = se->encoder.right;

  gyro_timestamp_old = gyro_timestamp_now;
  gyro_y_timestamp_old = gyro_y_timestamp_now;
  gyro_x_timestamp_old = gyro_x_timestamp_now;
  enc_r_timestamp_old = enc_r_timestamp_now;
  enc_l_timestamp_old = enc_l_timestamp_now;

  if (!enc_if.initialized) {
    return;
  }
  gyro_timestamp_now = esp_timer_get_time();
  // gyro_if.use_2 = false; // Use 1st SPI bus for gyro
  auto gyro = gyro_if.read_gyro_z();

  gyro_y_timestamp_now = esp_timer_get_time();
  auto gyro_y = gyro_if.read_gyro_y();

  gyro_x_timestamp_now = esp_timer_get_time();
  auto gyro_x = gyro_if.read_gyro_x();

  // printf("Gyro Z: %d\n", gyro);
  // gyro2_timestamp_now = esp_timer_get_time();
  // gyro_if.use_2 = true; // Use 2ndSPI bus for gyro
  // auto gyro2 = gyro_if.read_gyro_z();

  // AS5147Pエンコーダーの読み取り
  enc_r_timestamp_now = esp_timer_get_time();
  auto enc_r = enc_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;
  enc_l_timestamp_now = esp_timer_get_time();
  auto enc_l = enc_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;

  // // MT6835エンコーダーの読み取り
  // enc_r_timestamp_now = esp_timer_get_time();
  // auto enc_r = enc_if.read2byte(0xA0, 0x03, true) & 0x3FFF;
  // enc_l_timestamp_now = esp_timer_get_time();
  // auto enc_l = enc_if.read2byte(0xA0, 0x03, false) & 0x3FFF;
  // enc_r_timestamp_now = esp_timer_get_time();
  // auto enc_r = enc_if.read2byte(0x30, 0x03, true) & 0x3FFF;
  // enc_l_timestamp_now = esp_timer_get_time();
  // auto enc_l = enc_if.read2byte(0x30, 0x03, false) & 0x3FFF;

  auto gyro_dt = (float)(gyro_timestamp_now - gyro_timestamp_old) / 1000000;
  auto gyro_y_dt =
      (float)(gyro_y_timestamp_now - gyro_y_timestamp_old) / 1000000;
  auto gyro_x_dt =
      (float)(gyro_x_timestamp_now - gyro_x_timestamp_old) / 1000000;

  // auto gyro2_dt = (float)(gyro2_timestamp_now - gyro2_timestamp_old) /
  // 1000000;
  auto enc_r_dt = (float)(enc_r_timestamp_now - enc_r_timestamp_old) / 1000000;
  auto enc_l_dt = (float)(enc_l_timestamp_now - enc_l_timestamp_old) / 1000000;

  pt->kf_w.dt = gyro_dt;
  pt->kf_w_x.dt = gyro_x_dt;
  pt->kf_w_y.dt = gyro_y_dt;
  // pt->kf_w2.dt = gyro2_dt;

  pt->kf_v_r.dt = enc_r_dt;
  pt->kf_v_l.dt = enc_l_dt;

  if (std::isfinite(accl_l) && std::isfinite(accl_r)) {
    auto tmp_r_v = ABS(calc_enc_v(enc_r, se->encoder.right_old, pt->kf_v_r.dt));
    auto tmp_l_v = ABS(calc_enc_v(enc_l, se->encoder.left_old, pt->kf_v_l.dt));

    if (
        // (enc_r == se->encoder.right_old) ||
        false && ((enc_r == 0) ||
                  (enc_r == 0 && ABS(ABS(tmp_r_v) - ABS(se->ego.v_r)) > 50))) {
      for (int i = 0; i < 3; i++) {
        enc_r_timestamp_now = esp_timer_get_time();
        enc_r = enc_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;
        enc_r_dt = (float)(enc_r_timestamp_now - enc_r_timestamp_old) / 1000000;
        if (enc_r != 0) {
          break;
        }
      }
      if (enc_r == 0 && ABS(ABS(tmp_r_v) - ABS(se->ego.v_r)) > 50) {
        // エンコーダ取得失敗時更新中止
        enc_r_timestamp_now = enc_r_timestamp_old;
      } else if (enc_r_dt > 0) {
        se->encoder.right = enc_r;
        se->ego.v_r = -calc_enc_v(se->encoder.right, se->encoder.right_old,
                                  pt->kf_v_r.dt);
        pt->kf_v_r.dt = enc_r_dt;
        pt->kf_v_r.predict(accl_r);
        pt->kf_v_r.update(se->ego.v_r);
      }
    } else if (enc_r_dt > 0) {
      se->encoder.right = enc_r;
      se->ego.v_r =
          -calc_enc_v(se->encoder.right, se->encoder.right_old, pt->kf_v_r.dt);
      pt->kf_v_r.dt = enc_r_dt;
      pt->kf_v_r.predict(accl_r);
      pt->kf_v_r.update(se->ego.v_r);
    }

    if (
        //(enc_l == se->encoder.left_old) ||
        false && ((enc_l == 0) ||
                  (enc_l == 0 && ABS(ABS(tmp_l_v) - ABS(se->ego.v_l)) > 50))) {
      for (int i = 0; i < 3; i++) {
        enc_l_timestamp_now = esp_timer_get_time();
        enc_l = enc_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;
        enc_l_dt = (float)(enc_l_timestamp_now - enc_l_timestamp_old) / 1000000;
        if (enc_l != 0) {
          break;
        }
      }
      if (enc_l == 0 && ABS(ABS(tmp_l_v) - ABS(se->ego.v_l)) > 50) {
        // エンコーダ取得失敗時更新中止
        enc_l_timestamp_now = enc_l_timestamp_old;
      } else if (enc_l_dt > 0) {
        se->encoder.left = enc_l;
        se->ego.v_l =
            calc_enc_v(se->encoder.left, se->encoder.left_old, pt->kf_v_l.dt);
        pt->kf_v_l.dt = enc_l_dt;
        pt->kf_v_l.predict(accl_l);
        pt->kf_v_l.update(se->ego.v_l);
      }
    } else if (enc_l_dt > 0) {
      se->encoder.left = enc_l;
      se->ego.v_l =
          calc_enc_v(se->encoder.left, se->encoder.left_old, pt->kf_v_l.dt);
      pt->kf_v_l.dt = enc_l_dt;
      pt->kf_v_l.predict(accl_l);
      pt->kf_v_l.update(se->ego.v_l);
    }
  }
  if (param->enable_kalman_encoder == 1) {
    se->ego.v_l = pt->kf_v_l.get_state();
    se->ego.v_r = pt->kf_v_r.get_state();
  } else {
    se->ego.v_l = se->ego.v_l;
    se->ego.v_r = se->ego.v_r;
  }
  const auto alpha = (tgt_val->ego_in.w - w_old) / dt;

  se->ego.w_x_raw =
      param->gyro_param.gyro_x_gain * (gyro_x - tgt_val->gyro_zero_p_offset_x);
  se->ego.w_y_raw =
      param->gyro_param.gyro_y_gain * (gyro_y - tgt_val->gyro_zero_p_offset_y);

  pt->gyro_x = gyro_x;
  pt->gyro_y = gyro_y;

  pt->kf_w_x.predict(0);
  pt->kf_w_y.predict(0);
  pt->kf_w_x.update(se->ego.w_x_raw);
  pt->kf_w_y.update(se->ego.w_y_raw);

  if (std::isfinite(alpha) && std::isfinite(se->ego.w_lp)) {
    se->gyro.raw = se->gyro.data = gyro;
    if (gyro_dt > 0) {
      if ((gyro - tgt_val->gyro_zero_p_offset) >= 0) {
        se->ego.w_raw = param->gyro_param.gyro_w_gain_left *
                        (gyro - tgt_val->gyro_zero_p_offset);
      } else {
        se->ego.w_raw = param->gyro_param.gyro_w_gain_right *
                        (gyro - tgt_val->gyro_zero_p_offset);
      }
      // x,y,z
      pt->s.p = pt->kf_w_x.get_state();
      pt->s.q = pt->kf_w_y.get_state();
      pt->s.r = se->ego.w_raw;
      pt->s.ax = pt->s.ay = pt->s.az = 0;
      pt->s.dt = dt;

      se->ego.w_raw = pt->yaw_est.yaw_rate_kinematic(pt->s);

      pt->kf_w.predict(alpha);
      const float tread = param->tire_tread;
      const float w_enc = -(se->ego.v_r - se->ego.v_l) / tread;
      pt->kf_w.update(se->ego.w_raw);
    }
  }

  if (param->enable_kalman_gyro == 1) {
    se->ego.w_raw = se->ego.w_kf = pt->kf_w.get_state();
    // se->ego.w_raw2 = se->ego.w_kf2 = pt->kf_w2.get_state();
  } else if (param->enable_kalman_gyro == 2) {
    se->ego.w_kf = se->ego.w_raw;
    // se->ego.w_kf2 = se->ego.w_raw;
  } else {
    se->ego.w_kf = pt->kf_w.get_state();
    // se->ego.w_kf2 = pt->kf_w2.get_state();
  }
}

void SensingTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "sensing_task", 8192 * 1, this, 2,
                          &handle, xCoreID);
  // const esp_timer_create_args_t timer_200us_args = {
  //     .callback = &SensingTask::timer_200us_callback,
  //     .arg = this,
  //     .dispatch_method = ESP_TIMER_TASK,
  //     .name = "timer_200us"};
  // esp_timer_create(&timer_200us_args, &timer_200us);
}
void SensingTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void SensingTask::task_entry_point(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task();
}

void SensingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void SensingTask::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
}
void SensingTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}
void SensingTask::encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                               const gpio_num_t pinB) {}

void IRAM_ATTR SensingTask::exec_adc(adc2_channel_t channel,
                                     adc_bits_width_t width_bit, int *raw_out) {
  // auto start = esp_timer_get_time();
  adc2_get_raw(channel, width_bit, raw_out);
  // auto end = esp_timer_get_time();
  // printf("adc: %d , %d\n", (int16_t)(end - start), *raw_out);
}

void IRAM_ATTR SensingTask::task() {
  // timer_init_grp0_timer0();
  if (!GY_MODE) {
    gyro_if.init();
    // gyro_if.use_2 = false; // Use 1st SPI bus for gyro
    gyro_if.setup();
    // gyro_if.use_2 = true; // Use 2nd SPI bus for gyro
    // gyro_if.setup();
    // gyro_if.use_2 = false; // Use 1st SPI bus for gyro
    enc_if.init();

    pt->calib_.eps_zx = 0.0f;
    pt->calib_.eps_zy = 0.0f;
    pt->calib_.kgzx = 0.0f;
    pt->calib_.kgzy = 0.0f;
    pt->calib_.kgzz = 0.0f;

    pt->yaw_est.setCalibration(pt->calib_);
    pt->yaw_est.enableAccelFusion(false);
  }
  // esp_timer_start_periodic(timer_200us, 200);

  const auto se = get_sensing_entity();
  // sensing init
  adc2_config_channel_atten(SEN_R90, atten);
  adc2_config_channel_atten(SEN_R45, atten);
  // adc2_config_channel_atten(SEN_R45_2, atten);
  adc2_config_channel_atten(SEN_L45, atten);
  // adc2_config_channel_atten(SEN_L45_2, atten);
  adc2_config_channel_atten(SEN_L90, atten);
  adc2_config_channel_atten(BATTERY, atten);

  rtc_gpio_init(SEN_R90_GPIO);
  rtc_gpio_set_direction(SEN_R90_GPIO, RTC_GPIO_MODE_DISABLED);
  rtc_gpio_pulldown_dis(SEN_R90_GPIO);
  rtc_gpio_pullup_dis(SEN_R90_GPIO);

  rtc_gpio_init(SEN_R45_GPIO);
  rtc_gpio_set_direction(SEN_R45_GPIO, RTC_GPIO_MODE_DISABLED);
  rtc_gpio_pulldown_dis(SEN_R45_GPIO);
  rtc_gpio_pullup_dis(SEN_R45_GPIO);

  rtc_gpio_init(SEN_L45_GPIO);
  rtc_gpio_set_direction(SEN_L45_GPIO, RTC_GPIO_MODE_DISABLED);
  rtc_gpio_pulldown_dis(SEN_L45_GPIO);
  rtc_gpio_pullup_dis(SEN_L45_GPIO);

  rtc_gpio_init(SEN_L90_GPIO);
  rtc_gpio_set_direction(SEN_L90_GPIO, RTC_GPIO_MODE_DISABLED);
  rtc_gpio_pulldown_dis(SEN_L90_GPIO);
  rtc_gpio_pullup_dis(SEN_L90_GPIO);

  // esp_timer_start_periodic(timer_200us, 200);
  // esp_timer_start_periodic(timer_250us, 250);

  int64_t start = 0;
  int64_t end = 0;
  int64_t start2 = 0;
  int64_t end2 = 0;
  int64_t start_before = 0;
  bool battery_check = true;
  bool skip_sensing = false;

  int64_t last_gyro_time = 0;
  int64_t now_gyro_time = 0;

  int64_t last_enc_r_time = 0;
  int64_t now_enc_r_time = 0;

  int64_t last_enc_l_time = 0;
  int64_t now_enc_l_time = 0;

  ready = true;

  while (1) {

    last_gyro_time = now_gyro_time;
    last_enc_r_time = now_enc_r_time;
    last_enc_l_time = now_enc_l_time;

    skip_sensing = !skip_sensing;

    bool r90 = true;
    bool l90 = true;
    bool r45 = true;
    bool l45 = true;

    start_before = start;
    start = esp_timer_get_time();
    se->calc_time = (int16_t)(start - start_before);
    se->sensing_timestamp = start;
    start2 = now_gyro_time; // esp_timer_get_time();

    now_gyro_time = esp_timer_get_time();
    const auto gyro_dt = ((float)(now_gyro_time - last_gyro_time)) / 1000000.0;

    auto enc_r_dt = dt;
    auto enc_l_dt = dt;

    timer_200us_callback_main();

    calc_vel(gyro_dt, enc_l_dt, enc_r_dt);

    if (skip_sensing) {
      exec_adc(BATTERY, width, &sensing_result->battery.raw);
    }

    if (pt->search_mode && tgt_val->motion_type == MotionType::STRAIGHT) {
      // 加速中は正面は発光させない
      if (tgt_val->ego_in.state == 0) {
        r90 = l90 = false;
      }
    }
    if (pt->search_mode && tgt_val->nmr.sct == SensorCtrlType::NONE) {
      // 探索中、壁制御しないときはOFF
      r90 = l90 = false;
      r45 = l45 = false;
    }
    if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
      if (tgt_val->ego_in.state == 0) {
        // 斜め壁制御加速中は横は発光させない
        // r45 = l45 = false;
      }
      r45 = l45 = true;
    }
    if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
      r90 = l90 = true;
      r45 = l45 = true;
    }
    if (tgt_val->motion_type == MotionType::READY) {
      // motion check用
      r90 = l90 = true;
      r45 = l45 = false;
    }
    if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
      // 前壁制御中は横は発光させない
      r90 = l90 = true;
      r45 = l45 = false;
    }
    if (tgt_val->motion_type == MotionType::SENSING_DUMP) {
      r90 = l90 = true;
      r45 = l45 = true;
    }
    if (pt->tgt_val->motion_type == MotionType::SLALOM) {
      r90 = l90 = r45 = l45 = false;
      if (pt->tgt_val->tt == TurnType::Normal) {
        if (pt->tgt_val->td == TurnDirection::Right) {
          l45 = true;
        } else {
          r45 = true;
        }
      }
    }
    // LED_OFF ADC
    if (skip_sensing) {
      if (r90) {
        exec_adc(SEN_R90, width, &sensing_result->led_sen_before.right90.raw);
      } else {
        sensing_result->led_sen_before.right90.raw = 0;
      }
      if (l90) {
        exec_adc(SEN_L90, width, &sensing_result->led_sen_before.left90.raw);
      } else {
        sensing_result->led_sen_before.left90.raw = 0;
      }
    } else {
      if (r45) {
        exec_adc(SEN_R45, width, &sensing_result->led_sen_before.right45.raw);
        sensing_result->led_sen_before.right45_2.raw =
            sensing_result->led_sen_before.right45_3.raw =
                sensing_result->led_sen_before.right45.raw;
      } else {
        sensing_result->led_sen_before.right45.raw = 0;
      }
      if (l45) {
        exec_adc(SEN_L45, width, &sensing_result->led_sen_before.left45.raw);
        sensing_result->led_sen_before.left45_2.raw =
            sensing_result->led_sen_before.left45_3.raw =
                sensing_result->led_sen_before.left45.raw;
      } else {
        sensing_result->led_sen_before.left45.raw = 0;
      }
    }

    r90 = true;
    l90 = true;
    r45 = true;
    l45 = true;
    // LED_OFF ADC
    // 超信地旋回中は発光をサボる
    bool led_on = true;
    if (tgt_val->motion_type == MotionType::PIVOT) {
      led_on = false;
    }
    if (tgt_val->motion_type == MotionType::SLALOM) {
      led_on = true;
    }
    if (pt->mode_select) {
      led_on = false;
    }
    if (led_on) {
      if (pt->search_mode && pt->tgt_val->motion_type == MotionType::STRAIGHT) {
        // 加速中は正面は発光させない
        if (pt->tgt_val->ego_in.state == 0) {
          r90 = l90 = false;
        }
      }
      if (pt->search_mode && pt->tgt_val->nmr.sct == SensorCtrlType::NONE) {
        // 探索中、壁制御しないときはOFF
        r90 = l90 = false;
        r45 = l45 = false;
      }
      if (pt->tgt_val->nmr.sct == SensorCtrlType::Dia) {
        if (pt->tgt_val->ego_in.state == 0) {
          // 斜め壁制御加速中は横は発光させない
          // r45 = l45 = false;
        }
        r45 = l45 = true;
      }
      if (pt->tgt_val->nmr.sct == SensorCtrlType::Straight) {
        r90 = l90 = true;
        r45 = l45 = true;
      }
      if (pt->tgt_val->motion_type == MotionType::READY) {
        // motion check用
        r90 = l90 = true;
        r45 = l45 = false;
      }
      if (pt->tgt_val->motion_type == MotionType::FRONT_CTRL) {
        // 前壁制御中は横は発光させない
        r90 = l90 = true;
        r45 = l45 = false;
      }
      if (pt->tgt_val->motion_type == MotionType::SENSING_DUMP) {
        r90 = l90 = true;
        r45 = l45 = true;
      }
      if (pt->tgt_val->motion_type == MotionType::SLALOM) {
        r90 = l90 = r45 = l45 = false;
        if (pt->tgt_val->tt == TurnType::Normal) {
          if (pt->tgt_val->td == TurnDirection::Right) {
            l45 = true;
          } else {
            r45 = true;
          }
        }
        // if (tgt_val->ego_in.sla_param.counter >
        //     (tgt_val->ego_in.sla_param.limit_time_count / 2)) {
        //   r90 = l90 = r45 = l45 = true;
        // }
      }
      if (r90) { // R90
        led_driver(LED_EN_R90, 1, LED_EN_R45_2, 0, LED_EN_R45_1, 0);
        lec_cnt = 0;
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        exec_adc(SEN_R90, width, &se->led_sen_after.right90.raw);
        led_driver(LED_EN_R90, 0, LED_EN_R45_2, 0, LED_EN_R45_1, 0);
      } else {
        se->led_sen_after.right90.raw = 0;
      }
      if (l90) { // L90
        led_driver(LED_EN_L90, 1, LED_EN_L45_2, 0, LED_EN_L45_1, 0);
        lec_cnt = 0;
        for (int i = 0; i < param->led_light_delay_cnt2; i++) {
          lec_cnt++;
        }
        exec_adc(SEN_L90, width, &se->led_sen_after.left90.raw);
        led_driver(LED_EN_L90, 0, LED_EN_L45_2, 0, LED_EN_L45_1, 0);
      } else {
        se->led_sen_after.left90.raw = 0;
      }
      if (r45) { // R45

        // // R45
        led_driver(LED_EN_R90, 0, LED_EN_R45_2, 0, LED_EN_R45_1, 1);
        lec_cnt = 0;
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        exec_adc(SEN_R45, width, &se->led_sen_after.right45.raw);

        if (pt->tgt_val->motion_type == MotionType::WALL_OFF ||
            pt->tgt_val->motion_type == MotionType::WALL_OFF_DIA ||
            pt->tgt_val->motion_type == MotionType::SENSING_DUMP ||
            pt->tgt_val->motion_type == MotionType::SLA_BACK_STR) {
          // R45_2
          led_driver(LED_EN_R90, 0, LED_EN_R45_2, 1, LED_EN_R45_1, 1);
          lec_cnt = 0;
          for (int i = 0; i < param->led_light_delay_cnt; i++) {
            lec_cnt++;
          }
          exec_adc(SEN_R45, width, &se->led_sen_after.right45_2.raw);

          // R45_3
          led_driver(LED_EN_R90, 0, LED_EN_R45_2, 1, LED_EN_R45_1, 0);
          lec_cnt = 0;
          for (int i = 0; i < param->led_light_delay_cnt; i++) {
            lec_cnt++;
          }
          exec_adc(SEN_R45, width, &se->led_sen_after.right45_3.raw);
        } else {
          se->led_sen_after.right45_2.raw = se->led_sen_after.right45_3.raw = 0;
        }

        led_driver(LED_EN_R90, 0, LED_EN_R45_2, 0, LED_EN_R45_1, 0);

      } else {
        se->led_sen_after.right45.raw = se->led_sen_after.right45_2.raw =
            se->led_sen_after.right45_3.raw = 0;
      }
      if (l45) { // L45
        // L45
        led_driver(LED_EN_L90, 0, LED_EN_L45_2, 0, LED_EN_L45_1, 1);
        lec_cnt = 0;
        for (int i = 0; i < param->led_light_delay_cnt2; i++) {
          lec_cnt++;
        }
        exec_adc(SEN_L45, width, &se->led_sen_after.left45.raw);
        if (pt->tgt_val->motion_type == MotionType::WALL_OFF ||
            pt->tgt_val->motion_type == MotionType::WALL_OFF_DIA ||
            pt->tgt_val->motion_type == MotionType::SENSING_DUMP ||
            pt->tgt_val->motion_type == MotionType::SLA_BACK_STR) {
          // L45_2
          led_driver(LED_EN_L90, 0, LED_EN_L45_2, 1, LED_EN_L45_1, 1);
          lec_cnt = 0;
          for (int i = 0; i < param->led_light_delay_cnt2; i++) {
            lec_cnt++;
          }
          exec_adc(SEN_L45, width, &se->led_sen_after.left45_2.raw);

          // L45_3
          led_driver(LED_EN_L90, 0, LED_EN_L45_2, 1, LED_EN_L45_1, 0);
          lec_cnt = 0;
          for (int i = 0; i < param->led_light_delay_cnt2; i++) {
            lec_cnt++;
          }
          exec_adc(SEN_L45, width, &se->led_sen_after.left45_3.raw);
        } else {
          se->led_sen_after.left45_2.raw = se->led_sen_after.left45_3.raw = 0;
        }
        led_driver(LED_EN_L90, 0, LED_EN_L45_2, 0, LED_EN_L45_1, 0);
      } else {
        se->led_sen_after.left45.raw = se->led_sen_after.left45_2.raw =
            se->led_sen_after.left45_3.raw = 0;
      }
    }

    // end2 = esp_timer_get_time();
    led_driver(LED_EN_L90, 0, LED_EN_L45_2, 0, LED_EN_L45_1, 0);
    led_driver(LED_EN_R90, 0, LED_EN_R45_2, 0, LED_EN_R45_1, 0);

    se->battery.data = BATTERY_GAIN * 4 * sensing_result->battery.raw / 4096;
    if (led_on) {
      se->led_sen.right90.raw = std::max(
          se->led_sen_after.right90.raw - se->led_sen_before.right90.raw, 0);
      se->led_sen.right45.raw = std::max(
          se->led_sen_after.right45.raw - se->led_sen_before.right45.raw, 0);
      se->led_sen.right45_2.raw = std::max(se->led_sen_after.right45_2.raw -
                                               se->led_sen_before.right45_2.raw,
                                           0);
      se->led_sen.right45_3.raw = std::max(se->led_sen_after.right45_3.raw -
                                               se->led_sen_before.right45_3.raw,
                                           0);
      se->led_sen.left45.raw = std::max(
          se->led_sen_after.left45.raw - se->led_sen_before.left45.raw, 0);
      se->led_sen.left45_2.raw = std::max(
          se->led_sen_after.left45_2.raw - se->led_sen_before.left45_2.raw, 0);
      se->led_sen.left45_3.raw = std::max(
          se->led_sen_after.left45_3.raw - se->led_sen_before.left45_3.raw, 0);

      se->led_sen.left90.raw = std::max(
          se->led_sen_after.left90.raw - se->led_sen_before.left90.raw, 0);
      se->led_sen.front.raw =
          (se->led_sen.left90.raw + se->led_sen.right90.raw) / 2;
    } else {
      se->led_sen.right90.raw = se->led_sen.right45.raw =
          se->led_sen.right45_2.raw = se->led_sen.right45_3.raw =
              se->led_sen.left45.raw = se->led_sen.left45_2.raw =
                  se->led_sen.left45_3.raw = se->led_sen.left90.raw =
                      se->led_sen.front.raw = 0;
    }

    end = esp_timer_get_time();
    se->calc_time2 = (int16_t)(end - start);
    vTaskDelay(1.0 / portTICK_PERIOD_MS);
  }
}

float IRAM_ATTR SensingTask::calc_sensor(float data, float a, float b) {
  return 0;
}

float IRAM_ATTR SensingTask::calc_enc_v(float now, float old, float dt) {
  const float tire = pt->suction_en ? param->tire2 : param->tire;
  const auto enc_delta = now - old;
  float enc_ang = 0.f;
  if (ABS(enc_delta) <
      MIN(ABS(enc_delta - ENC_RESOLUTION), ABS(enc_delta + ENC_RESOLUTION))) {
    enc_ang = 2 * m_PI * (float)enc_delta / (float)ENC_RESOLUTION;
  } else {
    if (ABS(enc_delta - ENC_RESOLUTION) < ABS(enc_delta + ENC_RESOLUTION)) {
      enc_ang = 2 * m_PI * (float)(enc_delta - ENC_RESOLUTION) /
                (float)ENC_RESOLUTION;
    } else {
      enc_ang = 2 * m_PI * (float)(enc_delta + ENC_RESOLUTION) /
                (float)ENC_RESOLUTION;
    }
  }
  return tire * enc_ang / dt / 2;
}

void IRAM_ATTR SensingTask::calc_vel(float gyro_dt, float enc_r_dt,
                                     float enc_l_dt) {
  const auto se = get_sensing_entity();
  const float tire = pt->suction_en ? param->tire2 : param->tire;

  // se->ego.v_l = pt->kf_v_l.get_state();
  // se->ego.v_r = pt->kf_v_r.get_state();
  se->ego.v_c = (se->ego.v_l + se->ego.v_r) / 2;

  se->ego.rpm.right = 30.0 * se->ego.v_r / (m_PI * tire / 2);
  se->ego.rpm.left = 30.0 * se->ego.v_l / (m_PI * tire / 2);

  const auto dt = (enc_l_dt + enc_r_dt) / 2;
  tgt_val->ego_in.dist += se->ego.v_c * dt;
  tgt_val->global_pos.dist += se->ego.v_c * dt;

  if (param->enable_kalman_gyro == 1) {
    tgt_val->ego_in.ang += se->ego.w_kf * gyro_dt;
    tgt_val->global_pos.ang += se->ego.w_kf * gyro_dt;
  } else if (param->enable_kalman_gyro == 2) {
    tgt_val->ego_in.ang += se->ego.w_raw * gyro_dt;
    tgt_val->global_pos.ang += se->ego.w_raw * gyro_dt;
  } else {
    tgt_val->ego_in.ang += se->ego.w_raw * gyro_dt;
    tgt_val->global_pos.ang += se->ego.w_raw * gyro_dt;
  }

  pt->ang_x += pt->s.p * gyro_dt;
  pt->ang_y += pt->s.q * gyro_dt;
  pt->ang_x_raw += se->ego.w_x_raw * gyro_dt;
  pt->ang_y_raw += se->ego.w_y_raw * gyro_dt;

  w_old = tgt_val->ego_in.w;
  vl_old = se->ego.v_l;
  vr_old = se->ego.v_r;
}