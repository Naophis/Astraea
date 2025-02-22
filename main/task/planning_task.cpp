
#include "include/planning_task.hpp"

// constexpr int MOTOR_HZ = 250000;
// constexpr int MOTOR_HZ = 125000;
// constexpr int MOTOR_HZ = 100000;
constexpr unsigned long MOTOR_HZ = 75000 / 1;
// constexpr int MOTOR_HZ = 32500;
// constexpr int MOTOR_HZ = 17500;
// constexpr int MOTOR_HZ = 100000 / 1;
constexpr unsigned long SUCTION_MOTOR_HZ = 10000;
PlanningTask::PlanningTask() {}

PlanningTask::~PlanningTask() {}
void PlanningTask::create_task(const BaseType_t xCoreID) {
  xTaskCreatePinnedToCore(task_entry_point, "planning_task", 8192, this, 4, th,
                          xCoreID);
  motor_qh_enable = xQueueCreate(4, sizeof(motor_req_t *));
  suction_qh_enable = xQueueCreate(4, sizeof(motor_req_t *));
}

void PlanningTask::set_logging_task(std::shared_ptr<LoggingTask> &_lt) {
  lt = _lt; //
}
void IRAM_ATTR PlanningTask::motor_enable_main() {
  motor_en = true;
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  set_gpio_state(L_CW_CCW1, 1);
  set_gpio_state(R_CW_CCW1, 1);
  set_gpio_state(Motor_L_PWM, 0);
  set_gpio_state(Motor_L_PWM2, 0);
  set_gpio_state(Motor_R_PWM, 0);
  set_gpio_state(Motor_R_PWM2, 0);
}
void IRAM_ATTR PlanningTask::set_gpio_state(gpio_num_t gpio_num, int state) {
  const int num = (int)gpio_num;
  if (num < 32) {
    if (state) {
      GPIO.out_w1ts = BIT(num);
    } else {
      GPIO.out_w1tc = BIT(num);
    }
  } else {
    if (state) {
      GPIO.out1_w1ts.val = BIT(num - 32);
    } else {
      GPIO.out1_w1tc.val = BIT(num - 32);
    }
  }
}

void IRAM_ATTR PlanningTask::pid_gain_data(pid_error_t &in, pid_error_t &save) {
  // pid_val_data(in, save);
}

void IRAM_ATTR PlanningTask::set_ctrl_val(pid_error2_t &val, float error_p,
                                          float error_i, float error_i2,
                                          float error_d, float val_p,
                                          float val_i, float val_i2,
                                          float val_d, float zz, float z) {
  val.p = error_p;
  val.i = error_i;
  val.i2 = error_i2;
  val.d = error_d;
  val.p_val = val_p;
  val.i_val = val_i;
  val.i2_val = val_i2;
  val.d_val = val_d;
  val.zz = zz;
  val.z = z;
}
void IRAM_ATTR PlanningTask::pid_val_data(pid_error_t &in, pid_error_t &save) {
  // error_entity_ptr->error_p = in.error_p;
  // error_entity_ptr->error_i = in.error_i;
  // error_entity_ptr->error_d = in.error_d;
}

float IRAM_ATTR PlanningTask::interp1d(vector<float> &vx, vector<float> &vy,
                                       float x, bool extrapolate) {
  int size = vx.size();
  int i = 0;
  if (x >= vx[size - 2]) {
    i = size - 2;
  } else {
    while (x > vx[i + 1])
      i++;
  }
  float xL = vx[i], yL = vy[i], xR = vx[i + 1], yR = vy[i + 1];
  if (!extrapolate) {
    if (x < xL)
      yR = yL;
    if (x > xR)
      yL = yR;
  }

  float dydx = (yR - yL) / (xR - xL);

  return yL + dydx * (x - xL);
}

void IRAM_ATTR PlanningTask::motor_disable_main() {
  motor_en = false;
  gain_cnt = 0;
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
  set_gpio_state(L_CW_CCW1, 0);
  set_gpio_state(R_CW_CCW1, 0);
  set_gpio_state(Motor_L_PWM, 0);
  set_gpio_state(Motor_L_PWM2, 0);
  set_gpio_state(Motor_R_PWM, 0);
  set_gpio_state(Motor_R_PWM2, 0);
}

void IRAM_ATTR PlanningTask::motor_enable() {
  kf_dist.reset(0);
  kf_ang.reset(0);
  kf_v.reset(0);
  kf_w.reset(0);
  kf_w2.reset(0);
  kf_v_l.reset(0);
  kf_v_r.reset(0);
  motor_enable_send_msg.enable = true;
  motor_enable_send_msg.timestamp++;
  xQueueReset(motor_qh_enable);
  xQueueSendToFront(motor_qh_enable, &motor_enable_send_msg, 1);
}
void IRAM_ATTR PlanningTask::motor_disable(bool reset_req) {
  motor_enable_send_msg.enable = false;
  motor_enable_send_msg.timestamp++;
  xQueueReset(motor_qh_enable);
  xQueueSendToFront(motor_qh_enable, &motor_enable_send_msg, 1);
}
void IRAM_ATTR PlanningTask::motor_disable() {
  motor_disable(true); //
  vTaskDelay(1.0 / portTICK_PERIOD_MS);
}

void IRAM_ATTR PlanningTask::suction_motor_enable_main() {
  suction_en = true;
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                 std::abs(tgt_duty.duty_suction));
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_start(MCPWM_UNIT_1, MCPWM_TIMER_2);
  // ledc_channel_config(&suction_ch);
  // ledc_timer_config(&suction_timer);
}
void IRAM_ATTR PlanningTask::suction_motor_disable_main() {
  suction_en = false;
  tgt_duty.duty_suction = 0;
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
  mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B);
  // mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_2); //
  mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  // ledc_set_duty(suction_ch.speed_mode, suction_ch.channel, 0);
  // ledc_update_duty(suction_ch.speed_mode, suction_ch.channel);
}

void IRAM_ATTR PlanningTask::suction_enable(float duty, float duty2) {
  tgt_duty.duty_suction = duty;
  tgt_duty.duty_suction_low = duty2;
  suction_enable_send_msg.enable = true;
  suction_enable_send_msg.timestamp++;
  xQueueReset(suction_qh_enable);
  xQueueSendToFront(suction_qh_enable, &suction_enable_send_msg, 1);
}
void IRAM_ATTR PlanningTask::suction_disable() {
  suction_enable_send_msg.enable = false;
  suction_enable_send_msg.timestamp++;
  xQueueReset(suction_qh_enable);
  xQueueSendToFront(suction_qh_enable, &suction_enable_send_msg, 1);
}
void PlanningTask::task_entry_point(void *task_instance) {
  static_cast<PlanningTask *>(task_instance)->task();
}

void PlanningTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void PlanningTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param_ro) {
  param_ro = _param_ro;
}

void PlanningTask::reset_pos(float x, float y, float ang) {
  tgt_val->ego_in.pos_x = x;
  tgt_val->ego_in.pos_y = 0;
  pos.init(x, y, ang, param_ro->pos_init_cov, param_ro->pos_p_noise,
           param_ro->pos_m_noise);
}

void PlanningTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}
void PlanningTask::set_error_entity(
    std::shared_ptr<pid_error_entity_t> &_error_entity) {
  ee = _error_entity;
}
void PlanningTask::copy_error_entity(pid_error_entity_t &in) {}
void PlanningTask::active_logging(FILE *_f) {
  log_active = true;
  log_list2_size = 0;
}
void PlanningTask::inactive_logging() { log_active = false; }

void PlanningTask::buzzer(ledc_channel_config_t &buzzer_ch,
                          ledc_timer_config_t &buzzer_timer) {
  int duty = 0;
  bool buzzer = false;
  if (motor_en || suction_en) {
    return;
  }
  if (buzzer_timestamp != tgt_val->buzzer.timstamp) {
    buzzer_time_cnt = 0;
    buzzer_timestamp = tgt_val->buzzer.timstamp;
    buzzer_timer.freq_hz = tgt_val->buzzer.hz;
    ledc_channel_config(&buzzer_ch);
    ledc_timer_config(&buzzer_timer);
  }
  if (buzzer_time_cnt < tgt_val->buzzer.time) {
    duty = 50;
    buzzer_time_cnt++;
    buzzer = true;
  } else if (buzzer_time_cnt >= tgt_val->buzzer.time &&
             buzzer_time_cnt < tgt_val->buzzer.time + 10) {
    duty = 0;
    buzzer_time_cnt++;
    buzzer = true;
  }
  if (buzzer) {
    ledc_set_duty(buzzer_ch.speed_mode, buzzer_ch.channel, duty);
    ledc_update_duty(buzzer_ch.speed_mode, buzzer_ch.channel);
  }
}
void PlanningTask::calc_filter() {
  const auto alpha = param_ro->comp_param.accl_x_hp_gain;
  sensing_result->ego.filter_v = //
      alpha * (sensing_result->ego.filter_v +
               sensing_result->ego.accel_x_raw * dt) + //
      (1 - alpha) * sensing_result->ego.v_c;
  if (!std::isfinite(sensing_result->ego.filter_v)) {
    sensing_result->ego.filter_v = sensing_result->ego.v_c;
  }
}

void PlanningTask::set_motor_hz(unsigned long hz, int res) {
  const unsigned long int resolution = ((unsigned long int)res) * 100'000L;
  mcpwm_group_set_resolution(MCPWM_UNIT_0, resolution);

  memset(&motor_pwm_conf, 0, sizeof(motor_pwm_conf));
  motor_pwm_conf.frequency = hz; // PWM周波数= 10kHz,
  motor_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  motor_pwm_conf.cmpr_b = 0; // デューティサイクルの初期値（0%）
  motor_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  motor_pwm_conf.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motor_pwm_conf);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &motor_pwm_conf);

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, Motor_L_PWM);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, Motor_R_PWM);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);

  // 2PWM input mode configuration
  // if (param_ro->motor_driver_type == MotorDriveType::TWO_PWM) {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, Motor_L_PWM2);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, Motor_R_PWM2);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  // }
}
void PlanningTask::set_suction_motor_hz(unsigned long hz, int res) {
  const unsigned long int resolution = ((unsigned long int)res) * 100'000L;
  mcpwm_group_set_resolution(MCPWM_UNIT_1, resolution);
  memset(&suction_pwm_conf, 0, sizeof(suction_pwm_conf));
  suction_pwm_conf.frequency = hz; // PWM周波数= 10kHz,
  suction_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  suction_pwm_conf.cmpr_b = 0; // デューティサイクルの初期値（0%）
  suction_pwm_conf.counter_mode = MCPWM_UP_COUNTER;
  suction_pwm_conf.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
  suction_pwm_conf.cmpr_a = 0; // デューティサイクルの初期値（0%）
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &suction_pwm_conf);
}

void PlanningTask::reset_kf_state(bool reset_battery) {
  float initial_state = 0.0; // 初期姿勢の推定値
  // float initial_covariance = 0.950; // 初期姿勢の誤差共分散行列
  // float process_noise = 0.05;       // プロセスノイズの共分散
  // float measurement_noise = 0.35;   // 観測ノイズの共分散

  kf_w.dt = 0.001 / 1;
  kf_w2.dt = 0.001 / 1;
  kf_v_r.dt = 0.001 / 1;
  kf_v_l.dt = 0.001 / 1;

  if (reset_battery) {
    kf_batt.init(sensing_result->ego.battery_raw, //
                 param_ro->battery_init_cov,      //
                 param_ro->battery_p_noise,       //
                 param_ro->battery_m_noise);

    pos.init(-param_ro->offset_start_dist, 0, 0, //
             param_ro->pos_init_cov,             //
             param_ro->pos_p_noise,              //
             param_ro->pos_m_noise);             //
  } else {
    pos.reset_cov(param_ro->pos_init_cov, //
                  param_ro->pos_p_noise,  //
                  param_ro->pos_m_noise); //
  }
  kf_w.init(initial_state,        //
            param_ro->w_init_cov, //
            param_ro->w_p_noise,  //
            param_ro->w_m_noise);
  kf_v.init(initial_state,        //
            param_ro->v_init_cov, //
            param_ro->v_p_noise,  //
            param_ro->v_m_noise);
  kf_v_r.init(initial_state,              //
              param_ro->encoder_init_cov, //
              param_ro->encoder_p_noise,  //
              param_ro->encoder_m_noise);
  kf_v_l.init(initial_state,              //
              param_ro->encoder_init_cov, //
              param_ro->encoder_p_noise,  //
              param_ro->encoder_m_noise);
  kf_dist.init(initial_state,           //
               param_ro->dist_init_cov, //
               param_ro->dist_p_noise,  //
               param_ro->dist_m_noise);
  kf_ang.init(initial_state,          //
              param_ro->ang_init_cov, //
              param_ro->ang_p_noise,  //
              param_ro->ang_m_noise);
}

void PlanningTask::task() {
  int64_t start = 0;
  int64_t start2 = 0;
  int64_t start_before = 0;
  int64_t end = 0;
  int64_t start_calc_mpc = 0;
  int64_t end_calc_mpc = 0;
  int64_t start_que_rec = 0;
  int64_t end_que_rec = 0;
  // int64_t start2;
  // int64_t end2;
  // int64_t start2;
  // int64_t end2;
  const TickType_t xDelay = 1.0 / portTICK_PERIOD_MS;
  BaseType_t queue_recieved;
  init_gpio();
  for (int i = 0; i < 4097; i++) {
    log_table.emplace_back(std::log(i));
  }
  memset(&buzzer_ch, 0, sizeof(buzzer_ch));
  memset(&buzzer_timer, 0, sizeof(buzzer_timer));
  buzzer_ch.channel = (ledc_channel_t)LEDC_CHANNEL_0;
  buzzer_ch.duty = 0;
  buzzer_ch.gpio_num = BUZZER;
  buzzer_ch.speed_mode = (ledc_mode_t)LEDC_HIGH_SPEED_MODE;
  buzzer_ch.timer_sel = (ledc_timer_t)LEDC_TIMER_0;

  buzzer_timer.duty_resolution = (ledc_timer_bit_t)LEDC_TIMER_10_BIT;
  buzzer_timer.freq_hz = 440;
  buzzer_timer.speed_mode = (ledc_mode_t)LEDC_HIGH_SPEED_MODE; // timer mode
  buzzer_timer.timer_num = (ledc_timer_t)LEDC_TIMER_0;         // timer index
  ledc_channel_config(&buzzer_ch);
  ledc_timer_config(&buzzer_timer);

  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2A, SUCTION_PWM);
  mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);

  set_motor_hz(MOTOR_HZ, 10);
  set_suction_motor_hz(SUCTION_MOTOR_HZ, 10);

  motor_en = false;
  set_next_duty(0, 0, 0);
  gpio_set_level(SUCTION_PWM, 0);
  mpc_tgt_calc.initialize();
  vel_pid.initialize();
  gyro_pid.initialize();
  ang_pid.initialize();

  reset_kf_state(true);

  enc_v_q.clear();
  accl_x_q.clear();
  ready = false;

  while (1) {
    start_before = start;
    start = esp_timer_get_time();
    tgt_val->calc_time_diff = start - start_before;

    if (!ready) {
      vTaskDelay(xDelay);
    }

    // sensing
    if (xQueueReceive(motor_qh_enable, &motor_enable_send_msg, 0) == pdTRUE) {
      if (motor_req_timestamp != motor_enable_send_msg.timestamp) {
        if (motor_enable_send_msg.enable) {
          motor_enable_main();
        } else {
          motor_disable_main();
        }
        motor_req_timestamp = motor_enable_send_msg.timestamp;
      }
    }
    if (xQueueReceive(suction_qh_enable, &suction_enable_send_msg, 0) ==
        pdTRUE) {
      if (suction_req_timestamp != suction_enable_send_msg.timestamp) {
        if (suction_enable_send_msg.enable) {
          suction_motor_enable_main();
        } else {
          suction_motor_disable_main();
        }
        suction_req_timestamp = suction_enable_send_msg.timestamp;
      }
    }

    // 自己位置更新
    int64_t start_duty_cal = esp_timer_get_time();
    update_ego_motion(); // 30 usec
    int64_t end_duty_cal = esp_timer_get_time();
    calc_sensor_dist_all(); // 15 ~ 20 usec

    mpc_step = 1;
    tgt_val->tgt_in.time_step2 = param_ro->sakiyomi_time;

    recv_notify();

    start2 = esp_timer_get_time();

    // 物理量ベース計算
    float axel_degenerate_gain = 1;
    diff_old = diff;
    if (!search_mode && tgt_val->motion_type == MotionType::STRAIGHT) {
      // if (tgt_val->motion_type == MotionType::STRAIGHT) {
      if (axel_degenerate_x.size() > 0 &&
          tgt_val->nmr.sct == SensorCtrlType::Straight) {
        // if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
        SensingControlType type = SensingControlType::None;
        diff = ABS(check_sen_error(type));
        // } else if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
        //   diff = ABS(check_sen_error_dia());
        // }
        if (diff == 0) {
          diff = diff_old;
        }
        axel_degenerate_gain =
            interp1d(axel_degenerate_x, axel_degenerate_y, diff, false);
        tgt_val->tgt_in.axel_degenerate_gain =
            (1 - param_ro->sensor_gain.front2.b) *
                tgt_val->tgt_in.axel_degenerate_gain +
            param_ro->sensor_gain.front2.b * axel_degenerate_gain;
      }
    } else {
      diff = diff_old = 0;
      tgt_val->tgt_in.axel_degenerate_gain = axel_degenerate_gain;
    }

    start_calc_mpc = esp_timer_get_time();
    if (first_req) {

      if (search_mode && tgt_val->motion_type == MotionType::SLALOM) {
        tgt_val->tgt_in.enable_slip_decel = 1;
      } else {
        tgt_val->tgt_in.enable_slip_decel = 0;
      }
      mpc_tgt_calc.step(&tgt_val->tgt_in, &tgt_val->ego_in,
                        tgt_val->motion_mode, mpc_step, &mpc_next_ego,
                        &dynamics);
    }
    end_calc_mpc = esp_timer_get_time();
    // check 3

    if (tgt_val->motion_type == MotionType::STRAIGHT ||
        tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
        tgt_val->motion_type == MotionType::SLA_BACK_STR) {
      if (tgt_val->ego_in.img_dist >= tgt_val->tgt_in.tgt_dist) {
        mpc_next_ego.v = tgt_val->tgt_in.end_v;
      }
      if (tgt_val->ego_in.dist >= tgt_val->tgt_in.tgt_dist) {
        mpc_next_ego.v = tgt_val->tgt_in.end_v;
      }
    }

    // 算出結果をコピー
    cp_tgt_val(); // 1~2usec

    // Duty計算
    calc_tgt_duty(); // 15 ~ 20 usec

    check_fail_safe(); // 7 ~ 9 usec

    // chekc 4
    // 22 ~ 25 usec
    set_next_duty(tgt_duty.duty_l, tgt_duty.duty_r, tgt_duty.duty_suction);

    buzzer(buzzer_ch, buzzer_timer);
    global_msec_timer++;

    end = esp_timer_get_time();
    tgt_val->calc_time = (int16_t)(end - start);
    vTaskDelay(xDelay);
  }
}

float IRAM_ATTR PlanningTask::calc_sensor_pid() {
  float duty = 0;

  SensingControlType type = SensingControlType::None;
  ee->sen.error_i += ee->sen.error_p;
  ee->sen.error_d = ee->sen.error_p;
  ee->sen.error_p = check_sen_error(type);
  if (search_mode) {
    if (ee->sen.error_p > param_ro->search_sen_ctrl_limitter) {
      ee->sen.error_p = param_ro->search_sen_ctrl_limitter;
    } else if (ee->sen.error_p < -param_ro->search_sen_ctrl_limitter) {
      ee->sen.error_p = -param_ro->search_sen_ctrl_limitter;
    }
  }
  ee->sen.error_d = ee->sen.error_p - ee->sen.error_d;

  if (search_mode) {
    if (ee->sen.error_p != 0) {
      duty = param_ro->str_ang_pid.p * ee->sen.error_p -
             param_ro->str_ang_pid.i * ee->sen.error_d;

      //  (ee->sen_log.gain_z - ee->sen_log.gain_zz) * dt;
      set_ctrl_val(ee->s_val, ee->sen.error_p, 0, 0, ee->sen.error_d,
                   param_ro->str_ang_pid.p * ee->sen.error_p, 0, 0,
                   -param_ro->str_ang_pid.d * ee->sen.error_d,
                   ee->sen_log.gain_zz, ee->sen_log.gain_z);
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
    } else {
      duty = 0;
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
  } else {

    if (ee->sen.error_p != 0) {
      duty = param_ro->str_ang_pid.b * ee->sen.error_p -
             param_ro->str_ang_pid.d * ee->sen.error_d;

      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, ee->sen.error_p, 0, 0, ee->sen.error_d,
                   param_ro->str_ang_pid.b * ee->sen.error_p, 0, 0,
                   -param_ro->str_ang_pid.d * ee->sen.error_d,
                   ee->sen_log.gain_zz, ee->sen_log.gain_z);
    } else {
      duty = 0;
      ee->sen_log.gain_zz = ee->sen_log.gain_z;
      ee->sen_log.gain_z = duty;
      set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, ee->sen_log.gain_zz,
                   ee->sen_log.gain_z);
    }
  }
  // if (search_mode) {
  //   if (duty > param_ro->sensor_gain.front.a) {
  //     duty = param_ro->sensor_gain.front.a;
  //   } else if (duty < -param_ro->sensor_gain.front.a) {
  //     duty = -param_ro->sensor_gain.front.a;
  //   }
  // } else {
  //   if (duty > param_ro->sensor_gain.front2.a) {
  //     duty = param_ro->sensor_gain.front2.a;
  //   } else if (duty < -param_ro->sensor_gain.front2.a) {
  //     duty = -param_ro->sensor_gain.front2.a;
  //   }
  // }

  float limit = 0;
  if (type == SensingControlType::None) {
    return 0;
  } else if (type == SensingControlType::Wall) {
    limit = interp1d(sensor_deg_limitter_v, sensor_deg_limitter_str,
                     tgt_val->ego_in.v, false);
  } else if (type == SensingControlType::Piller) {
    limit = interp1d(sensor_deg_limitter_v, sensor_deg_limitter_piller,
                     tgt_val->ego_in.v, false);
  }
  duty = std::clamp(duty, -limit, limit);
  return duty;
}
float IRAM_ATTR PlanningTask::calc_sensor_pid_dia() {
  float duty = 0;
  SensingControlType type = SensingControlType::None;
  ee->sen_dia.error_i += ee->sen_dia.error_p;
  ee->sen_dia.error_d = ee->sen_dia.error_p;
  ee->sen_dia.error_p = check_sen_error_dia(type);
  ee->sen_dia.error_d = ee->sen_dia.error_p - ee->sen_dia.error_d;

  if (type == SensingControlType::DiaPiller && ee->sen_dia.error_p != 0) {
    duty = param_ro->sensor_pid_dia.p * ee->sen_dia.error_p -
           param_ro->sensor_pid_dia.d * sensing_result->ego.w_kf;
    set_ctrl_val(ee->s_val, ee->sen_dia.error_p, 0, 0, ee->sen_dia.error_d,
                 param_ro->sensor_pid_dia.p * ee->sen_dia.error_p,
                 param_ro->sensor_pid_dia.i * ee->sen_dia.error_i, 0,
                 param_ro->sensor_pid_dia.d * ee->sen_dia.error_d, 0, 0);
  } else {
    duty = 0;
    set_ctrl_val(ee->s_val, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }
  float limit = 0;
  if (type == SensingControlType::None) {
    return 0;
  } else if (type == SensingControlType::DiaPiller) {
    limit = interp1d(sensor_deg_limitter_v, sensor_deg_limitter_dia,
                     tgt_val->ego_in.v, false);
  }
  duty = std::clamp(duty, -limit, limit);
  return duty;
}
float IRAM_ATTR PlanningTask::check_sen_error(SensingControlType &type) {
  float error = 0;
  int check = 0;
  float dist_mod = (int)(tgt_val->ego_in.dist / param_ro->dist_mod_num);
  float tmp_dist = tgt_val->ego_in.dist - param_ro->dist_mod_num * dist_mod;

  bool expand_right = false;
  bool expand_left = false;
  bool expand_right_2 = false;
  bool expand_left_2 = false;

  auto wall_th = interp1d(param_ro->clear_dist_ragne_dist_list,
                          param_ro->clear_dist_ragne_th_list, tmp_dist, false);
  // printf("wall_th %f %f\n", wall_th, tmp_dist);

  const auto se = get_sensing_entity();
  const auto prm = get_param();

  auto exist_right45 = prm->sen_ref_p.normal.exist.right45;
  auto exist_left45 = prm->sen_ref_p.normal.exist.left45;
  // auto exist_right45_expand = prm->sen_ref_p.normal.expand.right45;
  // auto exist_left45_expand = prm->sen_ref_p.normal.expand.left45;
  auto exist_right45_expand = wall_th;
  auto exist_left45_expand = wall_th;

  if (search_mode && tgt_val->tgt_in.tgt_dist > 80 &&
      tgt_val->tgt_in.tgt_dist < 100 &&
      tgt_val->motion_type == MotionType::STRAIGHT) {
    expand_right = (10 < se->ego.right45_dist) &&
                   (se->ego.right45_dist < prm->sen_ref_p.search_exist.right45);
    expand_left = (10 < se->ego.left45_dist) &&
                  (se->ego.left45_dist < prm->sen_ref_p.search_exist.left45);
    // } else if (!search_mode && tgt_val->tgt_in.tgt_dist >= 90 &&
    //            tgt_val->motion_type == MotionType::STRAIGHT) {
    //   expand_right = (10 < se->ego.right45_dist) &&
    //                  (se->ego.right45_dist <
    //                  prm->sen_ref_p.search_exist.right45);
    //   expand_left = (10 < se->ego.left45_dist) &&
    //                 (se->ego.left45_dist <
    //                 prm->sen_ref_p.search_exist.left45);
  } else {
    exist_right45_expand = 0;
    exist_left45_expand = 0;
  }
  // auto exist_right45_expand_2 = prm->sen_ref_p.normal.expand.right45_2;
  // auto exist_left45_expand_2 = prm->sen_ref_p.normal.expand.left45_2;
  float val_left = 1000;
  float val_right = 1000;
  //前壁が近すぎるときはエスケープ

  bool range_check_right =
      (1 < se->ego.right45_dist) && (se->ego.right45_dist < exist_right45);
  bool range_check_left =
      (1 < se->ego.left45_dist) && (se->ego.left45_dist < exist_left45);
  bool range_check_right_expand = (1 < se->ego.right45_dist) &&
                                  (se->ego.right45_dist < exist_right45_expand);
  bool range_check_left_expand =
      (1 < se->ego.left45_dist) && (se->ego.left45_dist < exist_left45_expand);
  bool dist_check_right = ABS(tgt_val->global_pos.dist - right_keep.star_dist) >
                          prm->right_keep_dist_th;
  bool dist_check_left = ABS(tgt_val->global_pos.dist - left_keep.star_dist) >
                         prm->left_keep_dist_th;
  bool check_diff_right =
      ABS(se->ego.right45_dist_diff) < prm->sen_ref_p.normal.ref.kireme_r;
  bool check_diff_left =
      ABS(se->ego.left45_dist_diff) < prm->sen_ref_p.normal.ref.kireme_l;
  if (!search_mode) {
    check_diff_right = ABS(se->ego.right45_dist_diff) <
                       prm->sen_ref_p.normal.ref.kireme_r_fast;
    check_diff_left =
        ABS(se->ego.left45_dist_diff) < prm->sen_ref_p.normal.ref.kireme_l_fast;
  }

  bool check_front_left =
      (10 < se->ego.left90_mid_dist) &&
      (se->ego.left90_mid_dist < prm->sen_ref_p.normal.exist.front);
  bool check_front_right =
      (10 < se->ego.right90_mid_dist) &&
      (se->ego.right90_mid_dist < prm->sen_ref_p.normal.exist.front);

  if (!(check_front_left && check_front_right)) {
    if (range_check_right) {
      if (dist_check_right && check_diff_right) {
        error += prm->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
      } else if (expand_right && range_check_right_expand && dist_check_right &&
                 check_diff_right) {
        error += prm->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
      }
      check++;
    } else if (expand_right && range_check_right_expand) {
      if (dist_check_right && check_diff_right) {
        error += prm->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
      }
      check++;
    } else {
      right_keep.star_dist = tgt_val->global_pos.dist;
    }
    if (range_check_left) {
      if (dist_check_left && check_diff_left) {
        error -= prm->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
      } else if (expand_left && range_check_left_expand && dist_check_left &&
                 check_diff_left) {
        error -= param_ro->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
      }
      check++;
    } else if (expand_left && range_check_left_expand) {
      if (dist_check_left && check_diff_left) {
        error -= param_ro->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
      }
      check++;
    } else {
      left_keep.star_dist = tgt_val->global_pos.dist;
    }
    if (check != 0) {
      type = SensingControlType::Wall;
    }
  }

  if (check == 0) {
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
    bool right_check = false;
    bool left_check = false;
    float right_error = 0;
    float left_error = 0;

    const bool range_check_passed_right =
        (prm->sen_ref_p.normal2.ref.kireme_r < se->sen.r45.sensor_dist) &&
        (se->sen.r45.sensor_dist < prm->sen_ref_p.normal2.exist.right45) &&
        (se->sen.r45.sensor_dist + 5) < se->ego.right45_dist;

    const bool range_check_passed_left =
        (prm->sen_ref_p.normal2.ref.kireme_l < se->sen.l45.sensor_dist) &&
        (se->sen.l45.sensor_dist < prm->sen_ref_p.normal2.exist.left45) &&
        (se->sen.l45.sensor_dist + 5) < se->ego.left45_dist;

    const bool exist_right45 =
        se->ego.right45_dist < prm->sen_ref_p.search_exist.right45;
    const bool exist_left45 =
        se->ego.left45_dist < prm->sen_ref_p.search_exist.left45;

    if (!(check_front_left && check_front_right)) {
      if (range_check_passed_right && !exist_left45) {
        error += prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
        right_error +=
            prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
        check++;
        right_check = true;
      }
      if (range_check_passed_left && !exist_right45) {
        error -= prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist;
        left_error -=
            (prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist);
        check++;
        left_check = true;
      }

      if (check == 0) {
        const bool range_check_passed_right_near =
            (prm->sen_ref_p.normal2.ref.right90 < se->sen.r45.sensor_dist) &&
            (se->sen.r45.sensor_dist < prm->sen_ref_p.normal2.ref.kireme_r) &&
            (se->sen.r45.sensor_dist + 5) < se->ego.right45_dist;
        const bool range_check_passed_left_near =
            (prm->sen_ref_p.normal2.ref.left90 < se->sen.l45.sensor_dist) &&
            (se->sen.l45.sensor_dist < prm->sen_ref_p.normal2.ref.kireme_l) &&
            (se->sen.l45.sensor_dist + 5) < se->ego.left45_dist;
        if (!range_check_passed_left_near && range_check_passed_right_near &&
            !exist_left45) {
          error += prm->sen_ref_p.normal2.ref.right45 - se->sen.r45.sensor_dist;
          check++;
        } else if (range_check_passed_left_near &&
                   !range_check_passed_right_near && !exist_right45) {
          error -= prm->sen_ref_p.normal2.ref.left45 - se->sen.l45.sensor_dist;
          check++;
        }
      }
      if (check != 0 && //
          !(tgt_val->motion_type == MotionType::SLA_FRONT_STR)) {
        type = SensingControlType::Piller;
      }
      // error *= prm->sen_ref_p.normal2.exist.front;
    }
  } else {
    // TODO Uターン字は別ロジックに修正
    if (tgt_val->tgt_in.tgt_dist >= prm->clear_dist_order) {
      if (!(prm->clear_dist_ragne_from <= tmp_dist &&
            tmp_dist <= prm->clear_dist_ragne_to)) {
        if ((std::abs(tgt_val->ego_in.ang - tgt_val->ego_in.img_ang) * 180 /
             m_PI) < prm->clear_angle) {
          tgt_val->global_pos.ang = tgt_val->global_pos.img_ang;
          ee->w.error_i = 0;
          ee->w.error_d = 0;
          ee->w.error_dd = 0;
          ee->ang.error_i = 0;
          ee->ang.error_d = 0;
          ee->ang.error_dd = 0;
          w_reset = 0;
        }
      } else {
        // ee->sen.error_i = 0;
        // ee->sen_log.gain_zz = 0;
        // ee->sen_log.gain_z = 0;
        // if (check == 2) {
        //   return error;
        // } else if (check == 1) {
        //   return error * 2;
        // }
      }
    }
  }

  if (check == 2) {
    return error;
  } else if (check == 1) {
    return error * 2;
  }
  ee->sen.error_i = 0;
  ee->sen_log.gain_zz = 0;
  ee->sen_log.gain_z = 0;
  return 0;
}
float IRAM_ATTR PlanningTask::check_sen_error_dia(SensingControlType &type) {
  float error = 0;
  int check = 0;
  const auto se = get_sensing_entity();

  // 斜め移動が一定距離以上するとき、かつ、終わりまでの距離が一定距離未満
  if (tgt_val->tgt_in.tgt_dist > param_ro->sen_ctrl_front_th &&
      (tgt_val->tgt_in.tgt_dist - tgt_val->ego_in.dist) >
          param_ro->sen_ctrl_front_diff_th) {
    // 右前センサーが一定距離以内のとき
    const bool valid_right90 =
        1 < se->ego.right90_mid_dist &&
        se->ego.right90_mid_dist < param_ro->sen_ref_p.dia.exist.right90;
    // 左前センサーが一定距離以内のとき
    const bool valid_left90 =
        1 < se->ego.left90_mid_dist &&
        se->ego.left90_mid_dist < param_ro->sen_ref_p.dia.exist.left90;

    const bool valid_right45 =
        1 < se->sen.r45.sensor_dist &&
        se->sen.r45.sensor_dist < param_ro->sen_ref_p.dia.exist.right45 &&
        se->sen.r45.sensor_dist < se->ego.right45_dist;
    const bool valid_left45 =
        1 < se->sen.l45.sensor_dist &&
        se->sen.l45.sensor_dist < param_ro->sen_ref_p.dia.exist.left45 &&
        se->sen.l45.sensor_dist < se->ego.left45_dist;

    if (valid_right90) {
      error += param_ro->sen_ref_p.dia.ref.right90 - se->ego.right90_mid_dist;
      tgt_val->dia_state.right_old =
          param_ro->sen_ref_p.dia.ref.right90 - se->ego.right90_mid_dist;
      tgt_val->dia_state.right_save = true;
      tgt_val->dia_state.left_save = false;
      check++;
    }
    if (valid_left90) {
      error -= param_ro->sen_ref_p.dia.ref.left90 - se->ego.left90_mid_dist;
      tgt_val->dia_state.left_old =
          param_ro->sen_ref_p.dia.ref.left90 - se->ego.left90_mid_dist;
      tgt_val->dia_state.left_save = true;
      tgt_val->dia_state.right_save = false;
      check++;
    }
    if (!valid_left90 && !valid_right90 &&
        param_ro->sensor_gain.front4.a != 0) {

      if (valid_right45) {
        error += param_ro->sen_ref_p.dia.ref.right45 - se->sen.r45.sensor_dist;
        check++;
      }
      if (valid_left45) {
        error -= param_ro->sen_ref_p.dia.ref.left45 - se->sen.l45.sensor_dist;
        check++;
      }
      if (!(valid_left45 && valid_right45)) {
        if (tgt_val->dia_state.right_save) {
          error += tgt_val->dia_state.right_old;
          check++;
        }
        if (tgt_val->dia_state.left_save) {
          error -= tgt_val->dia_state.left_old;
          check++;
        }
      }
    }
  }
  // }
  if (check == 0) {
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  } else {
    type = SensingControlType::DiaPiller;
    // TODO Uターン字は別ロジックに修正
    // ee->sen_dia.error_i = 0;
    // ee->sen_log_dia.gain_zz = 0;
    // ee->sen_log_dia.gain_z = 0;
    if (tgt_val->tgt_in.tgt_dist >= param_ro->clear_dist_order) {
      if ((std::abs(tgt_val->ego_in.ang - tgt_val->ego_in.img_ang) * 180 /
           m_PI) < param_ro->clear_angle) {
        // tgt_val->global_pos.ang = tgt_val->global_pos.img_ang;
        // ee->w.error_i = 0;
        // ee->w.error_d = 0;
        // ee->ang.error_i = 0;
        // ee->ang.error_d = 0;
      } else {
        // ee->sen.error_i = 0;
        // ee->sen_log.gain_zz = 0;
        // ee->sen_log.gain_z = 0;
        // if (check == 2) {
        //   return error;
        // } else if (check == 1) {
        //   return error * 2;
        // }
      }
    }
  }

  if (check == 2) {
    return error;
  } else if (check == 1) {
    return error * 2;
  }
  return 0;
}

void IRAM_ATTR PlanningTask::calc_vel() {}

void IRAM_ATTR PlanningTask::update_ego_motion() {
  const auto se = get_sensing_entity();
  const float dt = param_ro->dt;
  const float tire = param_ro->tire;
  tgt_val->ego_in.ff_duty_low_th = param_ro->ff_front_dury;
  tgt_val->ego_in.ff_duty_low_v_th = param_ro->ff_v_th;
  if (!motor_en) {
    tgt_val->ego_in.v = 0;
    tgt_val->ego_in.w = 0;
  }
  se->ego.accel_x_raw = param_ro->accel_x_param.gain *
                        (se->accel_x.raw - tgt_val->accel_x_zero_p_offset);

  if (param_ro->comp_param.enable == 1) {
    se->ego.v_lp = (1 - param_ro->comp_param.v_lp_gain) * se->ego.v_c +
                   param_ro->comp_param.v_lp_gain * se->ego.v_lp;
  } else if (param_ro->comp_param.enable == 2) {
    // sensing_result->ego.v_lp =
    //     (1 - param_ro->comp_param.v_lp_gain) * sum_v / enc_v_q.size() +
    //     param_ro->comp_param.v_lp_gain * sensing_result->ego.v_lp;
    // printf("%f, %f %f %d\n", sensing_result->ego.v_lp,
    // sensing_result->ego.v_c,
    //        sum_v, enc_v_q.size());
  }

  // sensing_result->ego.main_v = sum_v / enc_v_q.size();
  // param_ro->comp_param.gain *
  //     (sensing_result->ego.main_v + sensing_result->ego.accel_x_raw * dt) +
  // (1 - param_ro->comp_param.gain) * sensing_result->ego.v_lp;

  // if (param_ro->comp_param.enable == 0) {
  // } else {
  //   tgt_val->ego_in.dist += sensing_result->ego.main_v * dt;
  //   tgt_val->global_pos.dist += sensing_result->ego.main_v * dt;
  // }

  se->ego.w_lp = se->ego.w_lp * (1 - param_ro->gyro_param.lp_delay) +
                 se->ego.w_raw * param_ro->gyro_param.lp_delay;

  // kf_w.predict(sensing_result->ego.w_raw);
  // kf_w.update(sensing_result->ego.w_raw);

  // if (std::isfinite(tgt_val->ego_in.alpha) && std::isfinite(se->ego.w_lp)) {
  //   for (const auto gyro : se->gyro_list) {
  //     if (tgt_val->motion_dir == MotionDirection::LEFT) {
  //       se->ego.w_raw = param_ro->gyro_param.gyro_w_gain_left *
  //                       (gyro - tgt_val->gyro_zero_p_offset);
  //     } else {
  //       se->ego.w_raw = param_ro->gyro_param.gyro_w_gain_right *
  //                       (gyro - tgt_val->gyro_zero_p_offset);
  //     }
  //     kf_w.predict(tgt_val->ego_in.alpha);
  //     kf_w.update(se->ego.w_raw);
  //   }
  //   // kf_w.predict(tgt_val->ego_in.alpha);
  //   // kf_w.update(sensing_result->ego.w_lp);
  //   se->ego.w_kf = kf_w.get_state();
  // }
  if (std::isfinite(tgt_val->ego_in.accl) && std::isfinite(se->ego.v_c)) {
    auto tmp_v_l = kf_v_l.get_state();
    auto tmp_v_r = kf_v_r.get_state();
    kf_v.predict(tgt_val->ego_in.accl);
    kf_v.update((tmp_v_l + tmp_v_r) / 2);
    se->ego.v_kf = kf_v.get_state();
    // printf("kf_v: %f\n", se->ego.v_kf);
    // kf_v.print_state();
  }

  if (std::isfinite(tgt_val->ego_in.v)) {
    kf_dist.predict(tgt_val->ego_in.v);
    kf_dist.update(tgt_val->ego_in.dist);
    se->ego.dist_kf = kf_dist.get_state();
  }
  if (std::isfinite(tgt_val->ego_in.w)) {
    kf_ang.predict(tgt_val->ego_in.w);
    kf_ang.update(tgt_val->ego_in.ang);

    if (param_ro->enable_kalman_gyro == 1) {
      se->ego.ang_kf = kf_ang.get_state();
    } else if (param_ro->enable_kalman_gyro == 2) {
      se->ego.ang_kf = tgt_val->ego_in.ang;
    } else {
      se->ego.ang_kf = tgt_val->ego_in.ang;
    }

    // const auto angle = kf_ang.get_state();
    // se->ego.ang_kf = fmod(angle + M_PI, 2 * M_PI) - M_PI;
  }

  if (!(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::FRONT_CTRL)) {
    if (std::isfinite(se->ego.v_kf) && std::isfinite(se->ego.ang_kf)) {
      pos.ang += se->ego.w_kf * dt;
      pos.ang = fmod(pos.ang + M_PI, 2 * M_PI) - M_PI;
      tgt_val->ego_in.pos_x += se->ego.v_kf * cosf(pos.ang) * dt;
      tgt_val->ego_in.pos_y += se->ego.v_kf * sinf(pos.ang) * dt;
      pos.predict(tgt_val->ego_in.v, tgt_val->ego_in.w, dt);
      const std::array<float, 3> z = {tgt_val->ego_in.pos_x, //
                                      tgt_val->ego_in.pos_y, //
                                      pos.ang};
      pos.update(z);
      const auto pos_state = pos.get_state();
      se->ego.pos_x = pos_state[0];
      se->ego.pos_y = pos_state[1];
    }
  }

  se->ego.battery_raw = se->battery.data;

  se->ego.battery_lp =
      se->ego.battery_lp * (1 - param_ro->battery_param.lp_delay) +
      se->ego.battery_raw * param_ro->battery_param.lp_delay;

  kf_batt.predict(0);
  kf_batt.update(se->ego.battery_raw);
  se->ego.batt_kf = kf_batt.get_state();

  se->ego.left45_lp_old = se->ego.left45_lp;
  se->ego.left90_lp_old = se->ego.left90_lp;
  se->ego.front_lp_old = se->ego.front_lp;
  se->ego.right45_lp_old = se->ego.right45_lp;
  se->ego.right90_lp_old = se->ego.right90_lp;

  // sensing_result->ego.right45_2_lp_old = sensing_result->ego.right45_2_lp;
  // sensing_result->ego.left45_2_lp_old = sensing_result->ego.left45_2_lp;

  se->ego.right90_raw = se->led_sen.right90.raw;
  se->ego.right90_lp = se->ego.right90_lp * (1 - param_ro->led_param.lp_delay) +
                       se->ego.right90_raw * param_ro->led_param.lp_delay;

  se->ego.right45_raw = se->led_sen.right45.raw;
  se->ego.right45_lp = se->ego.right45_lp * (1 - param_ro->led_param.lp_delay) +
                       se->ego.right45_raw * param_ro->led_param.lp_delay;

  se->ego.front_raw = se->led_sen.front.raw;
  se->ego.front_lp = se->ego.front_lp * (1 - param_ro->led_param.lp_delay) +
                     se->ego.front_raw * param_ro->led_param.lp_delay;

  se->ego.left45_raw = se->led_sen.left45.raw;
  se->ego.left45_lp = se->ego.left45_lp * (1 - param_ro->led_param.lp_delay) +
                      se->ego.left45_raw * param_ro->led_param.lp_delay;

  se->ego.left90_raw = se->led_sen.left90.raw;
  se->ego.left90_lp = se->ego.left90_lp * (1 - param_ro->led_param.lp_delay) +
                      se->ego.left90_raw * param_ro->led_param.lp_delay;

  // sensing_result->ego.right45_2_raw = sensing_result->led_sen.right45_2.raw;
  // sensing_result->ego.right45_2_lp =
  //     sensing_result->ego.right45_2_lp * (1 - param_ro->led_param.lp_delay) +
  //     sensing_result->ego.right45_2_raw * param_ro->led_param.lp_delay;

  // sensing_result->ego.left45_2_raw = sensing_result->led_sen.left45_2.raw;
  // sensing_result->ego.left45_2_lp =
  //     sensing_result->ego.left45_2_lp * (1 - param_ro->led_param.lp_delay) +
  //     sensing_result->ego.left45_2_raw * param_ro->led_param.lp_delay;
  // コピー
  tgt_val->ego_in.slip_point.w = se->ego.w_lp;
}
bool IRAM_ATTR PlanningTask::judge_motor_pwm(float duty, uint8_t type) {
  if (type == 1) {
    return duty > 0;
  } else {
    return duty < 0;
  }
  return true;
}

void IRAM_ATTR PlanningTask::limitter(float &kp, float &ki, float &kb,
                                      float &kd, pid_param_t &limitter) {
  if (limitter.mode == 0) {
    return;
  }
  if (kp > limitter.p) {
    kp = limitter.p;
  } else if (kp < -limitter.p) {
    kp = -limitter.p;
  }
  if (ki > limitter.i) {
    ki = limitter.i;
  } else if (ki < -limitter.i) {
    ki = -limitter.i;
  }
  if (kb > limitter.b) {
    kb = limitter.b;
  } else if (kb < -limitter.b) {
    kb = -limitter.b;
  }
  if (kd > limitter.d) {
    kd = limitter.d;
  } else if (kd < -limitter.d) {
    kd = -limitter.d;
  }
}

void IRAM_ATTR PlanningTask::change_pwm_freq(float duty_l, float duty_r) {
  if (param_ro->motor_debug_mode != 0) {
    duty_r = param_ro->motor_debug_mode_duty_r;
    duty_l = param_ro->motor_debug_mode_duty_l;
  }
  float duty_r_abs = duty_r > 0 ? duty_r : -duty_r;
  float duty_l_abs = duty_l > 0 ? duty_l : -duty_l;
  if (param_ro->motor_driver_type == MotorDriveType::TWO_PWM) {
    if (judge_motor_pwm(duty_l, param_ro->motor_l_cw_ccw_type)) {
      // printf("duty_l[A] %f\n", duty_l);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_l_abs);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                          MCPWM_DUTY_MODE_0);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                          MCPWM_DUTY_MODE_0);
    } else {
      // printf("duty_l[B] %f\n", duty_l);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                          MCPWM_DUTY_MODE_0);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_l_abs);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                          MCPWM_DUTY_MODE_0);
    }
    if (judge_motor_pwm(duty_r, param_ro->motor_r_cw_ccw_type)) {
      // printf("duty_r[A] %f\n", duty_r);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_r_abs);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                          MCPWM_DUTY_MODE_0);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
                          MCPWM_DUTY_MODE_0);
    } else {
      // printf("duty_r[B] %f\n", duty_r);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                          MCPWM_DUTY_MODE_0);
      mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty_r_abs);
      mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
                          MCPWM_DUTY_MODE_0);
    }
  } else if (param_ro->motor_driver_type == MotorDriveType::EN1_PH1) {
    if (judge_motor_pwm(duty_l, param_ro->motor_l_cw_ccw_type)) {
      set_gpio_state(L_CW_CCW1, true);
    } else {
      set_gpio_state(L_CW_CCW1, false);
    }
    if (judge_motor_pwm(duty_r, param_ro->motor_r_cw_ccw_type)) {
      set_gpio_state(R_CW_CCW1, true);
    } else {
      set_gpio_state(R_CW_CCW1, false);
    }
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_l_abs);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_r_abs);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
  } else if (param_ro->motor_driver_type == MotorDriveType::EN1_PH2) {
    uint32_t high = 0;
    uint32_t low = 0;

    if (judge_motor_pwm(duty_l, param_ro->motor_l_cw_ccw_type)) {
      high |= L_CW_CCW1_BIT;
      low |= L_CW_CCW2_BIT;
    } else {
      high |= L_CW_CCW2_BIT;
      low |= L_CW_CCW1_BIT;
    }
    if (judge_motor_pwm(duty_r, param_ro->motor_r_cw_ccw_type)) {
      high |= R_CW_CCW1_BIT;
      low |= R_CW_CCW2_BIT;
    } else {
      high |= R_CW_CCW2_BIT;
      low |= R_CW_CCW1_BIT;
    }

    GPIO.out1_w1ts.val = high;
    GPIO.out1_w1tc.val = low;
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_l_abs);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_r_abs);
  }
  // auto start_que_rec = esp_timer_get_time();

  // MCPWM[MCPWM_UNIT_0]->timer[MCPWM_TIMER_0].cmpr_value[MCPWM_OPR_A].cmpr_val
  // = duty_l_abs;
  // MCPWM[MCPWM_UNIT_0]->timer[MCPWM_TIMER_1].cmpr_value[MCPWM_OPR_A].cmpr_val
  // = duty_r_abs;

  // auto end_que_rec = esp_timer_get_time();
  // tgt_val->calc_time2 = end_que_rec - start_que_rec;
  // }
}

void IRAM_ATTR PlanningTask::set_next_duty(float duty_l, float duty_r,
                                           float duty_suction) {
  if (motor_en) {
    // if (receive_req->nmr.sys_id.enable) {
    //   duty_l =
    //       ABS(receive_req->nmr.sys_id.left_v / sensing_result->ego.batt_kf *
    //       100);
    //   duty_r = ABS(receive_req->nmr.sys_id.right_v /
    //   sensing_result->ego.batt_kf *
    //                100);
    //   // duty_suction = receive_req->nmr.sys_id.duty_suction;
    // }
    change_pwm_freq(duty_l, duty_r);
  }
  if (suction_en) {
    float duty_suction_in = 0;

    if (tgt_val->tgt_in.tgt_dist > 60 &&
        (tgt_val->ego_in.state == 0 || tgt_val->ego_in.state == 1) &&
        tgt_val->motion_type == MotionType::STRAIGHT) {
      duty_suction_in =
          100.0 * tgt_duty.duty_suction_low / sensing_result->ego.batt_kf;
    } else {
      duty_suction_in =
          100.0 * tgt_duty.duty_suction / sensing_result->ego.batt_kf;
    }
    if (duty_suction_in > 100) {
      duty_suction_in = 100.0;
    }
    gain_cnt += 1.0;
    if (gain_cnt > suction_gain) {
      gain_cnt = suction_gain;
    }
    duty_suction_in = duty_suction_in * gain_cnt / suction_gain;
    if (duty_suction_in > 100) {
      duty_suction_in = 100;
    }
    // printf("duty_suction_in: %f\n", duty_suction_in);
    // mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A, duty_suction_in);
    // mcpwm_set_duty_type(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A,
    //                     MCPWM_DUTY_MODE_0);
  }
}

void PlanningTask::init_gpio() {
  gpio_config_t io_conf;
  // 割り込みをしない
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // 出力モード
  io_conf.mode = GPIO_MODE_OUTPUT;
  // 設定したいピンのビットマスク
  io_conf.pin_bit_mask = 0;

  io_conf.pin_bit_mask |= 1ULL << L_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << R_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << L_CW_CCW2;
  io_conf.pin_bit_mask |= 1ULL << R_CW_CCW2;

  // 内部プルダウンしない
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // 内部プルアップしない
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // 設定をセットする
  gpio_config(&io_conf);
  gpio_set_level(SUCTION_PWM, 0);
  // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8,
}

void IRAM_ATTR PlanningTask::pl_req_activate() {
  if (receive_req->pl_req.time_stamp != pid_req_timestamp) {
    if (receive_req->pl_req.error_gyro_reset == 1) {
      ee->v.error_i = 0;
    }
    if (receive_req->pl_req.error_vel_reset == 1) {
      ee->dist.error_i = 0;
    }
    if (receive_req->pl_req.error_dist_reset == 1) {
      ee->w.error_i = 0;
    }
    if (receive_req->pl_req.error_ang_reset == 1) {
      ee->ang.error_i = 0;
    }
    if (receive_req->pl_req.error_led_reset == 1) {
      // ee->led.error_i = 0;
    }
    // if (tgt_val->pl_req.log_start == 1) {
    //   log_active = true;
    // }
    // if (tgt_val->pl_req.log_end == 1) {
    //   log_active = false;
    // }
    pid_req_timestamp = receive_req->pl_req.time_stamp;
  }
}

float IRAM_ATTR PlanningTask::get_feadforward_front(TurnDirection td) {
  return 0;
}
float IRAM_ATTR PlanningTask::get_feadforward_front() { return 0; }
float IRAM_ATTR PlanningTask::get_feadforward_roll() { return 0; }
float IRAM_ATTR PlanningTask::get_rpm_ff_val(TurnDirection td) { return 0; }
float IRAM_ATTR PlanningTask::satuate_sen_duty(float duty_sen) {
  return duty_sen;
}
void IRAM_ATTR PlanningTask::calc_tgt_duty() {

  float duty_ff_front = 0;
  float duty_ff_roll = 0;
  const unsigned char reset_req = motor_en ? 1 : 0;
  const unsigned char enable = 1;
  duty_sen = 0;
  sen_ang = 0;

  ee->s_val.p = ee->s_val.i = ee->s_val.d = 0;
  ee->s_val.p_val = ee->s_val.i_val = ee->s_val.d_val = 0;
  ee->s_val.z = ee->s_val.zz = 0;

  if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
    duty_sen = calc_sensor_pid();
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  } else if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
    duty_sen = calc_sensor_pid_dia();
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
  } else if (tgt_val->nmr.sct == SensorCtrlType::NONE) {
    duty_sen = sen_ang = 0;
    ee->sen.error_i = 0;
    ee->sen_log.gain_zz = 0;
    ee->sen_log.gain_z = 0;
    ee->sen_dia.error_i = 0;
    ee->sen_log_dia.gain_zz = 0;
    ee->sen_log_dia.gain_z = 0;
  }
  sensing_result->ego.duty.sen = duty_sen;
  sensing_result->ego.duty.sen_ang = sen_ang;

  // 重心速度PID偏差計算
  calc_pid_val();
  // 角度PID偏差計算
  calc_pid_val_ang();
  // 角速度PID偏差計算
  calc_pid_val_ang_vel();
  // 前壁制御PID偏差計算
  calc_pid_val_front_ctrl();

  duty_c = 0;
  duty_c2 = 0;
  duty_roll = 0;
  duty_front_ctrl_roll_keep = 0;
  duty_roll_ang = 0;
  duty_front_ctrl_trans = 0;
  duty_front_ctrl_roll = 0;
  reset_pid_val();

  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    calc_front_ctrl_duty();
  } else {
    // 重心速度制御
    calc_translational_ctrl();
    // 角速度制御
    calc_angle_velocity_ctrl();
  }
  sensing_result->ego.duty.sen = duty_sen;

  summation_duty();

  // Duty limitter
  apply_duty_limitter();

  if (tgt_val->motion_type == MotionType::NONE) {
    tgt_duty.duty_l = tgt_duty.duty_r = 0;
  }
  if (!motor_en) {
    clear_ctrl_val();
  }

  // set duty for log
  sensing_result->ego.duty.duty_r = tgt_duty.duty_r;
  sensing_result->ego.duty.duty_l = tgt_duty.duty_l;

  sensing_result->ego.duty.ff_duty_front = mpc_next_ego.ff_duty_front;
  sensing_result->ego.duty.ff_duty_roll = mpc_next_ego.ff_duty_roll;
  sensing_result->ego.duty.ff_duty_rpm_r = mpc_next_ego.ff_duty_rpm_r;
  sensing_result->ego.duty.ff_duty_rpm_l = mpc_next_ego.ff_duty_rpm_l;

  // sensing_result->ego.ff_duty.front = duty_ff_front;
  // sensing_result->ego.ff_duty.roll = duty_ff_roll;
  w_reset = 1;
  // copy_error_entity(error_entity);
}

void IRAM_ATTR PlanningTask::cp_tgt_val() {
  const auto se = get_sensing_entity();
  tgt_val->ego_in.accl = mpc_next_ego.accl;
  tgt_val->ego_in.alpha = mpc_next_ego.alpha;
  tgt_val->ego_in.pivot_state = mpc_next_ego.pivot_state;
  tgt_val->ego_in.sla_param = mpc_next_ego.sla_param;
  tgt_val->ego_in.state = mpc_next_ego.state;
  tgt_val->ego_in.decel_delay_cnt = mpc_next_ego.decel_delay_cnt;

  const auto tmp_v = tgt_val->ego_in.v;
  tgt_val->ego_in.v = mpc_next_ego.v;
  tgt_val->ego_in.v_l =
      mpc_next_ego.v - mpc_next_ego.w * param_ro->tire_tread / 2;
  tgt_val->ego_in.v_r =
      mpc_next_ego.v + mpc_next_ego.w * param_ro->tire_tread / 2;
  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.v < 10) {
      tgt_val->ego_in.v = tmp_v;
    }
    se->sen.r45.sensor_dist = 0;
    se->sen.l45.sensor_dist = 0;
  }
  tgt_val->ego_in.w = mpc_next_ego.w;
  tgt_val->ego_in.sla_param.state = mpc_next_ego.sla_param.state;
  tgt_val->ego_in.sla_param.counter = mpc_next_ego.sla_param.counter;
  tgt_val->ego_in.sla_param.state = mpc_next_ego.sla_param.state;

  tgt_val->ego_in.img_ang = mpc_next_ego.img_ang;
  tgt_val->ego_in.img_dist = mpc_next_ego.img_dist;

  tgt_val->global_pos.img_ang += mpc_next_ego.w * dt;

  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.pivot_state == 3) {
      tgt_val->global_pos.img_ang = //
          tgt_val->ego_in.img_ang = tgt_val->tgt_in.tgt_angle;
    }
  }

  tgt_val->global_pos.img_dist += mpc_next_ego.v * dt;

  tgt_val->ego_in.slip_point.slip_angle = mpc_next_ego.slip_point.slip_angle;

  tgt_val->ego_in.cnt_delay_accl_ratio = mpc_next_ego.cnt_delay_accl_ratio;
  tgt_val->ego_in.cnt_delay_decel_ratio = mpc_next_ego.cnt_delay_decel_ratio;

  // const auto theta = tgt_val->ego_in.img_ang + slip_param.beta;
  // const auto x = tgt_val->ego_in.v * std::cos(theta);
  // const auto y = tgt_val->ego_in.v * std::sin(theta);
  // tgt_val->p.x += x;
  // tgt_val->p.y += y;

  tgt_val->ego_in.slip.beta = mpc_next_ego.slip.beta;
  tgt_val->ego_in.slip.accl = mpc_next_ego.slip.accl;
  tgt_val->ego_in.slip.v = mpc_next_ego.slip.v;
  tgt_val->ego_in.slip.vx = mpc_next_ego.slip.vx;
  tgt_val->ego_in.slip.vy = mpc_next_ego.slip.vy;

  ideal_v_r = tgt_val->ego_in.v - tgt_val->ego_in.w * param_ro->tire_tread / 2;
  ideal_v_l = tgt_val->ego_in.v + tgt_val->ego_in.w * param_ro->tire_tread / 2;
}

void IRAM_ATTR PlanningTask::check_fail_safe() {
  bool no_problem = true;

  if (!motor_en) {
    tgt_val->fss.error = 0;
    return;
  }
  if (ABS(ee->ang.error_p) > param_ro->fail_check_ang_th) {
    fail_check_ang++;
  } else {
    fail_check_ang = 0;
  }
  if (tgt_val->motion_type == MotionType::WALL_OFF ||
      tgt_val->motion_type == MotionType::WALL_OFF_DIA) {
    keep_wall_off_cnt++;
  } else {
    keep_wall_off_cnt = 0;
  }

  if (ABS(ee->v.error_i) > param_ro->fail_check.v) {
    tgt_val->fss.error = 1;
  }
  if (ABS(ee->w.error_i) > param_ro->fail_check.w) {
    tgt_val->fss.error = 1;
  }
  if (fail_check_ang > param_ro->fail_check.ang) {
    tgt_val->fss.error = 1;
  }
  if (keep_wall_off_cnt > param_ro->fail_check.wall_off) {
    tgt_val->fss.error = 1;
  }

  // if (!std::isfinite(tgt_duty.duty_l) || !std::isfinite(tgt_duty.duty_r))
  // {
  //   tgt_val->fss.error = 1;
  // }
}

void IRAM_ATTR PlanningTask::cp_request() {
  // tgt_val->tgt_in.mass = param_ro->Mass;

  const auto se = get_sensing_entity();
  pl_req_activate();
  if (motion_req_timestamp == receive_req->nmr.timstamp) {
    return;
  }
  const float dt = param_ro->dt;
  slip_param.K = param_ro->slip_param_K;
  slip_param.k = param_ro->slip_param_k2;
  motion_req_timestamp = receive_req->nmr.timstamp;

  tgt_val->tgt_in.v_max = receive_req->nmr.v_max;
  // printf("v_max: %f\n", tgt_val->tgt_in.v_max);

  tgt_val->td = TurnDirection::None;
  tgt_val->tt = TurnType::None;
  if (receive_req->nmr.motion_type == MotionType::STRAIGHT ||
      receive_req->nmr.motion_type == MotionType::SLA_FRONT_STR) {
    if (tgt_val->ego_in.v > receive_req->nmr.v_max) {
      tgt_val->tgt_in.v_max = tgt_val->ego_in.v;
    }
  } else if (receive_req->nmr.motion_type == MotionType::SLALOM) {
    tgt_val->td = receive_req->nmr.td;
    tgt_val->tt = receive_req->nmr.tt;
  }

  tgt_val->tgt_in.end_v = receive_req->nmr.v_end;
  tgt_val->tgt_in.accl = receive_req->nmr.accl;
  tgt_val->tgt_in.decel = receive_req->nmr.decel;
  tgt_val->tgt_in.w_max = receive_req->nmr.w_max;
  tgt_val->tgt_in.end_w = receive_req->nmr.w_end;
  tgt_val->tgt_in.alpha = receive_req->nmr.alpha;

  tgt_val->tgt_in.tgt_dist = receive_req->nmr.dist;
  last_tgt_angle = tgt_val->tgt_in.tgt_angle;
  tgt_val->tgt_in.tgt_angle = receive_req->nmr.ang;

  tgt_val->motion_mode = (int)(receive_req->nmr.motion_mode);
  tgt_val->motion_type = receive_req->nmr.motion_type;

  tgt_val->ego_in.sla_param.base_alpha = receive_req->nmr.sla_alpha;
  tgt_val->ego_in.sla_param.base_time = receive_req->nmr.sla_time;
  tgt_val->ego_in.sla_param.limit_time_count =
      receive_req->nmr.sla_time * 2 / dt;
  tgt_val->ego_in.sla_param.pow_n = receive_req->nmr.sla_pow_n;

  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.pivot_state = 0;

  tgt_val->dia_state.left_save = receive_req->dia_state.right_save = false;
  tgt_val->dia_state.left_old = receive_req->dia_state.right_old = 0;

  tgt_val->dia_state.left_save = false;
  tgt_val->dia_state.right_save = false;
  tgt_val->dia_state.left_old = tgt_val->dia_state.right_old = 0;

  // if (!(tgt_val->motion_type == MotionType::NONE ||
  //       tgt_val->motion_type == MotionType::STRAIGHT ||
  //       tgt_val->motion_type == MotionType::PIVOT_PRE ||
  //       tgt_val->motion_type == MotionType::PIVOT_AFTER ||
  //       tgt_val->motion_type == MotionType::READY ||
  //       tgt_val->motion_type == MotionType::SENSING_DUMP ||
  //       tgt_val->motion_type == MotionType::WALL_OFF ||
  //       tgt_val->motion_type == MotionType::WALL_OFF_DIA ||
  //       tgt_val->motion_type == MotionType::BACK_STRAIGHT)) {
  //   tgt_val->ego_in.ang -= last_tgt_angle;
  //   tgt_val->ego_in.img_ang = 0;
  //   kf_ang.offset(-last_tgt_angle);
  // } else if (tgt_val->motion_type == MotionType::NONE) {
  //   tgt_val->ego_in.ang = tgt_val->ego_in.img_ang = last_tgt_angle = 0;
  //   kf_ang.reset(0);
  //   tgt_val->ego_in.img_dist = tgt_val->ego_in.dist = 0;
  //   kf_dist.reset(0);
  // }

  if (tgt_val->motion_type == MotionType::NONE ||
      tgt_val->motion_type == MotionType::PIVOT ||
      tgt_val->motion_type == MotionType::FRONT_CTRL ||
      tgt_val->motion_type == MotionType::BACK_STRAIGHT ||
      tgt_val->motion_type == MotionType::PIVOT_AFTER) {
    tgt_val->ego_in.ang = tgt_val->ego_in.img_ang = last_tgt_angle = 0;
    kf_ang.reset(0);
    tgt_val->ego_in.img_dist = tgt_val->ego_in.dist = 0;
    kf_dist.reset(0);
  } else {
    tgt_val->ego_in.ang -= last_tgt_angle;
    tgt_val->ego_in.img_ang = 0;
    kf_ang.offset(-last_tgt_angle);
  }

  if (tgt_val->motion_type == MotionType::NONE ||
      tgt_val->motion_type == MotionType::READY) {
    last_tgt_angle = 0;
    tgt_val->global_pos.ang -= tgt_val->global_pos.img_ang;
    tgt_val->global_pos.img_ang = 0;

    tgt_val->global_pos.dist -= tgt_val->global_pos.img_dist;
    tgt_val->global_pos.img_dist = 0;
  }

  right_keep.star_dist = tgt_val->global_pos.dist;
  left_keep.star_dist = tgt_val->global_pos.dist;

  if (tgt_val->tgt_in.tgt_angle != 0) {
    const auto tmp_ang = tgt_val->ego_in.ang;
    tgt_val->ego_in.img_ang -= last_tgt_angle;
    kf_ang.offset(-last_tgt_angle);
    tgt_val->ego_in.ang = 0;
    // } else {
    //   kf_ang.reset(0);
  }
  if (tgt_val->tgt_in.tgt_dist != 0) {
    const auto tmp_dist = tgt_val->ego_in.dist;
    tgt_val->ego_in.img_dist -= tmp_dist;
    kf_dist.offset(-tmp_dist);
    tgt_val->ego_in.dist = 0;
    // } else {
    //   kf_dist.reset(0);
  }

  sensing_result->ego.dist_kf = kf_dist.get_state();
  // sensing_result->ego.ang_kf = kf_ang.get_state();

  if (param_ro->enable_kalman_gyro == 1) {
    se->ego.ang_kf = kf_ang.get_state();
  } else if (param_ro->enable_kalman_gyro == 2) {
    se->ego.ang_kf = tgt_val->ego_in.ang;
  } else {
    se->ego.ang_kf = tgt_val->ego_in.ang;
  }

  tgt_val->ego_in.sla_param.counter = 1;
  tgt_val->ego_in.sla_param.state = 0;

  tgt_val->motion_dir = receive_req->nmr.motion_dir;
  tgt_val->dia_mode = receive_req->nmr.dia_mode;

  tgt_val->tgt_in.accl_param.limit = 5500;
  tgt_val->tgt_in.accl_param.n = 4;

  tgt_val->tgt_in.slip_gain_K1 = param_ro->slip_param_K;
  tgt_val->tgt_in.slip_gain_K2 = param_ro->slip_param_k2;
  if (receive_req->nmr.motion_type == MotionType::SLALOM) {
    tgt_val->ego_in.v = receive_req->nmr.v_max;
  }

  if (receive_req->nmr.motion_type == MotionType::STRAIGHT) {
    // if (se->sen.r45.sensor_dist == 0 || se->sen.r45.sensor_dist == 180) {
    //   se->sen.r45.sensor_dist = param_ro->sen_ref_p.normal.ref.right45;
    //   se->sen.r45.global_run_dist =
    //       tgt_val->global_pos.dist - param_ro->wall_off_hold_dist;
    // }
    // if (se->sen.l45.sensor_dist == 0 || se->sen.l45.sensor_dist == 180) {
    //   se->sen.l45.sensor_dist = param_ro->sen_ref_p.normal.ref.left45;
    //   se->sen.l45.global_run_dist =
    //       tgt_val->global_pos.dist - param_ro->wall_off_hold_dist;
    // }
  }
}
float IRAM_ATTR PlanningTask::calc_sensor(float data, float a, float b) {
  int idx = (int)data;
  if (idx <= param_ro->sensor_range_min || idx >= log_table.size()) {
    return param_ro->sensor_range_max;
  }
  auto res = a / log_table.at(idx) - b;
  if (res < param_ro->sensor_range_min || res > param_ro->sensor_range_max) {
    return param_ro->sensor_range_max;
  }
  if (!isfinite(res)) {
    return param_ro->sensor_range_max;
  }
  return res;
}

void IRAM_ATTR PlanningTask::calc_sensor_dist_all() {
  const auto se = get_sensing_entity();
  if (!(tgt_val->motion_type == MotionType::NONE ||
        tgt_val->motion_type == MotionType::PIVOT)) {
    se->ego.left90_dist_old = se->ego.left90_dist;
    se->ego.left45_dist_old = se->ego.left45_dist;
    se->ego.front_dist_old = se->ego.front_dist;
    se->ego.right45_dist_old = se->ego.right45_dist;
    se->ego.right90_dist_old = se->ego.right90_dist;

    se->ego.left90_dist =
        calc_sensor(se->ego.left90_lp, param_ro->sensor_gain.l90.a,
                    param_ro->sensor_gain.l90.b);
    se->ego.left45_dist =
        calc_sensor(se->ego.left45_lp, param_ro->sensor_gain.l45.a,
                    param_ro->sensor_gain.l45.b);
    se->ego.right45_dist =
        calc_sensor(se->ego.right45_lp, param_ro->sensor_gain.r45.a,
                    param_ro->sensor_gain.r45.b);
    se->ego.right90_dist =
        calc_sensor(se->ego.right90_lp, param_ro->sensor_gain.r90.a,
                    param_ro->sensor_gain.r90.b);

    se->ego.left90_far_dist =
        calc_sensor(se->ego.left90_lp, param_ro->sensor_gain.l90_far.a,
                    param_ro->sensor_gain.l90_far.b);
    se->ego.right90_far_dist =
        calc_sensor(se->ego.right90_lp, param_ro->sensor_gain.r90_far.a,
                    param_ro->sensor_gain.r90_far.b);

    se->ego.left90_mid_dist =
        calc_sensor(se->ego.left90_lp, param_ro->sensor_gain.l90_mid.a,
                    param_ro->sensor_gain.l90_mid.b);
    se->ego.right90_mid_dist =
        calc_sensor(se->ego.right90_lp, param_ro->sensor_gain.r90_mid.a,
                    param_ro->sensor_gain.r90_mid.b);

    if (se->ego.left90_dist < param_ro->sensor_range_far_max &&
        se->ego.right90_dist < param_ro->sensor_range_far_max) {
      se->ego.front_dist = (se->ego.left90_dist + se->ego.right90_dist) / 2;
    } else if (se->ego.left90_dist > param_ro->sensor_range_far_max &&
               se->ego.right90_dist < param_ro->sensor_range_far_max) {
      se->ego.front_dist = se->ego.right90_dist;
    } else if (se->ego.left90_dist < param_ro->sensor_range_far_max &&
               se->ego.right90_dist > param_ro->sensor_range_far_max) {
      se->ego.front_dist = se->ego.left90_dist;
    } else {
      se->ego.front_dist = param_ro->sensor_range_max;
    }

    if (se->ego.left90_far_dist < param_ro->sensor_range_far_max &&
        se->ego.right90_far_dist < param_ro->sensor_range_far_max) {
      se->ego.front_far_dist =
          (se->ego.left90_far_dist + se->ego.right90_far_dist) / 2;
    } else if (se->ego.left90_far_dist > param_ro->sensor_range_far_max &&
               se->ego.right90_far_dist < param_ro->sensor_range_far_max) {
      se->ego.front_far_dist = se->ego.right90_far_dist;
    } else if (se->ego.left90_far_dist < param_ro->sensor_range_far_max &&
               se->ego.right90_far_dist > param_ro->sensor_range_far_max) {
      se->ego.front_far_dist = se->ego.left90_far_dist;
    } else {
      se->ego.front_far_dist = param_ro->sensor_range_max;
    }
    if (se->ego.left90_mid_dist < param_ro->sensor_range_far_max &&
        se->ego.right90_mid_dist < param_ro->sensor_range_far_max) {
      se->ego.front_mid_dist =
          (se->ego.left90_mid_dist + se->ego.right90_mid_dist) / 2;
    } else if (se->ego.left90_mid_dist > param_ro->sensor_range_far_max &&
               se->ego.right90_mid_dist < param_ro->sensor_range_far_max) {
      se->ego.front_mid_dist = se->ego.right90_mid_dist;
    } else if (se->ego.left90_mid_dist < param_ro->sensor_range_far_max &&
               se->ego.right90_mid_dist > param_ro->sensor_range_far_max) {
      se->ego.front_mid_dist = se->ego.left90_mid_dist;
    } else {
      se->ego.front_mid_dist = param_ro->sensor_range_max;
    }
  } else {
    se->ego.left90_dist        //
        = se->ego.left45_dist  //
        = se->ego.front_dist   //
        = se->ego.right45_dist //
        = se->ego.right90_dist = param_ro->sensor_range_max;
    se->ego.left45_dist_diff = 0;
    se->ego.right45_dist_diff = 0;
  }

  se->ego.left45_dist_diff = se->ego.left45_dist - se->ego.left45_dist_old;

  se->ego.right45_dist_diff = se->ego.right45_dist - se->ego.right45_dist_old;

  // 壁からの距離に変換。あとで斜め用に変更
  calc_sensor_dist_diff();
}

void IRAM_ATTR PlanningTask::calc_sensor_dist_diff() {
  const auto se = get_sensing_entity();

  if (se->sen.l45.sensor_dist > se->ego.left45_dist ||
      se->sen.l45.sensor_dist == 0) {
    se->sen.l45.sensor_dist = se->ego.left45_dist;
    se->sen.l45.global_run_dist = tgt_val->global_pos.dist;
  } else {
    if (((tgt_val->global_pos.dist - se->sen.l45.global_run_dist) >
         param_ro->wall_off_hold_dist) &&
        se->ego.left45_dist < param_ro->sen_ref_p.normal2.exist.left90) {
      se->sen.l45.sensor_dist = se->ego.left45_dist;
    }
  }

  if (se->sen.r45.sensor_dist > se->ego.right45_dist ||
      se->sen.r45.sensor_dist == 0) {
    se->sen.r45.sensor_dist = se->ego.right45_dist;
    se->sen.r45.global_run_dist = tgt_val->global_pos.dist;
  } else {
    if (((tgt_val->global_pos.dist - se->sen.r45.global_run_dist) >
         param_ro->wall_off_hold_dist) &&
        se->ego.right45_dist < param_ro->sen_ref_p.normal2.exist.right90) {
      se->sen.r45.sensor_dist = se->ego.right45_dist;
    }
  }
}

void IRAM_ATTR PlanningTask::recv_notify() {
  uint32_t ulReceivedValue;
  // auto start_que_rec = esp_timer_get_time();
  BaseType_t xResult = xTaskNotifyWait(0x00,             // clear bit mask
                                       0xffffffff,       // recv bit mask
                                       &ulReceivedValue, // recieve data
                                       0 // us_to_ticks(1) // timeout
  );
  // auto end_que_rec = esp_timer_get_time();
  // printf("addr: %ld, %lld\n", ulReceivedValue, end_que_rec - start_que_rec);
  if (xResult == pdTRUE) {
    // printf("planning_task: %ld\n", ulReceivedValue);
    if ((uint32_t)tgt_val.get() == ulReceivedValue) {
      receive_req = (motion_tgt_val_t *)ulReceivedValue;
      cp_request();
    }
    first_req = true;
  }
}
void IRAM_ATTR PlanningTask::calc_front_ctrl_duty() {
  const unsigned char reset = 0;
  param_ro->motor_pid.i = param_ro->motor_pid.d = 0;
  vel_pid.step(&ee->v.error_p, &param_ro->motor_pid.p, &param_ro->motor_pid.i,
               &param_ro->motor_pid.d, &reset, &dt, &duty_c);
  set_ctrl_val(ee->v_val, ee->v.error_p, ee->v.error_i, 0, ee->v.error_d,
               param_ro->motor_pid.p * ee->v.error_p,
               vel_pid.simple_pid_controller_DW.Integrator_DSTATE, 0, 0, 0, 0);
  // reset
  ee->w.error_i = ee->w.error_d = 0;
  ee->w_log.gain_z = ee->w_log.gain_zz = 0;

  auto diff_ang = 0.0f;
  auto kp_gain = param_ro->front_ctrl_roll_pid.p * ee->w.error_p;
  auto ki_gain = 0.0f;
  auto kb_gain = 0.0f;
  auto kc_gain = 0.0f;
  auto kd_gain = param_ro->front_ctrl_roll_pid.d * ee->w_kf.error_d;

  limitter(kp_gain, ki_gain, kb_gain, kd_gain,
           param_ro->gyro_pid_gain_limitter);
  duty_roll = kp_gain + ki_gain + kb_gain + kc_gain + kd_gain +
              (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt;

  ee->ang_log.gain_zz = ee->ang_log.gain_z;
  ee->ang_log.gain_z = duty_roll;

  set_ctrl_val(ee->w_val,
               ee->w.error_p,                           // p
               diff_ang,                                // i
               ee->w.error_i,                           // i2
               ee->w_kf.error_d,                        // d
               param_ro->gyro_pid.p * ee->w.error_p,    // kp*p
               param_ro->gyro_pid.i * diff_ang,         // ki*i
               param_ro->gyro_pid.b * ee->w.error_i,    // kb*i2
               param_ro->gyro_pid.d * ee->w_kf.error_d, // kd*d
               ee->ang_log.gain_zz, ee->ang_log.gain_z);

  sensing_result->ego.duty.sen = duty_roll;
  sensing_result->ego.duty.sen = 0;
  // calc front dist ctrl
  duty_front_ctrl_trans =
      param_ro->front_ctrl_dist_pid.p * ee->dist.error_p +
      param_ro->front_ctrl_dist_pid.i * ee->dist.error_i +
      param_ro->front_ctrl_dist_pid.d * sensing_result->ego.v_c;
  // calc front roll ctrl
  duty_front_ctrl_roll = param_ro->front_ctrl_angle_pid.p * ee->ang.error_p +
                         param_ro->front_ctrl_angle_pid.i * ee->ang.error_i +
                         param_ro->front_ctrl_angle_pid.d * ee->w_kf.error_p;
  duty_front_ctrl_roll_keep =
      param_ro->front_ctrl_keep_angle_pid.p * ee->ang.error_p +
      param_ro->front_ctrl_keep_angle_pid.i * ee->ang.error_i +
      param_ro->front_ctrl_keep_angle_pid.d * ee->w_kf.error_p;
}

void IRAM_ATTR PlanningTask::calc_angle_velocity_ctrl_old() {}

void IRAM_ATTR PlanningTask::calc_angle_velocity_ctrl() {
  if (tgt_val->motion_type == MotionType::NONE) {
    duty_roll = param_ro->gyro_pid.p * ee->w.error_p +
                param_ro->gyro_pid.b * ee->w.error_i +
                param_ro->gyro_pid.c * ee->w.error_d;
    // (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt;
    ee->ang_log.gain_zz = ee->ang_log.gain_z;
    ee->ang_log.gain_z = duty_roll;

    set_ctrl_val(ee->w_val, ee->w.error_p, ee->w.error_i, 0, ee->w.error_d,
                 param_ro->gyro_pid.p * ee->w.error_p,
                 param_ro->gyro_pid.b * ee->w.error_i, param_ro->gyro_pid.b * 0,
                 param_ro->gyro_pid.c * ee->w.error_d, ee->ang_log.gain_zz,
                 ee->ang_log.gain_z);
  } else {
    // mode3 main
    auto diff_ang = (tgt_val->ego_in.img_ang - sensing_result->ego.ang_kf);
    auto ang_sum = ee->ang.error_i;
    if (tgt_val->motion_type == MotionType::SLALOM) {
      diff_ang = 0;
      ang_sum = 0;
    }
    if (!(tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
          tgt_val->motion_type == MotionType::SLA_BACK_STR ||
          tgt_val->motion_type == MotionType::PIVOT)) {
      diff_ang = 0;
      ang_sum = 0;
    }
    auto kp_gain = param_ro->gyro_pid.p * ee->w.error_p;
    auto ki_gain = param_ro->gyro_pid.i * diff_ang;
    auto kb_gain = param_ro->gyro_pid.b * ee->w.error_i;
    auto kc_gain = 0; // param_ro->gyro_pid.c * ang_sum;
    auto kd_gain = param_ro->gyro_pid.d * ee->w_kf.error_d;
    limitter(kp_gain, ki_gain, kb_gain, kd_gain,
             param_ro->gyro_pid_gain_limitter);
    duty_roll = kp_gain + ki_gain + kb_gain + kc_gain + kd_gain +
                (ee->ang_log.gain_z - ee->ang_log.gain_zz) * dt;

    ee->ang_log.gain_zz = ee->ang_log.gain_z;
    ee->ang_log.gain_z = duty_roll;
    set_ctrl_val(ee->w_val,
                 ee->w.error_p,    // p
                 diff_ang,         // i
                 ee->w.error_i,    // i2
                 ee->w_kf.error_d, // d
                 kp_gain,          // kp*p
                 ki_gain,          // ki*i
                 kb_gain,          // kb*i2
                 kd_gain,          // kd*d
                 ee->ang_log.gain_zz, ee->ang_log.gain_z);
  }
}

void IRAM_ATTR PlanningTask::apply_duty_limitter() {
  if (tgt_val->motion_type == MotionType::STRAIGHT ||
      tgt_val->motion_type == MotionType::SLALOM ||
      tgt_val->motion_type == MotionType::SLA_BACK_STR ||
      tgt_val->motion_type == MotionType::SLA_FRONT_STR ||
      tgt_val->motion_type == MotionType::PIVOT) {
    const auto min_duty = param_ro->min_duty;

    if (0 <= tgt_duty.duty_r && tgt_duty.duty_r < min_duty) {
      tgt_duty.duty_r = min_duty;
    } else if (-min_duty < tgt_duty.duty_r && tgt_duty.duty_r <= 0) {
      tgt_duty.duty_r = -min_duty;
    }
    if (0 <= tgt_duty.duty_l && tgt_duty.duty_l < min_duty) {
      tgt_duty.duty_l = min_duty;
    } else if (-min_duty < tgt_duty.duty_l && tgt_duty.duty_l <= 0) {
      tgt_duty.duty_l = -min_duty;
    }
  } else if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    const auto max_duty = param_ro->sen_ref_p.search_exist.offset_l;
    if (tgt_duty.duty_r > max_duty) {
      tgt_duty.duty_r = max_duty;
    } else if (tgt_duty.duty_r < -max_duty) {
      tgt_duty.duty_r = -max_duty;
    }
    if (tgt_duty.duty_l > max_duty) {
      tgt_duty.duty_l = max_duty;
    } else if (tgt_duty.duty_l < -max_duty) {
      tgt_duty.duty_l = -max_duty;
    }
  } else {
    const auto max_duty = param_ro->max_duty;
    if (tgt_duty.duty_r > max_duty) {
      tgt_duty.duty_r = max_duty;
    } else if (tgt_duty.duty_r < -max_duty) {
      tgt_duty.duty_r = -max_duty;
    }
    if (tgt_duty.duty_l > max_duty) {
      tgt_duty.duty_l = max_duty;
    } else if (tgt_duty.duty_l < -max_duty) {
      tgt_duty.duty_l = -max_duty;
    }
  }
}
void IRAM_ATTR PlanningTask::clear_ctrl_val() {
  duty_c = duty_c2 = duty_roll = duty_front_ctrl_roll_keep = duty_roll_ang = 0;
  ee->v.error_i = 0;
  ee->v.error_d = 0;
  ee->v.error_dd = 0;
  ee->dist.error_i = 0;
  ee->dist.error_d = 0;
  ee->dist.error_dd = 0;
  ee->w.error_i = 0;
  ee->w.error_d = 0;
  ee->w.error_dd = 0;
  ee->ang.error_i = 0;
  ee->ang.error_d = 0;
  ee->ang.error_dd = 0;
  ee->sen.error_i = 0;
  ee->sen.error_d = 0;
  ee->sen.error_dd = 0;
  ee->sen_dia.error_i = 0;
  ee->sen_dia.error_d = 0;
  ee->sen_dia.error_dd = 0;
  tgt_duty.duty_r = tgt_duty.duty_l = 0;
  ee->v_log.gain_zz = 0;
  ee->v_log.gain_z = 0;
  ee->dist_log.gain_zz = 0;
  ee->dist_log.gain_z = 0;
  ee->w_log.gain_zz = 0;
  ee->w_log.gain_z = 0;
  ee->ang_log.gain_zz = 0;
  ee->ang_log.gain_z = 0;
  ee->sen_log.gain_z = 0;
  ee->sen_log.gain_zz = 0;

  ee->v_l_log.gain_zz = 0;
  ee->v_l_log.gain_z = 0;
  ee->v_r_log.gain_zz = 0;
  ee->v_r_log.gain_z = 0;

  tgt_val->global_pos.ang = 0;
  tgt_val->global_pos.img_ang = 0;
  tgt_val->global_pos.dist = 0;
  tgt_val->global_pos.img_dist = 0;
  ee->v_val.p_val = 0;
}

void IRAM_ATTR PlanningTask::summation_duty() {

  auto ff_front = mpc_next_ego.ff_duty_front;
  auto ff_roll = mpc_next_ego.ff_duty_roll;

  if (tgt_val->motion_type == MotionType::WALL_OFF ||
      tgt_val->motion_type == MotionType::WALL_OFF_DIA) {
    ff_front = param_ro->ff_roll_gain_before * ff_front;
    mpc_next_ego.ff_duty_front = ff_front;
  }

  if (tgt_val->motion_type == MotionType::SLA_BACK_STR) {
    ff_front = param_ro->ff_front_gain_14 * ff_front;
    mpc_next_ego.ff_duty_front = ff_front;
  }

  if (tgt_val->motion_type == MotionType::SLALOM) {
    if (tgt_val->ego_in.sla_param.base_alpha > 0) {
      if (tgt_val->ego_in.alpha < 0) {
        ff_roll = param_ro->ff_roll_gain_after * ff_roll;
        sensing_result->ego.duty.ff_duty_roll = ff_roll;
        mpc_next_ego.ff_duty_roll = ff_roll;
      }
    } else if (tgt_val->ego_in.sla_param.base_alpha < 0) {
      if (tgt_val->ego_in.alpha > 0) {
        ff_roll = param_ro->ff_roll_gain_after * ff_roll;
        sensing_result->ego.duty.ff_duty_roll = ff_roll;
        mpc_next_ego.ff_duty_roll = ff_roll;
      }
    }
  }
  auto ff_duty_r = ff_front + ff_roll + mpc_next_ego.ff_duty_rpm_r;
  auto ff_duty_l = ff_front - ff_roll + mpc_next_ego.ff_duty_rpm_l;

  if (param_ro->FF_keV == 0) {
    ff_duty_l = ff_duty_r = 0;
  }
  if (param_ro->torque_mode == 0 ||
      tgt_val->motion_type == MotionType::FRONT_CTRL) {
    tgt_duty.duty_r =
        (duty_c + duty_front_ctrl_trans + duty_roll + duty_front_ctrl_roll +
         duty_front_ctrl_roll_keep + ff_duty_r) /
        sensing_result->ego.battery_lp * 100;

    tgt_duty.duty_l =
        (duty_c + duty_front_ctrl_trans - duty_roll - duty_front_ctrl_roll -
         duty_front_ctrl_roll_keep + ff_duty_l) /
        sensing_result->ego.battery_lp * 100;

  } else if (param_ro->torque_mode == 1) {
    auto ff_front = mpc_next_ego.ff_front_torque;
    auto ff_roll = mpc_next_ego.ff_roll_torque;
    auto ff_duty_r = mpc_next_ego.ff_duty_rpm_r;
    auto ff_duty_l = mpc_next_ego.ff_duty_rpm_l;
    if (param_ro->FF_keV == 0) {
      ff_front = ff_roll = ff_duty_r = ff_duty_l = 0;
    }
    float torque_r = (ff_front + ff_roll + duty_c + duty_roll + duty_sen);
    float torque_l = (ff_front - ff_roll + duty_c - duty_roll - duty_sen);

    const float km_gear = param_ro->Km * (param_ro->gear_a / param_ro->gear_b);
    float req_v_r = torque_r * param_ro->Resist / km_gear + ff_duty_r;
    float req_v_l = torque_l * param_ro->Resist / km_gear + ff_duty_l;

    tgt_duty.duty_r = req_v_r / sensing_result->ego.battery_lp * 100;
    tgt_duty.duty_l = req_v_l / sensing_result->ego.battery_lp * 100;
  } else if (param_ro->torque_mode == 2) {
    auto ff_front = mpc_next_ego.ff_front_torque;
    auto ff_roll = mpc_next_ego.ff_roll_torque;
    auto ff_duty_r = mpc_next_ego.ff_duty_rpm_r;
    auto ff_duty_l = mpc_next_ego.ff_duty_rpm_l;
    if (param_ro->FF_keV == 0) {
      ff_front = ff_roll = ff_duty_r = ff_duty_l = 0;
    }
    float torque_r = (ff_front + ff_roll + duty_c + duty_roll);
    float torque_l = (ff_front - ff_roll + duty_c - duty_roll);

    const float km_gear = param_ro->Km * (param_ro->gear_a / param_ro->gear_b);
    float req_v_r = torque_r * param_ro->Resist / km_gear + ff_duty_r;
    float req_v_l = torque_l * param_ro->Resist / km_gear + ff_duty_l;

    tgt_duty.duty_r = req_v_r / sensing_result->ego.battery_lp * 100;
    tgt_duty.duty_l = req_v_l / sensing_result->ego.battery_lp * 100;
  }
}

void IRAM_ATTR PlanningTask::reset_pid_val() {
  //
  if (tgt_val->motion_type == MotionType::FRONT_CTRL || !motor_en ||
      tgt_val->motion_type == MotionType::NONE) {
    ee->v.error_i = ee->v.error_d = 0;
    ee->v_kf.error_i = ee->v_kf.error_d = 0;
    ee->w_kf.error_i = ee->w_kf.error_d = 0;
    ee->v_log.gain_z = ee->v_log.gain_zz = 0;
    ee->v_l.error_i = ee->v_l.error_d = 0;
    ee->v_r.error_i = ee->v_r.error_d = 0;
    ee->v_l_log.gain_z = ee->v_l_log.gain_zz = 0;
    ee->v_r_log.gain_z = ee->v_r_log.gain_zz = 0;
    ee->sen.error_i = ee->sen.error_d = 0;
    ee->sen_log.gain_zz = ee->sen_log.gain_z = 0;
    ee->sen_dia.error_i = ee->sen_dia.error_d = 0;
    ee->sen_log_dia.gain_zz = ee->sen_log_dia.gain_z = 0;
  }

  // reset
  ee->v_val.p = ee->v_val.i = ee->v_val.d = 0;
  ee->w_val.p = ee->w_val.i = ee->w_val.d = 0;
  ee->v_val.p_val = ee->v_val.i_val = ee->v_val.d_val = 0;
  ee->w_val.p_val = ee->w_val.i_val = ee->w_val.d_val = 0;
  ee->v_val.z = ee->v_val.zz = 0;
  ee->w_val.z = ee->w_val.zz = 0;
}

void IRAM_ATTR PlanningTask::calc_translational_ctrl() {
  if (!motor_en) {
    const unsigned char reset = 0;
    vel_pid.step(&ee->v.error_p, &param_ro->motor_pid.p, &param_ro->motor_pid.i,
                 &param_ro->motor_pid.d, &reset, &dt, &duty_c);
    set_ctrl_val(ee->v_val, ee->v.error_p, ee->v.error_i, 0, ee->v.error_d,
                 param_ro->motor_pid.p * ee->v.error_p,
                 vel_pid.simple_pid_controller_DW.Integrator_DSTATE, 0, 0, 0,
                 0);
  } else {
    if (tgt_val->motion_type == MotionType::STRAIGHT) {
      //加速から減速に切り替わったら
      if (last_accl > 0 && tgt_val->ego_in.accl < 0) {
        ee->v.error_i *= param_ro->ff_front_gain_decel;
      }
    }
    const auto diff_dist =
        tgt_val->ego_in.img_dist - sensing_result->ego.dist_kf;
    auto kp_gain = param_ro->motor_pid2.p * ee->v.error_p;
    auto ki_gain = param_ro->motor_pid2.i * ee->v.error_i;
    auto kb_gain = param_ro->motor_pid2.b * diff_dist;
    auto kd_gain = param_ro->motor_pid2.d * ee->v_kf.error_d;
    limitter(kp_gain, ki_gain, kb_gain, kd_gain,
             param_ro->motor2_pid_gain_limitter);
    duty_c = kp_gain + ki_gain + kb_gain + kd_gain;

    set_ctrl_val(ee->v_val, ee->v.error_p, ee->v.error_i, diff_dist,
                 ee->v.error_d, kp_gain, ki_gain, kb_gain, kd_gain,
                 ee->v_log.gain_zz, ee->v_log.gain_z);
  }
  if (w_reset == 0 || !motor_en) {
    ee->w.error_i = ee->w.error_d = 0;
    ee->w_log.gain_z = ee->w_log.gain_zz = 0;
  }
  last_accl = tgt_val->ego_in.accl;
}
void IRAM_ATTR PlanningTask::calc_pid_val() {
  ee->v.error_dd = ee->v.error_d;
  ee->v.error_dd = ee->v.error_d;
  ee->v_kf.error_dd = ee->v_kf.error_d;
  ee->dist.error_dd = ee->dist.error_d;

  ee->v.error_d = ee->v.error_p;
  ee->v_r.error_d = ee->v_r.error_p;
  ee->v_l.error_d = ee->v_l.error_p;
  ee->v_kf.error_d = ee->v_kf.error_p;
  ee->dist.error_d = ee->dist.error_p;

  ee->v.error_p = tgt_val->ego_in.v - sensing_result->ego.v_c;

  ee->v_r.error_p = ideal_v_r - sensing_result->ego.v_r;
  ee->v_l.error_p = ideal_v_l - sensing_result->ego.v_l;
  ee->v_kf.error_p = tgt_val->ego_in.v - sensing_result->ego.v_kf;
  ee->w_kf.error_p = tgt_val->ego_in.w - sensing_result->ego.w_kf;

  ee->dist.error_p = tgt_val->global_pos.img_dist - tgt_val->global_pos.dist;

  if (ee->dist.error_p > param_ro->front_ctrl_error_th) {
    ee->dist.error_p = param_ro->front_ctrl_error_th;
  } else if (ee->dist.error_p < -param_ro->front_ctrl_error_th) {
    ee->dist.error_p = -param_ro->front_ctrl_error_th;
  }

  ee->v_kf.error_d = ee->v_kf.error_p - ee->v_kf.error_d;
  ee->v.error_d = ee->v.error_p - ee->v.error_d;
  ee->dist.error_d = ee->dist.error_p - ee->dist.error_d;

  ee->v_l.error_d = ee->v_l.error_p - ee->v_l.error_d;
  ee->v_r.error_d = ee->v_r.error_p - ee->v_r.error_d;

  ee->v_kf.error_dd = ee->v_kf.error_d - ee->v_kf.error_dd;
  ee->v.error_dd = ee->v.error_d - ee->v.error_dd;
  ee->dist.error_dd = ee->dist.error_d - ee->dist.error_dd;

  ee->v.error_i += ee->v.error_p;
  if (tgt_val->motion_type != MotionType::FRONT_CTRL) {
    ee->dist.error_i += ee->dist.error_p;
  }

  ee->v_l.error_i += ee->v_l.error_p;
  ee->v_r.error_i += ee->v_r.error_p;

  tgt_val->v_error = ee->v.error_i;
}
void IRAM_ATTR PlanningTask::calc_pid_val_ang() {

  const auto tgt = get_tgt_entity();
  const auto se = get_sensing_entity();

  ee->ang.error_dd = ee->ang.error_d;
  ee->ang.error_d = ee->ang.error_p;

  // TODO カスケード制御条件分岐

  float offset = 0;

  if (tgt->motion_type != MotionType::FRONT_CTRL) {
    offset += duty_sen;
  }
  // 壁制御できないときは角度を固定
  if ((tgt->motion_type == MotionType::STRAIGHT) ||
      tgt->motion_type == MotionType::SLA_FRONT_STR ||
      tgt->motion_type == MotionType::SLA_BACK_STR ||
      tgt->motion_type == MotionType::WALL_OFF ||
      tgt->motion_type == MotionType::WALL_OFF_DIA) {
  }
  ee->ang.error_p = (tgt->ego_in.img_ang + offset) - se->ego.ang_kf;

  // tgt_val->ego_in.ang
  ee->ang.error_d = ee->ang.error_p - ee->ang.error_d;
  // ee->ang.error_d = (ee->ang.error_p - offset) - ee->ang.error_d;

  ee->ang.error_dd = ee->ang.error_d - ee->ang.error_dd;

  ee->ang.error_i += (ee->ang.error_p);
  // ee->ang.error_i += (ee->ang.error_p - offset);

  duty_roll_ang = param_ro->angle_pid.p * ee->ang.error_p +
                  param_ro->angle_pid.i * ee->ang.error_i +
                  param_ro->angle_pid.d * ee->ang.error_d;

  set_ctrl_val(ee->ang_val,
               ee->ang.error_p,                         // p
               ee->ang.error_i,                         // i
               0,                                       // i2
               ee->ang.error_d,                         // d
               param_ro->angle_pid.p * ee->ang.error_p, // kp*p
               param_ro->angle_pid.i * ee->ang.error_i, // ki*i
               0,                                       // kb*i2
               param_ro->angle_pid.d * ee->ang.error_d, // kd*d
               0, 0);
}

void IRAM_ATTR PlanningTask::calc_pid_val_ang_vel() {
  const auto tgt = get_tgt_entity();
  const auto se = get_sensing_entity();

  ee->w.error_dd = ee->w.error_d;
  ee->w_kf.error_dd = ee->w_kf.error_d;
  ee->w.error_d = ee->w.error_p;
  ee->w_kf.error_d = ee->w_kf.error_p;

  float offset = 0;

  if (param_ro->torque_mode == 2) {
    if (!(tgt->motion_type == MotionType::PIVOT ||
          tgt->motion_type == MotionType::FRONT_CTRL
          // || tgt->motion_type == MotionType::SLALOM
          //
          )) {
      offset += duty_roll_ang;
    }
    // offset += duty_roll_ang;
  }

  ee->w.error_p = (tgt->ego_in.w + offset) - se->ego.w_lp;
  ee->w_kf.error_p = (tgt_val->ego_in.w + offset) - se->ego.w_kf;

  // ee->w.error_d = (ee->w.error_p - offset) - ee->w.error_d;
  // ee->w_kf.error_d = (ee->w_kf.error_p - offset) - ee->w_kf.error_d;

  ee->w.error_d = ee->w.error_p - ee->w.error_d;
  ee->w_kf.error_d = ee->w_kf.error_p - ee->w_kf.error_d;

  ee->w.error_dd = ee->w.error_d - ee->w.error_dd;
  ee->w_kf.error_dd = ee->w_kf.error_d - ee->w_kf.error_dd;

  // ee->w.error_i += (ee->w.error_p - offset);
  ee->w.error_i += ee->w.error_p;
  tgt_val->w_error = ee->w.error_i;
}

void IRAM_ATTR PlanningTask::calc_pid_val_front_ctrl() {
  const auto se = get_sensing_entity();
  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    ee->v.error_i = ee->v.error_d = 0;
    ee->w.error_i = ee->w.error_d = 0;
    ee->v_r.error_i = ee->v_r.error_d = 0;
    ee->v_l.error_i = ee->v_l.error_d = 0;
    if (se->ego.front_dist < param_ro->cell) {
      ee->dist.error_p =
          se->ego.front_dist - param_ro->sen_ref_p.search_exist.front_ctrl;
      ee->ang.error_p = (se->ego.right90_dist - se->ego.left90_dist) / 2 -
                        param_ro->sen_ref_p.search_exist.kireme_r;
      ee->dist.error_i += ee->dist.error_p;
    } else {
      ee->dist.error_p = ee->dist.error_i = ee->dist.error_d = 0;
      ee->ang.error_p = ee->ang.error_i = ee->ang.error_d = 0;
    }
  }
}