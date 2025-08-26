#include "include/logging_task.hpp"

void LoggingTask::create_task(const BaseType_t xCoreID) {
  qh = xQueueCreate(4, sizeof(motion_tgt_val_t *));
  xTaskCreatePinnedToCore(task_entry_point, "logging_task", 8192, this, 1,
                          &handle, xCoreID);
}

void LoggingTask::task_entry_point(void *task_instance) {
  static_cast<LoggingTask *>(task_instance)->task();
}

void LoggingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_entity) {
  sensing_result = _entity;
}
void LoggingTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void LoggingTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

void LoggingTask::set_error_entity(
    std::shared_ptr<pid_error_entity_t> &_error_entity) {
  error_entity = _error_entity;
}

void LoggingTask::start_slalom_log() {
  req_logging_active = true;
  // active_slalom_log = true;
  // log_mode = false;
  idx_slalom_log = 0;
  // std::vector<std::unique_ptr<log_data_t2>>().swap(log_vec);
  log_vec.clear();
  log_vec.shrink_to_fit();
  sysidlog_vec.clear();
  sysidlog_vec.shrink_to_fit();
  // log_vec.resize(param->log_size);
  // for (int i = 0; i < param->log_size; i++) {
  //   log_vec[i] = std::make_shared<log_data_t2>();
  // }
  xQueueSendToFront(qh, &req_logging_active, 1);
}

void LoggingTask::stop_slalom_log() {
  active_slalom_log = false; //
  logging_active = false;
}

void LoggingTask::change_sysid_mode(float d_l, float d_r, int t) {
  log_mode = false; //
  duty_l = d_l;
  duty_r = d_r;
  time = t;
  sysidlog_vec.clear();
  sysidlog_vec.shrink_to_fit();
  sysidlog_vec.resize(time);
  for (int i = 0; i < time; i++) {
    sysidlog_vec[i] = std::make_shared<sysid_log>();
  }
}

void LoggingTask::exec_log() {}

void IRAM_ATTR LoggingTask::task() {
  const TickType_t xDelay4 = 40.0 / portTICK_PERIOD_MS;
  const TickType_t xDelay1 = 1.0 / portTICK_PERIOD_MS;
  BaseType_t queue_recieved;
  logging_active = false;
  bool first = true;
  // 1,4MByteぐらいまで行ける
  // for (int i = 0; i < 26000; i++) {
  //   log_data_t2 *structPtr =
  //       (log_data_t2 *)heap_caps_malloc(sizeof(log_data_t2),
  //       MALLOC_CAP_SPIRAM);
  //   auto ld = std::shared_ptr<log_data_t2>(structPtr, heap_caps_free);
  //   log_vec.emplace_back(std::move(ld));
  //   printf("%d %p %p %p\n", i, &ld, structPtr, &log_vec.at(i));
  // }

  while (1) {

    if (!logging_active) {
      queue_recieved =
          xQueueReceive(qh, &receive_logging_active_req, portMAX_DELAY);
      logging_active = receive_logging_active_req;
    }
    if (idx_slalom_log > param->log_size) {
      logging_active = false;
    }
    if (log_mode) {
      if (logging_active) {
        if (idx_slalom_log <= param->log_size) {
          if (tgt_val->motion_type == MotionType::PIVOT) {
            first = false;
            set_data();
          } else if (idx_slalom_log < 4) {
            first = false;
            idx_slalom_log++;
          } else {
            set_data();
          }
        }
        if (param->set_param) {
          vTaskDelay(param->logging_time);
        } else {
          vTaskDelay(xDelay4);
        }
      } else {
        vTaskDelay(xDelay4);
      }
    } else {
      if (logging_active) {
        // printf("loggin active\n");
        if (active_slalom_log && idx_slalom_log <= time) {
          // printf("loggin active: push data\n");
          auto ld = std::make_shared<sysid_log>();

          ld->v_l = floatToHalf(sensing_result->ego.v_l);
          ld->v_c = floatToHalf(sensing_result->ego.v_c);
          ld->v_r = floatToHalf(sensing_result->ego.v_r);
          ld->w_lp = floatToHalf(sensing_result->ego.w_lp);
          ld->volt_l = floatToHalf(duty_l);
          ld->volt_r = floatToHalf(duty_r);

          sysidlog_vec.emplace_back(ld);
          idx_slalom_log++;
        }
      }
      vTaskDelay(xDelay1);
    }
  }
}
float IRAM_ATTR LoggingTask::calc_sensor(float data, float a, float b,
                                         char motion_type) {
  if ((motion_type == static_cast<char>(MotionType::NONE) ||
       motion_type == static_cast<char>(MotionType::PIVOT))) {
    return 0;
  }
  if (data <= 1 || data >= 4005) {
    return 0;
  }
  auto res = a / std::log(data) - b;
  if (res < param->sensor_range_min || res > param->sensor_range_max) {
    return 0;
  }
  if (!isfinite(res)) {
    return 0;
  }
  return res;
}

void LoggingTask::save(std::string file_name) {
  return; //
}

void LoggingTask::save_sysid(std::string file_name) {
  printf("usefile: %s\n", slalom_log_file.c_str());
  f_slalom_log = fopen(slalom_log_file.c_str(), "wb");
  if (f_slalom_log == NULL)
    printf("slalom_file_load_failed\n");

  int i = 0;

  if (f_slalom_log != NULL) {
    fclose(f_slalom_log);
    printf("close\n");
  }
}

// void LoggingTask::dump_log(std::string file_name) {
//   mount();
//   const TickType_t xDelay2 = 100.0 / portTICK_PERIOD_MS;
//   FILE *f = fopen(file_name.c_str(), "rb");
//   if (f == NULL)
//     return;
//   char line_buf[LINE_BUF_SIZE];
//   printf("start___\n"); // csvファイル作成トリガー
//   vTaskDelay(xDelay2);
//   printf("index,ideal_v,v_c,v_c2,v_l,v_r,accl,accl_x,ideal_w,w_lp,alpha,"
//          "ideal_dist,"
//          "dist,"
//          "ideal_ang,ang,left90,left45,front,right45,right90,left90_d,left45_d,"
//          "front_d,right45_d,right90_d,left90_far_d,front_far_d,right90_far_d,"
//          "battery,duty_l,"
//          "duty_r,motion_state,duty_sen,dist_mod90,"
//          "sen_dist_l45,sen_dist_r45,timestamp\n");
//   int c = 0;
//   while (fgets(line_buf, sizeof(line_buf), f) != NULL) {
//     printf("%s\n", line_buf);
//     c++;
//     if (c == 50) {
//       c = 0;
//       vTaskDelay(1.0 / portTICK_PERIOD_MS);
//     }
//   }
//   printf("end___\n"); // csvファイル追記終了トリガー

//   fclose(f);
//   printf("memory: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
//   log_vec.clear();
//   umount();
//   // std::vector<std::shared_ptr<log_data_t2>>().swap(log_vec);
// }

void IRAM_ATTR LoggingTask::print_header() {
  const TickType_t xDelay2 = 50.0 / portTICK_PERIOD_MS;

  const int size = sizeof(LogStruct1) + sizeof(LogStruct2) +
                   sizeof(LogStruct3) + sizeof(LogStruct4) +
                   sizeof(LogStruct5) + sizeof(LogStruct6) +
                   sizeof(LogStruct7) + sizeof(LogStruct8) +
                   sizeof(LogStruct9) + sizeof(LogStruct10);

  printf("ready___:%d\n", size);
  vTaskDelay(xDelay2);

  // LogStruct1
  printf("index:int:%d\n", sizeof(ls1.index));
  printf("ideal_v:float:%d\n", sizeof(ls1.ideal_v));
  printf("v_c:float:%d\n", sizeof(ls1.v_c));
  printf("v_c2:float:%d\n", sizeof(ls1.v_c2));

  printf("v_l:float:%d\n", sizeof(ls1.v_l));
  printf("v_r:float:%d\n", sizeof(ls1.v_r));
  printf("v_l_enc:int:%d\n", sizeof(ls1.v_l_enc));
  printf("v_r_enc:int:%d\n", sizeof(ls1.v_r_enc));

  printf("v_l_enc_sin:float:%d\n", sizeof(ls1.v_l_enc_sin));
  printf("v_r_enc_sin:float:%d\n", sizeof(ls1.v_r_enc_sin));
  printf("accl:float:%d\n", sizeof(ls1.accl));
  printf("accl_x:float:%d\n", sizeof(ls1.accl_x));

  // LogStruct2
  printf("ideal_w:float:%d\n", sizeof(ls2.ideal_w));
  printf("w_lp:float:%d\n", sizeof(ls2.w_lp));
  printf("alpha:float:%d\n", sizeof(ls2.alpha));
  printf("ideal_dist:float:%d\n", sizeof(ls2.ideal_dist));

  printf("dist:float:%d\n", sizeof(ls2.dist));
  printf("dist_kf:float:%d\n", sizeof(ls2.dist_kf));
  printf("ideal_ang:float:%d\n", sizeof(ls2.ideal_ang));
  printf("ang:float:%d\n", sizeof(ls2.ang));

  printf("ang_kf:float:%d\n", sizeof(ls2.ang_kf));
  printf("left90:int:%d\n", sizeof(ls2.left90));
  printf("left45:int:%d\n", sizeof(ls2.left45));
  printf("front:int:%d\n", sizeof(ls2.front));

  // LogStruct3
  printf("right45:int:%d\n", sizeof(ls3.right45));
  printf("right90:int:%d\n", sizeof(ls3.right90));
  printf("left90_d:float:%d\n", sizeof(ls3.left90_d));
  printf("left45_d:float:%d\n", sizeof(ls3.left45_d));

  printf("front_d:float:%d\n", sizeof(ls3.front_d));
  printf("right45_d:float:%d\n", sizeof(ls3.right45_d));
  printf("right90_d:float:%d\n", sizeof(ls3.right90_d));
  printf("left90_far_d:float:%d\n", sizeof(ls3.left90_far_d));

  printf("front_far_d:float:%d\n", sizeof(ls3.front_far_d));
  printf("right90_far_d:float:%d\n", sizeof(ls3.right90_far_d));
  printf("battery:float:%d\n", sizeof(ls3.battery));
  printf("duty_l:float:%d\n", sizeof(ls3.duty_l));

  // LogStruct4
  printf("duty_r:float:%d\n", sizeof(ls4.duty_r));
  printf("motion_state:int:%d\n", sizeof(ls4.motion_state));
  printf("duty_sen:float:%d\n", sizeof(ls4.duty_sen));
  printf("dist_mod90:float:%d\n", sizeof(ls4.dist_mod90));

  printf("sen_dist_l45:float:%d\n", sizeof(ls4.sen_dist_l45));
  printf("sen_dist_r45:float:%d\n", sizeof(ls4.sen_dist_r45));
  printf("timestamp:int:%d\n", sizeof(ls4.timestamp));
  printf("sen_calc_time:int:%d\n", sizeof(ls4.sen_calc_time));

  printf("sen_calc_time2:int:%d\n", sizeof(ls4.sen_calc_time2));
  printf("pln_calc_time:int:%d\n", sizeof(ls4.pln_calc_time));
  printf("pln_calc_time2:int:%d\n", sizeof(ls4.pln_calc_time2));
  printf("pln_time_diff:int:%d\n", sizeof(ls4.pln_time_diff));

  // LogStruct5
  printf("m_pid_p:float:%d\n", sizeof(ls5.m_pid_p));
  printf("m_pid_i:float:%d\n", sizeof(ls5.m_pid_i));
  printf("m_pid_i2:float:%d\n", sizeof(ls5.m_pid_i2));
  printf("m_pid_d:float:%d\n", sizeof(ls5.m_pid_d));

  printf("m_pid_p_v:float:%d\n", sizeof(ls5.m_pid_p_v));
  printf("m_pid_i_v:float:%d\n", sizeof(ls5.m_pid_i_v));
  printf("m_pid_i2_v:float:%d\n", sizeof(ls5.m_pid_i2_v));
  printf("m_pid_d_v:float:%d\n", sizeof(ls5.m_pid_d_v));

  printf("g_pid_p:float:%d\n", sizeof(ls5.g_pid_p));
  printf("g_pid_i:float:%d\n", sizeof(ls5.g_pid_i));
  printf("g_pid_i2:float:%d\n", sizeof(ls5.g_pid_i2));
  printf("g_pid_d:float:%d\n", sizeof(ls5.g_pid_d));

  // LogStruct6
  printf("g_pid_p_v:float:%d\n", sizeof(ls6.g_pid_p_v));
  printf("g_pid_i_v:float:%d\n", sizeof(ls6.g_pid_i_v));
  printf("g_pid_i2_v:float:%d\n", sizeof(ls6.g_pid_i2_v));
  printf("g_pid_d_v:float:%d\n", sizeof(ls6.g_pid_d_v));

  printf("s_pid_p:float:%d\n", sizeof(ls6.s_pid_p));
  printf("s_pid_i:float:%d\n", sizeof(ls6.s_pid_i));
  printf("s_pid_i2:float:%d\n", sizeof(ls6.s_pid_i2));
  printf("s_pid_d:float:%d\n", sizeof(ls6.s_pid_d));

  printf("s_pid_p_v:float:%d\n", sizeof(ls6.s_pid_p_v));
  printf("s_pid_i_v:float:%d\n", sizeof(ls6.s_pid_i_v));
  printf("s_pid_i2_v:float:%d\n", sizeof(ls6.s_pid_i2_v));
  printf("s_pid_d_v:float:%d\n", sizeof(ls6.s_pid_d_v));

  // LogStruct7
  printf("ang_pid_p:float:%d\n", sizeof(ls7.ang_pid_p));
  printf("ang_pid_i:float:%d\n", sizeof(ls7.ang_pid_i));
  printf("ang_pid_d:float:%d\n", sizeof(ls7.ang_pid_d));
  printf("ang_pid_p_v:float:%d\n", sizeof(ls7.ang_pid_p_v));

  printf("ang_pid_i_v:float:%d\n", sizeof(ls7.ang_pid_i_v));
  printf("ang_pid_d_v:float:%d\n", sizeof(ls7.ang_pid_d_v));
  printf("ff_duty_front:float:%d\n", sizeof(ls7.ff_duty_front));
  printf("ff_duty_roll:float:%d\n", sizeof(ls7.ff_duty_roll));

  printf("ff_duty_rpm_r:float:%d\n", sizeof(ls7.ff_duty_rpm_r));
  printf("ff_duty_rpm_l:float:%d\n", sizeof(ls7.ff_duty_rpm_l));
  printf("x:float:%d\n", sizeof(ls7.x));
  printf("y:float:%d\n", sizeof(ls7.y));

  // LogStruct8
  printf("right45_2:int:%d\n", sizeof(ls8.right45_2));
  printf("right45_3:int:%d\n", sizeof(ls8.right45_3));
  printf("left45_2:int:%d\n", sizeof(ls8.left45_2));
  printf("left45_3:int:%d\n", sizeof(ls8.left45_3));

  printf("right45_2_d:float:%d\n", sizeof(ls8.right45_2_d));
  printf("right45_3_d:float:%d\n", sizeof(ls8.right45_3_d));
  printf("left45_2_d:float:%d\n", sizeof(ls8.left45_2_d));
  printf("left45_3_d:float:%d\n", sizeof(ls8.left45_3_d));

  printf("sen_dist_l45_2:float:%d\n", sizeof(ls8.sen_dist_l45_2));
  printf("sen_dist_r45_2:float:%d\n", sizeof(ls8.sen_dist_r45_2));
  printf("sen_dist_l45_3:float:%d\n", sizeof(ls8.sen_dist_l45_3));
  printf("sen_dist_r45_3:float:%d\n", sizeof(ls8.sen_dist_r45_3));

  // LogStruct9
  printf("knym_v:float:%d\n", sizeof(ls9.knym_v));
  printf("knym_w:float:%d\n", sizeof(ls9.knym_w));
  printf("odm_x:float:%d\n", sizeof(ls9.odm_x));
  printf("odm_y:float:%d\n", sizeof(ls9.odm_y));

  printf("odm_theta:float:%d\n", sizeof(ls9.odm_theta));
  printf("kim_x:float:%d\n", sizeof(ls9.kim_x));
  printf("kim_y:float:%d\n", sizeof(ls9.kim_y));
  printf("kim_theta:float:%d\n", sizeof(ls9.kim_theta));

  printf("ang_i_bias:float:%d\n", sizeof(ls9.ang_i_bias));
  printf("ang_i_bias_val:float:%d\n", sizeof(ls9.ang_i_bias_val));
  printf("left90_d_diff:float:%d\n", sizeof(ls9.left90_d_diff));
  printf("right90_d_diff:float:%d\n", sizeof(ls9.right90_d_diff));

  // LogStruct10
  printf("right45_3_d_diff:float:%d\n", sizeof(ls10.right45_3_d_diff));
  printf("right45_2_d_diff:float:%d\n", sizeof(ls10.right45_2_d_diff));
  printf("right45_d_diff:float:%d\n", sizeof(ls10.right45_d_diff));
  printf("left45_d_diff:float:%d\n", sizeof(ls10.left45_d_diff));

  printf("left45_2_d_diff:float:%d\n", sizeof(ls10.left45_2_d_diff));
  printf("left45_3_d_diff:float:%d\n", sizeof(ls10.left45_3_d_diff));
  printf("duty_suction:float:%d\n", sizeof(ls10.duty_suction));
  printf("reserve1:int:%d\n", sizeof(ls10.reserve1));
  printf("reserve2:int:%d\n", sizeof(ls10.reserve2));

  printf("reserve3:int:%d\n", sizeof(ls10.reserve3));
  printf("reserve4:int:%d\n", sizeof(ls10.reserve4));
  printf("reserve5:int:%d\n", sizeof(ls10.reserve5));

  vTaskDelay(xDelay2);
  printf("start___\n");
  vTaskDelay(xDelay2);
}

void IRAM_ATTR LoggingTask::dump_log(std::string file_name) {
  const TickType_t xDelay2 = 100.0 / portTICK_PERIOD_MS;
  const float PI = 3.141592653589793238;
  const float th = 10;
  int i = 0;
  int c = 0;
  print_header();

  float left90_d_z = 0;
  float right90_d_z = 0;
  float left45_3_d_z = 0;
  float left45_2_d_z = 0;
  float left45_d_z = 0;
  float right45_d_z = 0;
  float right45_2_d_z = 0;
  float right45_3_d_z = 0;

  const auto len = log_vec.size();
  for (const auto &ld : log_vec) {
    ls1.index = i++;
    if (i == len) {
      break;
    }

    left90_d_z = ls3.left90_d;
    left45_d_z = ls3.left45_d;
    right45_d_z = ls3.right45_d;
    right90_d_z = ls3.right90_d;

    left45_2_d_z = ls8.left45_2_d;
    right45_2_d_z = ls8.right45_2_d;
    left45_3_d_z = ls8.left45_3_d;
    right45_3_d_z = ls8.right45_3_d;

    if (ls1.index == 0) {
      left90_d_z = right90_d_z = left45_3_d_z = left45_2_d_z = left45_d_z =
          right45_d_z = right45_2_d_z = right45_3_d_z = 0;
    }
    ls1.ideal_v = halfToFloat(ld->img_v);
    ls1.v_c = halfToFloat(ld->v_c);
    ls1.v_c2 = halfToFloat(ld->v_c2);
    ls1.v_l = halfToFloat(ld->v_l);
    ls1.v_r = halfToFloat(ld->v_r);
    ls1.v_l_enc = ld->v_l_enc;
    ls1.v_r_enc = ld->v_r_enc;
    ls1.v_l_enc_sin = std::sin(2.0 * PI * ld->v_l_enc / ENC_RESOLUTION);
    ls1.v_r_enc_sin = std::sin(2.0 * PI * ld->v_r_enc / ENC_RESOLUTION);
    ls1.accl = halfToFloat(ld->accl) * 1000;
    ls1.accl_x = halfToFloat(ld->accl_x);

    ls2.ideal_w = halfToFloat(ld->img_w);
    ls2.w_lp = halfToFloat(ld->w_lp);
    ls2.alpha = halfToFloat(ld->alpha);
    ls2.ideal_dist = halfToFloat(ld->img_dist);
    ls2.dist = halfToFloat(ld->dist);
    ls2.dist_kf = halfToFloat(ld->dist_kf);
    ls2.ideal_ang = halfToFloat(ld->img_ang);
    ls2.ang = halfToFloat(ld->ang);
    ls2.ang_kf = halfToFloat(ld->ang_kf);

    ls2.left90 = (ld->left90_lp);
    ls2.left45 = (ld->left45_lp);
    ls2.front = (((ld->left90_lp) + (ld->right90_lp)) / 2);

    ls3.right45 = (ld->right45_lp);
    ls3.right90 = (ld->right90_lp);

    auto l90 = calc_sensor((ld->left90_lp), param->sensor_gain.l90.a,
                           param->sensor_gain.l90.b, ld->motion_type);
    auto l45 = calc_sensor((ld->left45_lp), param->sensor_gain.l45.a,
                           param->sensor_gain.l45.b, ld->motion_type);
    auto r45 = calc_sensor((ld->right45_lp), param->sensor_gain.r45.a,
                           param->sensor_gain.r45.b, ld->motion_type);
    auto r45_2 = calc_sensor((ld->right45_2_lp), param->sensor_gain.r45_2.a,
                             param->sensor_gain.r45_2.b, ld->motion_type);
    auto l45_2 = calc_sensor((ld->left45_2_lp), param->sensor_gain.l45_2.a,
                             param->sensor_gain.l45_2.b, ld->motion_type);
    auto l45_3 = calc_sensor((ld->left45_3_lp), param->sensor_gain.l45_3.a,
                             param->sensor_gain.l45_3.b, ld->motion_type);
    auto r45_3 = calc_sensor((ld->right45_3_lp), param->sensor_gain.r45_3.a,
                             param->sensor_gain.r45_3.b, ld->motion_type);

    auto r90 = calc_sensor((ld->right90_lp), param->sensor_gain.r90.a,
                           param->sensor_gain.r90.b, ld->motion_type);

    auto l90_far = calc_sensor((ld->left90_lp), param->sensor_gain.l90_far.a,
                               param->sensor_gain.l90_far.b, ld->motion_type);
    auto r90_far = calc_sensor((ld->right90_lp), param->sensor_gain.r90_far.a,
                               param->sensor_gain.r90_far.b, ld->motion_type);
    float front = 0;
    if (l90 > 0 && r90 > 0) {
      front = (l90 + r90) / 2;
    } else if (l90 == 0 && r90 > 0) {
      front = r90;
    } else if (l90 > 0 && r90 == 0) {
      front = l90;
    } else {
      front = 0;
    }
    float front_far = 0;
    if (l90_far > 0 && r90_far > 0) {
      front_far = (l90_far + r90_far) / 2;
    } else if (l90_far > 0 && r90_far == 0) {
      front_far = l90_far;
    } else if (l90_far == 0 && r90_far > 0) {
      front_far = r90_far;
    } else {
      front_far = 0;
    }
    auto dist = halfToFloat(ld->img_dist);
    float dist_mod = (int)(dist / param->dist_mod_num);
    float tmp_dist = dist - param->dist_mod_num * dist_mod;

    ls3.left90_d = l90;
    ls3.left45_d = l45;
    ls3.front_d = front;
    ls3.right45_d = r45;
    ls3.right90_d = r90;
    ls3.left90_far_d = l90_far;
    ls3.front_far_d = front_far;
    ls3.right90_far_d = r90_far;
    ls3.battery = halfToFloat(ld->battery_lp);
    ls3.duty_l = halfToFloat(ld->duty_l);

    ls4.duty_r = halfToFloat(ld->duty_r);
    ls4.motion_state = ld->motion_type;

    ls4.duty_sen = halfToFloat(ld->duty_sensor_ctrl);
    ls4.dist_mod90 = tmp_dist;
    ls4.sen_dist_l45 = halfToFloat(ld->sen_log_l45);
    ls4.sen_dist_r45 = halfToFloat(ld->sen_log_r45);

    ls4.timestamp = ld->motion_timestamp;
    ls4.sen_calc_time = ld->sen_calc_time;
    ls4.sen_calc_time2 = ld->sen_calc_time2;
    ls4.pln_calc_time = ld->pln_calc_time;
    ls4.pln_calc_time2 = ld->pln_calc_time2;
    ls4.pln_time_diff = ld->pln_time_diff;

    ls5.m_pid_p = halfToFloat(ld->m_pid_p);
    ls5.m_pid_i = halfToFloat(ld->m_pid_i);
    ls5.m_pid_i2 = halfToFloat(ld->m_pid_i2);
    ls5.m_pid_d = halfToFloat(ld->m_pid_d);
    ls5.m_pid_p_v = halfToFloat(ld->m_pid_p_v);
    ls5.m_pid_i_v = halfToFloat(ld->m_pid_i_v);
    ls5.m_pid_i2_v = halfToFloat(ld->m_pid_i2_v);
    ls5.m_pid_d_v = halfToFloat(ld->m_pid_d_v);

    ls5.g_pid_p = halfToFloat(ld->g_pid_p);
    ls5.g_pid_i = halfToFloat(ld->g_pid_i);
    ls5.g_pid_i2 = halfToFloat(ld->g_pid_i2);
    ls5.g_pid_d = halfToFloat(ld->g_pid_d);

    ls6.g_pid_p_v = halfToFloat(ld->g_pid_p_v);
    ls6.g_pid_i_v = halfToFloat(ld->g_pid_i_v);
    ls6.g_pid_i2_v = halfToFloat(ld->g_pid_i2_v);
    ls6.g_pid_d_v = halfToFloat(ld->g_pid_d_v);

    ls6.s_pid_p = halfToFloat(ld->s_pid_p);
    ls6.s_pid_i = halfToFloat(ld->s_pid_i);
    ls6.s_pid_i2 = halfToFloat(ld->s_pid_i2);
    ls6.s_pid_d = halfToFloat(ld->s_pid_d);
    ls6.s_pid_p_v = halfToFloat(ld->s_pid_p_v);
    ls6.s_pid_i_v = halfToFloat(ld->s_pid_i_v);
    ls6.s_pid_i2_v = halfToFloat(ld->s_pid_i2_v);
    ls6.s_pid_d_v = halfToFloat(ld->s_pid_d_v);

    ls7.ang_pid_p = halfToFloat(ld->ang_pid_p);
    ls7.ang_pid_i = halfToFloat(ld->ang_pid_i);
    ls7.ang_pid_d = halfToFloat(ld->ang_pid_d);
    ls7.ang_pid_p_v = halfToFloat(ld->ang_pid_p_v);
    ls7.ang_pid_i_v = halfToFloat(ld->ang_pid_i_v);
    ls7.ang_pid_d_v = halfToFloat(ld->ang_pid_d_v);

    ls7.ff_duty_front = halfToFloat(ld->ff_duty_front);
    ls7.ff_duty_roll = halfToFloat(ld->ff_duty_roll);
    ls7.ff_duty_rpm_r = halfToFloat(ld->ff_duty_rpm_r);
    ls7.ff_duty_rpm_l = halfToFloat(ld->ff_duty_rpm_l);
    ls7.x = halfToFloat(ld->pos_x);
    ls7.y = halfToFloat(ld->pos_y);

    ls8.right45_2 = (ld->right45_2_lp);
    ls8.right45_3 = (ld->right45_3_lp);
    ls8.left45_2 = (ld->left45_2_lp);
    ls8.left45_3 = (ld->left45_3_lp);
    ls8.sen_dist_l45_2 = halfToFloat(ld->sen_log_l45_2);
    ls8.sen_dist_r45_2 = halfToFloat(ld->sen_log_r45_2);
    ls8.sen_dist_l45_3 = halfToFloat(ld->sen_log_l45_3);
    ls8.sen_dist_r45_3 = halfToFloat(ld->sen_log_r45_3);

    ls8.right45_2_d = r45_2;
    ls8.right45_3_d = r45_3;
    ls8.left45_2_d = l45_2;
    ls8.left45_3_d = l45_3;

    ls9.knym_v = halfToFloat(ld->knym_v);
    ls9.knym_w = halfToFloat(ld->knym_w);
    ls9.odm_x = halfToFloat(ld->odm_x);
    ls9.odm_y = halfToFloat(ld->odm_y);

    ls9.odm_theta = halfToFloat(ld->odm_theta);
    ls9.kim_x = halfToFloat(ld->kim_x);
    ls9.kim_y = halfToFloat(ld->kim_y);
    ls9.kim_theta = halfToFloat(ld->kim_theta);
    ls9.ang_i_bias = halfToFloat(ld->ang_i_bias);
    ls9.ang_i_bias_val = halfToFloat(ld->ang_i_bias_val);

    ls9.left90_d_diff = std::clamp(l90 - left90_d_z, -th, th);
    ls9.right90_d_diff = std::clamp(r90 - right90_d_z, -th, th);

    ls10.right45_3_d_diff = std::clamp(r45_3 - right45_3_d_z, -th, th);
    ls10.right45_2_d_diff = std::clamp(r45_2 - right45_2_d_z, -th, th);
    ls10.right45_d_diff = std::clamp(r45 - right45_d_z, -th, th);

    ls10.left45_d_diff = std::clamp(l45 - left45_d_z, -th, th);
    ls10.left45_2_d_diff = std::clamp(l45_2 - left45_2_d_z, -th, th);
    ls10.left45_3_d_diff = std::clamp(l45_3 - left45_3_d_z, -th, th);
    ls10.duty_suction = tgt_val->duty_suction / ls3.battery * 100;

    uart_write_bytes(UART_NUM_0, &ls1, sizeof(LogStruct1));
    uart_write_bytes(UART_NUM_0, &ls2, sizeof(LogStruct2));
    uart_write_bytes(UART_NUM_0, &ls3, sizeof(LogStruct3));
    uart_write_bytes(UART_NUM_0, &ls4, sizeof(LogStruct4));
    uart_write_bytes(UART_NUM_0, &ls5, sizeof(LogStruct5));
    uart_write_bytes(UART_NUM_0, &ls6, sizeof(LogStruct6));
    uart_write_bytes(UART_NUM_0, &ls7, sizeof(LogStruct7));
    uart_write_bytes(UART_NUM_0, &ls8, sizeof(LogStruct8));
    uart_write_bytes(UART_NUM_0, &ls9, sizeof(LogStruct9));
    uart_write_bytes(UART_NUM_0, &ls10, sizeof(LogStruct10));

    c++;
    if (c == 50) {
      c = 0;
      vTaskDelay(1.0 / portTICK_PERIOD_MS);
    }
  }

  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  ls1.index = -1;
  uart_write_bytes(UART_NUM_0, &ls1, sizeof(LogStruct1));
  uart_write_bytes(UART_NUM_0, &ls2, sizeof(LogStruct2));
  uart_write_bytes(UART_NUM_0, &ls3, sizeof(LogStruct3));
  uart_write_bytes(UART_NUM_0, &ls4, sizeof(LogStruct4));
  uart_write_bytes(UART_NUM_0, &ls5, sizeof(LogStruct5));
  uart_write_bytes(UART_NUM_0, &ls6, sizeof(LogStruct6));
  uart_write_bytes(UART_NUM_0, &ls7, sizeof(LogStruct7));
  uart_write_bytes(UART_NUM_0, &ls8, sizeof(LogStruct8));
  uart_write_bytes(UART_NUM_0, &ls9, sizeof(LogStruct9));
  uart_write_bytes(UART_NUM_0, &ls10, sizeof(LogStruct10));

  vTaskDelay(10.0 / portTICK_PERIOD_MS);

  printf("end___\n"); // csvファイル追記終了トリガー

  vTaskDelay(2500.0 / portTICK_PERIOD_MS);

  printf("memory: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

  printf("gyro bias: %f\n", tgt_val->gyro_zero_p_offset);
  printf("gyro var_robust_dps2: %f\n", tgt_val->var_robust_dps2);
  printf("gyro var_unbiased_dps2: %f\n", tgt_val->var_unbiased_dps2);
  printf("gyro retry: %d\n", tgt_val->gyro_retry);
  log_vec.clear();
  // umount();
  // std::vector<std::shared_ptr<log_data_t2>>().swap(log_vec);
}

void IRAM_ATTR LoggingTask::dump_log2(std::string file_name) {}

void LoggingTask::dump_log_sysid(std::string file_name) {

  const TickType_t xDelay2 = 100.0 / portTICK_PERIOD_MS;
  printf("start___\n"); // csvファイル作成トリガー
  vTaskDelay(xDelay2);
  printf("index,v_l,v_c,v_r,w,volt_l,volt_r\n");
  int c = 0;
  for (const auto &ld : sysidlog_vec) {
    printf("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n", //
           ++c,                                        //
           halfToFloat(ld->v_l),                       //
           halfToFloat(ld->v_c),                       //
           halfToFloat(ld->v_r),                       //
           halfToFloat(ld->w_lp),                      //
           halfToFloat(ld->volt_l),                    //
           halfToFloat(ld->volt_r));                   //
  }
  printf("end___\n"); // csvファイル追記終了トリガー
}

void IRAM_ATTR LoggingTask::set_data() {

  if (tgt_val->calc_time_diff > 3000) {
    return;
  }

  log_data_t2 *structPtr =
      (log_data_t2 *)heap_caps_malloc(sizeof(log_data_t2), MALLOC_CAP_SPIRAM);
  auto ld = std::shared_ptr<log_data_t2>(structPtr, heap_caps_free);

  ld->img_v = floatToHalf(tgt_val->ego_in.v);
  ld->v_l = floatToHalf(sensing_result->ego.v_l);
  // ld->v_l = floatToHalf(sensing_result->ego.v_kf);
  ld->v_c = floatToHalf(sensing_result->ego.v_c);
  ld->v_c2 = floatToHalf(sensing_result->ego.v_kf);
  ld->v_r = floatToHalf(sensing_result->ego.v_r);
  ld->v_r_enc = (sensing_result->encoder.right);
  ld->v_l_enc = (sensing_result->encoder.left);
  ld->accl = floatToHalf(tgt_val->ego_in.accl / 1000);
  ld->accl_x = floatToHalf(sensing_result->ego.w_kf);
  ld->dist_kf = floatToHalf(sensing_result->ego.dist_kf);

  // ld->accl_x = floatToHalf((float)tgt_val->ego_in.state);
  // ld->accl = floatToHalf(tgt_val->v_error);
  // ld->accl_x = floatToHalf(tgt_val->w_error);

  ld->img_w = floatToHalf(tgt_val->ego_in.w);
  ld->w_lp = floatToHalf(sensing_result->ego.w_lp);
  // ld->w_lp2 = floatToHalf(sensing_result->ego.w_lp2);
  // ld->w_kf2 = floatToHalf(sensing_result->ego.w_kf2);
  ld->alpha = floatToHalf(tgt_val->ego_in.alpha);

  ld->img_dist = floatToHalf(tgt_val->ego_in.img_dist);
  ld->dist = floatToHalf(tgt_val->ego_in.dist);

  ld->img_ang = floatToHalf((tgt_val->ego_in.img_ang) * 180 / m_PI);
  ld->ang = floatToHalf(tgt_val->ego_in.ang * 180 / m_PI);
  ld->ang_kf = floatToHalf(sensing_result->ego.ang_kf * 180 / m_PI);
  // ld->ang_kf2 = floatToHalf(sensing_result->ego.ang_kf2 * 180 / m_PI);

  ld->left90_lp = (sensing_result->led_sen.left90.raw);
  ld->left45_lp = (sensing_result->led_sen.left45.raw);
  // ld->front_lp = floatToHalf(sensing_result->ego.front_lp);
  ld->right45_lp = (sensing_result->led_sen.right45.raw);
  ld->right90_lp = (sensing_result->led_sen.right90.raw);

  ld->left45_2_lp = (sensing_result->led_sen.left45_2.raw);
  ld->right45_2_lp = (sensing_result->led_sen.right45_2.raw);
  ld->left45_3_lp = (sensing_result->led_sen.left45_3.raw);
  ld->right45_3_lp = (sensing_result->led_sen.right45_3.raw);

  ld->battery_lp = floatToHalf(sensing_result->ego.batt_kf);
  ld->duty_l = floatToHalf(sensing_result->ego.duty.duty_l);
  ld->duty_r = floatToHalf(sensing_result->ego.duty.duty_r);

  ld->ff_duty_front = floatToHalf(sensing_result->ego.duty.ff_duty_front);
  ld->ff_duty_roll = floatToHalf(sensing_result->ego.duty.ff_duty_roll);
  ld->ff_duty_rpm_r = floatToHalf(sensing_result->ego.duty.ff_duty_rpm_r);
  ld->ff_duty_rpm_l = floatToHalf(sensing_result->ego.duty.ff_duty_rpm_l);

  ld->motion_type = static_cast<int>(tgt_val->motion_type);

  ld->duty_sensor_ctrl = floatToHalf(sensing_result->ego.duty.sen);

  ld->sen_log_l45 = floatToHalf(sensing_result->sen.l45.sensor_dist);
  ld->sen_log_r45 = floatToHalf(sensing_result->sen.r45.sensor_dist);
  ld->sen_log_l45_2 = floatToHalf(sensing_result->sen.l45_2.sensor_dist);
  ld->sen_log_r45_2 = floatToHalf(sensing_result->sen.r45_2.sensor_dist);
  ld->sen_log_l45_3 = floatToHalf(sensing_result->sen.l45_3.sensor_dist);
  ld->sen_log_r45_3 = floatToHalf(sensing_result->sen.r45_3.sensor_dist);

  ld->motion_timestamp = tgt_val->nmr.timstamp;
  ld->sen_calc_time = sensing_result->calc_time;
  ld->sen_calc_time2 = sensing_result->calc_time2;
  ld->pln_calc_time = tgt_val->calc_time;
  ld->pln_calc_time2 = tgt_val->calc_time2;
  ld->pln_time_diff = tgt_val->calc_time_diff;

  ld->m_pid_p = floatToHalf(error_entity->v_val.p);
  ld->m_pid_i = floatToHalf(error_entity->v_val.i);
  ld->m_pid_i2 = floatToHalf(error_entity->v_val.i2);
  ld->m_pid_d = floatToHalf(error_entity->v_val.d);
  if (idx_slalom_log == 0 || static_cast<int>(tgt_val->motion_type) == 0 ||
      (error_entity->v_val.i_val == 0)) {
    ld->m_pid_p_v = (real16_T)0;
  } else {
    ld->m_pid_p_v = floatToHalf(error_entity->v_val.p_val);
  }
  ld->m_pid_i_v = floatToHalf(error_entity->v_val.i_val);
  ld->m_pid_i2_v = floatToHalf(error_entity->v_val.i2_val);
  ld->m_pid_d_v = floatToHalf(error_entity->v_val.d_val);

  ld->g_pid_p = floatToHalf(error_entity->w_val.p);
  ld->g_pid_i = floatToHalf(error_entity->w_val.i);
  ld->g_pid_i2 = floatToHalf(error_entity->w_val.i2);
  ld->g_pid_d = floatToHalf(error_entity->w_val.d);
  ld->g_pid_p_v = floatToHalf(error_entity->w_val.p_val);
  ld->g_pid_i_v = floatToHalf(error_entity->w_val.i_val);
  ld->g_pid_i2_v = floatToHalf(error_entity->w_val.i2_val);
  if (idx_slalom_log == 0 || static_cast<int>(tgt_val->motion_type) == 0 ||
      (error_entity->v_val.i_val == 0)) {
    ld->g_pid_d_v = (real16_T)0;
  } else {
    ld->g_pid_d_v = floatToHalf(error_entity->w_val.d_val);
  }

  ld->ang_pid_p = floatToHalf(error_entity->ang_val.p);
  ld->ang_pid_i = floatToHalf(error_entity->ang_val.i);
  ld->ang_pid_d = floatToHalf(error_entity->ang_val.d);
  ld->ang_pid_p_v = floatToHalf(error_entity->ang_val.p_val);
  ld->ang_pid_i_v = floatToHalf(error_entity->ang_val.i_val);
  ld->ang_pid_d_v = floatToHalf(error_entity->ang_val.d_val);

  ld->s_pid_p = floatToHalf(error_entity->s_val.p);
  ld->s_pid_i = floatToHalf(error_entity->s_val.i);
  ld->s_pid_i2 = floatToHalf(error_entity->s_val.i2);
  ld->s_pid_d = floatToHalf(error_entity->s_val.d);
  ld->s_pid_p_v = floatToHalf(error_entity->s_val.p_val);
  ld->s_pid_i_v = floatToHalf(error_entity->s_val.i_val);
  ld->s_pid_i2_v = floatToHalf(error_entity->s_val.i2_val);
  ld->s_pid_d_v = floatToHalf(error_entity->s_val.d_val);

  // ld->pos_x = floatToHalf(sensing_result->ego.pos_x);
  // ld->pos_y = floatToHalf(sensing_result->ego.pos_y);
  ld->pos_x = floatToHalf(sensing_result->ego.pos_x);
  ld->pos_y = floatToHalf(sensing_result->ego.pos_y);

  ld->knym_v = floatToHalf(sensing_result->ego.knym_v);
  ld->knym_w = floatToHalf(sensing_result->ego.knym_w);
  ld->odm_x = floatToHalf(sensing_result->ego.odm_x);
  ld->odm_y = floatToHalf(sensing_result->ego.odm_y);
  ld->odm_theta = floatToHalf(sensing_result->ego.odm_theta);
  ld->kim_x = floatToHalf(sensing_result->ego.kim_x);
  ld->kim_y = floatToHalf(sensing_result->ego.kim_y);
  ld->kim_theta = floatToHalf(sensing_result->ego.kim_theta);
  ld->ang_i_bias = floatToHalf(error_entity->ang_val.i2);
  ld->ang_i_bias_val = floatToHalf(error_entity->ang_val.i2_val);

  if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > 7500) {
    log_vec.emplace_back(std::move(ld));
    idx_slalom_log++;
  }
}