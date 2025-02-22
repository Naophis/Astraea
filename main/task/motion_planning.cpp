#include "include/motion_planning.hpp"

void MotionPlanning::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

void MotionPlanning::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void MotionPlanning::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_entity) {
  sensing_result = _entity;
}

void MotionPlanning::set_userinterface(std::shared_ptr<UserInterface> &_ui) {
  ui = _ui;
}

void MotionPlanning::set_path_creator(std::shared_ptr<PathCreator> &_pc) {
  pc = _pc;
}
void MotionPlanning::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
}
void MotionPlanning::set_logging_task(std::shared_ptr<LoggingTask> &_lt) {
  lt = _lt; //
}
MotionResult IRAM_ATTR MotionPlanning::go_straight(
    param_straight_t &p, std::shared_ptr<Adachi> &adachi, bool search_mode) {
  tgt_val->nmr.v_max = p.v_max;
  tgt_val->nmr.v_end = p.v_end;
  tgt_val->nmr.accl = p.accl;
  tgt_val->nmr.decel = p.decel;
  tgt_val->nmr.dist = p.dist;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::ST_RUN;
  tgt_val->nmr.motion_type = MotionType::STRAIGHT;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = p.dia_mode;

  const auto ego_v = tgt_val->ego_in.v;
  const auto req_dist =
      std::abs((ego_v * ego_v - p.v_end * p.v_end) / (2 * p.decel));
  if (req_dist > p.dist) {
    p.accl =
        std::abs((ego_v * ego_v - p.v_end * p.v_end) / (2 * p.dist)) + 1000;
    p.decel = -p.accl;
  }

  // if (p.motion_type == MotionType::SLA_FRONT_STR) {
  //   // 次のタイミングで通り過ぎてしまう場合は早めに切り上げる
  //   const auto tmp_dist = tgt_val->ego_in.dist + tgt_val->ego_in.v * dt;
  //   if (std::abs(tmp_dist) >= std::abs(p.dist)) {
  //     auto diff = std::abs(p.dist - tgt_val->ego_in.dist);
  //     volatile auto time = diff / tgt_val->ego_in.v * 240000000.0 * dt;
  //     for (int i = 0; i < time; i++)
  //       ;
  //     return MotionResult::NONE;
  //   }
  // }

  const auto left = param->sen_ref_p.normal.exist.left45;
  const auto right = param->sen_ref_p.normal.exist.right45;

  if (adachi != nullptr) {
    // if (p.search_str_wide_ctrl_l) {
    //   param->sen_ref_p.normal.exist.left45 =
    //       param->wall_off_dist.go_straight_wide_ctrl_th;
    // }
    // if (p.search_str_wide_ctrl_r) {
    //   param->sen_ref_p.normal.exist.right45 =
    //       param->wall_off_dist.go_straight_wide_ctrl_th;
    // }
  }
  tgt_val->nmr.sct = p.sct;
  if (p.motion_type != MotionType::NONE) {
    tgt_val->nmr.motion_type = p.motion_type;
  }
  tgt_val->nmr.timstamp++;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
  vTaskDelay(1.0 / portTICK_RATE_MS);
  if (search_mode && adachi != nullptr) {
    adachi->update();
  }

  if (p.motion_type == MotionType::PIVOT_PRE ||
      p.motion_type == MotionType::PIVOT_PRE2 ||
      p.motion_type == MotionType::SLA_FRONT_STR ||
      p.motion_type == MotionType::SLA_BACK_STR ||
      p.motion_type == MotionType::BACK_STRAIGHT) {
    if (std::abs(p.dist) > 5) {
      vTaskDelay(1.0 / portTICK_RATE_MS);
    }
  }

  unsigned int cnt = 0;

  float tmp_dist_before = tgt_val->global_pos.dist;
  float tmp_dist_after = tmp_dist_before;
  int wall_off_state = 0;

  bool exist = false;
  bool exist_right = false;
  bool exist_left = false;
  if (search_mode) {
    exist_right = sensing_result->ego.right45_dist <
                  param->wall_off_dist.ctrl_exist_wall_th_r;
    exist_left = sensing_result->ego.left45_dist <
                 param->wall_off_dist.ctrl_exist_wall_th_l;
  }

  while (1) {
    cnt++;
    auto now_dist = (cnt == 1) ? 0.0f : tgt_val->ego_in.dist;
    if (std ::abs(now_dist) >= std::abs(p.dist)) {
      break;
    }

    if (p.motion_type == MotionType::SLA_FRONT_STR) {
      // 次のタイミングで通り過ぎてしまう場合は早めに切り上げる
      const auto tmp_dist = now_dist + tgt_val->ego_in.v * dt;
      if (std::abs(tmp_dist) >= std::abs(p.dist)) {
        auto diff = std::abs(p.dist - now_dist);
        volatile auto time = diff / tgt_val->ego_in.v * 240000000.0 * dt;
        param->sen_ref_p.normal.exist.left45 = left;
        param->sen_ref_p.normal.exist.right45 = right;
        for (int i = 0; i < time; i++)
          ;
        return MotionResult::NONE;
      }
    }
    if (search_mode && param->wall_off_dist.search_wall_off_enable) {
      // watch wall off
      if (wall_off_state == 0) {
        if (sensing_result->ego.right45_dist <
                param->wall_off_dist.exist_dist_r &&
            exist_right) {
          wall_off_state |= 1;
        }
        if (sensing_result->ego.left45_dist <
                param->wall_off_dist.exist_dist_l &&
            exist_left) {
          wall_off_state |= 2;
        }
      } else if (wall_off_state == 3 && exist_right && exist_left) {
        if (sensing_result->ego.right45_dist >
            param->wall_off_dist.noexist_th_r) {
          wall_off_state = 4;
          p.dist = param->wall_off_dist.search_wall_off_r_dist_offset;
          param->sen_ref_p.normal.exist.left45 = left;
          param->sen_ref_p.normal.exist.right45 = right;
          return go_straight(p, fake_adachi, false);
        } else if (sensing_result->ego.left45_dist >
                   param->wall_off_dist.noexist_th_l) {
          wall_off_state = 4;
          p.dist = param->wall_off_dist.search_wall_off_l_dist_offset;
          param->sen_ref_p.normal.exist.left45 = left;
          param->sen_ref_p.normal.exist.right45 = right;
          return go_straight(p, fake_adachi, false);
        }
      } else if (wall_off_state == 1 && exist_right) {
        if (sensing_result->ego.right45_dist >
            param->wall_off_dist.noexist_th_r) {
          wall_off_state = 4;
          p.dist = param->wall_off_dist.search_wall_off_r_dist_offset;
          param->sen_ref_p.normal.exist.left45 = left;
          param->sen_ref_p.normal.exist.right45 = right;
          return go_straight(p, fake_adachi, false);
        }
      } else if (wall_off_state == 2 && exist_left) {
        if (sensing_result->ego.left45_dist >
            param->wall_off_dist.noexist_th_l) {
          wall_off_state = 4;
          p.dist = param->wall_off_dist.search_wall_off_l_dist_offset;
          param->sen_ref_p.normal.exist.left45 = left;
          param->sen_ref_p.normal.exist.right45 = right;
          return go_straight(p, fake_adachi, false);
        }
      }
    }

    tmp_dist_after = tgt_val->global_pos.dist;

    if (std::abs(tmp_dist_after - tmp_dist_before) >=
        param->clear_dist_ragne_to2) {
      param->sen_ref_p.normal.exist.left45 = left;
      param->sen_ref_p.normal.exist.right45 = right;
    }
    if (p.v_end <= 10) {
      if (cnt > 1000) {
        break;
      }
    }

    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      if (p.motion_type == MotionType::STRAIGHT &&
          p.wall_ctrl_mode == WallCtrlMode::LEFT_ONLY && p.dist < 90) {
      } else if (p.motion_type == MotionType::SLA_FRONT_STR ||
                 p.motion_type == MotionType::SLA_BACK_STR) {
      } else {
        param->sen_ref_p.normal.exist.left45 = left;
        param->sen_ref_p.normal.exist.right45 = right;
        req_error_reset();
        tgt_val->nmr.v_max = 0.1;
        tgt_val->nmr.v_end = 0.1;
        tgt_val->nmr.accl = 1000;
        tgt_val->nmr.decel = p.decel;
        tgt_val->nmr.dist = 10;
        tgt_val->nmr.timstamp += 10;

        xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
        vTaskDelay(1.0 / portTICK_RATE_MS);

        return MotionResult::ERROR;
      }
    }
    vTaskDelay(1.0 / portTICK_RATE_MS);
  }
  param->sen_ref_p.normal.exist.left45 = left;
  param->sen_ref_p.normal.exist.right45 = right;
  return MotionResult::NONE;
}

MotionResult IRAM_ATTR MotionPlanning::go_straight(param_straight_t &p) {
  return go_straight(p, fake_adachi, false);
}

MotionResult IRAM_ATTR MotionPlanning::pivot_turn(param_roll_t &p) {
  // 一度初期化
  pt->motor_enable();
  reset_tgt_data();
  reset_ego_data();
  tgt_val->motion_type = MotionType::NONE;
  tgt_val->nmr.timstamp++;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
  vTaskDelay(1.0 / portTICK_RATE_MS);

  tgt_val->nmr.v_max = 0;
  tgt_val->nmr.v_end = 0;
  tgt_val->nmr.accl = 0;
  tgt_val->nmr.decel = 0;
  tgt_val->nmr.dist = 0;
  if (p.RorL == TurnDirection::Left) {
    tgt_val->nmr.w_max = p.w_max;
    tgt_val->nmr.w_end = p.w_end;
    tgt_val->nmr.alpha = p.alpha;
    tgt_val->nmr.ang = p.ang;
    tgt_val->nmr.motion_dir = MotionDirection::LEFT;
  } else {
    tgt_val->nmr.w_max = -p.w_max;
    tgt_val->nmr.w_end = -p.w_end;
    tgt_val->nmr.alpha = -p.alpha;
    tgt_val->nmr.ang = -p.ang;
    tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  }
  tgt_val->nmr.motion_mode = RUN_MODE2::PIVOT_TURN;
  tgt_val->nmr.motion_type = MotionType::PIVOT;
  tgt_val->nmr.sct = SensorCtrlType::NONE;
  tgt_val->nmr.timstamp++;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
  vTaskDelay(10.0 / portTICK_RATE_MS);
  const auto sr = sensing_result;
  int c = 0;
  while (1) {
    vTaskDelay(1.0 / portTICK_RATE_MS);
    c++;
    if (std::abs(tgt_val->ego_in.ang) >= std::abs(p.ang) &&
        std::abs(tgt_val->ego_in.ang * 180 / m_PI) > 10) {
      break;
    }
    if (c == 250) { //動き出さないとき
      if (std::abs(tgt_val->ego_in.ang * 180 / m_PI) < 10) {
        pt->motor_disable();
        vTaskDelay(10.0 / portTICK_RATE_MS);
        pt->motor_enable();
        reset_tgt_data();
        reset_ego_data();
        tgt_val->motion_type = MotionType::NONE;
        tgt_val->nmr.timstamp++;

        xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
        vTaskDelay(1.0 / portTICK_RATE_MS);

        tgt_val->nmr.v_max = 0;
        tgt_val->nmr.v_end = 0;
        tgt_val->nmr.accl = 0;
        tgt_val->nmr.decel = 0;
        tgt_val->nmr.dist = 0;
        if (p.RorL == TurnDirection::Left) {
          tgt_val->nmr.w_max = p.w_max;
          tgt_val->nmr.w_end = p.w_end;
          tgt_val->nmr.alpha = p.alpha;
          tgt_val->nmr.ang = p.ang;
          tgt_val->nmr.motion_dir = MotionDirection::LEFT;
        } else {
          tgt_val->nmr.w_max = -p.w_max;
          tgt_val->nmr.w_end = -p.w_end;
          tgt_val->nmr.alpha = -p.alpha;
          tgt_val->nmr.ang = -p.ang;
          tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
        }
        tgt_val->nmr.motion_mode = RUN_MODE2::PIVOT_TURN;
        tgt_val->nmr.motion_type = MotionType::PIVOT;
        tgt_val->nmr.sct = SensorCtrlType::NONE;
        tgt_val->nmr.timstamp++;
        c = 0;

        xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
        vTaskDelay(10.0 / portTICK_RATE_MS);
      }
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      // return MotionResult::ERROR;
    }
  }
  return MotionResult::NONE;
}
void IRAM_ATTR MotionPlanning::req_error_reset() {
  tgt_val->pl_req.error_vel_reset = 1;
  tgt_val->pl_req.error_gyro_reset = 1;
  tgt_val->pl_req.error_ang_reset = 1;
  tgt_val->pl_req.error_dist_reset = 1;
  tgt_val->pl_req.time_stamp++;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
}
MotionResult IRAM_ATTR MotionPlanning::slalom(slalom_param2_t &sp,
                                              TurnDirection td,
                                              next_motion_t &next_motion) {
  return slalom(sp, td, next_motion, false);
}

MotionResult IRAM_ATTR MotionPlanning::slalom(slalom_param2_t &sp,
                                              TurnDirection td,
                                              next_motion_t &next_motion,
                                              bool dia) {
  return slalom(sp, td, next_motion, dia, fake_adachi, false);
}
MotionResult IRAM_ATTR MotionPlanning::slalom(
    slalom_param2_t &sp, TurnDirection td, next_motion_t &next_motion, bool dia,
    std::shared_ptr<Adachi> &adachi, bool search_mode) {
  bool find = false;
  bool find_r = false;
  bool find_l = false;
  const auto se = get_sensing_entity();

  ps_front.search_str_wide_ctrl_l = ps_front.search_str_wide_ctrl_r =
      ps_back.search_str_wide_ctrl_l = ps_back.search_str_wide_ctrl_r = false;

  ps_front.v_max = sp.v;
  ps_front.v_end = sp.v;
  ps_front.accl = next_motion.accl;
  ps_front.decel = next_motion.decel;

  ps_front.dist = (td == TurnDirection::Right) ? sp.front.right : sp.front.left;
  if (adachi != nullptr) {
    ps_front.dist -= adachi->diff;
  }
  ps_front.skil_wall_off = next_motion.skip_wall_off;
  ps_front.motion_type = MotionType::SLA_FRONT_STR;
  ps_front.sct = SensorCtrlType::Straight;
  if (sp.type == TurnType::Dia45_2 || sp.type == TurnType::Dia135_2 ||
      sp.type == TurnType::Dia90) {
    ps_front.sct = SensorCtrlType::NONE;
  }
  // ps_front.sct = SensorCtrlType::NONE;
  ps_front.wall_off_req = WallOffReq::NONE;

  ps_back.dist = (td == TurnDirection::Right) ? sp.back.right : sp.back.left;
  next_motion.carry_over_dist = 0;
  bool offset_r = false;
  bool offset_l = false;
  if (sp.type == TurnType::Normal) {
    // search_front_ctrl(ps_front); // 前壁制御
    ps_front.v_max = next_motion.v_max;
    if (param->front_dist_offset_pivot_th <
            sensing_result->ego.left90_mid_dist &&
        sensing_result->ego.left90_mid_dist < param->sla_front_ctrl_th &&
        param->front_dist_offset_pivot_th <
            sensing_result->ego.right90_mid_dist &&
        sensing_result->ego.right90_mid_dist < param->sla_front_ctrl_th) {
      float diff =
          (sensing_result->ego.front_mid_dist - param->front_dist_offset);
      diff = std::clamp(diff, -param->normal_sla_offset_front,
                        param->normal_sla_offset_front);
      ps_front.dist += diff;
      if (ps_front.dist < 0) {
        ps_front.dist = 1;
      }
      // (sensing_result->ego.front_dist - param->front_dist_offset);
    }
    if (td == TurnDirection::Right) {
      if ((10 < sensing_result->ego.left45_dist) &&
          (sensing_result->ego.left45_dist < param->th_offset_dist)) {
        offset_l = true;
        float diff = (param->sla_wall_ref_l - sensing_result->ego.left45_dist);
        diff = std::clamp(diff, -param->normal_sla_offset_back,
                          param->normal_sla_offset_back);
        ps_back.dist += diff;
        if (ps_back.dist < 0) {
          ps_back.dist = 1;
        }
      }
    } else {
      if ((10 < sensing_result->ego.right45_dist) &&
          (sensing_result->ego.right45_dist < param->th_offset_dist)) {
        offset_r = true;
        float diff = (param->sla_wall_ref_r - sensing_result->ego.right45_dist);
        diff = std::clamp(diff, -param->normal_sla_offset_back,
                          param->normal_sla_offset_back);
        ps_back.dist += diff;
        if (ps_back.dist < 0) {
          ps_back.dist = 1;
        }
      }
    }
    if (ps_front.dist > (0)) {
      res_f = go_straight(ps_front);
      if (res_f != MotionResult::NONE) {
        return MotionResult::ERROR;
      }
    }
  } else if (sp.type == TurnType::Large) {
    bool b = true;
    float dist_r = ps_back.dist;
    float dist_l = ps_back.dist;
    if (td == TurnDirection::Right) {
      if (sensing_result->ego.right45_dist < param->th_offset_dist) {
        dist_r -= param->sla_wall_ref_r - sensing_result->ego.right45_dist;
        find_r = true;
      }
      if (sensing_result->ego.left45_dist < param->th_offset_dist) {
        dist_l += param->sla_wall_ref_l - sensing_result->ego.left45_dist;
        find_l = true;
      }
    } else {
      if (sensing_result->ego.left45_dist < param->th_offset_dist) {
        dist_l -= param->sla_wall_ref_l - sensing_result->ego.left45_dist;
        find_l = true;
      }
      if (sensing_result->ego.right45_dist < param->th_offset_dist) {
        dist_r += param->sla_wall_ref_r - sensing_result->ego.right45_dist;
        find_r = true;
      }
    }
    if (find_r && find_l) {
      ps_back.dist =
          (std::abs(ps_back.dist - dist_r) < std::abs(ps_back.dist - dist_l))
              ? dist_r
              : dist_l;
    } else if (find_r) {
      ps_back.dist = dist_r;
    } else if (find_l) {
      ps_back.dist = dist_l;
    }
    ps_back.dist -= (td == TurnDirection::Right) ? param->offset_after_turn_r2
                                                 : param->offset_after_turn_l2;
    if (b && !next_motion.skip_wall_off) {
      wall_off(td, ps_front);
    }
    if (ps_front.dist > 0 && !next_motion.skip_wall_off) {
      res_f = go_straight(ps_front);
      if (res_f != MotionResult::NONE) {
        return MotionResult::ERROR;
      }
    }
  } else if (sp.type == TurnType::Orval) {
    bool b = true;
    if (param->orval_front_ctrl_min < sensing_result->ego.left90_mid_dist &&
        sensing_result->ego.left90_mid_dist < param->orval_front_ctrl_max &&
        param->orval_front_ctrl_min < sensing_result->ego.right90_mid_dist &&
        sensing_result->ego.right90_mid_dist < param->orval_front_ctrl_max) {
      ps_front.dist +=
          (sensing_result->ego.front_mid_dist - param->front_dist_offset2);
      // ps_front.dist = 0;
      b = false;
    }
    // float default_rad = sp.rad;
    float rad_r = sp.rad;
    float rad_l = sp.rad;

    if (sp.pow_n > 20) {
      if (td == TurnDirection::Left) {
        rad_r = sp.rad;
        rad_l = sp.rad;
      } else {
        rad_r = sp.pow_n;
        rad_l = sp.pow_n;
      }
    }
    ps_back.dist -= (td == TurnDirection::Right) ? param->offset_after_turn_r2
                                                 : param->offset_after_turn_l2;
    if (b) {
      wall_off(td, ps_front);
    }
    if (ps_front.dist > (0)) {
      res_f = go_straight(ps_front);
      if (res_f != MotionResult::NONE) {
        return MotionResult::ERROR;
      }
    }
    // if (ps_back.dist < 0) {
    //   ps_back.dist = 2;
    // }

  } else if (sp.type == TurnType::Dia45 || sp.type == TurnType::Dia135) {
    bool b = true;
    // if (sensing_result->ego.left90_dist < 150 &&
    //     sensing_result->ego.right90_dist < 150) {
    //   ps_front.dist -=
    //       (param->front_dist_offset2 - sensing_result->ego.front_dist);
    //   b = false;
    // }
    if (b && !next_motion.skip_wall_off) {
      wall_off(td, ps_front);
    }
    if (sp.type == TurnType::Dia135) {
      // calc_dia135_offset(ps_front, ps_back, td, !b);
    } else if (sp.type == TurnType::Dia45) {
      // calc_dia45_offset(ps_front, ps_back, td, !b);
    }
    if (ps_front.dist > (0) && !next_motion.skip_wall_off) {
      res_f = go_straight(ps_front);
      if (res_f != MotionResult::NONE) {
        return MotionResult::ERROR;
      }
    }
    ps_back.dist -= (td == TurnDirection::Right) ? param->offset_after_turn_r
                                                 : param->offset_after_turn_l;
  } else if (sp.type == TurnType::Dia45_2 || sp.type == TurnType::Dia135_2 ||
             sp.type == TurnType::Dia90) {
    bool exist = false;
    bool use_oppo_wall = false;
    bool result = wall_off_dia(td, ps_front, exist, use_oppo_wall);
    float dist = 0;
    if (result && !use_oppo_wall) {
      if (sp.type == TurnType::Dia135_2) {
        if (td == TurnDirection::Right) {
          const auto ref = param->dia_wall_off_ref_r;
          // exist ? param->dia_wall_off_ref_r : param->dia_wall_off_ref_r2;
          dist = (ref - se->sen.r45.sensor_dist) / ROOT2;
        } else {
          const auto ref = param->dia_wall_off_ref_l;
          // exist ? param->dia_wall_off_ref_l : param->dia_wall_off_ref_l2;
          dist = (ref - se->sen.l45.sensor_dist) / ROOT2;
        }
        dist = std::clamp(dist, -param->dia_offset_max_dist,
                          param->dia_offset_max_dist);
      } else if (sp.type == TurnType::Dia90) {
        if (td == TurnDirection::Right) {
          const auto ref = param->dia_wall_off_ref_r;
          // exist ? param->dia_wall_off_ref_r : param->dia_wall_off_ref_r2;
          dist = (ref - se->sen.r45.sensor_dist) / ROOT2;
        } else {
          const auto ref = param->dia_wall_off_ref_l;
          // exist ? param->dia_wall_off_ref_l : param->dia_wall_off_ref_l2;
          dist = (ref - se->sen.l45.sensor_dist) / ROOT2;
        }
        dist = std::clamp(dist, -param->dia_offset_max_dist,
                          param->dia_offset_max_dist);
      }
    }
    // TODO: ここでdistを使ってps_front.distを調整する
    ps_front.dist = ps_front.dist - dist;
    if (ps_front.dist > (0)) {
      res_f = go_straight(ps_front);
      if (res_f != MotionResult::NONE) {
        return MotionResult::ERROR;
      }
    }
    ps_back.dist -= (td == TurnDirection::Right)
                        ? param->offset_after_turn_dia_r
                        : param->offset_after_turn_dia_l;
  } else {
    if (ps_front.dist > (sp.v * dt)) {
      res_f = go_straight(ps_front);
      if (res_f != MotionResult::NONE) {
        return MotionResult::ERROR;
      }
    }
  }

  float alphaTemp = ((td == TurnDirection::Right) ? -1 : 1) * (sp.v / sp.rad);

  tgt_val->nmr.motion_dir = (td == TurnDirection::Left)
                                ? MotionDirection::LEFT
                                : MotionDirection::RIGHT;

  tgt_val->nmr.v_max = sp.v;
  tgt_val->nmr.v_end = sp.v;
  tgt_val->nmr.accl = next_motion.accl;
  tgt_val->nmr.decel = next_motion.decel;
  tgt_val->nmr.dist = 180 * 1000;

  tgt_val->nmr.sla_alpha = alphaTemp;
  tgt_val->nmr.sla_time = sp.time;
  tgt_val->nmr.sla_pow_n = sp.pow_n;

  tgt_val->nmr.motion_mode = RUN_MODE2::SLAROM_RUN;
  tgt_val->nmr.motion_type = MotionType::SLALOM;
  tgt_val->nmr.sct = SensorCtrlType::NONE;
  tgt_val->nmr.td = td;
  tgt_val->nmr.tt = sp.type;
  // tgt_val->ego_in.v = sp.v;//強制的に速度を指定

  if (sp.type == TurnType::Orval) {
    if (td == TurnDirection::Left) {
      tgt_val->nmr.sla_time = sp.time;
      tgt_val->nmr.sla_rad = sp.rad;
      tgt_val->nmr.sla_alpha = (sp.v / sp.rad);
    } else {
      tgt_val->nmr.sla_time = sp.time2;
      tgt_val->nmr.sla_rad = sp.rad2;
      tgt_val->nmr.sla_alpha = -(sp.v / sp.rad2);
    }
  }

  if (sp.type == TurnType::Orval && (sp.pow_n < 2 || sp.pow_n > 40)) {
    // tgt_val->nmr.motion_mode = RUN_MODE2::SLALOM_RUN2;
    // tgt_val->nmr.ang = sp.ang;
    // tgt_val->nmr.sla_rad = sp.rad;
    // tgt_val->nmr.w_end = 0;
    // tgt_val->nmr.alpha = (2 * sp.v * sp.v / (sp.rad * sp.rad * sp.ang / 3));
    // tgt_val->nmr.w_max = 200000;
    // if (td == TurnDirection::Right) {
    //   tgt_val->nmr.w_max = -200000;
    //   tgt_val->nmr.alpha = -(2 * sp.v * sp.v / (sp.rad * sp.rad * sp.ang /
    //   3));
    // }
  } else if (sp.type == TurnType::Dia45 && sp.time == 0) {
    tgt_val->nmr.motion_mode = RUN_MODE2::SLALOM_RUN2;
    tgt_val->nmr.ang = sp.ang;
    tgt_val->nmr.sla_rad = sp.rad;
    tgt_val->nmr.w_end = 0;
    tgt_val->nmr.alpha = (2 * sp.v * sp.v / (sp.rad * sp.rad * sp.ang / 3));
    tgt_val->nmr.w_max = 200000;
    if (td == TurnDirection::Right) {
      tgt_val->nmr.w_max = -200000;
      tgt_val->nmr.alpha = -(2 * sp.v * sp.v / (sp.rad * sp.rad * sp.ang / 3));
    }
  } else {
    tgt_val->nmr.ang = 0;
    tgt_val->nmr.w_max = 0;
    tgt_val->nmr.w_end = 0;
    tgt_val->nmr.alpha = 0;
  }
  tgt_val->ego_in.sla_param.counter = 1;
  tgt_val->ego_in.sla_param.limit_time_count =
      (int)(tgt_val->nmr.sla_time * 2 / dt);
  tgt_val->nmr.timstamp++;

  tgt_val->nmr.ang = (td == TurnDirection::Left) ? sp.ref_ang : -sp.ref_ang;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
  vTaskDelay(1.0 / portTICK_RATE_MS);
  if (search_mode) {
    vTaskDelay(1.0 / portTICK_RATE_MS);
    adachi->update();
  }
  bool find_in = false;
  bool find_out = false;
  int count = 0;
  int count_save = 0;
  int limit_count = tgt_val->ego_in.sla_param.limit_time_count;
  while (1) {
    count++;
    if (tgt_val->nmr.motion_mode == RUN_MODE2::SLALOM_RUN2) {
      if (tgt_val->ego_in.pivot_state == 3 &&
          std::abs(tgt_val->ego_in.ang * 180 / m_PI) > 10) {
        tgt_val->ego_in.w = 0;
        break;
      }
      if (std::abs(tgt_val->ego_in.img_ang) + 0.001 >= std::abs(sp.ang)) {
        tgt_val->ego_in.w = 0;
        break;
      }
    } else {
      if (count > limit_count / 2) {
        if (sp.type == TurnType::Normal && !find_in && !find_out) {
          if (td == TurnDirection::Right) {
            if (10 < sensing_result->ego.left45_dist &&
                sensing_result->ego.left45_dist <
                    param->normal_sla_l_wall_off_th_in) {
              find_in = true;
            }
          } else if (td == TurnDirection::Left) {
            if (10 < sensing_result->ego.right45_dist &&
                sensing_result->ego.right45_dist <
                    param->normal_sla_r_wall_off_th_in) {
              find_in = true;
            }
          }
        } else if (sp.type == TurnType::Normal && find_in && !find_out) {
          if (td == TurnDirection::Right) {
            if (sensing_result->ego.left45_dist >
                    param->normal_sla_l_wall_off_th_out &&
                sensing_result->ego.left45_dist < 180) {
              find_out = true;
              count_save = count;
            }
          } else {
            if (sensing_result->ego.right45_dist >
                    param->normal_sla_r_wall_off_th_out &&
                sensing_result->ego.right45_dist < 180) {
              find_out = true;
              count_save = count;
            }
          }
        }
      }
      if (tgt_val->ego_in.sla_param.counter >= (sp.time * 2 / dt)) {
        tgt_val->ego_in.w = 0;
        tgt_val->ego_in.img_ang =
            (td == TurnDirection::Left) ? sp.ref_ang : -sp.ref_ang;
        break;
      }
    }
    if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
      return MotionResult::ERROR;
    }
    vTaskDelay(1.0 / portTICK_RATE_MS);
  }
  if (sp.type == TurnType::Normal && find_in && find_out && count_save > 0
      // &&     !(offset_l || offset_r)
  ) {
    if (td == TurnDirection::Right) {
      if (ABS(count_save - param->normal_sla_l_wall_off_ref_cnt) >
          param->normal_sla_l_wall_off_margin) {
        if ((count_save > param->normal_sla_l_wall_off_ref_cnt)) {
          ps_back.dist += param->normal_sla_l_wall_off_dist;
        } else {
          ps_back.dist -= param->normal_sla_l_wall_off_dist;
        }
        if (ps_back.dist < 1) {
          ps_back.dist = 1;
        }
      }
    } else {
      if (ABS(count_save - param->normal_sla_r_wall_off_ref_cnt) >
          param->normal_sla_r_wall_off_margin) {
        if ((count_save > param->normal_sla_r_wall_off_ref_cnt)) {
          ps_back.dist += param->normal_sla_r_wall_off_dist;
        } else {
          ps_back.dist -= param->normal_sla_r_wall_off_dist;
        }
        if (ps_back.dist < 1) {
          ps_back.dist = 1;
        }
      }
    }
  }
  ps_back.v_max = MAX(sp.v, next_motion.v_max);
  if (sp.type == TurnType::Normal) {
    ps_back.v_max = sp.v;
  }
  ps_back.v_end = next_motion.v_end;
  ps_back.accl = next_motion.accl;

  ps_back.accl =
      MAX(ABS((ps_back.v_end + tgt_val->ego_in.v) *
                  (ps_back.v_end - tgt_val->ego_in.v) / (2.0 * ps_back.dist) +
              500),
          next_motion.accl);

  ps_back.decel = next_motion.decel;
  ps_back.motion_type = MotionType::SLA_BACK_STR;
  ps_back.sct = SensorCtrlType::Straight;

  if (sp.type == TurnType::Dia45 || sp.type == TurnType::Dia135 ||
      sp.type == TurnType::Dia90) {
    ps_back.sct = SensorCtrlType::NONE;
  }
  // ps_back.sct = SensorCtrlType::NONE;
  ps_back.wall_off_req = WallOffReq::NONE;
  //  : SensorCtrlType::Dia;
  MotionResult res_b = MotionResult::NONE;

  if (!next_motion.is_turn) {
    next_motion.carry_over_dist = ps_back.dist;
    return MotionResult::NONE;
  }
  if (ps_back.dist > 0) {
    res_b = go_straight(ps_back);
    if (res_b != MotionResult::NONE) {
      return MotionResult::ERROR;
    }
  }
  return MotionResult::NONE;
}
void IRAM_ATTR MotionPlanning::normal_slalom(param_normal_slalom_t &p,
                                             param_straight_t &p_str) {}

void IRAM_ATTR MotionPlanning::reset_tgt_data() {
  tgt_val->tgt_in.v_max = 0;
  tgt_val->tgt_in.end_v = 0;
  tgt_val->tgt_in.accl = 0;
  tgt_val->tgt_in.decel = 0;
  tgt_val->tgt_in.w_max = 0;
  tgt_val->tgt_in.end_w = 0;
  tgt_val->tgt_in.alpha = 0;
  tgt_val->tgt_in.tgt_dist = 0;
  tgt_val->tgt_in.tgt_angle = 0;

  tgt_val->motion_mode = 0;

  tgt_val->tgt_in.accl_param.limit = 5500;
  tgt_val->tgt_in.accl_param.n = 4;
  tgt_val->global_pos.ang = 0;
  tgt_val->global_pos.img_ang = 0;
  tgt_val->global_pos.dist = 0;
  tgt_val->global_pos.img_dist = 0;
  tgt_val->nmr.tgt_reset_req = true;
  pt->last_tgt_angle = 0;
  // TODO
  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
  vTaskDelay(1.0 / portTICK_RATE_MS);
  tgt_val->nmr.tgt_reset_req = false;
}

void IRAM_ATTR MotionPlanning::reset_ego_data() {
  pt->reset_kf_state(false);
  tgt_val->ego_in.accl = 0;
  tgt_val->ego_in.alpha = 0;
  tgt_val->ego_in.ang = 0;
  tgt_val->ego_in.dist = 0;
  tgt_val->ego_in.pivot_state = 0;
  tgt_val->ego_in.sla_param.base_alpha = 0;
  tgt_val->ego_in.sla_param.base_time = 0;
  tgt_val->ego_in.sla_param.counter = 0;
  tgt_val->ego_in.sla_param.limit_time_count = 0;
  tgt_val->ego_in.sla_param.pow_n = 0;
  tgt_val->ego_in.sla_param.state = 0;
  tgt_val->ego_in.state = 0;
  tgt_val->ego_in.img_ang = 0;
  tgt_val->ego_in.img_dist = 0;

  tgt_val->ego_in.v = 0;
  tgt_val->ego_in.w = 0;

  tgt_val->motion_mode = 0;

  tgt_val->nmr.motion_mode = RUN_MODE2::NONE_MODE;
  tgt_val->nmr.motion_type = MotionType::NONE;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.ego_reset_req = true;
  // 一度初期化
  tgt_val->motion_type = MotionType::NONE;
  tgt_val->nmr.timstamp++;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
  vTaskDelay(1.0 / portTICK_RATE_MS);
  tgt_val->nmr.ego_reset_req = false;

  req_error_reset();
  vTaskDelay(1.0 / portTICK_RATE_MS);
}

void IRAM_ATTR MotionPlanning::reset_gyro_ref() {
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  float gyro_raw_data_sum = 0;
  float gyro2_raw_data_sum = 0;
  float accel_x_raw_data_sum = 0;
  float accel_y_raw_data_sum = 0;

  for (int i = 0; i < RESET_GYRO_LOOP_CNT; i++) {
    gyro_raw_data_sum += sensing_result->gyro.raw;
    gyro2_raw_data_sum += sensing_result->gyro2.raw;
    accel_x_raw_data_sum += sensing_result->accel_x.raw;
    accel_y_raw_data_sum += sensing_result->accel_y.raw;
    vTaskDelay(xDelay); //他モジュールの起動待ち
  }
  tgt_val->gyro_zero_p_offset = gyro_raw_data_sum / RESET_GYRO_LOOP_CNT;
  tgt_val->gyro2_zero_p_offset = gyro2_raw_data_sum / RESET_GYRO_LOOP_CNT;
  tgt_val->accel_x_zero_p_offset = accel_x_raw_data_sum / RESET_GYRO_LOOP_CNT;
  tgt_val->accel_y_zero_p_offset = accel_y_raw_data_sum / RESET_GYRO_LOOP_CNT;

  printf("gyro initial data : %f\n", pt->get_sensing_entity()->ego.battery_raw);

  printf("kf_batt:\n");
  pt->kf_batt.print_state();

  printf("kf_v:\n");
  pt->kf_v.print_state();

  printf("kf_enc_r:\n");
  pt->kf_v_r.print_state();

  printf("kf_enc_l:\n");
  pt->kf_v_l.print_state();

  printf("kf_dist:\n");
  pt->kf_dist.print_state();

  printf("kf_w:\n");
  pt->kf_w.print_state();

  printf("kf_ang:\n");
  pt->kf_ang.print_state();

  pt->reset_kf_state(true);
}
void IRAM_ATTR MotionPlanning::reset_gyro_ref_with_check() {
  // return;
  ui->motion_check();
  reset_gyro_ref();
  pt->reset_kf_state(true);
  pt->reset_pos(-param->offset_start_dist, 0, 0);
}

void IRAM_ATTR MotionPlanning::coin() { ui->coin(120); }

MotionResult IRAM_ATTR MotionPlanning::front_ctrl(bool limit) {
  req_error_reset();
  vTaskDelay(2 / portTICK_PERIOD_MS);
  tgt_val->nmr.v_max = 0;
  tgt_val->nmr.v_end = 0;
  tgt_val->nmr.accl = 0;
  tgt_val->nmr.decel = 0;
  tgt_val->nmr.dist = 0;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.sla_alpha = 0;
  tgt_val->nmr.sla_time = 0;
  tgt_val->nmr.sla_pow_n = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::KEEP;
  tgt_val->nmr.motion_type = MotionType::FRONT_CTRL;
  tgt_val->nmr.dia_mode = false;
  tgt_val->ego_in.sla_param.counter = 1;
  tgt_val->nmr.timstamp++;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);

  unsigned int cnt = 0;
  unsigned int max_cnt = 0;
  while (1) {
    vTaskDelay(1.0 / portTICK_RATE_MS);
    if (ui->button_state_hold()) {
      break;
    }
    // if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
    //   return MotionResult::ERROR;
    // }
    if (std::abs(sensing_result->ego.front_dist -
                 param->sen_ref_p.search_exist.front_ctrl) <
            param->sen_ref_p.search_exist.kireme_l &&
        std::abs((sensing_result->ego.right90_dist -
                  sensing_result->ego.left90_dist) /
                     2 -
                 param->sen_ref_p.search_exist.kireme_r) <
            param->sen_ref_p.search_exist.offset_r) {
      cnt++;
    } else {
      cnt = 0;
    }
    // printf("%f %f %f\n", sensing_result->ego.left90_dist,
    //        sensing_result->ego.front_dist,
    //        sensing_result->ego.right90_dist);
    max_cnt++;
    if (!limit) {
      if (cnt > param->sen_ref_p.search_exist.front_ctrl_th)
        break;
      if (max_cnt > 250)
        break;
    }
  }
  return MotionResult::NONE;
}

void IRAM_ATTR MotionPlanning::keep() {
  tgt_val->nmr.v_max = 0;
  tgt_val->nmr.v_end = 0;
  tgt_val->nmr.accl = 0;
  tgt_val->nmr.decel = 0;
  tgt_val->nmr.dist = 0;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.sla_alpha = 0;
  tgt_val->nmr.sla_time = 0;
  tgt_val->nmr.sla_pow_n = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::KEEP;
  tgt_val->nmr.motion_type = MotionType::NONE;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = false;
  tgt_val->nmr.sct = SensorCtrlType::NONE;
  tgt_val->nmr.timstamp++;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);

  while (1) {
    vTaskDelay(1.0 / portTICK_RATE_MS);
    if (ui->button_state_hold()) {
      break;
    }
  }
}
void IRAM_ATTR MotionPlanning::exec_path_running(param_set_t &p_set) {
  ego.x = ego.y = ego.ang = 0;
  ego.dir = Direction::North;
  dia = false;
  bool fast_mode = false;
  bool start_turn = false;
  bool fast_turn_mode = false;
  float carry_over_dist = 0;

  // default straight parma
  ps.v_max = p_set.str_map[StraightType::FastRun].v_max;
  ps.v_end = p_set.str_map[StraightType::FastRun].v_max;
  ps.accl = p_set.str_map[StraightType::FastRun].accl;
  ps.decel = p_set.str_map[StraightType::FastRun].decel;
  ps.motion_type = MotionType::STRAIGHT;
  ps.sct = SensorCtrlType::Straight;
  ps.wall_ctrl_mode = WallCtrlMode::NONE;
  ps.search_str_wide_ctrl_l = ps.search_str_wide_ctrl_r = false;

  reset_gyro_ref_with_check();

  if (p_set.suction) {
    pt->suction_enable(p_set.suction_duty, p_set.suction_duty_low);
    vTaskDelay(800.0 / portTICK_PERIOD_MS);
  }
  if (param->fast_log_enable > 0) {
    tgt_val->global_pos.ang = 0;
    tgt_val->global_pos.dist = 0;
    lt->start_slalom_log();
    tgt_val->global_pos.ang = 0;
    tgt_val->global_pos.dist = 0;
  }
  // reset();
  reset_tgt_data();
  reset_ego_data();
  pt->motor_enable();
  auto path_size = pc->path_s.size();
  for (int i = 0; i < path_size; i++) {
    float dist = 0.5 * pc->path_s[i] - 1;
    auto turn_dir = tc.get_turn_dir(pc->path_t[i]);
    auto turn_type = tc.get_turn_type(pc->path_t[i], dia);
    start_turn = false;
    fast_turn_mode = false;
    if (dist > 0) {
      fast_mode = true;
    }
    if (dist > 0 || i == 0) {
      auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
      ps.v_max = p_set.str_map[st].v_max;
      ps.v_end =
          fast_mode ? p_set.map[turn_type].v : p_set.map_slow[turn_type].v;
      bool exist_next_idx = (i + 1) < path_size; //絶対true
      if (exist_next_idx) {
        float next_dist = 0.5 * pc->path_s[i + 1] - 1;
        auto next_turn_type = tc.get_turn_type(pc->path_t[i + 1]);
        if (next_dist > 0 && (next_turn_type == TurnType::Orval ||
                              next_turn_type == TurnType::Large)) {
          ps.v_end = p_set.map_fast[turn_type].v;
          fast_turn_mode = true;
        }
      }
      ps.accl = p_set.str_map[st].accl;
      ps.decel = p_set.str_map[st].decel;
      ps.dia_mode = dia;
      ps.search_str_wide_ctrl_l = ps.search_str_wide_ctrl_r = false;
      ps.wall_ctrl_mode = WallCtrlMode::LEFT_ONLY;

      ps.dist = !dia ? (dist * param->cell) : (dist * param->cell * ROOT2);
      if (i == 0) {
        tgt_val->global_pos.ang = 0;
        if (dist == 0) { // 初手ターンの場合は距離合成して加速区間を増やす
          if (fast_mode) {
            ps.dist = (turn_dir == TurnDirection::Left)
                          ? p_set.map[turn_type].front.left
                          : p_set.map[turn_type].front.right;
          } else {
            ps.dist = (turn_dir == TurnDirection::Left)
                          ? p_set.map_slow[turn_type].front.left
                          : p_set.map_slow[turn_type].front.right;
          }
          start_turn = true;
        }
        ps.dist += param->offset_start_dist; // 初期加速距離を加算
        auto tmp_v2 = 2 * ps.accl * ps.dist;
        if (ps.v_end * ps.v_end > tmp_v2) {
          ps.accl = (ps.v_end * ps.v_end) / (2 * ps.dist) + 1000;
          ps.decel = -ps.accl;
        }
      }
      if (turn_type == TurnType::Finish) {
        ps.dist -= param->cell / 2;
        if (p_set.suction) {
          ps.v_end = 2500;
        } else {
          ps.v_end = p_set.map[TurnType::Large].v;
        }
      }
      ps.motion_type = MotionType::STRAIGHT;
      ps.sct = !dia ? SensorCtrlType::Straight : SensorCtrlType::Dia;
      ps.dist += carry_over_dist;

      // if (i == 0 && start_turn) {
      //   if (turn_dir == TurnDirection::Left) {
      //     ps.dist = param->offset_start_dist +
      //     p_set.map[turn_type].front.left;
      //   } else {
      //     ps.dist = param->offset_start_dist +
      //     p_set.map[turn_type].front.right;
      //   }
      // }

      // printf("%f %f %f %f\n", ps.dist, ps.v_max, ps.v_end, ps.accl);
      ps.dist -= param->long_run_offset_dist;
      auto res = go_straight(ps);
      carry_over_dist = 0;
      if (res == MotionResult::ERROR) {
        break;
      }
      if (turn_type == TurnType::Finish) {
        break;
      }
    }

    if (!((turn_type == TurnType::None) || (turn_type == TurnType::Finish))) {
      auto st = !dia ? StraightType::FastRun : StraightType::FastRunDia;
      bool exist_next_idx = (i + 1) < path_size; //絶対true
      float dist3 = 0;
      float dist4 = 0;
      if (exist_next_idx) {
        dist3 = 0.5 * pc->path_s[i + 1] * param->cell;
        dist4 = 0.5 * pc->path_s[i + 1] - 1;
      }
      // スラロームの後距離の目標速度を指定
      // nm.v_max =
      //     fast_mode ? p_set.map[turn_type].v : p_set.map_slow[turn_type].v;
      // nm.v_end =
      //     fast_mode ? p_set.map[turn_type].v : p_set.map_slow[turn_type].v;

      // nm.v_max = p_set.map[turn_type].v;
      nm.v_max = MAX(p_set.map[turn_type].v, p_set.str_map[st].v_max);
      nm.v_end = p_set.map[turn_type].v;

      nm.accl = p_set.str_map[st].accl;
      nm.decel = p_set.str_map[st].decel;
      nm.is_turn = false;
      nm.skip_wall_off = start_turn;

      if (exist_next_idx && !(dist3 > 0 && dist4 > 0)) {
        //連続スラロームのとき、次のスラロームの速度になるように加速
        auto next_turn_type = tc.get_turn_type(pc->path_t[i + 1]);
        nm.is_turn = true;
        nm.v_max = MAX(p_set.map[next_turn_type].v, p_set.str_map[st].v_max);
        nm.v_end = p_set.map[next_turn_type].v;
      }

      if (fast_turn_mode) {
        nm.v_max = MAX(p_set.map_fast[turn_type].v, p_set.str_map[st].v_max);
        nm.v_end = p_set.map_fast[turn_type].v;
        auto res = slalom(p_set.map_fast[turn_type], turn_dir, nm, dia);
        if (res == MotionResult::ERROR) {
          break;
        }
      } else {
        auto res =
            slalom(fast_mode ? p_set.map[turn_type] : p_set.map_slow[turn_type],
                   turn_dir, nm, dia);
        if (res == MotionResult::ERROR) {
          break;
        }
      }
      if (!nm.is_turn) {
        carry_over_dist = nm.carry_over_dist;
      }
      fast_mode = true;
      ego.dir = tc.get_next_dir(ego.dir, turn_type, turn_dir);
      // ego.ang = trj_ele.ang;
      dia =
          (ego.dir == Direction::NorthEast || ego.dir == Direction::NorthWest ||
           ego.dir == Direction::SouthEast || ego.dir == Direction::SouthWest);
    }
  }
  float dist = sensing_result->ego.front_dist - param->cell2 / 2;
  if (dist < 0) {
    dist = 1;
  }
  ps.v_max = 1500;
  ps.v_end = 20;
  ps.dist = dist;
  ps.accl = p_set.str_map[StraightType::FastRun].accl;
  ps.decel = p_set.str_map[StraightType::FastRun].decel;
  ps.sct = !dia ? SensorCtrlType::Straight : SensorCtrlType::Dia;
  go_straight(ps);
  reset_tgt_data();
  reset_ego_data();
  vTaskDelay(250.0 / portTICK_RATE_MS);
  pt->motor_disable();
  pt->suction_disable();

  // pt->motor_enable();
  // front_ctrl(false);
  // reset_tgt_data();
  // reset_ego_data();
  // vTaskDelay(25.0 / portTICK_RATE_MS);
  // pt->motor_disable(false);

  lt->stop_slalom_log();
  lt->save(slalom_log_file);
  coin();
  while (1) {
    if (ui->button_state_hold())
      break;
    vTaskDelay(10.0 / portTICK_RATE_MS);
  }
  lt->dump_log(slalom_log_file);
}

MotionResult IRAM_ATTR MotionPlanning::search_front_ctrl(param_straight_t &p) {
  return MotionResult::NONE;
}

MotionResult IRAM_ATTR MotionPlanning::wall_off(param_straight_t &p, bool dia) {
  return MotionResult::NONE;
}

void IRAM_ATTR MotionPlanning::wall_off(TurnDirection td,
                                        param_straight_t &ps_front) {
  const auto se = get_sensing_entity();
  bool exist = false;
  const auto r45_start_dist = se->ego.right45_dist;
  const auto l45_start_dist = se->ego.left45_dist;
  if (td == TurnDirection::Right) {
    exist =
        se->ego.right45_dist < param->wall_off_dist.wall_off_exist_wall_th_r;
  } else {
    exist = se->ego.left45_dist < param->wall_off_dist.wall_off_exist_wall_th_l;
  }
  tgt_val->nmr.v_max = ps_front.v_max;
  tgt_val->nmr.v_end = ps_front.v_end;
  tgt_val->nmr.accl = ps_front.accl;
  tgt_val->nmr.decel = ps_front.decel;
  tgt_val->nmr.dist = param->wall_off_wait_dist;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::ST_RUN;
  tgt_val->motion_type = MotionType::WALL_OFF;
  tgt_val->nmr.motion_type = MotionType::WALL_OFF;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = ps_front.dia_mode;
  tgt_val->nmr.sct = SensorCtrlType::Straight;
  tgt_val->nmr.timstamp++;
  float tmp_dist_before = tgt_val->global_pos.dist;
  float tmp_dist_after = tmp_dist_before;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);
  vTaskDelay(1.0 / portTICK_RATE_MS);
  if (td == TurnDirection::Right) {
    while (true) {
      tmp_dist_after = tgt_val->global_pos.dist;
      if (se->ego.right45_dist < param->wall_off_dist.exist_dist_r) {
        break;
      }
      if (!exist) {
        if (se->ego.right45_dist < param->wall_off_dist.exist_dist_r2) {
          break;
        }
        if (se->ego.right45_dist_diff > param->wall_off_dist.div_th_r2 &&
            se->ego.right45_dist < 100) {
          ps_front.dist += param->wall_off_dist.right_str;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      } else {
        if (se->ego.right45_dist_diff > param->wall_off_dist.div_th_r2 &&
            se->ego.right45_dist < 100) {
          ps_front.dist += param->wall_off_dist.right_str;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
        break;
      }
      //　見逃し対応：見切れた場合
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_dist.diff_check_dist)) {
        if ((se->ego.right45_dist_diff > param->wall_off_dist.diff_dist_th_r) &&
            (se->ego.right45_dist_diff > 0)) {
          ps_front.dist -= param->wall_off_pass_through_offset_r;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      }
      //　見逃し対応：通過して完全に壁がなくなった場合
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_dist.diff_check_dist)) {
        if (se->ego.right45_dist > 170) {
          ps_front.dist -= param->wall_off_pass_through_offset_r;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      }

      // 見逃し対策、壁がないときに見切れた場合
      if (!exist) {
        if (se->ego.right45_dist_diff > param->wall_off_dist.right_diff_th &&
            se->ego.right45_dist < 100) {
          ps_front.dist -= param->wall_off_pass_through_offset_r;
          ps_front.dist = MAX(ps_front.dist, 0.1);
        }
      }

      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_front_move_dist_th)) {
        if (param->wall_off_front_ctrl_min < se->ego.left90_far_dist &&
            se->ego.left90_far_dist < param->front_dist_offset4 &&
            param->wall_off_front_ctrl_min < se->ego.right90_far_dist &&
            se->ego.right90_far_dist < param->front_dist_offset4) {
          if (se->ego.front_far_dist < param->front_dist_offset3) {
            ps_front.dist -=
                (param->front_dist_offset2 - se->ego.front_far_dist);
            ps_front.dist = MAX(ps_front.dist, 0.1);
            return;
          }
        }
      }

      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_front_move_dist_th)) {
        if (std::abs(tgt_val->ego_in.dist) >= std::abs(tgt_val->nmr.dist)) {
          ps_front.dist -= param->wall_off_pass_through_offset_r;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      }
      if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
        return;
      }
      vTaskDelay(1.0 / portTICK_RATE_MS);
    }
    while (true) {
      tmp_dist_after = tgt_val->global_pos.dist;
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_front_move_dist_th)) {
        if (40 < se->ego.left90_far_dist &&
            se->ego.left90_far_dist < param->front_dist_offset4 &&
            40 < se->ego.right90_far_dist &&
            se->ego.right90_far_dist < param->front_dist_offset4) {
          if (se->ego.front_far_dist < param->front_dist_offset3) {
            ps_front.dist -=
                (param->front_dist_offset2 - se->ego.front_far_dist);
            ps_front.dist = MAX(ps_front.dist, 0.1);
            return;
          }
        }
      }
      if (exist) {
        if (se->ego.right45_dist > param->wall_off_dist.noexist_th_r &&
            se->ego.right45_dist_diff > 0) {
          ps_front.dist += param->wall_off_dist.right_str_exist;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      } else {
        // 壁が見切れかけ始めたら
        if (se->ego.right45_dist_diff > param->wall_off_dist.div_th_r &&
            se->ego.right45_dist < 100) {
          ps_front.dist += param->wall_off_dist.right_str;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
        if (se->ego.right45_dist > param->wall_off_dist.noexist_th_r2) {
          ps_front.dist += param->wall_off_dist.right_str;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      }

      if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
        return;
      }
      vTaskDelay(1.0 / portTICK_RATE_MS);
    }
  } else {
    while (true) {
      tmp_dist_after = tgt_val->global_pos.dist;
      if (se->ego.left45_dist < param->wall_off_dist.exist_dist_l) {
        break;
      }
      if (!exist) {
        if (se->ego.left45_dist < param->wall_off_dist.exist_dist_l2) {
          break;
        }
        if (se->ego.left45_dist_diff > param->wall_off_dist.div_th_l3 &&
            se->ego.left45_dist < 100) {
          ps_front.dist += param->wall_off_dist.left_str;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      } else {
        if (se->ego.left45_dist_diff > param->wall_off_dist.div_th_l2 &&
            se->ego.left45_dist < 100) {
          ps_front.dist += param->wall_off_dist.left_str;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
        break;
      }
      //　見逃し対応：見切れた場合
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_dist.diff_check_dist)) {
        if ((se->ego.left45_dist_diff > param->wall_off_dist.diff_dist_th_l) &&
            (se->ego.left45_dist_diff > 0)) {
          ps_front.dist -= param->wall_off_pass_through_offset_l;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      }
      //　見逃し対応：通過して完全に壁がなくなった場合
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_dist.diff_check_dist)) {
        if (se->ego.left45_dist > 170) {
          ps_front.dist -= param->wall_off_pass_through_offset_l;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      }

      // 見逃し対策、壁がないときに見切れた場合
      if (!exist) {
        if (se->ego.left45_dist_diff > param->wall_off_dist.left_diff_th &&
            se->ego.left45_dist < 100) {
          ps_front.dist -= param->wall_off_pass_through_offset_l;
          ps_front.dist = MAX(ps_front.dist, 0.1);
        }
      }
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_front_move_dist_th)) {
        if (40 < se->ego.left90_far_dist &&
            se->ego.left90_far_dist < param->front_dist_offset4 &&
            40 < se->ego.right90_far_dist &&
            se->ego.right90_far_dist < param->front_dist_offset4) {
          if (se->ego.front_far_dist < param->front_dist_offset3) {
            ps_front.dist -=
                (param->front_dist_offset2 - se->ego.front_far_dist);
            ps_front.dist = MAX(ps_front.dist, 0.1);
            return;
          }
        }
      }
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_front_move_dist_th)) {
        if (std::abs(tgt_val->ego_in.dist) >= std::abs(tgt_val->nmr.dist)) {
          ps_front.dist -= param->wall_off_pass_through_offset_r;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      }
      if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
        return;
      }
      vTaskDelay(1.0 / portTICK_RATE_MS);
    }
    while (true) {
      tmp_dist_after = tgt_val->global_pos.dist;
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_front_move_dist_th)) {
        if (40 < se->ego.left90_far_dist &&
            se->ego.left90_far_dist < param->front_dist_offset4 &&
            40 < se->ego.right90_far_dist &&
            se->ego.right90_far_dist < param->front_dist_offset4) {
          if (se->ego.front_far_dist < param->front_dist_offset3) {
            ps_front.dist -=
                (param->front_dist_offset2 - se->ego.front_far_dist);
            ps_front.dist = MAX(ps_front.dist, 0.1);
            return;
          }
        }
      }
      if (exist) {
        if (se->ego.left45_dist > param->wall_off_dist.noexist_th_l &&
            se->ego.left45_dist_diff > 0) {
          ps_front.dist += param->wall_off_dist.left_str_exist;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      } else {
        // 壁が見切れかけ始めたら
        if (se->ego.left45_dist_diff > param->wall_off_dist.div_th_l &&
            se->ego.left45_dist < 100) {
          ps_front.dist += param->wall_off_dist.left_str;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
        if (se->ego.left45_dist > param->wall_off_dist.noexist_th_l2) {
          ps_front.dist += param->wall_off_dist.left_str;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return;
        }
      }
      if (tgt_val->fss.error != static_cast<int>(FailSafe::NONE)) {
        return;
      }
      vTaskDelay(1.0 / portTICK_RATE_MS);
    }
  }
}

bool IRAM_ATTR MotionPlanning::wall_off_dia(TurnDirection td,
                                            param_straight_t &ps_front,
                                            bool &exist, bool &use_oppo_wall) {
  const auto se = get_sensing_entity();
  tgt_val->nmr.v_max = ps_front.v_max;
  tgt_val->nmr.v_end = ps_front.v_end;
  tgt_val->nmr.accl = ps_front.accl;
  tgt_val->nmr.decel = ps_front.decel;
  tgt_val->nmr.dist = param->wall_off_wait_dist_dia;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.motion_mode = RUN_MODE2::ST_RUN;
  tgt_val->motion_type = MotionType::WALL_OFF_DIA;
  tgt_val->nmr.motion_type = MotionType::WALL_OFF_DIA;
  tgt_val->nmr.motion_dir = MotionDirection::RIGHT;
  tgt_val->nmr.dia_mode = ps_front.dia_mode;
  tgt_val->nmr.sct = SensorCtrlType::Dia;
  tgt_val->nmr.timstamp++;

  float tmp_dist_before = tgt_val->global_pos.dist;
  float tmp_dist_after = tmp_dist_before;

  float init_r45_dist = sensing_result->ego.right45_dist;
  float init_l45_dist = sensing_result->ego.left45_dist;

  exist = false;
  if (td == TurnDirection::Right) {
    exist = se->ego.right45_dist <
            param->wall_off_dist.wall_off_exist_dia_wall_th_r;
  } else {
    exist =
        se->ego.left45_dist < param->wall_off_dist.wall_off_exist_dia_wall_th_l;
  }
  use_oppo_wall = false;

  if (td == TurnDirection::Right) {
    while (true) {
      // 壁切れ開始
      if (exist) {
        if (se->ego.right45_dist < param->wall_off_dist.exist_dia_th_r) {
          break;
        }
        break;
      } else {
        if (se->ego.right45_dist < param->wall_off_dist.exist_dia_th_r2) {
          break;
        }
      }
      // 反対側の壁あり
      tmp_dist_after = tgt_val->global_pos.dist;
      // 見切れた場合
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_dist.diff_check_dist_dia_2)) {
        if ((se->ego.right45_dist - init_r45_dist) >
            param->wall_off_pass_dist) {
          ps_front.dist += param->wall_off_dist.right_dia_oppo;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return false;
        }
      }
      // 反対側の壁あり
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_dist.diff_check_dist_dia)) {
        if (se->ego.left45_dist < param->dia_turn_th_l) {
          // ps_front.dist += param->wall_off_dist.right_dia_oppo;
          float diff = (se->ego.left45_dist - param->dia_turn_ref_l);
          diff = std::clamp(diff, -param->dia_turn_max_dist_l,
                            param->dia_turn_max_dist_l);
          ps_front.dist = ps_front.dist + diff;
          use_oppo_wall = true;
          return false;
        }
      }
      vTaskDelay(1.0 / portTICK_RATE_MS);
    }
    while (true) {
      // 壁切れ終了
      if (exist) {
        if (se->ego.right45_dist > param->wall_off_dist.noexist_dia_th_r &&
            se->ego.right45_dist_diff > 0) {
          ps_front.dist += param->wall_off_dist.right_dia;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return true;
        }
        if (se->ego.right45_dist_diff > param->wall_off_dist.div_th_dia_r &&
            se->ego.right45_dist < 100) {
          ps_front.dist += param->wall_off_dist.right_dia;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return true;
        }
      } else {
        if (se->ego.right45_dist > param->wall_off_dist.noexist_dia_th_r2) {
          ps_front.dist += param->wall_off_dist.right_dia2;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return true;
        }
      }
      vTaskDelay(1.0 / portTICK_RATE_MS);
    }
  } else {
    while (true) {
      // 壁切れ開始
      if (exist) {
        if (se->ego.left45_dist < param->wall_off_dist.exist_dia_th_l) {
          break;
        }
        break;
      } else {
        if (se->ego.left45_dist < param->wall_off_dist.exist_dia_th_l2) {
          break;
        }
      }
      // 反対側の壁あり
      tmp_dist_after = tgt_val->global_pos.dist;
      //  見切れた場合
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_dist.diff_check_dist_dia_2)) {
        if ((se->ego.left45_dist - init_l45_dist) > param->wall_off_pass_dist) {
          ps_front.dist += param->wall_off_dist.left_dia_oppo;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return false;
        }
      }
      // 反対側の壁あり
      if (std::abs(tmp_dist_after - tmp_dist_before) >=
          std::abs(param->wall_off_dist.diff_check_dist_dia)) {
        if (se->ego.right45_dist < param->dia_turn_th_r) {
          // ps_front.dist += param->wall_off_dist.left_dia_oppo;
          float diff = (se->ego.right45_dist - param->dia_turn_ref_r);
          diff = std::clamp(diff, -param->dia_turn_max_dist_r,
                            param->dia_turn_max_dist_r);
          ps_front.dist = ps_front.dist + diff;
          use_oppo_wall = true;
          return false;
        }
      }
      vTaskDelay(1.0 / portTICK_RATE_MS);
    }
    while (true) {
      // 壁切れ終了
      if (exist) {
        if (se->ego.left45_dist > param->wall_off_dist.noexist_dia_th_l &&
            se->ego.left45_dist_diff > 0) {
          ps_front.dist += param->wall_off_dist.left_dia;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return true;
        }
        if (se->ego.left45_dist_diff > param->wall_off_dist.div_th_dia_l &&
            se->ego.left45_dist < 100) {
          ps_front.dist += param->wall_off_dist.left_dia;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return true;
        }
      } else {
        if (se->ego.left45_dist > param->wall_off_dist.noexist_dia_th_l2) {
          ps_front.dist += param->wall_off_dist.left_dia2;
          ps_front.dist = MAX(ps_front.dist, 0.1);
          return true;
        }
      }
      vTaskDelay(1.0 / portTICK_RATE_MS);
    }
  }
}

void IRAM_ATTR MotionPlanning::calc_dia135_offset(param_straight_t &front,
                                                  param_straight_t &back,
                                                  TurnDirection dir,
                                                  bool exec_wall_off) {
  const auto se = get_sensing_entity();
  float offset_l = 0;
  float offset_r = 0;
  float offset = 0;
  bool valid_l = false;
  bool valid_r = false;
  if (dir == TurnDirection::Left) {
    if (exec_wall_off) {
      if (1 < se->sen.l45.sensor_dist &&
          se->sen.l45.sensor_dist < param->dia_turn_offset_calc_th) {
        offset_l = se->sen.l45.sensor_dist - param->sen_ref_p.normal.ref.left45;
        valid_l = true;
      }
    } else {
      if (1 < se->ego.left45_dist &&
          se->ego.left45_dist < param->dia_turn_offset_calc_th) {
        offset_l = se->ego.left45_dist - param->sen_ref_p.normal.ref.left45;
        valid_l = true;
      }
    }
    if (1 < se->ego.right45_dist &&
        se->ego.right45_dist < param->dia_turn_offset_calc_th) {
      offset_r = param->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
      valid_r = true;
    }
  } else {
    if (exec_wall_off) {
      if (1 < se->sen.r45.sensor_dist &&
          se->sen.r45.sensor_dist < param->dia_turn_offset_calc_th) {
        offset_r =
            se->sen.r45.sensor_dist - param->sen_ref_p.normal.ref.right45;
        valid_r = true;
      }
    } else {
      if (1 < se->ego.right45_dist &&
          se->ego.right45_dist < param->dia_turn_offset_calc_th) {
        offset_r = se->ego.right45_dist - param->sen_ref_p.normal.ref.right45;
        valid_r = true;
      }
    }
    if (1 < se->ego.left45_dist &&
        se->ego.left45_dist < param->dia_turn_offset_calc_th) {
      offset_l = param->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
      valid_l = true;
    }
  }
  if (valid_l && valid_r) {
    offset = (std::abs(offset_l) < std::abs(offset_r)) ? offset_l : offset_r;
  } else if (valid_l) {
    offset = offset_l;
  } else if (valid_r) {
    offset = offset_r;
  }
  if (param->dia135_offset_enable) {
    front.dist += offset;
    back.dist += offset * ROOT2;
  }
}

void IRAM_ATTR MotionPlanning::calc_dia45_offset(param_straight_t &front,
                                                 param_straight_t &back,
                                                 TurnDirection dir,
                                                 bool exec_wall_off) {
  const auto se = get_sensing_entity();
  float offset_l = 0;
  float offset_r = 0;
  float offset = 0;
  bool valid_l = false;
  bool valid_r = false;
  if (dir == TurnDirection::Left) {
    if (exec_wall_off) {
      if (1 < se->sen.l45.sensor_dist &&
          se->sen.l45.sensor_dist < param->dia_turn_offset_calc_th) {
        offset_l = se->sen.l45.sensor_dist - param->sen_ref_p.normal.ref.left45;
        valid_l = true;
      }
    } else {
      if (1 < se->ego.left45_dist &&
          se->ego.left45_dist < param->dia_turn_offset_calc_th) {
        offset_l = se->ego.left45_dist - param->sen_ref_p.normal.ref.left45;
        valid_l = true;
      }
    }
    if (1 < se->ego.right45_dist &&
        se->ego.right45_dist < param->dia_turn_offset_calc_th) {
      offset_r = param->sen_ref_p.normal.ref.right45 - se->ego.right45_dist;
      valid_r = true;
    }
  } else {
    if (exec_wall_off) {
      if (1 < se->sen.r45.sensor_dist &&
          se->sen.r45.sensor_dist < param->dia_turn_offset_calc_th) {
        offset_r =
            se->sen.r45.sensor_dist - param->sen_ref_p.normal.ref.right45;
        valid_r = true;
      }
    } else {
      if (1 < se->ego.right45_dist &&
          se->ego.right45_dist < param->dia_turn_offset_calc_th) {
        offset_r = se->ego.right45_dist - param->sen_ref_p.normal.ref.right45;
        valid_r = true;
      }
    }
    if (1 < se->ego.left45_dist &&
        se->ego.left45_dist < param->dia_turn_offset_calc_th) {
      offset_l = param->sen_ref_p.normal.ref.left45 - se->ego.left45_dist;
      valid_l = true;
    }
  }
  if (valid_l && valid_r) {
    if (std::abs(offset_l) < std::abs(offset_r)) {
      offset = offset_l;
    } else {
      offset = offset_r;
    }
  } else if (valid_l) {
    offset = offset_l;
  } else if (valid_r) {
    offset = offset_r;
  }
  if (param->dia45_offset_enable) {
    front.dist -= offset;
    back.dist += offset * ROOT2;
  }
}
void IRAM_ATTR MotionPlanning::system_identification(MotionType mt,
                                                     float volt_l, float volt_r,
                                                     float time) {
  req_error_reset();
  vTaskDelay(2 / portTICK_PERIOD_MS);
  tgt_val->nmr.v_max = 0;
  tgt_val->nmr.v_end = 0;
  tgt_val->nmr.accl = 0;
  tgt_val->nmr.decel = 0;
  tgt_val->nmr.dist = 0;
  tgt_val->nmr.w_max = 0;
  tgt_val->nmr.w_end = 0;
  tgt_val->nmr.alpha = 0;
  tgt_val->nmr.ang = 0;
  tgt_val->nmr.sla_alpha = 0;
  tgt_val->nmr.sla_time = 0;
  tgt_val->nmr.sla_pow_n = 0;
  tgt_val->nmr.motion_type = mt;
  tgt_val->nmr.dia_mode = false;
  tgt_val->ego_in.sla_param.counter = 1;
  tgt_val->nmr.sys_id.left_v = volt_l;
  tgt_val->nmr.sys_id.right_v = volt_r;
  tgt_val->nmr.sys_id.enable = true;
  tgt_val->nmr.timstamp++;

  xTaskNotify(*th, (uint32_t)tgt_val.get(), eSetValueWithOverwrite);

  vTaskDelay(time / portTICK_RATE_MS);
  tgt_val->nmr.sys_id.enable = false;
}