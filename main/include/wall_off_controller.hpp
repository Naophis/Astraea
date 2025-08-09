#pragma once

#include "defines.hpp"
#include "enums.hpp"
#include "structs.hpp"
#include <algorithm>
#include <functional>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct WallSensorStrategy {
  std::function<bool()> wall_missing;
  std::function<bool()> exist_wall;
  std::function<bool()> find_vertical_wall;
  std::function<bool(float, float)> detect_pass_through_case1;
  std::function<bool(float, float)> detect_pass_through_case2;
  std::function<bool(float, float)> detect_distance;
  std::function<bool()> detect_missing_by_deviation;
  std::function<bool(float)> detect_wall_off;
  std::function<bool(float)> detect_wall_missing_by_deviation;
  std::function<bool()> detect_wall_off_vertical;
};

struct DiagonalWallOffStrategy {
  std::function<bool()> exist_dia_wall_start;
  std::function<bool()> exist_dia_wall_start_alt;
  std::function<bool(float, float, float)> detect_pass_through_case1;
  std::function<bool(float, float, float)> detect_pass_through_case2;
  std::function<bool()> detect_wall_off_exist;
  std::function<bool()> detect_wall_off_by_deviation;
  std::function<bool()> detect_wall_off_alt;
};

class WallOffController {
public:
  WallSensorStrategy &get_right_strategy();
  WallSensorStrategy &get_left_strategy();
  DiagonalWallOffStrategy &get_left_dia_strategy();
  DiagonalWallOffStrategy &get_right_dia_strategy();
  WallOffController();

  ~WallOffController() = default;

  // 直進時の壁切れ処理
  void execute_wall_off(TurnDirection td, param_straight_t &ps_front);

  // 斜め走行時の壁切れ処理
  bool execute_wall_off_dia(TurnDirection td, param_straight_t &ps_front,
                            bool &use_oppo_wall, bool &exist_wall);

  // 検索時の壁切れ処理（go_straight内で使用）
  MotionResult execute_search_wall_off(param_straight_t &p, bool search_mode);

  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);

  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_task_handler(TaskHandle_t &_th);
  // 斜め走行用の壁切れ距離計算
  float calculate_dia_wall_off_distance(TurnDirection td, TurnType turn_type,
                                        bool &exist_wall);

private:
  std::shared_ptr<input_param_t> param;
  std::shared_ptr<motion_tgt_val_t> tgt_val;
  std::shared_ptr<sensing_result_entity_t> sensing_result;
  TaskHandle_t *th;

  // センサーデータ取得
  std::shared_ptr<sensing_result_entity_t> get_sensing_entity();
  wall_off_hold_dist_t &get_wall_off_param();

  // 右壁の壁切れ処理
  void process_right_wall_off(param_straight_t &ps_front);

  // 左壁の壁切れ処理
  void process_left_wall_off(param_straight_t &ps_front);

  // フロントセンサーによる距離補正
  bool apply_front_sensor_correction(param_straight_t &ps_front,
                                     float tmp_dist_before,
                                     float tmp_dist_after);

  // 壁の存在判定
  bool is_wall_exist(TurnDirection td, float threshold_l, float threshold_r);

  // 右壁の斜め壁切れ処理
  bool process_right_wall_off_dia(param_straight_t &ps_front,
                                  bool &use_oppo_wall, bool &exist_wall);

  // 左壁の斜め壁切れ処理
  bool process_left_wall_off_dia(param_straight_t &ps_front,
                                 bool &use_oppo_wall, bool &exist_wall);

  std::optional<WallSensorStrategy> right_strategy;
  std::optional<WallSensorStrategy> left_strategy;
  std::optional<DiagonalWallOffStrategy> left_dia_strategy;
  std::optional<DiagonalWallOffStrategy> right_dia_strategy;
};