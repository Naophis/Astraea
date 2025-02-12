#ifndef PLANNING_TASK_HPP
#define PLANNING_TASK_HPP

#include "defines.hpp"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gen_code_mpc/mpc_tgt_calc.h"
#include "include/logging_task.hpp"
#include "include/maze_solver.hpp"

#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
// #include "gen_code_pid/pid_controller.h"
// #include "gen_code_pid_2dof/pid_controller_2dof.h"
#include "gen_code_simple_pid/simple_pid_controller.h"
#include <cmath>

#include "include/kalman_filter.hpp"
#include "include/kalman_filter_matrix.hpp"
#include <algorithm>
// #include "hal/mcpwm_ll.h"

class PlanningTask {
public:
  PlanningTask();
  virtual ~PlanningTask();
  void create_task(const BaseType_t xCoreID);
  void motor_enable();
  void suction_enable(float duty, float duty2);
  void motor_disable();
  void motor_disable(bool reset_req);
  void suction_disable();
  bool ready = false;

  void reset_pos(float x, float y, float ang);

  void
  set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_sensing_result);
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param_ro);
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);
  void set_error_entity(std::shared_ptr<pid_error_entity_t> &_error_entity);
  void set_logging_task(std::shared_ptr<LoggingTask> &_lt);

  void buzzer(ledc_channel_config_t &buzzer_ch,
              ledc_timer_config_t &buzzer_timer);
  static void task_entry_point(void *task_instance);
  virtual void task();

  void active_logging(FILE *_f);
  void inactive_logging();
  void dump_log();

  void set_motor_hz(unsigned long hz, int res);
  void set_suction_motor_hz(unsigned long hz, int res);

  float interp1d(vector<float> &vx, vector<float> &vy, float x,
                 bool extrapolate);

  unsigned long long global_msec_timer = 0;

  bool motor_en = false;
  bool suction_en = false;
  bool search_mode = false;
  bool mode_select = false;

  t_dynamics dynamics;
  QueueHandle_t *qh;
  QueueHandle_t motor_qh_enable;
  QueueHandle_t suction_qh_enable;

  motor_req_t motor_enable_send_msg;
  motor_req_t motor_enable_status;

  motor_req_t suction_enable_send_msg;
  motor_req_t suction_enable_status;

  motion_tgt_val_t *receive_req;

  void set_queue_handler(QueueHandle_t &_qh) { qh = &_qh; }
  TaskHandle_t *th;
  void set_task_handler(TaskHandle_t &_th) { th = &_th; }
  std::vector<float> log_table;

  t_ego mpc_next_ego;

  std::vector<float> axel_degenerate_x;
  std::vector<float> axel_degenerate_y;
  std::vector<float> sensor_deg_limitter_v;
  std::vector<float> sensor_deg_limitter_str;
  std::vector<float> sensor_deg_limitter_dia;
  std::vector<float> sensor_deg_limitter_piller;

  KalmanFilter kf_w;
  KalmanFilter kf_w2;
  KalmanFilter kf_v;
  KalmanFilter kf_v_r;
  KalmanFilter kf_v_l;
  KalmanFilter kf_dist;
  KalmanFilter kf_ang;
  KalmanFilter kf_batt;
  KalmanFilterMatrix pos;

  float suction_gain = 200;

  std::shared_ptr<motion_tgt_val_t> tgt_val;
  std::shared_ptr<pid_error_entity_t> ee;
  float ideal_v_r, ideal_v_l;
  std::shared_ptr<sensing_result_entity_t> get_sensing_entity() {
    return sensing_result;
  }
  std::shared_ptr<motion_tgt_val_t> get_tgt_entity() { return tgt_val; }

  void reset_kf_state(bool reset_battery);
  float last_tgt_angle = 0;
private:
  sensor_ctrl_keep_dist_t right_keep;
  sensor_ctrl_keep_dist_t left_keep;

  void clear_ctrl_val();
  void reset_pid_val();
  void summation_duty();
  void calc_translational_ctrl();
  void calc_angle_velocity_ctrl();
  void calc_angle_velocity_ctrl_old();
  void calc_pid_val_ang();
  void calc_pid_val_ang_vel();
  void calc_pid_val_front_ctrl();

  void calc_pid_val();

  void limitter(float &kp, float &ki, float &kb, float &kd,
                pid_param_t &limitter);

  bool judge_motor_pwm(float duty, uint8_t type);
  void change_pwm_freq(float duty_l, float duty_r);
  void copy_error_entity(pid_error_entity_t &in);

  void pid_gain_data(pid_error_t &in, pid_error_t &save);
  void pid_val_data(pid_error_t &in, pid_error_t &save);

  void set_ctrl_val(pid_error2_t &val, float error_p, float error_i,
                    float error_i2, float error_d, float val_p, float val_i,
                    float val_i2, float val_d, float zz, float z);

  void apply_duty_limitter();

  void set_gpio_state(gpio_num_t gpio_num, int state);
  float diff_old = 0;
  float diff = 0;
  float gain_cnt = 0;
  void motor_enable_main();
  void motor_disable_main();

  void suction_motor_enable_main();
  void suction_motor_disable_main();

  bool first_req = false;

  xTaskHandle handle = 0;

  void check_fail_safe();
  void update_ego_motion();
  void set_next_duty(float duty_l, float duty_r, float duty_suction);
  void init_gpio();
  void calc_tgt_duty();

  void cp_request();

  void calc_sensor_dist_diff();

  void calc_sensor_dist_all();

  float satuate_sen_duty(float duty_sen);

  float calc_sensor(float date, float a, float b);

  void calc_vel();

  void calc_filter();

  void recv_notify();

  duty_t tgt_duty;
  int fail_check_ang = 0;
  int keep_wall_off_cnt = 0;
  int buzzer_time_cnt = 0;
  int buzzer_timestamp = 0;
  int motion_req_timestamp = 0;
  int pid_req_timestamp = 0;
  int motor_req_timestamp = 0;
  int suction_req_timestamp = 0;

  void pl_req_activate();
  void cp_tgt_val();
  mpc_tgt_calcModelClass mpc_tgt_calc;

  bool log_active = false;
  // log_t log_list2[10];
  int log_list2_size = 0;
  // char line[1024];
  // FILE *f;

  std::shared_ptr<sensing_result_entity_t> sensing_result;

  std::shared_ptr<input_param_t> param_ro;
  std::shared_ptr<input_param_t> get_param() {
    return param_ro; //
  }
  std::shared_ptr<LoggingTask> lt;

  t_tgt *mpc_tgt;
  t_ego *mpc_now_ego;
  int32_t mpc_mode;
  int32_t mpc_step;
  t_ego mpc_next_ego2;

  fail_safe_t fail_safe;
  float get_feadforward_front();
  float get_feadforward_front(TurnDirection td);
  float get_feadforward_roll();
  float get_rpm_ff_val(TurnDirection td);
  float calc_sensor_pid();
  float calc_sensor_pid_dia();
  float check_sen_error(SensingControlType &type);
  float check_sen_error_dia(SensingControlType &type);

  void calc_front_ctrl_duty();
  float error_right = 0;
  float error_left = 0;
  bool check_right = false;
  bool check_left = false;

  ledc_channel_config_t buzzer_ch;
  ledc_timer_config_t buzzer_timer;

  ledc_channel_config_t suction_ch;
  ledc_timer_config_t suction_timer;

  mcpwm_config_t motor_pwm_conf;
  mcpwm_config_t suction_pwm_conf;
  float duty_c = 0;
  float duty_c2 = 0;
  float duty_roll = 0;
  float duty_roll_ang = 0;
  float duty_front_ctrl_roll = 0;
  float duty_front_ctrl_trans = 0;
  float duty_front_ctrl_roll_keep = 0;
  float duty_sen = 0;
  float sen_ang = 0;

  float last_accl;
  sen_log2_t sen_log;
  slip_t slip_param;
  std::deque<float> enc_v_q;
  std::deque<float> accl_x_q;
  float sum_v = 0;

  // PID_Controller vel_pid;
  // PID_Controller dist_pid;
  // PID_Controller sen_pid;
  // PID_Controller sen_dia_pid;
  // PID_Controller gyro_pid;
  // PID_Controller angle_pid;

  Simple_PID_Controller vel_pid;
  Simple_PID_Controller gyro_pid;
  Simple_PID_Controller ang_pid;

  // Simple_PID_Controller dist_pid;
  // Simple_PID_Controller sen_pid;
  // Simple_PID_Controller sen_dia_pid;
  // Simple_PID_Controller gyro_pid;
  // Simple_PID_Controller angle_pid;

  // PID_Controller_2dof vel_pid_2dof;
  // PID_Controller_2dof gyro_pid_2dof;
  unsigned char w_reset = 0;
};

#endif