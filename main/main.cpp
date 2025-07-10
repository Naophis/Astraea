#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/adachi.hpp"
#include "include/defines.hpp"
#include "sdkconfig.h"
#include "soc/adc_channel.h"
#include "soc/ledc_periph.h"
#include "xtensa/core-macros.h"
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_task_wdt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include "VL53L0X.h"
// #include "esp_efuse_rtc_calib.h"

#include "include/logging_task.hpp"
#include "include/main_task.hpp"
#include "include/planning_task.hpp"
#include "include/sensing_task.hpp"

#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "esp_debug_helpers.h"
#include "rom/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <eigen3/Eigen/Eigen>

// #include <Eigen/Core>
void init_uart() {
  uart_config_t uart_config;
  uart_config.baud_rate = 3 * 1000 * 1000;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_APB;
  int intr_alloc_flags = 0; // ESP_INTR_FLAG_IRAM;
  uart_driver_delete(UART_NUM_0);
  // uart_driver_delete(UART_NUM_1);
  uart_param_config(UART_NUM_0, &uart_config);
  uart_driver_install(UART_NUM_0, BUF_SIZE * 4, BUF_SIZE * 4, 0, NULL,
                      intr_alloc_flags);
  uart_set_pin(UART_NUM_0, TXD, RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void init_gpio() {
  gpio_config_t io_conf;
  // 割り込みをしない
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // 出力モード
  io_conf.mode = GPIO_MODE_OUTPUT;
  // 設定したいピンのビットマスク
  io_conf.pin_bit_mask = 0;
  // io_conf.pin_bit_mask |= 1ULL << LED_R90;
  // io_conf.pin_bit_mask |= 1ULL << LED_R45;
  // io_conf.pin_bit_mask |= 1ULL << LED_L45;
  // io_conf.pin_bit_mask |= 1ULL << LED_L90;
  io_conf.pin_bit_mask |= 1ULL << LED_EN_L45_1;
  io_conf.pin_bit_mask |= 1ULL << LED_EN_L45_2;
  io_conf.pin_bit_mask |= 1ULL << LED_EN_L90;
  io_conf.pin_bit_mask |= 1ULL << LED_EN_R45_1;
  io_conf.pin_bit_mask |= 1ULL << LED_EN_R45_2;
  io_conf.pin_bit_mask |= 1ULL << LED_EN_R90;

  io_conf.pin_bit_mask |= 1ULL << L_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << R_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << SUCTION_PWM;

  io_conf.pin_bit_mask |= 1ULL << Motor_L_PWM;
  io_conf.pin_bit_mask |= 1ULL << Motor_R_PWM;

  io_conf.pin_bit_mask |= 1ULL << L_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << R_CW_CCW1;
  io_conf.pin_bit_mask |= 1ULL << L_CW_CCW2;
  io_conf.pin_bit_mask |= 1ULL << R_CW_CCW2;

  io_conf.pin_bit_mask |= 1ULL << BUZZER;

  // 内部プルダウンしない
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // 内部プルアップしない
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // 設定をセットする
  gpio_config(&io_conf);
  gpio_set_level(SUCTION_PWM, 0);
  // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8,
}

DRAM_ATTR std::shared_ptr<input_param_t> param =
    std::make_shared<input_param_t>();
DRAM_ATTR std::shared_ptr<sensing_result_entity_t> sensing_entity =
    std::make_shared<sensing_result_entity_t>();
DRAM_ATTR std::shared_ptr<motion_tgt_val_t> tgt_val =
    std::make_shared<motion_tgt_val_t>();
DRAM_ATTR std::shared_ptr<pid_error_entity_t> error_entity =
    std::make_shared<pid_error_entity_t>();
DRAM_ATTR std::shared_ptr<PlanningTask> pt = std::make_shared<PlanningTask>();
DRAM_ATTR std::shared_ptr<LoggingTask> lt = std::make_shared<LoggingTask>();
DRAM_ATTR std::shared_ptr<MainTask> mt = std::make_shared<MainTask>();
DRAM_ATTR std::shared_ptr<SensingTask> st = std::make_shared<SensingTask>();
TaskHandle_t xTaskHandler;
// SensingTask st;

std::shared_ptr<sensing_result_entity_t> get_sensing_entity() {
  return sensing_entity;
}
mcpwm_config_t motor_pwm_conf;
void set_motor_hz(unsigned long hz, int res) {
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
  // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, Motor_R_PWM);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                      MCPWM_DUTY_MODE_0);
  // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
  //                     MCPWM_DUTY_MODE_0);

  // 2PWM input mode configuration
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, Motor_L_PWM2);
  // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, Motor_R_PWM2);
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                      MCPWM_DUTY_MODE_0);
  // mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
  //                     MCPWM_DUTY_MODE_0);
}

// LogStruct1 ls1;
// LogStruct2 ls2;
// LogStruct3 ls3;
// LogStruct4 ls4;
// LogStruct5 ls5;
// LogStruct6 ls6;
// LogStruct7 ls7;

// void IRAM_ATTR print_log() {}

extern "C" void app_main() {
  // Adachi adachi;

  init_gpio();
  init_uart();

  // LSM6DSR lsm6dsr;
  // lsm6dsr.init();
  // lsm6dsr.setup();
  printf("ESP32-S3\n"
         "Build time: %s %s\n",
         __DATE__, __TIME__);
  // print_log();

  QueueHandle_t xQueue;
  xQueue = xQueueCreate(4, sizeof(motion_tgt_val_t *));

  gpio_set_level(SUCTION_PWM, 0);
  int c = 0;

  gpio_set_level(SUCTION_PWM, 0);
  param->tire = 12.0;
  param->dt = 0.001;
  param->motor_pid.p = 0.175;
  param->motor_pid.i = 0.0175;
  param->motor_pid.d = 0.0;
  param->gyro_pid.p = 2.5;
  param->gyro_pid.i = 0.75;
  param->gyro_pid.d = 0.0;
  tgt_val->ego_in.v = 0;
  tgt_val->ego_in.w = 0;
  tgt_val->motion_type = MotionType::NONE;
  param->gyro_param.gyro_w_gain_left = 0.0002645;

  st->set_sensing_entity(sensing_entity);
  st->set_tgt_val(tgt_val);
  st->set_main_task(mt);
  st->set_input_param_entity(param);
  st->set_planning_task(pt);
  st->set_queue_handler(xQueue);
  st->set_task_handler(xTaskHandler);
  st->create_task(0);

  // sn->init();

  pt->set_sensing_entity(sensing_entity);
  pt->set_input_param_entity(param);
  pt->set_tgt_val(tgt_val);
  pt->set_error_entity(error_entity);
  pt->set_queue_handler(xQueue);
  pt->set_task_handler(xTaskHandler);
  pt->create_task(0);

  lt->set_sensing_entity(sensing_entity);
  lt->set_input_param_entity(param);
  lt->set_tgt_val(tgt_val);
  lt->set_error_entity(error_entity);
  lt->create_task(1);
  pt->set_logging_task(lt);

  mt->set_sensing_entity(sensing_entity);
  mt->set_input_param_entity(param);
  mt->set_tgt_val(tgt_val);
  mt->set_planning_task(pt);
  mt->set_logging_task(lt);
  mt->set_queue_handler(xQueue);
  mt->set_task_handler(xTaskHandler);
  // mt->set_task_handler((TaskHandle_t) NULL);
  mt->create_task(1);

  esp_task_wdt_reset();
  esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));
  esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1));

  while (1) {
    vTaskDelay(5000.0 / portTICK_RATE_MS);
    if (mt->ui->button_state()) {
      printf("time_stamp: %d\n", tgt_val->nmr.timstamp);
      printf("motion_type: %d\n", static_cast<int>(tgt_val->motion_type));
      printf("fss.error: %d\n", tgt_val->fss.error);
      printf("motor_en: %d\n", (pt->motor_en) ? 1 : 0);
      printf("suction_en: %d\n", (pt->suction_en) ? 1 : 0);
    }
  }
}