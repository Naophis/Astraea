#ifndef SENSING_TASK_HPP
#define SENSING_TASK_HPP

#include "as5147p.hpp"
#include "defines.hpp"
#include "driver/pcnt.h"
#include "driver/rtc_io.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "icm20689.hpp"
#include "lsm6dsr.hpp"
#include "main_task.hpp"
#include <algorithm> // std::min_element, std::max_element
#include <deque>
#include <driver/adc.h>
#include <numeric> // std::accumulate
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <array>

// リングバッファのサイズを定義
constexpr size_t BUFFER_SIZE = 5;

class RingBuffer {
public:
  // 配列と重みの初期化
  std::array<double, BUFFER_SIZE> buffer{};
  std::array<double, BUFFER_SIZE> weights{1, 2, 3, 4, 5}; // 重みを設定
  int head = 0; // 次にデータを追加する位置
  int tail = 0; // 現在の先頭要素の位置
  int size = 0; // 現在の要素数
  bool is_full = false;

  // 新しいデータをリングバッファに追加
  void add(double value) {
    buffer[head] = value;
    head = (head + 1) % BUFFER_SIZE;
    if (is_full) {
      tail = (tail + 1) % BUFFER_SIZE;
      size = BUFFER_SIZE;
    } else {
      size++; // バッファが満杯でない場合のみサイズを増加
    }
    is_full = head == tail;
  }
  double pop() {
    if (!is_full && head == tail) {
      return 0;
    }
    double value = buffer[tail];
    tail = (tail + 1) % BUFFER_SIZE;
    is_full = false; // データを削除したので full フラグをリセット
    size--;
    return value;
  }
  bool isEmpty() const {
    return (!is_full && head == tail); // バッファが空かどうか
  }
  int getSize() const { return size; }
  double getLatest() const {
    if (!is_full && head == tail) {
      return 0;
    }
    size_t latestIndex = (head == 0) ? BUFFER_SIZE - 1 : head - 1;
    return buffer[latestIndex];
  }
  // 重み付き平均を計算（minとmaxを除外）
  double calculateWeightedAverageWithoutMinMax() {
    // minとmaxの要素を見つける
    auto minIt = std::min_element(buffer.begin(), buffer.end());
    auto maxIt = std::max_element(buffer.begin(), buffer.end());
    size_t minIndex = std::distance(buffer.begin(), minIt);
    size_t maxIndex = std::distance(buffer.begin(), maxIt);

    // minとmaxの一つを除外して加重平均を計算
    double weightedSum = 0;
    double weightSum = 0;
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
      if (i != minIndex && i != maxIndex) { // 最初のmin, maxのみスキップ
        weightedSum += buffer[i] * weights[i];
        weightSum += weights[i];
      }
    }

    return weightedSum / weightSum;
  }
};

class SensingTask {
public:
  SensingTask();
  virtual ~SensingTask();
  void create_task(const BaseType_t xCoreID);

  std::shared_ptr<sensing_result_entity_t> sensing_result;
  std::shared_ptr<PlanningTask> pt;
  void set_sensing_entity(std::shared_ptr<sensing_result_entity_t> &_entity);
  void set_planning_task(std::shared_ptr<PlanningTask> &_pt);

  QueueHandle_t *qh;
  void set_queue_handler(QueueHandle_t &_qh) { qh = &_qh; }
  TaskHandle_t *th;
  void set_task_handler(TaskHandle_t &_th) { th = &_th; }
  std::shared_ptr<MainTask> mt;
  void set_main_task(std::shared_ptr<MainTask> &_mt) {
    mt = _mt; //
  }

  static void task_entry_point(void *task_instance);
  virtual void task();
  static void task_entry_point0(void *task_instance);

  void timer_200us_callback_main();
  void set_input_param_entity(std::shared_ptr<input_param_t> &_param);

  bool is_ready() { return ready; }
  std::deque<int> gyro_q;
  void set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val);

private:
  LSM6DSR gyro_if;
  AS5147P enc_if;
  float w_old = 0;
  float vr_old = 0;
  float vl_old = 0;
  int64_t gyro_timestamp_old = 0;
  int64_t gyro_timestamp_now = 0;
  int64_t gyro2_timestamp_old = 0;
  int64_t gyro2_timestamp_now = 0;
  int64_t enc_r_timestamp_old = 0;
  int64_t enc_r_timestamp_now = 0;
  int64_t enc_l_timestamp_old = 0;
  int64_t enc_l_timestamp_now = 0;
  volatile int cnt_a = 0;
  esp_timer_handle_t timer_200us;

  motion_tgt_val_t *receive_req;

  static void timer_200us_callback(void *arg);

  void exec_adc(adc2_channel_t channel, adc_bits_width_t width_bit,
                int *raw_out);

  void calc_vel(float gyro_dt, float enc_l_dt, float enc_r_dt);
  void IRAM_ATTR set_gpio_state(gpio_num_t gpio_num, int state) {
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
  void IRAM_ATTR led_driver(gpio_num_t io1, int state1, gpio_num_t io2,
                            int state2, gpio_num_t io3, int state3) {
    const int num1 = (int)io1;
    const int num2 = (int)io2;
    const int num3 = (int)io3;
    uint32_t out_wlts = 0;
    uint32_t out_wltc = 0;
    if (state1) {
      out_wlts |= BIT(num1);
    } else {
      out_wltc |= BIT(num1);
    }
    if (state2) {
      out_wlts |= BIT(num2);
    } else {
      out_wltc |= BIT(num2);
    }
    if (state3) {
      out_wlts |= BIT(num3);
    } else {
      out_wltc |= BIT(num3);
    }
    GPIO.out_w1ts = out_wlts;
    GPIO.out_w1tc = out_wltc;
  }

  float calc_sensor(float data, float a, float b);
  volatile int lec_cnt = 0;
  std::shared_ptr<input_param_t> param;
  int led_light_delay_cnt = 10000;
  xTaskHandle handle = 0;
  bool ready;
  timer_isr_handle_t handle_isr;
  void encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                    const gpio_num_t pinB);
  const TickType_t xDelay = 1.0 / portTICK_PERIOD_MS;

  // void timer_isr(void *parameters);
  std::shared_ptr<motion_tgt_val_t> tgt_val;

  bool itr_state = true;
  static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
  static const adc_atten_t atten = ADC_ATTEN_DB_11;

  float offset_enc_val(float val, vector<float> &coef_list);
  float calc_enc_v(float now, float old, float dt);
};

#endif