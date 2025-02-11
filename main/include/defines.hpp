#ifndef DEFINES_HPP
#define DEFINES_HPP

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "gen_code_mpc/bus.h"
#include "include/maze_solver.hpp"
#include "sdkconfig.h"
#include "soc/adc_channel.h"
#include "soc/gpio_struct.h"
#include "soc/ledc_periph.h"

#include "fs.hpp"
#include "include/enums.hpp"
#include "include/structs.hpp"
#include "sdkconfig.h"
#include <bitset>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "esp_heap_caps.h"
#include "icm20689.hpp"
#include "lsm6dsr.hpp"

#define portTICK_RATE_MS portTICK_PERIOD_MS
#define xTaskHandle TaskHandle_t

#define ABS(IN) ((IN) < 0 ? -(IN) : (IN))

constexpr int GY_DQ_SIZE = 2;
constexpr int GY_CYCLE = 2500; // 2500=1/4msec
// constexpr int GY_CYCLE = 1250; // 1250=1/8msec
constexpr int GY_MODE = 0;
constexpr float cell_size = 90;

// Banshee
//  constexpr gpio_num_t SDA_PIN = GPIO_NUM_16;
//  constexpr gpio_num_t SCL_PIN = GPIO_NUM_14;
// Exia
constexpr gpio_num_t SDA_PIN = GPIO_NUM_15;
constexpr gpio_num_t SCL_PIN = GPIO_NUM_14;

constexpr gpio_num_t LED_EN = GPIO_NUM_10;
constexpr gpio_num_t LED_A0 = GPIO_NUM_11;
constexpr gpio_num_t LED_A1 = GPIO_NUM_12;

constexpr gpio_num_t TXD = GPIO_NUM_43;
constexpr gpio_num_t RXD = GPIO_NUM_44;
constexpr gpio_num_t RTS = GPIO_NUM_15;
constexpr gpio_num_t CTS = GPIO_NUM_16;

constexpr gpio_num_t L_CW_CCW1 = GPIO_NUM_38;
constexpr gpio_num_t L_CW_CCW2 = GPIO_NUM_38;
constexpr gpio_num_t R_CW_CCW1 = GPIO_NUM_46;
constexpr gpio_num_t R_CW_CCW2 = GPIO_NUM_46;

constexpr uint32_t L_CW_CCW1_BIT = BIT(GPIO_NUM_38 - 32);
constexpr uint32_t L_CW_CCW2_BIT = BIT(GPIO_NUM_38 - 32);

constexpr uint32_t R_CW_CCW1_BIT = BIT(GPIO_NUM_38 - 32);
constexpr uint32_t R_CW_CCW2_BIT = BIT(GPIO_NUM_38 - 32);

constexpr gpio_num_t Motor_L_PWM = GPIO_NUM_39;
constexpr gpio_num_t Motor_L_PWM2 = GPIO_NUM_40;

constexpr gpio_num_t Motor_R_PWM = GPIO_NUM_41;
constexpr gpio_num_t Motor_R_PWM2 = GPIO_NUM_42;

constexpr gpio_num_t BUZZER = GPIO_NUM_5;

constexpr gpio_num_t SW1 = GPIO_NUM_34;

constexpr gpio_num_t EN_MISO = GPIO_NUM_1;
constexpr gpio_num_t EN_MOSI = GPIO_NUM_2;
constexpr gpio_num_t EN_CLK = GPIO_NUM_3;
constexpr gpio_num_t EN_GN_SSL = GPIO_NUM_4;

constexpr gpio_num_t ENC_R_CS = GPIO_NUM_6;
constexpr gpio_num_t GYRO2_CS = GPIO_NUM_46;
constexpr gpio_num_t ENC_L_CS = GPIO_NUM_37;
constexpr gpio_num_t ENC_CLK = GPIO_NUM_7;
constexpr gpio_num_t ENC_MISO = GPIO_NUM_8; // B
constexpr gpio_num_t ENC_MOSI = GPIO_NUM_9; // A

constexpr gpio_num_t SUCTION_PWM = GPIO_NUM_46;

// Banshee
//  #define SEN_R90 ADC2_CHANNEL_2
//  #define SEN_L90 ADC2_CHANNEL_4
//  #define SEN_R45 ADC2_CHANNEL_6
//  #define SEN_L45 ADC2_CHANNEL_7
//  #define BATTERY ADC2_CHANNEL_9
// Exia
#define SEN_R90 ADC2_CHANNEL_2
#define SEN_L90 ADC2_CHANNEL_5
#define SEN_R45 ADC2_CHANNEL_7
#define SEN_L45 ADC2_CHANNEL_6
#define BATTERY ADC2_CHANNEL_9

// Banshee
//  #define SEN_R90_GPIO GPIO_NUM_13
//  #define SEN_L90_GPIO GPIO_NUM_15
//  #define SEN_R45_GPIO GPIO_NUM_17
//  #define SEN_L45_GPIO GPIO_NUM_18
//  #define BATTERY_GPIO GPIO_NUM_20
// Exia
#define SEN_R90_GPIO GPIO_NUM_13
#define SEN_L90_GPIO GPIO_NUM_16
#define SEN_R45_GPIO GPIO_NUM_18
#define SEN_L45_GPIO GPIO_NUM_17
#define BATTERY_GPIO GPIO_NUM_20

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// constexpr adc2_channel_t SEN_R90 = ADC2_CHANNEL_0;
// constexpr adc2_channel_t SEN_R45 = ADC2_CHANNEL_1;
// constexpr adc2_channel_t SEN_F = ADC2_CHANNEL_4;
// constexpr adc2_channel_t SEN_L45 = ADC2_CHANNEL_6;
// constexpr adc2_channel_t SEN_L90 = ADC2_CHANNEL_8;
// constexpr adc2_channel_t BATTERY = ADC2_CHANNEL_9;

constexpr int16_t ENCODER_H_LIM_VAL = 32767;
constexpr int16_t ENCODER_L_LIM_VAL = -32767;

const int16_t ENC_RESOLUTION = 16384 - 1;
constexpr uint8_t READ_FLAG = 0x80;
constexpr uint16_t READ_FLAG2 = 0b01000000;
constexpr uint16_t PARITY_FLAG = 0b10000000;
constexpr uint8_t ESC = 0x1B;
constexpr uint16_t BUF_SIZE = 4096;

constexpr uint16_t MOTION_CHECK_TH = 1000;
constexpr uint16_t ENC_OPE_V_R_TH = 90 * 1;

constexpr uint16_t LOG_SIZE = 1300;
constexpr uint16_t LINE_BUF_SIZE = 1024;
// constexpr float BATTERY_GAIN = 3.2075; // 2.97324;
constexpr float BATTERY_GAIN = 3.740556718095668; // 2.97324;

constexpr uint8_t LEDC_HIGH_SPEED_MODE = 0;
constexpr float LOW_BATTERY_TH = 11.5;

constexpr uint16_t RESET_GYRO_LOOP_CNT = 512;

static const std::string slalom_log_file("/spiflash/sla.log");
static const std::string sysid_log_file("/spiflash/sysid.log");
static const std::string maze_log_file("/spiflash/maze.txt");
static const std::string maze_log_kata_file("/spiflash/maze_kata.log");
static const std::string maze_log_return_file("/spiflash/maze_return.log");

static const std::string format1("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%0.3f,"
                                 "%0.3f,%0.3f,%0.3f,");
static const std::string
    format2("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
static const std::string format3("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0."
                                 "3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,"
                                 "%0.3f,%d,");
static const std::string format4("%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%d,%d,%d,");
static const std::string format5(
    "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
static const std::string
    format6("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,"
            "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,");
static const std::string format7("%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n");
static const std::string
    formatsysid("%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n");
static char line_buf[LINE_BUF_SIZE];
// static const std::string format1("%d,%d,%d,%d,%d,%d,");
// static const std::string format2("%d,%d,%d,%d,%d,%d,%d,");
// static const std::string format3("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n");

std::shared_ptr<sensing_result_entity_t> get_sensing_entity();

#endif