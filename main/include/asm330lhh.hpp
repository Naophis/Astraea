#ifndef ASM330LHH_HPP
#define ASM330LHH_HPP

#include "defines.hpp"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <string.h>

#define ASM330LHH_CTRL1_XL 0x10U
#define ASM330LHH_CTRL2_G 0x11U
#define ASM330LHH_CTRL3_C 0x12U
#define ASM330LHH_CTRL4_C 0x13U
#define ASM330LHH_CTRL6_C 0x15U
#define ASM330LHH_CTRL8_XL 0x17U
#define ASM330LHH_CTRL9_XL 0x18U
#define ASM330LHH_CTRL1_OIS 0x70U
#define ASM330LHH_FIFO_CTRL4 0x0AU
#define ASM330LHH_CTRL10_C 0x19U
#define ASM330LHH_EMB_FUNC_SRC 0x64U
#define ASM330LHH_FIFO_STATUS1 0x3AU
#define ASM330LHH_FIFO_DATA_OUT_TAG 0x78U
#define ASM330LHH_FIFO_DATA_OUT_Z_L 0x7DU
#define ASM330LHH_FIFO_DATA_OUT_Z_H 0x7EU

class ASM330LHH {
public:
  ASM330LHH();
  virtual ~ASM330LHH();

  void init();
  uint8_t write1byte(const uint8_t address, const uint8_t data);
  uint8_t read1byte(const uint8_t address);
  int16_t read_2byte(const uint8_t address);
  void setup();
  int16_t read_gyro_z();
  int16_t read_accel_x();
  int16_t read_accel_y();
  int get_unread_fifo_data_length();
  int get_fifo_data();
  int get_fifo_tag();

private:
  spi_device_handle_t spi;
  spi_transaction_t itr_t;
  spi_transaction_t *r_trans;
};

#endif