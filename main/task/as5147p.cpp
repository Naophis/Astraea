#include "include/as5147p.hpp"

AS5147P::AS5147P() {}
AS5147P::~AS5147P() {}
void AS5147P::init() {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = ENC_MOSI,
      .miso_io_num = ENC_MISO,
      .sclk_io_num = ENC_CLK,
      .quadwp_io_num = -1,  // unused
      .quadhd_io_num = -1,  // unused
      .max_transfer_sz = 3, // bytes
      .flags = SPICOMMON_BUSFLAG_MASTER,
      .intr_flags = 0 // 割り込みをしない
  };
  spi_device_interface_config_t devcfg = {
      .mode = 3,
      .clock_speed_hz = 10 * 1000 * 1000,
      // .clock_speed_hz = 1 * 1000 * 1000,
      .spics_io_num = ENC_L_CS,
      .queue_size = 1,
  };
  // Initialize the SPI bus
  ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED);
  ESP_ERROR_CHECK(ret);
  // Attach the LCD to the SPI bus
  ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_l);
  devcfg.spics_io_num = ENC_R_CS;
  ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_r);
  // devcfg.spics_io_num = GYRO2_CS;
  // ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi_gyro);

  // high keep
  GPIO.out_w1ts = BIT(6);
  GPIO.out1_w1ts.val = BIT(33 - 32);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  initialized = true;
}
uint8_t AS5147P::write1byte(const uint8_t address, const uint8_t data) {
  return 0;
}
bool AS5147P::_spiCalcEvenParity(uint16_t value) {
  bool even = 0;
  uint8_t i;
  for (i = 0; i < 16; i++) {
    if (value & (0x0001 << i)) {
      even = !even;
    }
  }
  return even;
}
uint8_t AS5147P::read1byte(const uint8_t address) { return 0; }

int16_t AS5147P::read2byte(const uint16_t address) { return 0; }

uint32_t IRAM_ATTR AS5147P::read2byte(const uint8_t address1,
                                      const uint8_t address2, bool rorl) {
  esp_err_t ret;

  DRAM_ATTR static spi_transaction_t t;
  static bool is_initialized = false;

  if (!is_initialized) {
    memset(&t, 0, sizeof(t)); // Zero out the transaction once
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
    is_initialized = true;
  }

  auto res = _spiCalcEvenParity(address1 | READ_FLAG2);
  if (res) {
    t.tx_data[0] = (address1 | READ_FLAG2 | PARITY_FLAG);
  } else {
    t.tx_data[0] = (address1 | READ_FLAG2);
  }
  t.tx_data[1] = (address2);
  t.tx_data[2] = 0;
  t.tx_data[3] = 0;
  if (!rorl) {
    ret = spi_device_polling_transmit(spi_l, &t); // Transmit!
  } else {
    ret = spi_device_polling_transmit(spi_r, &t); // Transmit!
  }
  // assert(ret == ESP_OK);

  return (int32_t)((uint16_t)(t.rx_data[0]) << 8) | (uint16_t)(t.rx_data[1]);
}

int16_t AS5147P::read2byte_2(const uint8_t address1, const uint8_t address2) {
  return 0;
}

#define LSM6DSRX_CTRL1_XL 0x10U
#define LSM6DSRX_CTRL2_G 0x11U
#define LSM6DSRX_CTRL3_C 0x12U
#define LSM6DSRX_CTRL4_C 0x13U
#define LSM6DSRX_CTRL8_XL 0x17U
#define LSM6DSRX_CTRL9_XL 0x18U
#define LSM6DSRX_FIFO_CTRL4 0x0AU
#define LSM6DSRX_CTRL10_C 0x19U
#define LSM6DSRX_EMB_FUNC_SRC 0x64U
#define LSM6DSRX_FIFO_STATUS1 0x3AU
#define LSM6DSRX_FIFO_DATA_OUT_TAG 0x78U
#define LSM6DSRX_FIFO_DATA_OUT_Z_L 0x7DU
#define LSM6DSRX_FIFO_DATA_OUT_Z_H 0x7EU
void AS5147P::setup() {
  uint8_t whoami = read1byte_gy(0x0F);
  // begin();
  printf("setup gyro2\n");
  write1byte_gy(LSM6DSRX_CTRL3_C, 0x81); // LSM6DSRXをリセット
  vTaskDelay(200.0 / portTICK_PERIOD_MS);
  while ((read1byte(LSM6DSRX_CTRL3_C) & 0x01) == 0x01)
    ;
  if (true) {
    // lsm6sr
    write1byte_gy(LSM6DSRX_CTRL9_XL, 0xE2); // I3CモードをDisableに設定
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
    write1byte_gy(LSM6DSRX_CTRL4_C, 0x06); // I2CモードをDisableに設定
    vTaskDelay(10.0 / portTICK_PERIOD_MS);

    // 加速度計の設定
    write1byte_gy(LSM6DSRX_CTRL1_XL, 0xAA); // 4g
    // write1byte_gy(LSM6DSRX_CTRL1_XL, 0xAE); //8g
    // write1byte_gy(LSM6DSRX_CTRL1_XL, 0xA6); // 16g
    // 加速度計のスケールを±8gに設定
    // 加速度計の出力データレートを416Hzに設定
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
    write1byte_gy(LSM6DSRX_CTRL8_XL, 0xB0); // 加速度計のLPFを100Hzに設定
    vTaskDelay(10.0 / portTICK_PERIOD_MS);

    // ジャイロの設定
    write1byte_gy(LSM6DSRX_CTRL2_G, 0xA1);
    auto ctrl1 = read1byte_gy(LSM6DSRX_CTRL1_XL);
    auto ctrl2 = read1byte_gy(LSM6DSRX_CTRL2_G);
    auto ctrl3 = read1byte_gy(LSM6DSRX_CTRL3_C);
    auto ctrl4 = read1byte_gy(LSM6DSRX_CTRL4_C);
    auto ctrl9 = read1byte_gy(LSM6DSRX_CTRL9_XL);
    printf("%d, %d, %d, %d, %d\n", ctrl1, ctrl2, ctrl3, ctrl4, ctrl9);
  } else { // lsm6sdv16
    // write1byte(LSM6DSRX_CTRL9_XL, 0xE2); // I3CモードをDisableに設定
    write1byte_gy(0x03, 0x01); // I2C/I3CモードをDisableに設定
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
    write1byte_gy(0x15, 0x0c); // 4000deg/s,  LPF 279hz
    vTaskDelay(10.0 / portTICK_PERIOD_MS);

    write1byte_gy(0x11, 0x0c); // 7.68kHz
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
    auto ctrl1 = read1byte_gy(LSM6DSRX_CTRL1_XL);
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
    auto ctrl2 = read1byte_gy(LSM6DSRX_CTRL2_G);
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
    auto ctrl3 = read1byte_gy(LSM6DSRX_CTRL3_C);
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
    auto ctrl4 = read1byte_gy(LSM6DSRX_CTRL4_C);
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
    auto ctrl9 = read1byte_gy(LSM6DSRX_CTRL9_XL);
    vTaskDelay(10.0 / portTICK_PERIOD_MS);
    printf("%d, %d, %d, %d, %d\n", ctrl1, ctrl2, ctrl3, ctrl4, ctrl9);
    // ジャイロのスケールを±4000deg/sに設定
    // ジャイロの出力データレートを6.66Hzに設定
  }
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
}
int AS5147P::read_gyro_z() { return read2byte(0x47); }
int AS5147P::read_accel_x() { return read2byte(0x3B); }
int AS5147P::read_accel_y() { return read2byte(0x3D); }
void AS5147P::req_read1byte_itr(const uint8_t address) {}
uint8_t AS5147P::read_1byte_itr() {
  // spi_device_get_trans_result(spi_r, &r_trans, 1 / portTICK_RATE_MS);
  return (uint8_t)(((unsigned short)(r_trans->rx_data[1] & 0xff)));
}

void AS5147P::req_read2byte_itr(const uint8_t address) {
  memset(&itr_t, 0, sizeof(itr_t)); // Zero out the transaction
  itr_t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  itr_t.length = 24; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  itr_t.tx_data[0] = (address | READ_FLAG2);
  itr_t.tx_data[1] = 0;
  itr_t.tx_data[2] = 0;
  // spi_device_queue_trans(spi, &itr_t, 1 / portTICK_RATE_MS); // Transmit!
}
int16_t AS5147P::read_2byte_itr() {
  // spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}
signed short AS5147P::read_2byte_itr2(std::vector<int> &list) {
  // spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  list[0] = r_trans->rx_data[0];
  list[1] = r_trans->rx_data[1];
  list[2] = r_trans->rx_data[2];
  list[3] = r_trans->rx_data[3];
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}

uint8_t AS5147P::read1byte_gy(const uint8_t address) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.flags = SPI_TRANS_USE_RXDATA;
  t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  uint16_t tx_data = (address | READ_FLAG) << 8;
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  t.tx_buffer = &tx_data;
  ret = spi_device_polling_transmit(spi_gyro, &t); // Transmit!
  assert(ret == ESP_OK);                           // Should have had no issues.
  printf("%d, %d\n", t.rx_data[0], t.rx_data[1]);
  uint8_t data =
      SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; // FF + Data
  return data;
}

uint8_t AS5147P::write1byte_gy(const uint8_t address, const uint8_t data) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction

  // printf("%2x, %2x\n", address, data);
  // printf("%2x, %2x\n", address, 0x0f & data);
  t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.tx_data[0] = address;
  t.tx_data[1] = (uint8_t)(0xff & data);
  // t.tx_data[2] = 0;

  // uint16_t tx_data = (address) << 8 | (0xff & data);
  // tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  // t.tx_buffer = &(t.tx_data);
  ret = spi_device_polling_transmit(spi_gyro, &t); // Transmit!
  assert(ret == ESP_OK);                           // Should have had no issues.
  return 0;
}

int16_t IRAM_ATTR AS5147P::read_2byte_gy(const uint8_t address) {
  esp_err_t ret;
  DRAM_ATTR static spi_transaction_t t;
  static bool is_initialized = false;

  if (!is_initialized) {
    memset(&t, 0, sizeof(t)); // Zero out the transaction once
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 24; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
    is_initialized = true;
  }

  t.tx_data[0] = (address | READ_FLAG);
  t.tx_data[1] = 0;
  t.tx_data[2] = 0;

  ret = spi_device_polling_transmit(spi_gyro, &t); // Transmit!

  return (signed short)((((unsigned short)(t.rx_data[2] & 0xff)) << 8) |
                        ((unsigned short)(t.rx_data[1] & 0xff)));
}