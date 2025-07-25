#include "include/asm330lhh.hpp"

ASM330LHH::ASM330LHH() {}
ASM330LHH::~ASM330LHH() {}
void ASM330LHH::init() {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = EN_MOSI,
      .miso_io_num = EN_MISO,
      .sclk_io_num = EN_CLK,
      .quadwp_io_num = -1,  // unused
      .quadhd_io_num = -1,  // unused
      .max_transfer_sz = 2, // bytes
      .flags = SPICOMMON_BUSFLAG_MASTER,
      .intr_flags = 0 // 割り込みをしない
  };
  spi_device_interface_config_t devcfg = {
      .mode = 3,
      .clock_speed_hz = 10 * 1000 * 1000, // aaaaaaaaaaa
      .spics_io_num = EN_GN_SSL,
      .queue_size = 12,
  };
  // Initialize the SPI bus
  ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
  ESP_ERROR_CHECK(ret);
  // Attach the LCD to the SPI bus
  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
  // devcfg.spics_io_num = EN_GN_SSL2;
  // ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_2);
  ESP_ERROR_CHECK(ret);
}

uint8_t IRAM_ATTR ASM330LHH::write1byte(const uint8_t address,
                                        const uint8_t data) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = 16;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.tx_data[0] = address;
  t.tx_data[1] = (uint8_t)(0xff & data);
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  // if (!use_2) {
  // } else {
  //   ret = spi_device_polling_transmit(spi_2, &t); // Transmit!
  // }
  assert(ret == ESP_OK); // Should have had no issues.
  return 0;
}

uint8_t IRAM_ATTR ASM330LHH::read1byte(const uint8_t address) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.flags = SPI_TRANS_USE_RXDATA;
  t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  uint16_t tx_data = (address | READ_FLAG) << 8;
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  t.tx_buffer = &tx_data;
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  // if (!use_2) {
  // } else {
  //   ret = spi_device_polling_transmit(spi_2, &t); // Transmit!
  // }
  assert(ret == ESP_OK); // Should have had no issues.
  printf("%d, %d\n", t.rx_data[0], t.rx_data[1]);
  uint8_t data =
      SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; // FF + Data
  return data;
}

int16_t IRAM_ATTR ASM330LHH::read_2byte(const uint8_t address) {
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

  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  // if (!use_2) {
  // } else {
  //   ret = spi_device_polling_transmit(spi_2, &t); // Transmit!
  // }

  return (signed short)((((unsigned short)(t.rx_data[2] & 0xff)) << 8) |
                        ((unsigned short)(t.rx_data[1] & 0xff)));
}

int ASM330LHH::get_unread_fifo_data_length() {
  return read1byte(ASM330LHH_FIFO_STATUS1); //
}
int ASM330LHH::get_fifo_data() {
  return read1byte(ASM330LHH_FIFO_DATA_OUT_Z_L); //
}
int ASM330LHH::get_fifo_tag() {
  return read1byte(ASM330LHH_FIFO_DATA_OUT_TAG); //
}

void ASM330LHH::setup() {
  uint8_t whoami = read1byte(0x0F);
  // begin();

  write1byte(ASM330LHH_CTRL3_C, 0x83); // ASM330LHHをリセット
  vTaskDelay(200.0 / portTICK_PERIOD_MS);
  while ((read1byte(ASM330LHH_CTRL3_C) & 0x01) == 0x01)
    ;
  write1byte(ASM330LHH_CTRL4_C, 0x04); // I2C/I3CモードをDisableに設定
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  write1byte(ASM330LHH_CTRL2_G, 0x91); // ODR: 6667Hz, scale: 4000deg/s
  vTaskDelay(10.0 / portTICK_PERIOD_MS);

  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  auto ctrl1 = read1byte(ASM330LHH_CTRL1_XL);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  auto ctrl2 = read1byte(ASM330LHH_CTRL2_G);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  auto ctrl3 = read1byte(ASM330LHH_CTRL3_C);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  auto ctrl4 = read1byte(ASM330LHH_CTRL4_C);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  auto ctrl9 = read1byte(ASM330LHH_CTRL9_XL);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  printf("%d, %d, %d, %d, %d\n", ctrl1, ctrl2, ctrl3, ctrl4, ctrl9);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
}
int16_t ASM330LHH::read_gyro_z() { return read_2byte(0x26); }
int16_t ASM330LHH::read_accel_x() { return read_2byte(0x3B); }
int16_t ASM330LHH::read_accel_y() { return read_2byte(0x3D); }
