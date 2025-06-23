#include "include/mt6835.hpp"

MT6835::MT6835() {}
MT6835::~MT6835() {}
void MT6835::init() {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = ENC_MOSI,
      .miso_io_num = ENC_MISO,
      .sclk_io_num = ENC_CLK,
      .quadwp_io_num = -1,  // unused
      .quadhd_io_num = -1,  // unused
      .max_transfer_sz = 5, // bytes
      .flags = SPICOMMON_BUSFLAG_MASTER,
      .intr_flags = 0 // 割り込みをしない
  };
  spi_device_interface_config_t devcfg = {
      .mode = 3,
      .clock_speed_hz = 1 * 1000 * 1000,
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

  // high keep
  GPIO.out_w1ts = BIT(6);
  GPIO.out1_w1ts.val = BIT(35 - 32);
  vTaskDelay(10.0 / portTICK_PERIOD_MS);
  initialized = true;
}
uint8_t MT6835::write1byte(const uint8_t address, const uint8_t data) {
  return 0;
}

bool MT6835::_spiCalcEvenParity(uint16_t value) {
  bool even = 0;
  uint8_t i;
  for (i = 0; i < 16; i++) {
    if (value & (0x0001 << i)) {
      even = !even;
    }
  }
  return even;
}

uint32_t IRAM_ATTR MT6835::read2byte(const uint8_t address1,
                                     const uint8_t address2, bool rorl) {
  esp_err_t ret;

  DRAM_ATTR static spi_transaction_t t;
  static bool is_initialized = false;

  if (!is_initialized) {
    memset(&t, 0, sizeof(t)); // Zero out the transaction once
    // t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 5 * 8; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
    is_initialized = true;
  }

  uint8_t tx[5] = {address1, address2, 0x00, 0x00, 0x00};
  uint8_t rx[5] = {0};

  t.tx_buffer = tx;
  t.rx_buffer = rx;
  
  if (!rorl) {
    ret = spi_device_polling_transmit(spi_l, &t); // Transmit!
  } else {
    ret = spi_device_polling_transmit(spi_r, &t); // Transmit!
  }
  // assert(ret == ESP_OK);

  // printf("%d, %d, %d, %d\n", t.rx_data[0], t.rx_data[1], t.rx_data[2],
  //        t.rx_data[3]);
  printf("%d, %d, %d, %d, %d\n", rx[0], rx[1], rx[2], rx[3], rx[4]);
  return 0;
  // return (int32_t)((uint16_t)(t.rx_data[0]) << 8) | (uint16_t)(t.rx_data[1]);
}

int16_t MT6835::read2byte_2(const uint8_t address1, const uint8_t address2) {
  return 0;
}

void MT6835::setup() {}