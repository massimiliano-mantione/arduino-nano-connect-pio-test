#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DSOX.h>
#include <vl53lx_class.h>
#include <arducam.h>

#include <rust_lib.h>

extern "C" void ffi_usb_serial_write(const char* buffer, size_t size) {
  Serial.write(buffer, size);
}

uint8_t image_buf[324*324];
uint8_t image_tmp[162*162];
uint8_t image[96*96];
uint8_t header[2] = {0x55,0xAA};

void setup() {
  Serial.begin(115200);

  //gpio_init(PIN_LED);
	//gpio_set_dir(PIN_LED, GPIO_OUT);
	struct arducam_config config;
	config.sccb = i2c0;
	config.sccb_mode = I2C_MODE_16_8;
	config.sensor_address = 0x24;
	config.pin_sioc = PIN_CAM_SIOC;
	config.pin_siod = PIN_CAM_SIOD;
	config.pin_resetb = PIN_CAM_RESETB;
	config.pin_xclk = PIN_CAM_XCLK;
	config.pin_vsync = PIN_CAM_VSYNC;
	config.pin_y2_pio_base = PIN_CAM_Y2_PIO_BASE;

	config.pio = pio0;
	config.pio_sm = 0;

	config.dma_channel = 0;
	config.image_buf = image_buf;
	config.image_buf_size = sizeof(image_buf);

	arducam_init(&config);
}

int count = 0;

void loop() {
  count++;
  print_tick(count);
  sleep_ms(1000);
}
