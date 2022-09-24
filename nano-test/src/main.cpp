#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DSOX.h>
#include <vl53lx_class.h>
#include <arducam.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_SSD1306.h>
#include <IRremote.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

// Components.
VL53LX sensor_vl53lx_sat(&DEV_I2C, A1);

#include <rust_lib.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000
};

void testdrawline() {
  int16_t i;

  display.clearDisplay(); // Clear display buffer

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn line
    delay(1);
  }
  for(i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();

  for(i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  for(i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000); // Pause for 2 seconds
}

void testdrawrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=2) {
    display.drawRect(i, i, display.width()-2*i, display.height()-2*i, SSD1306_WHITE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testfillrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2; i+=3) {
    // The INVERSE color is used so rectangles alternate white/black
    display.fillRect(i, i, display.width()-i*2, display.height()-i*2, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn rectangle
    delay(1);
  }

  delay(2000);
}

void testdrawcircle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=2) {
    display.drawCircle(display.width()/2, display.height()/2, i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillcircle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, SSD1306_INVERSE);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testdrawroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    display.drawRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfillroundrect(void) {
  display.clearDisplay();

  for(int16_t i=0; i<display.height()/2-2; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, display.width()-2*i, display.height()-2*i,
      display.height()/4, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawtriangle(void) {
  display.clearDisplay();

  for(int16_t i=0; i<max(display.width(),display.height())/2; i+=5) {
    display.drawTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_WHITE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testfilltriangle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/2; i>0; i-=5) {
    // The INVERSE color is used so triangles alternate white/black
    display.fillTriangle(
      display.width()/2  , display.height()/2-i,
      display.width()/2-i, display.height()/2+i,
      display.width()/2+i, display.height()/2+i, SSD1306_INVERSE);
    display.display();
    delay(1);
  }

  delay(2000);
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println(F("scroll"));
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrollleft(0x00, 0x0F);
  delay(2000);
  display.stopscroll();
  delay(1000);
  display.startscrolldiagright(0x00, 0x07);
  delay(2000);
  display.startscrolldiagleft(0x00, 0x07);
  delay(2000);
  display.stopscroll();
  delay(1000);
}

void testdrawbitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}

#define XPOS   0 // Indexes into the 'icons' array in function below
#define YPOS   1
#define DELTAY 2

void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for(f=0; f< NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  for(;;) { // Loop forever...
    display.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for(f=0; f< NUMFLAKES; f++) {
      display.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display.display(); // Show the display buffer on the screen
    delay(200);        // Pause for 1/10 second

    // Then update coordinates of each flake...
    for(f=0; f< NUMFLAKES; f++) {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display.height()) {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
        icons[f][YPOS]   = -LOGO_HEIGHT;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }
}

void setup_display() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
      delay(1);
  }

  // Initialize I2C bus.
  DEV_I2C.begin();

  Serial.println(F("SSD1306 initialization..."));
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  Serial.println(F("SSD1306 initialization done."));

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  // display.display() is NOT necessary after every single drawing command,
  // unless that's what you want...rather, you can batch up a bunch of
  // drawing operations and then update the screen all at once by calling
  // display.display(). These examples demonstrate both approaches...

  testdrawline();      // Draw many lines

  testdrawrect();      // Draw rectangles (outlines)

  testfillrect();      // Draw rectangles (filled)

  testdrawcircle();    // Draw circles (outlines)

  testfillcircle();    // Draw circles (filled)

  testdrawroundrect(); // Draw rounded rectangles (outlines)

  testfillroundrect(); // Draw rounded rectangles (filled)

  testdrawtriangle();  // Draw triangles (outlines)

  testfilltriangle();  // Draw triangles (filled)

  testdrawchar();      // Draw characters of the default font

  testdrawstyles();    // Draw 'stylized' characters

  testscrolltext();    // Draw scrolling text

  testdrawbitmap();    // Draw a small bitmap image

  // Invert and restore display, pausing in-between
  display.invertDisplay(true);
  delay(1000);
  display.invertDisplay(false);
  delay(1000);

  testanimate(logo_bmp, LOGO_WIDTH, LOGO_HEIGHT); // Animate bitmaps
}

void loop_display() {
  Serial.println(F("SSD1306 test done."));
  sleep_ms(1000);
}


extern "C" void ffi_usb_serial_write(const char* buffer, size_t size) {
  Serial.write(buffer, size);
}

uint8_t image_buf[324*324];
uint8_t image_tmp[162*162];
uint8_t image[96*96];
uint8_t header[2] = {0x55,0xAA};

void setup_with_arducam() {
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

// VL53LX_GetDistanceMode
// VL53LX_SetMeasurementTimingBudgetMicroSeconds
// VL53LX_set_inter_measurement_period_ms
// VL53LX_get_inter_measurement_period_ms
// VL53LX_set_zone_config
// VL53LX_set_preset_mode
// VL53LX_set_zone_preset
// VL53LX_get_measurement_results


void setup_with_VL53LX() {
  Serial.begin(115200);

  pinMode(LedPin, OUTPUT);

   SerialPort.println("Starting...");

   // Initialize I2C bus.
   DEV_I2C.begin();

   // Configure VL53LX satellite component.
   sensor_vl53lx_sat.begin();

   // Switch off VL53LX satellite component.
   sensor_vl53lx_sat.VL53LX_Off();

   //Initialize VL53LX satellite component.
   sensor_vl53lx_sat.InitSensor(0x12);

   // Public setup API (where should it be invoked? how should it be used?)
   // sensor_vl53lx_sat.SetPresetModeL3CX(VL53LX_DISTANCEMODE_MEDIUM, 0);
   // sensor_vl53lx_sat.VL53LX_run_zone_calibration(
   //   VL53LX_DEVICEPRESETMODE_NONE,
   //   VL53LX_DEVICEZONEPRESET_XTALK_PLANAR, NULL, 0, 0, NULL);

   //sensor_vl53lx_sat.VL53LX_set_zone_preset(VL53LX_DEVICEZONEPRESET_2X2_SIZE_8X8);
   //sensor_vl53lx_sat.VL53LX_set_preset_mode(VL53LX_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE_LONG_RANGE);

   // Start Measurements
   sensor_vl53lx_sat.VL53LX_StartMeasurement();
}

int count = 0;

void loop_test_ffi() {
  count++;
  print_tick(count);
  sleep_ms(1000);
}

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup_with_vl53l0x() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X test.");
  if (!lox.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while(1);
  }
  // power
  Serial.println("VL53L0X API Continuous Ranging example\n\n");

  // start continuous ranging
  lox.startRangeContinuous();
}

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
#define SHT_LOX1 2
#define SHT_LOX2 4
#define SHT_LOX3 6
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32

void setup_with_vl53l0x_3() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  Serial.println("Adafruit VL53L0X reset");
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(100);
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(100);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(100);

  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println("Failed to boot VL53L0X 1");
    while(1);
  } else {
    Serial.println("Booted VL53L0X 1");
  }
  delay(100);

  digitalWrite(SHT_LOX2, HIGH);
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println("Failed to boot VL53L0X 2");
    while(1);
  } else {
    Serial.println("Booted VL53L0X 2");
  }
  delay(100);

  digitalWrite(SHT_LOX3, HIGH);
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println("Failed to boot VL53L0X 3");
    while(1);
  } else {
    Serial.println("Booted VL53L0X 3");
  }
  delay(10);


  // start continuous ranging
  Serial.println("Start VL53L0X Continuous Ranging \n");
  lox1.startRangeContinuous();
  lox2.startRangeContinuous();
  lox3.startRangeContinuous();
}

const char* ble_device_name = "BLE-TEST-DEVICE";
const char* ble_service_uuid = "934642e9-db09-4802-90b2-aec03c8bd146";
const char* ble_characteristic_data_uuid = "2a47b36e-3e2e-496f-9a8e-2f14313acb53";
const char* ble_characteristic_command_uuid = "2d4b86f0-2342-4d67-8734-9c2ca4b380d6";

#define BLE_CHARACTERISTIC_DATA_LENGTH 50
#define BLE_CHARACTERISTIC_COMMAND_LENGTH 10
uint8_t ble_characteristic_data_bytes[BLE_CHARACTERISTIC_DATA_LENGTH + 1];
uint8_t ble_characteristic_command_bytes[BLE_CHARACTERISTIC_COMMAND_LENGTH + 1];

BLEService ble_service(ble_service_uuid);
BLECharacteristic ble_characteristic_data(
      ble_characteristic_data_uuid,
      BLERead | BLENotify,
      BLE_CHARACTERISTIC_DATA_LENGTH,
      false
   );
BLECharacteristic ble_characteristic_command(
      ble_characteristic_command_uuid,
      BLERead | BLEWrite,
      "          "
   );

void setup_with_ble() {
   Serial.begin(115200);

   // wait until serial port opens for native USB devices
   while (! Serial) {
      delay(1);
   }

   Serial.println("INIT ble_characteristic_data_bytes");
   memset(ble_characteristic_data_bytes, ' ', BLE_CHARACTERISTIC_DATA_LENGTH);
   ble_characteristic_data_bytes[BLE_CHARACTERISTIC_DATA_LENGTH] = 0;

   Serial.println("INIT ble_characteristic_command_bytes");
   memset(ble_characteristic_command_bytes, ' ', BLE_CHARACTERISTIC_COMMAND_LENGTH);
   ble_characteristic_command_bytes[BLE_CHARACTERISTIC_COMMAND_LENGTH] = 0;

   Serial.println("SETUP ble_service");
   ble_service.addCharacteristic(ble_characteristic_data);
   ble_service.addCharacteristic(ble_characteristic_command);

   Serial.println("BLE begin");
   if (!BLE.begin()) {
      Serial.println("starting Bluetooth® Low Energy failed!");
      while(false);
   }
   BLE.setLocalName(ble_device_name);
   BLE.setAdvertisedService(ble_service);
   BLE.addService(ble_service);
   BLE.setConnectable(true);
   BLE.advertise();

   Serial.print("Local address is: ");
   Serial.println(BLE.address());

   Serial.println("BLE waiting for connections");
}


void print_multi_ranging_data(VL53LX_MultiRangingData_t *pMultiRangingData) {
      uint8_t count=pMultiRangingData->StreamCount;
      uint8_t no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
      SerialPort.print("count=");
      SerialPort.print(count);
      SerialPort.print(", objs=");
      SerialPort.print(no_of_object_found);
      for(int j=0;j<no_of_object_found;j++)
      {
         if(j!=0)SerialPort.print("\r\n                               ");
         SerialPort.print(" [status=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
         SerialPort.print(", D=");
         SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
         SerialPort.print("mm");
         SerialPort.print(", Signal=");
         SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0);
         SerialPort.print(" Mcps, Ambient=");
         SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
         SerialPort.print(" Mcps]");
      }

}

void loop_with_VL53LX()
{
   VL53LX_MultiRangingData_t MultiRangingData;
   VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
   uint8_t NewDataReady = 0;
   int status;

   do
   {
      status = sensor_vl53lx_sat.VL53LX_GetMeasurementDataReady(&NewDataReady);
   } while (!NewDataReady);

   //Led on
   digitalWrite(LedPin, HIGH);

   if((!status)&&(NewDataReady!=0))
   {
      status = sensor_vl53lx_sat.VL53LX_GetMultiRangingData(pMultiRangingData);
      print_multi_ranging_data(pMultiRangingData);

      //VL53LX_zone_results_t  zone_results;
      //sensor_vl53lx_sat.V53L1_get_zone_results(&zone_results);
      //for (int z = 0; z < zone_results.active_zones; z++) {
      //   print_zone_objects(&zone_results.VL53LX_p_003[z]);
      //}

      SerialPort.println("");
      if (status==0)
      {
         status = sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }
   }

   digitalWrite(LedPin, LOW);
}

void loop_with_vl53l0x() {
  if (lox.isRangeComplete()) {
    Serial.print("Distance in mm: ");
    Serial.println(lox.readRange());
  }
}

void loop_with_vl53l0x_3() {
   delay(20);
   lox1.waitRangeComplete();
   lox2.waitRangeComplete();
   lox3.waitRangeComplete();
   
   Serial.print("LOX MM1 ");
   Serial.print(lox1.readRange());
   Serial.print(" MM2 ");
   Serial.print(lox2.readRange());
   Serial.print(" MM3 ");
   Serial.print(lox3.readRange());
   Serial.println("");
}

void loop_with_ble() {
   Serial.println("Waiting for central...");

   // listen for BLE peripherals to connect:
   BLEDevice central = BLE.central();
   // if a central is connected to peripheral:
   if (central) {
      Serial.print("Connected to central: ");
      // print the central's MAC address:
      Serial.println(central.address());
      
      // while the central is still connected to peripheral:
      int tick = 0;
      while (central.connected()) {
         memset(ble_characteristic_data_bytes, ' ', BLE_CHARACTERISTIC_DATA_LENGTH);
         ble_characteristic_data_bytes[BLE_CHARACTERISTIC_DATA_LENGTH] = 0;
         ble_characteristic_data_bytes[tick] = 'X';
         tick += 1;
         tick = tick % BLE_CHARACTERISTIC_DATA_LENGTH;
         ble_characteristic_data.writeValue(ble_characteristic_data_bytes, BLE_CHARACTERISTIC_DATA_LENGTH, false);

         memcpy(ble_characteristic_command_bytes, ble_characteristic_command.value(), BLE_CHARACTERISTIC_COMMAND_LENGTH);
         ble_characteristic_command_bytes[BLE_CHARACTERISTIC_COMMAND_LENGTH] = 0;
         Serial.print("tick: ");
         Serial.print(tick);
         Serial.print(" written: ");
         Serial.print(ble_characteristic_command.written());
         Serial.print(" command: '");
         Serial.print((const char*) ble_characteristic_command_bytes);
         Serial.println("'");
         
         delay(500);
      }

      // when the central disconnects, print it out:
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
   } else {
      delay(1000);
   }
}

#define JOY_X 17
#define JOY_Y 16
#define JOY_B 15

void setup_test_lines() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);
  pinMode(JOY_B, INPUT);

  Serial.println("Testing lines...");
}

char pinStatus(PinStatus s) {
   switch (s) {
   case LOW: return 'L';
   case HIGH: return 'H';
   case CHANGE: return 'C';
   case FALLING: return 'F';
   case RISING: return 'R';
   default: return 'U';
   }
}

void loop_test_lines() {
   for(;;) {
      int x = analogRead(JOY_X);
      int y = analogRead(JOY_Y);
      //char b = pinStatus(digitalRead(JOY_B));
      int b = analogRead(JOY_B);
      Serial.print("B: ");
      Serial.print(b);
      Serial.print(" X: ");
      Serial.print(x);
      Serial.print(" Y: ");
      Serial.print(y);
      Serial.println("");
      delay(100);
   }
}

void setup_bt() {
  Serial.begin(115200);
  while (! Serial) {
      delay(1);
  }
  Serial.println("Main serial OK");

  Serial1.begin(9600);
  //while (! Serial1) {
  //    delay(1);
  //}
  Serial.println("BT serial?");

  Serial1.write("AT+VERR?");
}

void loop_bt() {
   int from_main = Serial.read();
   if (from_main >= 0) {
      Serial1.write((char) from_main);
   }

   int from_bt = Serial1.read();
   if (from_bt >= 0) {
      Serial.write((char) from_bt);
   }
}

//#define IR_PIN 19
#define IR_PIN 15

void setup_ir() {
  Serial.begin(115200);
  while (! Serial) {
      delay(1);
  }
  Serial.println("Main serial OK");
  //ir_recv.enableIRIn();
  IrReceiver.begin(IR_PIN);
  Serial.println("IR receiver enabled");
  Serial.print("Protocols: ");
  printActiveIRProtocols(&Serial);
  Serial.println();
}

void loop_ir() {
  int skipped = 0;
  for(;;) {
    if (IrReceiver.decode()) {
      Serial.print("RECV: ");
      IrReceiver.printIRResultShort(&Serial);
      Serial.println();
      IrReceiver.resume();
    }

    skipped += 1;
    if (skipped > 100) {
      skipped = 0;
      Serial.println("RECV: NONE");
    }
    delay(100);
  }
}

void setup() {
   setup_ir();
   //setup_test_lines();
   //setup_display();
   //setup_bt();
   //setup_with_vl53l0x();
}
void loop() {
   loop_ir();
   //loop_test_lines();
   //loop_display();
   //loop_bt();
   //loop_with_vl53l0x();
}
