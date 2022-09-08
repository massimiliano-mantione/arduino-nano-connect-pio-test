#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM6DSOX.h>
#include <vl53lx_class.h>
#include <arducam.h>
#include "Adafruit_VL53L0X.h"

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

void setup() {
   setup_with_ble();
}
void loop() {
   loop_with_ble();
}
