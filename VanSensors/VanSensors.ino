
/*
 *  This version will only work with Arduino MKR1010
 *  
 */


/*
 *  Hardware
 *  ========
 *  
 *  Arduino:                        MKR1010           https://www.amazon.co.uk/Arduino-ARDUINO-MKR-WIFI-1010/dp/B07FYFF5YZ
 *  Internal Temp/Press/Hum/Gas:    Adafruit BME680   https://www.amazon.co.uk/Waveshare-1-5inch-OLED-Module-Communicating/dp/B079NNZ9V1
 *  External Temp/Hum:              SHT30             https://thepihut.com/products/sht-30-mesh-protected-weather-proof-temperature-humidity-sensor
 *  GPS:                            Neo-6M            https://www.amazon.co.uk/AZDelivery-GPS-GY-GPS6MV2-Antenna-including/dp/B01N38EMBF/ref=sr_1_1_sspa?dchild=1&keywords=neo-6m&qid=1591250133&s=electronics&sr=1-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUExTU00Q1NMMUxQVkNQJmVuY3J5cHRlZElkPUEwMTQ0NDE5MTNOWktUSlQ0MU5SUSZlbmNyeXB0ZWRBZElkPUEwNzA0MjYxMkNPN0JLQUU5UjI3OCZ3aWRnZXROYW1lPXNwX2F0ZiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU=
 *  GPS Aerial:                     Generic           https://www.amazon.co.uk/Waterproof-Active-Antenna-28dB-Gain-Black/dp/B00LXRQY9A/ref=sr_1_4?crid=2B7WKPX8682G2&dchild=1&keywords=gps+antenna+sma&qid=1591250167&s=electronics&sprefix=gps+ante%2Celectronics%2C177&sr=1-4
 *  
 */
 
/*
 *  To do:
 *  
 *  Tidy up OLED after voltage cal
 *  SB cal
 *  128bit UUIDs
 *  Field test
 *  
 */
#if defined (ARDUINO_SAMD_MKRWIFI1010)   
  #include "config.h"           // user info
  #include "vanSensors.h"       // common data structures with peripherals
  
  //Libraries
  #include <SPI.h>                  // SPI interface library for BME680
  #include <U8x8lib.h>              // OLED display
  #include <WiFiNINA.h>             // WiFi driver **** NOTE this needs to have the modified WiFi.cpp to allow BLE to work with WiFi//
  #include <PubSubClient.h>         // MQTT interface
  #include <FuGPS.h>                // GPS library
  #include "Adafruit_SHT31.h"       // SHT30 sensor
  #include <Wire.h>                 // I2C interface library for SHT31 and OLED
  #include "Adafruit_BME680.h"      // BME680 sensor
  #include <ArduinoBLE.h>           // Bluetooth LE
  #include <Adafruit_SleepyDog.h>   // Reset tools
  
  #define IO_CYCLE              10000
  #define FAN_TEMP              30
  #define pinFan                5
  
  #define CONNECTION_ATTEMPTS 3
  #define BLE_SCAN_TIMEOUT      10000   // wait 10 seconds to find any given Service UUID
  #define BLE_ATTR_TIMEOUT       5000   // wait  5 seconds to find BLE attributes
  #define BLE_RESCAN_INT       120000   // wait  2 minutes to look for a peripheral if it isn't found
  #define BLE_SCAN_ATTEMPTS         3
  
  #include <avr/dtostrf.h>
  #include <FlashAsEEPROM.h>
  
  //Flash Memory addresses for saving defaults
  #define addrSsid  0x0000
  #define addrPwd   0x0021
  #define addrLbCal 0x0060
  #define addrSbCal 0x0080
  
  //SPI pin definition for the BME680 sesnsor
  #define BME_SCK             10 // Orange  
  #define BME_MISO             9 // Yellow
  #define BME_MOSI             8 // Green
  #define BME_CS               7 // Blue
  
  //Leisure Battery ADC
  #define VOLTDIVRATIO        0.017584      // defuault conversion from ADC reading to volts (depends on voltage divider used)
  #define PIN_LB              A0  
  #define ADC_COUNT           500           // take 500 voltage readings to get a decent average
  
  // Battery Auto-Cal settings comment these two lines out to turn off auto-cal
  #define BATTERY_CAPACITY    260           
  #define BMS_CUTOFF          14.4
  #define LBVM_MIN            0.8
  #define LBVM_MAX            1.2
  
  
  //Bluetooth LE Services for setting up WiFi and/or calibrating voltage
  BLEService vanCentralService("4A00");
  BLEUnsignedIntCharacteristic adcCalChar("4A01", BLERead | BLEWrite);
  BLECharacteristic ssidChar("4A02", BLERead | BLEWrite, "DefaultSSID"); 
  BLECharacteristic pwdChar("4A02", BLERead | BLEWrite, "12345678901234567890"); 
  BLEDescriptor adcCalDesc("2901", "ADC Voltage Cal");
  BLEDescriptor ssidDesc("2901", "Van SSID");
  BLEDescriptor pwdDesc("2901", "Van SSID");
   
  // rows and columns for the OLED display
  #define OLED_RCOL          11
  #define OLED_RSSI           3
  #define OLED_MQTT           4
  #define OLED_LAT            5
  #define OLED_LON            6
  #define OLED_LB             7
  #define OLED_SB             8
  #define OLED_TEMP           9
  #define OLED_HUM           10
  #define OLED_ETEMP         11 
  #define OLED_EHUM          12 
  #define OLED_PRESS         13
  #define OLED_BAR           15
  
  // Neo-6M GPS
  #define NEO_6M_TIMEOUT    30000
  
  // Settings for manual voltage calibration
  #define pinCal                6
  #define CAL_TIMEOUT       60000
  
  //BME680 parameters
  #define BME_GAS_TEMP      320  // 320*C for 150 ms
  #define BME_GAS_TIME      150  //
  #define SEALEVELPRESSURE_HPA (1013.25)
  
  
  WiFiClient                        wifiClient;
  PubSubClient                      mqttClient;
  Adafruit_SHT31                    sht =  Adafruit_SHT31();
  FuGPS                             fuGPS(Serial1);
  U8X8_SSD1327_WS_128X128_HW_I2C    oled(U8X8_PIN_NONE);
  Adafruit_BME680                   bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
  
  bool    boolBME680;
  bool    boolSHT31;
  bool    boolGPS;
  bool    boolTank;
  bool    gpsFix;
  
  //Default to home
  float   lat =  DEFAULT_LAT;
  float   lon =  DEFAULT_LON;
  float   alt =  DEFAULT_ALT;
  
  char    ssid[32];
  char    pwd[64];
  
  char    default_ssid[] = WLAN_SSID;
  char    default_pwd[] = WLAN_PASS;
  
  float   lbvMult = 1.0;
  float   sbvMult = 1.0;
  bool    runCal = false;
  
  bool    lastRelay;
  long    chargeRelayCount;

  int     mqttLogLen;
  String  mqttLog;
  
  #if defined(BATTERY_CAPACITY)
    float ampHours = BATTERY_CAPACITY;
    long lastAmpHourCalc = millis();
  #endif
  
  long bleNotFoundTime[SERVICES];
  int  bleScanCount[SERVICES];
  
  
  
  // callback routine for MQTT (not used)
  void callback(char* topic, byte* payload, unsigned int length) {
    // handle message arrived
  }
  
  
  
  void setup() {
    Serial.begin(115200);
    delay(2000);
    // wait for serial monitor to open
  
    Serial.println();
    for (int j = 0; j< 80; j++){
      Serial.print("=");
    }
    Serial.println();
    for (int j = 0; j< 38; j++){
      Serial.print(" ");
    }
    Serial.println("SETUP");
    minDiv();

    mqttLogPrint("--------------------------------");
    mqttLogPrint("            SETUP");
    mqttLogPrint("--------------------------------");
    
    if (oled.begin()){
      Serial.println("OLED OK");    
      oled.setFont(u8x8_font_amstrad_cpc_extended_f);   
      oledHeader();
    } else {
      Serial.println("OLED Fail!"); 
    }
  
    pinMode(pinFan, OUTPUT);
    digitalWrite(pinFan, LOW);
  
  //Set up interrupt for calibration
    pinMode(pinCal, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinCal), calCheck, FALLING);  
  
  /*  Serial.println(UPDATED_WIFI_CPP);
    #if (!defined(UPDATED_WIFI_CPP))
      Serial.println("Updated WiFi.cpp required!");
      oled.println("************\n*  Updated  *\n* WiFi.cpp  *\n* required! *\n*************");
      while(1){};
    #endif */
    
    readFlash();
    
    Serial1.begin(9600);
    delay(1000);
    fuGPS.sendCommand(FUGPS_PMTK_API_SET_NMEA_OUTPUT_RMCGGA);
    Serial.print("Neo-6M GPS ");
    oled.print("Neo-6M: ");
    if(boolGPS = checkGPS()){
      Serial.println("found");
      oled.println(("ok"));
      mqttLogPrint("  NEO-6M GPS    ok");
    } else {
      Serial.println("not found");
      oled.println(("fail"));
      mqttLogPrint("  NEO-6M GPS    fail");
    }
    // *** BME680 (Internal Sensor) ***
    Serial.print(("BME680 sensor "));
    oled.print(("BME680: "));
    boolBME680 = bme.begin();
    if (!boolBME680) {
      Serial.print(("not "));
      oled.println(("fail"));
      mqttLogPrint("  BME680 sensor ok");
    } else {
      bme.setTemperatureOversampling(BME680_OS_8X);
      bme.setHumidityOversampling(BME680_OS_2X);
      bme.setPressureOversampling(BME680_OS_4X);
      bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      bme.setGasHeater(BME_GAS_TEMP, BME_GAS_TIME);
      oled.println(("ok"));
      mqttLogPrint("  BME680 sensor ok");
    }
    Serial.println(("found."));
    
    // *** SHT31 (External Sensor) ****
    Serial.print(("SHT31 sensor "));
    oled.print(("SHT31:  "));
    boolSHT31 = sht.begin(0x44);// Set to 0x45 for alternate i2c addr
    if (!boolSHT31) {   
      Serial.print(("not "));
      oled.println(("fail"));
      mqttLogPrint("  SHT31  sensor fail");
    } else {
      mqttLogPrint("  SHT31  sensor ok");
      oled.println(("ok"));
    }      
    
    Serial.println(("found."));  
    
    delay(5000);
    
    oledTemplate();
    WiFi_connect();
  }
  
  
  void loop() {
    mqttLogPrint("--------------------------------");
    Serial.println();
    for (int j = 0; j< 80; j++){
      Serial.print("=");
    }
    Serial.println();
    for (int j = 0; j< 38; j++){
      Serial.print(" ");
    }
    Serial.println("LOOP");
    
    if (boolGPS){
      minDiv();
      Serial.print("GPS: ");
      gpsFix = getGPS(lat, lon, alt);
      if (gpsFix){
        Serial.print("Lat: ");
        Serial.print(lat, 6);
        Serial.print("  Lon: ");
        Serial.print(lon, 6);
        Serial.print(" Alt: ");
        Serial.println(alt, 2);    
     
        oledXYprint(OLED_LAT, OLED_RCOL, lat, 2);
        oledXYprint(OLED_LON, OLED_RCOL, lon, 2); 
      } else {
        Serial.println("No GPS Fix"); 
      } 
    }
    
    float r = WiFi.RSSI();
    oledXYprint(OLED_RSSI, OLED_RCOL, r, 0);
    saveSensorVal(SENSOR_IDX_RSSI, r);
  
    if (boolSHT31){
      minDiv();
      Serial.println("SHT31 Data");
      float t = sht.readTemperature();
      float h = sht.readHumidity();
        
      if (! isnan(t)) {  // check if 'is not a number'
        Serial.print(("  Ext Temp:   "));
        Serial.print(t);
        Serial.println(("*C. "));
        
        if (t >= MIN_TEMP && t <= MAX_TEMP){
          oledXYprint(OLED_ETEMP, OLED_RCOL, t, 1);
          saveSensorVal(SENSOR_IDX_EXT_TEMP, t);
        } 
      }    
      if (! isnan(h)) {  // check if 'is not a number'
        Serial.print(("  Ext Hum:    "));
        Serial.print(h);
        Serial.println(("%.    ")); 
         
        if (h >= MIN_HUM && h <= MAX_HUM){
          oledXYprint(OLED_EHUM , OLED_RCOL, h, 1);
          saveSensorVal(SENSOR_IDX_EXT_HUMIDITY, h);
        } 
      }      
    }
  
    if (boolBME680){
      Serial.println("BME680 Data");
      if (bme.performReading()){   
        float t = bme.temperature;
        digitalWrite(pinFan,(t > FAN_TEMP));
        Serial.print(("  Int Temp:   "));
        Serial.print(t);
        Serial.println(("*C. "));
        if (t >= MIN_TEMP && t <= MAX_TEMP){
          oledXYprint(OLED_TEMP, OLED_RCOL, t, 1);
          saveSensorVal(SENSOR_IDX_INT_TEMP, t);
        } 
        
        Serial.print(("  Int Hum:    "));
        Serial.print(bme.humidity);
        Serial.println(("%.    "));
        if (bme.humidity >= MIN_HUM && bme.humidity <= MAX_HUM){
          oledXYprint(OLED_HUM, OLED_RCOL, bme.humidity, 1);
          saveSensorVal(SENSOR_IDX_INT_HUMIDITY, bme.humidity);
        }
        
        Serial.print(("  Pressure: "));
        Serial.print(bme.pressure / 100.0);
        Serial.println((" hPa.        "));
        if (bme.pressure / 100 >= MIN_PRESS && bme.pressure / 100 <= MAX_PRESS){
          oledXYprint( OLED_PRESS, OLED_RCOL,bme.pressure / 100.0, 0);
          saveSensorVal(SENSOR_IDX_AIR_PRESSURE, bme.pressure / 100.0);
        } 
      } 
    }  
  
/*    float var; 
    for (int i = 0; i < ADC_COUNT; i++){
      var = var + analogRead(PIN_LB) *  VOLTDIVRATIO / ADC_COUNT * lbvMult;  
    }
    minDiv();
    Serial.println("ADC Data:");
    Serial.print(("  LB Voltage:  "));
    Serial.print(var);
    Serial.println(("V.           "));
  
    if (var >= MIN_VOLT && var <= MAX_VOLT){
     oledXYprint(OLED_LB, OLED_RCOL, var, 1);
    } 
  
    saveSensorVal(SENSOR_IDX_LEISURE_BATT, var);  */
  
    minDiv();
    WiFi_disconnect();
  
    minDiv();
    Serial.print("\nStarting BLE: ");
    if (BLE.begin()){
      Serial.println("OK");
      bool uuid_scanned[SENSORS];
      for (int i = 0; i < SERVICES; i++){
        Serial.print("  ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(uuidServ[i]);
        Serial.print(" {");
        unsigned long startLoop = millis();
        Serial.print(startLoop - bleNotFoundTime[i]);
        Serial.print("} ");
        if ((bleNotFoundTime[i] == 0) || ((startLoop - bleNotFoundTime[i]) > BLE_RESCAN_INT)){
          Serial.print("  Scanning for UUID[");
          Serial.print(bleScanCount[i]);
          Serial.print("]: ");
          if (bleScanCount[i] == BLE_SCAN_ATTEMPTS){
            bleScanCount[i] = 0;
          }
          if (bleNotFoundTime[i] > 0){
            bleNotFoundTime[i] = 0;    
            mqttLogPrint("UUID " + String(uuidServ[i]) + " [" + String(bleScanCount[i]) + "]: Re-Scan");
          }
          oled.setCursor(0,OLED_BAR);
          oled.print("BLE ");
          oled.print(uuidServ[i]);
          oled.print(":     ");
          oled.setCursor(10,OLED_BAR);
          BLE.stopScan();
          BLE.scanForUuid(uuidServ[i]);
          bool p = false;
          while ((millis() - startLoop) < BLE_SCAN_TIMEOUT && !p){
            BLEDevice peripheral = BLE.available();
            if (peripheral) {
              p = true;
              bleNotFoundTime[i] = 0;
              // discovered a peripheral, print out address, local name, and advertised service
              Serial.print("Found ");
              oled.print("OK (");
              Serial.print(peripheral.address());
              Serial.print(" in ");
              Serial.print(millis() - startLoop);
              Serial.print("ms. ");
              Serial.print(" '");
              Serial.print(peripheral.localName());
              Serial.println("' ");
              Serial.print("    Connecting            : ");
              BLE.stopScan();                
              long startconn = millis();
              if (peripheral.connect()){
                Serial.println("OK");
                // discover peripheral attributes
                Serial.print("    Discovering attributes: ");
                long startDisc = millis();
                while (!peripheral.discoverAttributes() && ((millis() - startDisc) < BLE_ATTR_TIMEOUT)){
                 
                }
                if (peripheral.discoverAttributes()) {
                  mqttLogPrint("UUID " + String(uuidServ[i]) + " [" + String(bleScanCount[i]) + "]: OK");
                  Serial.println("OK");
                  for (int j = 0; j < SENSORS; j++){
                    if(vanSensors[j].uuid_s == uuidServ[i]){
                       if(peripheral.characteristic(vanSensors[j].uuid_c)){
                        Serial.print("    Found characteristic:   ");
                        Serial.print(vanSensors[j].uuid_c);
                        Serial.print(". Value: ");
                        peripheral.characteristic(vanSensors[j].uuid_c).readValue(vanSensors[j].val);
                        Serial.println(vanSensors[j].val);
                        vanSensors[j].updated=(vanSensors[j].val != 0);
                      }
                    }
                  }
                } else {
                 mqttLogPrint("UUID " + String(uuidServ[i]) + " [" + String(bleScanCount[i]) + "]: Discovery fail");
                 Serial.println("Fail");
                }
                Serial.print("    Disconnecting from ");
                Serial.println(peripheral.address());
                peripheral.disconnect();                             
              } else {
                mqttLogPrint("UUID " + String(uuidServ[i]) + " [" + String(bleScanCount[i]) + "]: Connect fail");
                Serial.println("Fail");
              }
            } 
          } 
          if (!p){
            oled.setCursor(10,OLED_BAR);
            oled.println("fail");
            Serial.print("Not found.");
            mqttLogPrint("UUID " + String(uuidServ[i]) + " [" + String(bleScanCount[i]) + "]: Scan fail");
            bleScanCount[i]++;
            if (bleScanCount[i] == BLE_SCAN_ATTEMPTS){
              bleNotFoundTime[i] = millis();
              Serial.print("bleNotFound[");
              Serial.print(i);
              Serial.print("] Set to ");
              Serial.print(bleNotFoundTime[i]);
              Serial.println("mS");
            } else {
              Serial.println();
            }            
            delay(500);
          }
        } else {
          mqttLogPrint("UUID " + String(uuidServ[i]) + " [" + String(bleScanCount[i]) + "]: Skipped");
          Serial.println(" Skipped");
        }
      }
      Serial.println("BLE Disconnect..");
      BLE.disconnect();
      oled.setCursor(0,OLED_BAR);
      oled.print("               ");
      Serial.println("BLE End..");
      BLE.end();  
    } else {
      mqttLogPrint("BLE Fail");
      Serial.println("Fail");
    }   
  
    if(runCal){
      minDiv();
      calibrateVoltage();
      runCal = false;
    }
  
    minDiv();
    WiFi_connect();
  
  // Auto-Cal logic
    #if defined (BMS_CUTOFF)
      bool  currentRelay = (readSensorVal(SENSOR_IDX_RELAY) > 0x01ff);
      if (!currentRelay && lastRelay  && (chargeRelayCount > 1)){
        minDiv();
        mqttLogPrint("Charge Relay Cutoff detected");

        Serial.println("\nCharge Relay cutoff detected:");
        Serial.println(  "-----------------------------");
        Serial.print("  BMS Cutoff Voltage   : ");
        Serial.print(BMS_CUTOFF);
        Serial.println("V");
        Serial.print("  Last Measured Voltage: ");
        Serial.print(readSensorVal(SENSOR_IDX_LEISURE_BATT));
        Serial.println("V");
        if (readSensorVal(SENSOR_IDX_LEISURE_BATT) > -999){
          float calcLbvM = lbvMult * BMS_CUTOFF / readSensorVal(SENSOR_IDX_LEISURE_BATT);
          if ((calcLbvM > LBVM_MIN) && (calcLbvM  < LBVM_MAX)){
            lbvMult = calcLbvM;
            Serial.print("  New Voltage Mult     : ");
            Serial.println(lbvMult);
            char buf[10];
            dtostrf(lbvMult,10,8,buf);
            for (int i = 0; i < 10; i++){
              EEPROM.write(addrLbCal + i, buf[i]);
            }
            EEPROM.write(addrLbCal + 10,0);
            Serial.print("  Committing changes to Flash Memory:");
            EEPROM.commit();
            if (EEPROM.isValid()){
              Serial.println("OK");            
            } else {
              Serial.println("Fail");            
            }
            mqttLogPrint("New LB Vmult:" + String(buf));
            mqttLogPrint("LB SOC reset to 100%");
          }
        }      
        ampHours = BATTERY_CAPACITY;  
      }  
      chargeRelayCount += (readSensorVal(SENSOR_IDX_RELAY) > -999);
      lastRelay = currentRelay;
  
      if (readSensorVal(SENSOR_IDX_CURRENT) > -999){
        minDiv();
        Serial.println("\nBattery Capacity Calcs:");
        Serial.println("-----------------------");
        Serial.print("  Old Capacity     :");
        Serial.print(ampHours);
        Serial.println(" Ah");
        Serial.print("  Current     :");
        Serial.print(readSensorVal(SENSOR_IDX_CURRENT));
        Serial.print(" A x ");
        Serial.print((millis() - lastAmpHourCalc) / 1000);
        Serial.print(" seconds = ");
        long millisNow = millis();
        Serial.print(readSensorVal(SENSOR_IDX_CURRENT) * (millisNow - lastAmpHourCalc) / 3600000);
        Serial.println(" Ah");
        ampHours = ampHours + readSensorVal(SENSOR_IDX_CURRENT) * (millisNow - lastAmpHourCalc) / 3600000;
        Serial.print("  New Capacity     :");
        Serial.print(ampHours);
        Serial.println(" Ah");
        saveSensorVal(SENSOR_IDX_LB_SOC, ampHours / BATTERY_CAPACITY * 100);
        lastAmpHourCalc = millisNow;
      }
    #endif
    
    if(MQTT_connect()){
      minDiv();
      Serial.println("Sending MQTT Data:");
      for (int i = 0; i < SENSORS; i++){
        if (vanSensors[i].updated){
          Serial.print("  ");
          Serial.print(vanSensors[i].feedname);
          Serial.print(": ");
          Serial.print(readSensorVal(i));
          vanSensors[i].updated = !(sendMqtt(vanSensors[i].feedname, readSensorVal(i)));
          delay(1000); 
        }
      }
    }
      
  /*  oled.setCursor(0, OLED_BAR);
    oled.print("OOOOOOOOO");
    for (int lcol = 9; lcol >= 0; lcol--){    
      for (int i = 0; i<IO_CYCLE/10000; i++){
        delay(500);
        oled.setCursor(lcol, OLED_BAR);
        oled.print("O");
        delay(500);
        oled.setCursor(lcol, OLED_BAR);
        oled.print(" ");
      }
    } */
  }
  
  bool sendMqtt(const char* f, float v){
    char buf[120];
    char *p = buf;
  
    strcpy(p, f);
    p += strlen(f);
    p[0] = ',';
    p++;
  
    dtostrf(v, 10, 8, p); 
    p += strlen(p);
    
    strcat(p, "\nlocation,");
    p += strlen(p);
    
    dtostrf(lat, 10, 8, p); 
    p += strlen(p);
    p[0] = ',';
    p++;
    dtostrf(lon, 10, 8, p); 
    p += strlen(p);
    p[0] = ',';
    p++;
    dtostrf(alt, 10, 8, p); 
    p += strlen(p);
    p[0] = 0;
  
    if(mqttClient.publish(AIO_GROUP ,buf)){
      Serial.println(" - OK");  
    } else {
      Serial.println(" - Fail");      
    }
  }
  
  void oledHeader(){
    oled.clear();
    oled.print(("VanSensors v"));    //0
    char vers[3] ; 
    dtostrf(float(VERSION),3,1,vers);
    oled.println(vers);    //0
    oled.println("---------------\n");  //1-2
  }
  
  void oledTemplate(){
    oledHeader();
    oled.println(("RSSI:       dB"));     //3
    oled.println( "MQTT:")           ;     //4
    oled.println(("Lat:        *N"));     //5
    oled.println(("Lon:        *E"));     //6
    oled.println(("LB:         v"));      //7
    oled.println(("SB:         v"));      //8 
    oled.println(("iTemp:      *C"));     //9
    oled.println(("iHum:       %"));      //10
    oled.println(("eTemp:      *C"));     //11
    oled.println(("eHum:       %"));      //12
    oled.println(("Press:      hPa"));    //13 
  }
  
  
  
  bool getGPS(float &lat, float &lon, float &alt){
    int iGPS = 10;
    int iValid = 5;
    
    #if defined(ARDUINO_SAMD_MKRWIFI1010) || !defined(DEBUG_BLE)
      while(iValid > 0 && iGPS > 0){
        bool gpsAlive;
        if (fuGPS.read()){
          iGPS--;
          gpsAlive = true;
          
          
          if (fuGPS.hasFix()){
     //       acc = fuGPS.Accuracy;
            alt = fuGPS.Altitude;
            lat = fuGPS.Latitude;
            lon = fuGPS.Longitude;
            iValid--;
          } 
        } 
        gpsAlive = fuGPS.isAlive();
      }
    #endif
    return(iValid == 0);
  }
  
  
  void oledXYprint(int row, int rcol, float val, int dec){  
    int lcol = log(abs(val))/log(10);
  
    if (lcol > 0){lcol++;} else {lcol = 1;} 
    if (val < 0){lcol++;}
    if (dec > 0){lcol++;}
    lcol = rcol - lcol - dec;
  
    oled.setCursor(lcol, row);
    if (dec == 0){
      int ival = val;
      oled.print(ival);
    } else {
      oled.print(val, dec);
    }
  }
  
  
  
  bool MQTT_connect() {
    bool ret;
    
    Serial.print("Connecting to MQTT: ");
    oled.setCursor(6, OLED_MQTT);
    oled.print("Connect");
  
  //  WiFiClient wifiClient;
    mqttClient.setClient(wifiClient);
    mqttClient.setServer(AIO_SERVER, AIO_SERVERPORT);  
    mqttClient.setCallback(callback);  
    
    uint8_t retries = 3;
    while (!mqttClient.connect(AIO_GUID,  AIO_USERNAME, AIO_KEY) and retries > 0){  
      oled.setCursor(6, OLED_MQTT);
      oled.print("Error ");
      oled.print(mqttClient.state());
      Serial.print("  MQTT status: ");
      Serial.print(mqttClient.state());
      Serial.print(".  WiFi Client status: ");
      Serial.println(wifiClient.status());    
      delay(5000);  // wait 5 seconds
      retries--;
    }
    ret = mqttClient.connected();
    if (ret){
      Serial.println("OK");
      oled.setCursor(6, OLED_MQTT);
      oled.print("   OK    ");
      mqttLogPub();
    } else {
      Serial.println("Fail");
      oled.setCursor(6, OLED_MQTT);
      oled.print(" FAIL ");
    }
    return ret;
  }
  
  void WiFi_disconnect(){
    int status = WL_IDLE_STATUS;     // the WiFi radio's status
  
  //  Serial.println("Shutting down MQTT and WiFi:");
    Serial.print("WiFi/MQTT Disconnect: ");
    Serial.print("  MQTT Disconnect: ");
    if (mqttClient.connected()){
      mqttClient.disconnect();
    }
    if (!mqttClient.connected()){
      Serial.println("OK");
    } else {
      Serial.println("Fail");
    }
    
    Serial.print("  WiFi Client Stop: ");
    wifiClient.stop();
    wifiClient.flush();
    if (!wifiClient.connected()){
      Serial.println("OK");
    } else {
      Serial.println("Fail");
    }
    
    Serial.println("  WiFi Disconnect:  ");
    WiFi.disconnect();
    Serial.print("  Status: ");
    status = WiFi.status();
    Serial.print(status);
    Serial.print(" : ");
    printWiFiStatusCode(status);
    Serial.println();
    Serial.print("  WiFi End:         ");
    WiFi.disconnect();
    Serial.print("  Status: ");
    status = WiFi.status();
    Serial.print(status);
    Serial.print(" : ");
    printWiFiStatusCode(status);
    if (WiFi.status() == WL_DISCONNECTED){
      Serial.println("  OK");
    } else {
      Serial.println("  Fail");
    }
  }
  
  void WiFi_connect(){
    int status = WL_IDLE_STATUS;     // the WiFi radio's status
  
    Serial.print("Connecting to ");
    Serial.println(ssid);
    
  
  //  WiFi.setTimeout(2000);
    int retries = CONNECTION_ATTEMPTS;
    status = WiFi.begin(ssid, pwd);
    while (status != WL_CONNECTED) {     
      Serial.print("  Status: ");
      Serial.print(status);
      Serial.print(" : ");
      printWiFiStatusCode(status);
      Serial.println();
      oled.setCursor(0, OLED_RSSI);
      oled.print("WiFi Status: ");
      oledXYprint(OLED_RSSI, OLED_RCOL+3, status, 0);
      delay(1000);
      status = WiFi.begin(ssid, pwd);
      retries--;
      if (retries == 0){
        getWiFi_from_BLE();
        retries = CONNECTION_ATTEMPTS;
        status = WiFi.begin(ssid, pwd);
      }
    }
  
    Serial.print("  Status: ");
    Serial.print(status);
    Serial.print(" : ");
    printWiFiStatusCode(status);
    Serial.print("  IP address: "); 
    Serial.println(WiFi.localIP());
  
    oled.setCursor(0, OLED_RSSI);
    oled.println(("RSSI:       dB"));     //3
    oledXYprint(OLED_RSSI, OLED_RCOL, WiFi.RSSI(), 0);
  }
  
  void printWiFiStatusCode(int i){
    switch(i){  
      case 0:
        Serial.print("WL_IDLE_STATUS    ");   
        break;      
      case 1:
        Serial.print("WL_NO_SSID_AVAIL  ");   
        break;      
      case 2:
        Serial.print("WL_SCAN_COMPLETED ");   
        break; 
      case 3:
        Serial.print("WL_CONNECTED      ");   
        break;    
      case 4:
        Serial.print("WL_CONNECT_FAILED ");   
        break;      
      case 5:
        Serial.print("WL_CONNECTION_LOST");   
        break;
      case 6:
        Serial.print("WL_DISCONNECTED   ");   
        break;
      default:
        Serial.print("WL_NO_SHIELD      ");        
    }    
  }
  
  bool checkGPS(){
    bool gpsAlive = false;
    long startTest = millis();
  
    while (!gpsAlive && (millis()-startTest < NEO_6M_TIMEOUT)){
      gpsAlive = (fuGPS.read());
      if (!fuGPS.isAlive() && gpsAlive){
          gpsAlive = false;
      }
    }
    return gpsAlive;
  }
  
  void minDiv(){
    Serial.println();
    for (int j = 0; j< 80; j++){
      Serial.print("-");
    }
    Serial.println();
  }
  
  void readFlash(){
    char lbvM[11] = "1.00000000";
    char sbvM[11] = "1.00000000";
    
    if(EEPROM.isValid()){
      int l = 0;
      Serial.print("Saved SSID:  ");
      for (int i = 0; i < 32; i++){
        ssid[i] = (EEPROM.read(addrSsid + i));
        l = i;
        if (ssid[i] == 0){
          break;
        }
      }
      Serial.println(ssid);
      oled.print("\nSSID:   ");
      oled.println(ssid);
  
      l = 0;
      Serial.print("Saved Password:");
      for (int i = 0; i < 64; i++){
        pwd[i] = (EEPROM.read(addrPwd + i));
        l = i;
        if (pwd[i] == 0){
          break;
        }
      }
      Serial.println(pwd);
                  
      l = 0;
      Serial.print("Saved lbvMult:");
      for (int i = 0; i < 10; i++){
        lbvM[i] = (EEPROM.read(addrLbCal + i));
        l = i;
        if (lbvM[i] == 0){
          break;
        }
      }
      Serial.print(lbvM);
      Serial.print(" = ");
      lbvMult = atof(lbvM);
      dtostrf(lbvMult,7,5,lbvM);
      Serial.println(lbvMult);                
      oled.print("LB Cal: ");
      oled.println(lbvM);
      oled.println();
      mqttLogPrint("Flash SSID:" + String(ssid));
      mqttLogPrint("Flash LB Cal:" + String(lbvM));

    } else {
      Serial.println("No Saved Data in Flash Memory");
      Serial.println("Using defaults:");
      oled.println("\nNo data saved\nin Flash\n\nUsing defaults\n");
      
      int l = 0;
      Serial.print("  SSID:      ");
      for (int i = 0; i < 32; i++){
        ssid[i] = default_ssid[i];
        Serial.print(ssid[i]);
        EEPROM.write(addrSsid + i,ssid[i]);
        l = i;
        if (ssid[i] == 0){
          break;
        }
      }
      Serial.println();
      ssid[l] = 0;
          
      l = 0;
      Serial.print("  Password:  ");
      for (int i = 0; i < 64; i++){
        pwd[i] = default_pwd[i];
        if ((i == 0) || (default_pwd[i + 1]==0) || (default_pwd[i]==0)){
          Serial.print(pwd[i]);
        } else {
          Serial.print("*");
        }
        EEPROM.write(addrPwd + i,pwd[i]);
        l = i;
        if (pwd[i] == 0){
          break;
        }
      }
      Serial.println();
      pwd[l] = 0;
  
      l = 0;
      Serial.print("  lbvMult:   ");
      for (int i = 0; i < 10; i++){
        Serial.print(lbvM[i]);
        EEPROM.write(addrLbCal + i,lbvM[i]);
      }
      Serial.println();        
      EEPROM.write(addrLbCal + 10, 0);
  
      Serial.print("  Saving to Flash: ");
      EEPROM.commit();
      if (EEPROM.isValid()){
        Serial.println("OK");
      } else {
        Serial.println("Fail");      
      }
    }  
  }
  
  void getWiFi_from_BLE(){
    BLEService vanWiFiService("4A00");
    BLECharacteristic ssidChar("4A01", BLERead | BLEWrite, "12345678901234567890123456789012"); 
    BLECharacteristic pwdChar("4A02", BLEWrite, "1234567890123456789012345678901234567890123456789012345678901234"); 
    BLEDescriptor ssidDesc("2901", "Van WiFi SSID");
    BLEDescriptor pwdDesc("2901", "Van WiFi Password");  
  
    WiFi.end();
  
    Serial.println("\nBluetooth LE");
    Serial.println("============");
    Serial.println();
    
    if (!BLE.begin()) {
      Serial.println("starting BLE failed!");
      oled.println("BLE fail");
      int countdownMS = Watchdog.enable(4000);
      } else {
      Serial.print("BLE started with address ");
      Serial.println(BLE.address().c_str());
    
      Serial.print("Setting Device Name to ");
      String strName = String("Van:" + BLE.address().substring(0,5));
      Serial.println(strName);
  
      oledHeader();
      oled.println("\nUnable to\nconnect to WiFi\n\nUse BLE to set\nSSID & Password\n\n\nDevice Name:\n------------\n  ");
      oled.println(strName);
    
      BLE.setDeviceName(strName.c_str());
      BLE.setLocalName(strName.c_str());
    
      Serial.println(); 
    
      BLE.setAdvertisedService(vanWiFiService);          // add the service UUID
      vanWiFiService.addCharacteristic(ssidChar);  // add the battery level characteristic
      vanWiFiService.addCharacteristic(pwdChar);  // add the battery level characteristic 
      ssidChar.addDescriptor(ssidDesc);
      pwdChar.addDescriptor(pwdDesc); 
    
      BLE.addService(vanWiFiService);                     // Add the battery service 
      BLE.advertise(); 
      
      bool burnFlash = false;
      long startTime = millis();
      
      while((!burnFlash) && (millis() < startTime + CAL_TIMEOUT)){
        BLEDevice central = BLE.central();
      
        if (central) {      
          Serial.print("Connected to central: ");
          // print the central's BT address:
          Serial.println(central.address());
          // turn on the LED to indicate the connection:
          digitalWrite(LED_BUILTIN, HIGH);
        
          while (central.connected()){
            central.poll();
            if (ssidChar.written()){
              Serial.print("New SSID:    ");
              if (!burnFlash){oled.clear();};
              oled.println("New SSID:");
      
              int l = ssidChar.valueLength();
              char buf[l];
              ssidChar.readValue(buf, l);
              Serial.println(buf);
              oled.println(buf);
              oled.println();
              for (int i = 0; i< l; i++){
                EEPROM.write(addrSsid + i, buf[i]);
                ssid[i] = buf[i];
              }        
              burnFlash = true;    
              EEPROM.write(addrSsid + l, 0);
              ssid[l] = 0;
            }
      
            if (pwdChar.written()){
              Serial.print("New Password:");
              if (!burnFlash){oled.clear();};
              oled.println("New password:");
      
              int l = pwdChar.valueLength();
              char buf[l];
              pwdChar.readValue(buf, l);
              Serial.println(buf);
              oled.println(buf);
              oled.println();
              for (int i = 0; i< l; i++){
                EEPROM.write(addrPwd + i, buf[i]);
                pwd[i] = buf[i];
              }        
              burnFlash = true;    
              EEPROM.write(addrPwd + l, 0);
              pwd[l] = 0;
            }
      
          }
          Serial.print("Disconnected from central: ");
          Serial.println(central.address());
          if (burnFlash){
            Serial.print("Committing changes to Flash Memory:");
            EEPROM.commit();
            if (EEPROM.isValid()){
              Serial.println("OK");            
            } else {
              Serial.println("Fail");            
            }
          }  
          BLE.disconnect();
          BLE.end();
          Serial.println("Connecting to WiFi:\n");  
    
        }
      }
    }
    oledTemplate();
  }
  
  void calibrateVoltage(){
    char sbvC[5];  
    char lbvC[5];
  
    float lbvF = readSensorVal(SENSOR_IDX_LEISURE_BATT);
    float sbvF = readSensorVal(SENSOR_IDX_STARTER_BATT);
    dtostrf(lbvF, 4, 1, lbvC); 
    dtostrf(sbvF, 4, 1, sbvC); 
  
    Serial.print("LB: ");
    Serial.print(lbvC);
    Serial.print(" = ");
    Serial.println(lbvF);
    Serial.print("SB: ");
    Serial.println(sbvC);
    
    BLEService voltCalService("4B00");
    BLECharacteristic lbVoltChar("4B01", BLERead | BLEWrite, lbvC); 
    BLECharacteristic sbVoltChar("4B02", BLERead | BLEWrite, sbvC); 
    BLEDescriptor lbVoltDesc("2901", "Leisure Battery Voltage (Cal)");
    BLEDescriptor sbVoltDesc("2901", "Starter Battery Voltage (Cal)");  
  
    Serial.println("\nBluetooth LE");
    Serial.println("============");
    Serial.println();
    
    if (!BLE.begin()) {
      Serial.println("starting BLE failed!");
      oled.println("BLE fail");
    } else {
      Serial.print("BLE started with address ");
      Serial.println(BLE.address().c_str());
    
      Serial.print("Setting Device Name to ");
      String strName = String("Van:" + BLE.address().substring(0,5));
      Serial.println(strName);
  
      oled.clear();
      oled.println("Voltage Cal");
      oled.println("===========\n");
      oled.println("Use BLE to set\nvoltage levels\n\nEnter 0 to \ncancel.\n\n\nBLE Device:\n-----------\n  ");
      oled.println(strName);
    
      BLE.setDeviceName(strName.c_str());
      BLE.setLocalName(strName.c_str());
  
      BLE.setAdvertisedService(voltCalService);          // add the service UUID
      voltCalService.addCharacteristic(lbVoltChar);  // add the battery level characteristic
      voltCalService.addCharacteristic(sbVoltChar);  // add the battery level characteristic 
      lbVoltChar.addDescriptor(lbVoltDesc);
      sbVoltChar.addDescriptor(sbVoltDesc); 
    
      BLE.addService(voltCalService);                     // Add the battery service 
      BLE.advertise(); 
      
      bool burnFlash = false;
      long startTime = millis();
      
      while((!burnFlash) && (millis() < startTime + CAL_TIMEOUT)){
        BLEDevice central = BLE.central();
      
        if (central) {      
          Serial.print("Connected to central: ");
          // print the central's BT address:
          Serial.println(central.address());
          // turn on the LED to indicate the connection:
          digitalWrite(LED_BUILTIN, HIGH);
        
          while (central.connected()){
            central.poll();
            if (lbVoltChar.written()){
              Serial.print("LB Voltage Cal:    ");
       
              int l = lbVoltChar.valueLength();
              char buf[l];
              lbVoltChar.readValue(buf, l);
              Serial.print(buf);
              Serial.println("V");
    
              Serial.print("Float Voltage: ");
              float v1 = atof(buf);
              Serial.print(v1);
              
              if((v1 >= vanSensors[SENSOR_IDX_LEISURE_BATT].valMin ) && (v1 <= vanSensors[SENSOR_IDX_LEISURE_BATT].valMax )){
                Serial.println(" - OK");
                Serial.print("LB Voltage Read:   ");
                Serial.print(lbvC);
                Serial.println("V");
                Serial.print("Old lbVmult:     ");
                Serial.println(lbvMult);
                lbvMult = lbvMult * v1 / lbvF;
                Serial.print("New lbVmult:     ");
                Serial.println(lbvMult);
                if (!burnFlash){oled.clear();};
                oled.println("LB Volt Cal:");
  
                char buf2[10];
                dtostrf(lbvMult,10,8,buf2);
                oled.println(buf2);
                for (int i = 0; i < 10; i++){
                  EEPROM.write(addrLbCal + i, buf2[i]);
                }
                EEPROM.write(addrLbCal + 10,0);
                burnFlash = true;
              }
            }
          }
          Serial.print("Disconnected from central: ");
          Serial.println(central.address());
          if (burnFlash){
            Serial.print("Committing changes to Flash Memory:");
            EEPROM.commit();
            if (EEPROM.isValid()){
              Serial.println("OK");            
            } else {
              Serial.println("Fail");            
            }
          }  
          BLE.disconnect();
          BLE.end();
          Serial.println("Connecting to WiFi:\n");  
        }
      }
    }
  }
  
  void calCheck(){
    if (!runCal){Serial.println("****** Cal Button Pressed ******");}
    runCal = true;
  }


  void mqttLogPrint(String s){
    if (mqttClient.connected()){
      mqttClient.publish(AIO_LOG, s.c_str());      
    } else {
      mqttLog += s;
      mqttLog += "\n";
    }
  }

  void mqttLogPub(){
    const char *c;
    char e[64];
    int k = 1;

    minDiv();

    Serial.println("Unpublished MQTT Log:");
    Serial.println("=====================");
    Serial.println(mqttLog);
    c =  mqttLog.c_str();
    Serial.println();
/*    Serial.println("C Version:");
    Serial.println(c);
    Serial.println(); */

    int j = 0;
    for (int i = 0; i < mqttLog.length(); i++){
      if (c[i] == 10){
        e[i - j] = 0;
/*        Serial.println();
        Serial.print(k);
        Serial.print(": ");
        Serial.println(e); */
        mqttClient.publish(AIO_LOG, e);      
        delay(250);
        j = i + 1;
        k++;
      } else {
        e[i - j] = c[i];
      }
    } 

    mqttLog = "";
  }
#endif
