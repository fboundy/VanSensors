/*
 *  Hardware: initially built using Arduino Uno WiFi Rev2 but should be able to be ported to Arduino Nano BLE which is half the price!
 *  
 *  
 *  
 *  Fluid Level
 *  ===========
 *  
 *  Arduino:    Uno WiFi Rev2  
 *  Ultrasonic: HC-SR04           https://www.amazon.co.uk/HC-SR04-Ultrasonic-Distance-Rangefinder-Detection/dp/B0066X9V5K
 *  
 *  Power
 *  =====
 *  
 *  Mains Detect:   Uses an LED PSU to generate 12v when EHU is on (also drives a siren via a relay).
 *  Current:        CPZ-1085 Current Sensor   https://www.amazon.co.uk/Electronics-Salon-150Amp-Current-Sensor-Module/dp/B016M81RLY/ref=sxts_sxwds-bia-wc-p13n1_0?cv_ct_cx=electronics+salon+current&dchild=1&keywords=electronics+salon+current&pd_rd_i=B016M81RLY&pd_rd_r=855c286a-2189-482b-8a82-3c8ac6955461&pd_rd_w=6ZtSk&pd_rd_wg=2eXXN&pf_rd_p=e3a968a9-db34-4cb6-962f-ddcde7323cf7&pf_rd_r=4Z14TAA9TQPE6GJV23PP&psc=1&qid=1591251657&sr=1-1-91e9aa57-911e-4628-99b3-09163b7d9294
 *  
 *  IMU
 *  ===
 *  
 *  Uses the Arduino's internal IMU sensors
 *  
 */


//#define FLUID_LEVEL    1
#define POWER_IMU      1
//#define STARTER_BATTERY 1
#define DEBUG_0        1 // high level debug
//#define DEBUG_1        1 // low level debug


#include <ArduinoBLE.h>
#include <EEPROM.h>
#include "vanSensors.h"

#define UPDATE_PERIOD 500

// Fluid Level Sensor 
#if defined FLUID_LEVEL
  #define echoPin         2    // attach pin D2 Arduino to pin Echo of HC-SR04
  #define trigPin         3    //attach pin D3 Arduino to pin Trig of HC-SR04
  #define emptyPin        4
  #define fullPin         5
  #define CAL_INTERVAL    10000 
  
  #define FULL_DIST       50    // default distance to fluid when full (mm)
  #define EMPTY_DIST      550  //  default distance to fluid when full (mm)
  #define ADDR_FULL       0
  
  BLEService fluidLevelService(vanSensors[SENSOR_IDX_FRESH_WATER].uuid_s);
  BLEUnsignedShortCharacteristic fluidLevelChar(vanSensors[SENSOR_IDX_FRESH_WATER].uuid_c, BLERead | BLENotify);
  BLEUnsignedIntCharacteristic fluidDistChar("4E02", BLERead | BLENotify);
  BLEDescriptor currentLevelLabelDescriptor("2901", "Fluid Level (%)");
  BLEDescriptor currentDistLabelDescriptor("2901", "Fluid Distance (mm)");
  
  BLEService fluidLevelCalService("4F00");
  BLEUnsignedIntCharacteristic fluidLevelFullDist("4F01", BLERead | BLEWrite);
  BLEUnsignedIntCharacteristic fluidLevelEmptyDist("4F02", BLERead | BLEWrite); 
  BLEDescriptor fullDistLabelDescriptor("2901", "Full Dist (mm)");
  BLEDescriptor emptyDistLabelDescriptor("2901", "Empty Dist (mm)");
  
  // defines variables
  long duration; // variable for the duration of sound wave travel
  int  distance; // variable for the distance measurement
  int  level;
  int  oldFluidLevel = 0;  // last battery level reading from analog input
  int  cal_high = 100;
  int  cal_low = 0;
  int  dist_full = FULL_DIST;
  int  dist_empty = EMPTY_DIST;
  bool readFull;
  bool readEmpty;
  long lastFull;
  long lastEmpty;
#endif

#if defined STARTER_BATTERY
  #define PIN_SB          A0
  #define ALT_THRESHOLD   14.0
  #define VOLTAGE_MULT    0.0175

  BLEService starterBattService(vanSensors[SENSOR_IDX_STARTER_BATT].uuid_s);
  BLEUnsignedShortCharacteristic starterBatteryChar(vanSensors[SENSOR_IDX_STARTER_BATT].uuid_c, BLERead | BLENotify);
  BLEUnsignedShortCharacteristic alternatorChar(vanSensors[SENSOR_IDX_ALTERNATOR].uuid_c, BLERead | BLENotify);
  BLEDescriptor starterBatteryLabelDescriptor("2901", "Starter Battery Voltage (V)");
  BLEDescriptor alternatorLabelDescriptor("2901", "Alternator Flag");

#endif

#if defined POWER_IMU
  #include <Arduino_LSM6DS3.h>
  
  #define PIN_IGN         A1
  #define PIN_MAINS       A2
  #define VOLTAGE_MULT    0.015
  #define PIN_CURRENT     A0
  #define PIN_RELAY       A3
  

  BLEService powerImuService(vanSensors[SENSOR_IDX_IGNITION].uuid_s);
  BLEUnsignedShortCharacteristic ignitionChar(vanSensors[SENSOR_IDX_IGNITION].uuid_c, BLERead | BLENotify);
  BLEUnsignedShortCharacteristic mainsPowerChar(vanSensors[SENSOR_IDX_MAINS_POWER].uuid_c, BLERead | BLENotify);
  BLEUnsignedShortCharacteristic currentChar(vanSensors[SENSOR_IDX_CURRENT].uuid_c, BLERead | BLENotify);
  BLEUnsignedShortCharacteristic relayChar(vanSensors[SENSOR_IDX_RELAY].uuid_c, BLERead | BLENotify);
  BLEUnsignedShortCharacteristic pitchChar(vanSensors[SENSOR_IDX_PITCH].uuid_c, BLERead | BLENotify);
  BLEUnsignedShortCharacteristic rollChar(vanSensors[SENSOR_IDX_ROLL].uuid_c, BLERead | BLENotify);
  BLEDescriptor pitchLabelDescriptor("2901", "Pitch Angle (deg)");
  BLEDescriptor rollLabelDescriptor("2901", "Roll Angle (deg)");  
  BLEDescriptor ignitionLabelDescriptor("2901", "Ignition Flag");
  BLEDescriptor mainsPowerLabelDescriptor("2901", "Mains Power Flag");
  BLEDescriptor currentLabelDescriptor("2901", "Current (A)");  
  BLEDescriptor relayLabelDescriptor("2901", "Charge Relay Flag");
  
  float ampSeconds;
  long  lastCurr = millis();
  long  lastUpload = millis();
  bool  boolIMU;
#endif

long  previousMillis = 0;  // last time the battery level was checked, in ms

  
void setup() {
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

  Serial.println("Bluetooth LE");
  Serial.println("============");
  Serial.println();

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
  } else {
    Serial.print("BLE started with address ");
    Serial.println(BLE.address().c_str());

    Serial.print("Setting Device Name to ");
    String strName = String("Van_Sensor_" + BLE.address().substring(0,5));
    Serial.println(strName);

    BLE.setDeviceName(strName.c_str());
    BLE.setLocalName(strName.c_str());
  }
  Serial.println();
  
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  #if defined FLUID_LEVEL
    Serial.println("Fluid Level Sensor");
    Serial.println("=================");
    Serial.println();
    
    
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
    pinMode(fullPin, INPUT_PULLUP);
    pinMode(emptyPin, INPUT_PULLUP);

    Serial.println("Reading calibration from EEPROM");
    Serial.println("-------------------------------");
    
    int eeAddress = ADDR_FULL;
    EEPROM.get(eeAddress, dist_full);
    eeAddress += sizeof(dist_full);
    EEPROM.get(eeAddress, dist_empty);
  
    Serial.print("  Full distance:  ");
    Serial.print(dist_full);
    Serial.print("mm\n  Empty distance: ");
    Serial.print(dist_empty);
    Serial.println("mm");
    Serial.println();
  
    if ((dist_full >= dist_empty) || (dist_full < 0) || (dist_empty < 0) || (dist_full > 500) ||(dist_empty > 1500)){
      dist_full = FULL_DIST;
      dist_empty = EMPTY_DIST;        
      Serial.println(" Calibration data invalid. Reset to defaults (50, 550)");
    } else {
      Serial.println(" Calibration data OK");
    } 
  
    lastFull = millis() - CAL_INTERVAL;
    lastEmpty = lastFull;
  
    BLE.setAdvertisedService(fluidLevelService);          // add the service UUID
    fluidLevelService.addCharacteristic(fluidLevelChar);  // add the battery level characteristic
    fluidLevelService.addCharacteristic(fluidDistChar);  // add the battery level characteristic
    fluidLevelChar.addDescriptor(currentLevelLabelDescriptor);
    fluidDistChar.addDescriptor(currentDistLabelDescriptor);
    
  //  BLE.setAdvertisedService(fluidLevelCalService);          // add the service UUID
    fluidLevelCalService.addCharacteristic(fluidLevelFullDist);  // add the battery level characteristic
    fluidLevelCalService.addCharacteristic(fluidLevelEmptyDist);  // add the battery level characteristic 
    fluidLevelFullDist.addDescriptor(fullDistLabelDescriptor);
    fluidLevelEmptyDist.addDescriptor(emptyDistLabelDescriptor); 
  
    BLE.addService(fluidLevelService);                    // Add the battery service
    BLE.addService(fluidLevelCalService);                    // Add the battery service
    fluidLevelChar.writeValue(oldFluidLevel);             // set initial value for this characteristic
    fluidLevelFullDist.writeValue(dist_full);             // set initial value for this characteristic
    fluidLevelEmptyDist.writeValue(dist_empty);             // set initial value for this characteristic 
    
    readDistance();

    Serial.print("\nDistance  ");
    Serial.print(distance);
    Serial.print(". Level  ");
    Serial.print(level);
    Serial.println("%");

  #endif

  #if defined STARTER_BATTERY  
    Serial.println("Starter Battery Sensor");
    Serial.println("======================");
    Serial.println();

    BLE.setAdvertisedService(starterBatteryService);          // add the service UUID
    powerImuService.addCharacteristic(starterBatteryChar);  // add the battery level characteristic
    powerImuService.addCharacteristic(alternatorChar);  // add the battery level characteristic
    starterBatteryChar.addDescriptor(starterBatteryLabelDescriptor);
    alternatorChar.addDescriptor(alternatorLabelDescriptor);
 
    BLE.addService(starterBatteryService);                    // Add the battery service
      
    Serial.print(" Starter Battery:  ");
    Serial.print(analogRead(PIN_SB) * VOLTAGE_MULT);
    Serial.println("V");      
    Serial.print(" Alternator:       ");
    Serial.println(0x3ff * ((analogRead(PIN_SB) * VOLTAGE_MULT) > ALT_THRESHOLD));      
  #endif

  #if defined POWER_IMU   
    Serial.println("External/Vehicle Power Sensor");
    Serial.println("==============================");
    Serial.println();

    BLE.setAdvertisedService(powerImuService);          // add the service UUID
    powerImuService.addCharacteristic(ignitionChar);  // add the battery level characteristic
    powerImuService.addCharacteristic(mainsPowerChar);  // add the battery level characteristic
    powerImuService.addCharacteristic(currentChar);  // add the battery level characteristic
    powerImuService.addCharacteristic(relayChar);  // add the battery level characteristic
    powerImuService.addCharacteristic(pitchChar);  // add the battery level characteristic
    powerImuService.addCharacteristic(rollChar);  // add the battery level characteristic
    ignitionChar.addDescriptor(ignitionLabelDescriptor);    
    mainsPowerChar.addDescriptor(mainsPowerLabelDescriptor);
    currentChar.addDescriptor(currentLabelDescriptor);
    relayChar.addDescriptor(relayLabelDescriptor);
    pitchChar.addDescriptor(pitchLabelDescriptor);
    rollChar.addDescriptor(rollLabelDescriptor);    

 
    BLE.addService(powerImuService);                    // Add the battery service
      
    Serial.print(" Ignition:         ");
    Serial.println(analogRead(PIN_IGN));      
    Serial.print(" Mains Power:      ");
    Serial.println(analogRead(PIN_MAINS));
    Serial.print(" Charge Relay:     ");
    Serial.println(analogRead(PIN_RELAY));
    Serial.print(" Current:          ");
    float curr = analogRead(PIN_CURRENT);
    Serial.print(curr);
    Serial.print(" = ");
    curr = curr  * (MAX_CURRENT - MIN_CURRENT) / 0x3fc + MIN_CURRENT;
    Serial.println(curr);
    Serial.println();

    Serial.println("Vehicle Tilt Sensor");
    Serial.println("===================");
    Serial.println();

    boolIMU = IMU.begin();

    readIMU();
    
    Serial.print(" Pitch:            ");
    Serial.print(readSensorVal(SENSOR_IDX_PITCH));
    Serial.print(" deg   ");
    Serial.print(" Roll:             ");
    Serial.print(readSensorVal(SENSOR_IDX_ROLL));
    Serial.println(" deg");
    
  #endif
  BLE.advertise();
}

void loop() {

// Read sensors etc. prior to advertising checking BLE

  #if defined FLUID_LEVEL  
    readDistance();
    checkManualCal();
  #endif

  #if defined POWER_IMU
    readExternal();
    readIMU();
  #endif

  
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);
  
    while (central.connected()){
      central.poll();
      
      long currentMillis = millis();
      if(currentMillis - previousMillis >= UPDATE_PERIOD) {
        previousMillis = currentMillis;
        #if defined FLUID_LEVEL 
          readDistance();
          checkManualCal();
          checkBleCal();
        #endif
  
        #if defined STARTER_BATTERY
          readStarter();
        #endif
      
        #if defined POWER_IMU
          readExternal();
          ampSeconds = 0;
          lastUpload = millis();
          readIMU();
        #endif     
      }
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

#if defined FLUID_LEVEL
  void readDistance(){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.34 / 2; // Speed of sound wave divided by 2 (go and back)
    // Displays the distance on the Serial Monitor
    float slope = float(cal_high - cal_low) / float(dist_full - dist_empty);
    float intercept = cal_high - (dist_full * slope);
    level = distance * slope + intercept;
    if (level > 100){
      level = 100;
    } else if (level < 0){
      level = 0;
    }
    #if defined(DEBUG_0)
      Serial.print("Distance  ");
      Serial.print(distance);
      Serial.print(". Level  ");
      Serial.print(level);
      Serial.print("%  BLE val:");
      Serial.println(vanSensors[SENSOR_IDX_FRESH_WATER].val, HEX); 
    #endif

    saveSensorVal(SENSOR_IDX_FRESH_WATER, level);
    fluidLevelChar.writeValue(vanSensors[SENSOR_IDX_FRESH_WATER].val);
 
    fluidDistChar.writeValue(distance);  
  }
  
  void checkBleCal(){
    if (fluidLevelFullDist.written()){
      dist_full=fluidLevelFullDist.value();
      Serial.print("Full distance updated to: ");
      Serial.print(dist_full);
      Serial.println(" mm");
      EEPROM.put(ADDR_FULL, dist_full);
      EEPROM.put(ADDR_FULL + sizeof(dist_full), dist_empty);
    } 
    
    if (fluidLevelEmptyDist.written()){
      dist_empty=fluidLevelEmptyDist.value();
      Serial.print("Empty distance updated to: ");
      Serial.print(dist_empty);
      Serial.println(" mm");
      EEPROM.put(ADDR_FULL, dist_full);
      EEPROM.put(ADDR_FULL + sizeof(dist_full), dist_empty);
    } 
  }

  void checkManualCal(){
    if ((!digitalRead(fullPin)) && ((millis() - lastFull) > CAL_INTERVAL)){
      dist_full = distance;
      Serial.println((millis() - lastFull) );
      lastFull = millis();
      Serial.print("Full distance updated to: ");
      Serial.print(dist_full);
      Serial.println(" mm");
      fluidLevelFullDist.writeValue(dist_full);             // set initial value for this characteristic
      EEPROM.put(ADDR_FULL, dist_full);
      EEPROM.put(ADDR_FULL + sizeof(dist_full), dist_empty);
     
    } else if ((!digitalRead(emptyPin)) && ((millis() - lastEmpty) > CAL_INTERVAL)){
      dist_empty = distance;
      lastEmpty = millis();
      Serial.print("Empty distance updated to: ");
      Serial.print(dist_empty);
      Serial.println(" mm");
      fluidLevelEmptyDist.writeValue(dist_empty);             // set initial value for this characteristic 
      EEPROM.put(ADDR_FULL, dist_full);
      EEPROM.put(ADDR_FULL + sizeof(dist_full), dist_empty);
    }
    readFull = (!digitalRead(fullPin) && !lastFull);
    readEmpty = (!digitalRead(emptyPin) && !readEmpty);
  }
#endif

#if defined STARTER_BATTERY
  void readStarter(){
    float sb = analogRead(PIN_SB) * VOLTAGE_MULT;
    saveSensorVal(SENSOR_IDX_STARTER_BATT, sb);
    int alt = 0x3ff * ((analogRead(PIN_SB) * VOLTAGE_MULT) > ALT_THRESHOLD);
    saveSensorVal(SENSOR_IDX_ALTERNATOR, alt);

    starterBatteryChar.writeValue(vanSensors[SENSOR_IDX_STARTER_BATT].val);
    alternatorChar.writeValue(vanSensors[SENSOR_IDX_ALTERNATOR].val);
 
    #if defined(DEBUG_0)
      Serial.print(" Starter Battery:  ");
      Serial.print(sb);
      Serial.print("   ");
      Serial.print(vanSensors[SENSOR_IDX_STARTER_BATT].val);
      Serial.print("   ");
      Serial.print(readSensorVal(SENSOR_IDX_STARTER_BATT));
      Serial.println("V");
      
      Serial.print(" Alternator:       ");
      Serial.print(alt);
      Serial.print("   ");
      Serial.print(vanSensors[SENSOR_IDX_ALTERNATOR].val);
      Serial.print("   ");
      Serial.println(readSensorVal(SENSOR_IDX_ALTERNATOR));
    #endif  
  }
#endif

#if defined POWER_IMU
  void readExternal(){
    int ign = analogRead(PIN_IGN);
    saveSensorVal(SENSOR_IDX_IGNITION, ign);
    int mains = analogRead(PIN_MAINS);
    saveSensorVal(SENSOR_IDX_MAINS_POWER, mains);
    int relay = analogRead(PIN_RELAY);
    saveSensorVal(SENSOR_IDX_RELAY, relay);

    int intCurr = analogRead(PIN_CURRENT);
    float curr = intCurr  * (MAX_CURRENT - MIN_CURRENT) / 0x3fc + MIN_CURRENT; 
    ampSeconds = ampSeconds + curr * (millis() - lastCurr) / 1000;
    lastCurr = millis();
    float avgCurr = ampSeconds * 1000 / (millis() - lastUpload);

    
    saveSensorVal(SENSOR_IDX_CURRENT, avgCurr);

    ignitionChar.writeValue(vanSensors[SENSOR_IDX_IGNITION].val);
    mainsPowerChar.writeValue(vanSensors[SENSOR_IDX_MAINS_POWER].val);
    currentChar.writeValue(vanSensors[SENSOR_IDX_CURRENT].val);
    relayChar.writeValue(vanSensors[SENSOR_IDX_RELAY].val);
 
    #if defined(DEBUG_0)
      Serial.print(" Ignition:         ");
      Serial.print(ign);
      Serial.print("   ");
      Serial.print(vanSensors[SENSOR_IDX_IGNITION].val);
      Serial.print("   ");
      Serial.println(readSensorVal(SENSOR_IDX_IGNITION));
      
      Serial.print(" Mains Power:      ");
      Serial.print(mains);
      Serial.print("   ");
      Serial.print(vanSensors[SENSOR_IDX_MAINS_POWER].val);
      Serial.print("   ");
      Serial.println(readSensorVal(SENSOR_IDX_MAINS_POWER));

      Serial.print(" Charge Relay:     ");
      Serial.print(relay);
      Serial.print("   ");
      Serial.print(vanSensors[SENSOR_IDX_RELAY].val);
      Serial.print("   ");
      Serial.println(readSensorVal(SENSOR_IDX_RELAY));

      Serial.print(" Inst Current:      ");
      Serial.print(intCurr);
      Serial.print("   ");
      Serial.println(curr);
      Serial.print(" Avg Current:       ");
      Serial.print(avgCurr);
      Serial.print("   ");
      Serial.print(vanSensors[SENSOR_IDX_CURRENT].val);
      Serial.print("   ");
      Serial.println(readSensorVal(SENSOR_IDX_CURRENT));
    
    #endif  
  }

  void readIMU(){
    float x, y, z;
    const float pi = 4 * atan(1);

    if (boolIMU){
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        #if defined(DEBUG_0)
          Serial.print(" Pitch:            ");
          Serial.print(atan(x / z) * 180 / pi);
          Serial.print(" deg   ");
          Serial.print(" Roll:             ");
          Serial.print(atan(y / z) * 180 / pi);        
          Serial.println(" deg");
       #endif
        saveSensorVal(SENSOR_IDX_PITCH, atan(x / z) * 180 / pi);
        saveSensorVal(SENSOR_IDX_ROLL, atan(y / z) * 180 / pi);   

        pitchChar.writeValue(vanSensors[SENSOR_IDX_PITCH].val);
        rollChar.writeValue(vanSensors[SENSOR_IDX_ROLL].val);
           
//        Serial.println("IMU Readings:");    
      }   
    }
  }
#endif
