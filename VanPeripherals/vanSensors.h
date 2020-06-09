#include <Arduino.h>

#define SENS_TYPE_ADC     0
#define SENS_TYPE_BME     1
#define SENS_TYPE_SHT     2
#define SENS_TYPE_BLE     3

#define SENSOR_IDX_FRESH_WATER   0
#define SENSOR_IDX_STARTER_BATT  1
#define SENSOR_IDX_IGNITION      2
#define SENSOR_IDX_ALTERNATOR    3
#define SENSOR_IDX_MAINS_POWER   4
#define SENSOR_IDX_LEISURE_BATT  5
#define SENSOR_IDX_EXT_TEMP      6
#define SENSOR_IDX_EXT_HUMIDITY  7
#define SENSOR_IDX_INT_TEMP      8
#define SENSOR_IDX_INT_HUMIDITY  9
#define SENSOR_IDX_AIR_PRESSURE 10
#define SENSOR_IDX_RSSI         11
#define SENSOR_IDX_PITCH        12
#define SENSOR_IDX_ROLL         13
#define SENSOR_IDX_CURRENT      14
#define SENSOR_IDX_RELAY        15
#define SENSOR_IDX_LB_SOC       16

#define SENSORS                 17

#define MIN_TEMP          -25.0
#define MAX_TEMP           45.0
#define MIN_PRESS         700.0
#define MAX_PRESS        1100.0
#define MIN_VOLT            5.0
#define MAX_VOLT           15.0
#define MIN_HUM             1.0
#define MAX_HUM           100.0
#define MIN_CURRENT       -187.97  // 13.3 mV/A  => 2.5V for 2500/13.3 = 187.97 A
#define MAX_CURRENT        187.97  // resolution is 187.97 / 512 = 0.37 A

struct vanSensorSet{
  char                *feedname;
  unsigned short int  val;      //Value 
  bool                updated;  //Updated so needs uploading to AIO
  int                 type;
  char                *uuid_s;  //Service UUID
  char                *uuid_c;  //Primary Characteristic UUID
  float               valMin;
  float               valMax;
} ;

#define SERVICES           3
char *uuidServ[] = {"4C00", "4D00", "4E00"};

vanSensorSet vanSensors[] = {
                             {"freshwater-level", 0, false, SENS_TYPE_BLE,   "4E00", "4E01", 0, 100}, 
                             {"starter-battery",  0, false, SENS_TYPE_BLE,   "4C00", "4C01", MIN_VOLT, MAX_VOLT},
                             {"ignition",         0, false, SENS_TYPE_BLE,   "4D00", "4D02", 0, 0x3ff}, 
                             {"alternator",       0, false, SENS_TYPE_BLE,   "4D00", "4D03", 0, 0x3ff},  
                             {"mains-power",      0, false, SENS_TYPE_BLE,   "4D00", "4D04", 0, 0x3ff},
                             {"leisure-battery",  0, false, SENS_TYPE_BLE,   "4D00", "4D08", MIN_VOLT, MAX_VOLT},
                             {"ext-temp",         0, false, SENS_TYPE_SHT,   "", "", MIN_TEMP, MAX_TEMP},
                             {"ext-hum",          0, false, SENS_TYPE_SHT,   "", "", MIN_HUM, MAX_HUM},
                             {"int-temp",         0, false, SENS_TYPE_BME,   "", "", MIN_TEMP, MAX_TEMP},
                             {"int-hum",          0, false, SENS_TYPE_BME,   "", "", MIN_HUM, MAX_HUM},
                             {"air-pressure",     0, false, SENS_TYPE_BME,   "", "", MIN_PRESS, MAX_PRESS},
                             {"rssi",             0, false, SENS_TYPE_ADC,   "", "", -100, 0},
                             {"pitch",            0, false, SENS_TYPE_BLE,   "4D00", "4D05", -20, 20},
                             {"roll",             0, false, SENS_TYPE_BLE,   "4D00", "4D06", -20, 20},
                             {"current",          0, false, SENS_TYPE_BLE,   "4D00", "4D07", MIN_CURRENT, MAX_CURRENT},
                             {"charge-relay",     0, false, SENS_TYPE_BLE,   "4D00", "4D01", 0, 0x3ff},
                             {"lb-soc",           0, false, SENS_TYPE_ADC,   "", "", 0, 100},        
                            };
                            
void saveSensorVal(int i, float x){
  float v1;
  if (x < vanSensors[i].valMin || x > vanSensors[i].valMax){
    vanSensors[i].val =  0x0000;
    vanSensors[i].updated = false;
  } else {
    vanSensors[i].val = 0xfffe * (x - vanSensors[i].valMin) / (vanSensors[i].valMax - vanSensors[i].valMin) + 0x0001;    
    vanSensors[i].updated = true;
  }
  
  #if defined(DEBUG_1)
    Serial.print("saveSensorVal: ");
    Serial.print("  i: ");
    Serial.print(i);
    Serial.print("  x: ");
    Serial.print(x);
    Serial.print("  Min: ");
    Serial.print(vanSensors[i].valMin);
    Serial.print("  Max: ");
    Serial.print(vanSensors[i].valMax);  Serial.print("  Val: ");
    Serial.print(v1);
    Serial.print("   ");
    Serial.println(vanSensors[i].val);
  #endif
}

float readSensorVal(int i){
  float x = vanSensors[i].val;
  if (x !=0){
    x = (x - 0x0001) * (vanSensors[i].valMax - vanSensors[i].valMin) / 0xfffe + vanSensors[i].valMin;
  } else { 
    x = -999.99;
  }
  return x;
}                             
