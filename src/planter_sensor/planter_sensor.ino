#include <Arduino.h>
#include <bluefruit.h>

#define PIN_VBAT        (32)  // D32 battery voltage
#define PIN_VBAT_ENABLE (14)  // D14 LOW:read enable
#define PIN_HICHG       (22)  // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23)  // D23 charge indicatore LOW:charge HIGH:no charge

// Setting this parameter to true enables a direct reading from the ADC pin of the battery level to be sent to a BTHome counter.
// The goal is to determine the maximum and minimum battery levels through testing in order to calculate the battery percentage. 
static const bool Calibration = false;

static const int MinBatteryLvl = 1300;
static const int MaxBatteryLvl = 1600;


static const uint8_t data_index_batLvl = 18;
static const uint8_t data_index_charging = 20;

static const uint8_t Power_sensor_1  = D10;
static const uint8_t Value_sensor_1  = D3;
static const uint8_t data_index_sensor_1 = 22;

static const uint8_t Power_sensor_2  = D9;
static const uint8_t Value_sensor_2  = D4;
static const uint8_t data_index_sensor_2 = 24;

static const uint8_t Power_sensor_3  = D8;
static const uint8_t Value_sensor_3  = D5;
static const uint8_t data_index_sensor_3 = 26;

static const uint8_t Power_sensor_4  = D7;
static const uint8_t Value_sensor_4  = D6;
static const uint8_t data_index_sensor_4 = 28;

static const uint8_t Power_sensor_5  = D2;
static const uint8_t Value_sensor_5  = D1;
static const uint8_t data_index_sensor_5 = 30;

// Bluetooth data send to Home Assistant via BTHome
static uint8_t ble_adv_data[31];
static uint8_t ble_adv_data2[22];

// Adc value filter
const int samples = 20;
int AdcBatterySmoothedValues[samples];
int AdcBatterySmoothedValuesIndex = 0;
int AdcBatterySmoothedValue = 0;


void setup()
{
  Serial.begin(115200);

  pinMode(LED_BLUE, OUTPUT);


  Bluefruit.begin();
  Bluefruit.setTxPower(2);

  SetupSensors();
  SetupBleData();
  SetupBleData2();
  SetupBattery();

  digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
  for (int i = 0; i < samples; i++)
  {
    AdcBatterySmoothedValues[i] = analogRead(PIN_VBAT);
  }
  digitalWrite(PIN_VBAT_ENABLE, HIGH); // VBAT read disable
}

void SetupSensors()
{
  SetupSensor(Power_sensor_1, Value_sensor_1);
  SetupSensor(Power_sensor_2, Value_sensor_2);
  SetupSensor(Power_sensor_3, Value_sensor_3);
  SetupSensor(Power_sensor_4, Value_sensor_4);
  SetupSensor(Power_sensor_5, Value_sensor_5);
}

void SetupSensor(uint8_t power, uint8_t value)
{
  pinMode(power, OUTPUT);
  digitalWrite(power, LOW);
  pinMode(value, INPUT_PULLDOWN);
}

void SetupBattery()
{
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);

  digitalWrite(PIN_HICHG, LOW);       // charge current 100mA
  
  // initialise ADC wireing_analog_nRF52.c:73
  analogReference(AR_DEFAULT);        // default 0.6V*6=3.6V  wireing_analog_nRF52.c:73
  analogReadResolution(12);           // wireing_analog_nRF52.c:39
}

uint8_t ReadValueOfSensor(uint8_t power, uint8_t value)
{
  uint8_t result = 0;
  
  digitalWrite(power, HIGH);
  delay(1);

  if (digitalRead(value) == HIGH)
  {
    result = LOW;
  }
  else
  {
    result = HIGH;
  }

  digitalWrite(power, LOW);

  return result;
}

void SetupBleData()
{
  // Header
  ble_adv_data[0] = 0x02;
  ble_adv_data[1] = 0x01;
  ble_adv_data[2] = 0x06;
 
  // Sensor name
  ble_adv_data[3] = 0x08; // Length
  ble_adv_data[4] = 0x09; // Local name
  ble_adv_data[5] = 0x50; // Planter : 50 6C 61 6E 74 65 72
  ble_adv_data[6] = 0x6C;
  ble_adv_data[7] = 0x61;
  ble_adv_data[8] = 0x6E;
  ble_adv_data[9] = 0x74;
  ble_adv_data[10] = 0x65;
  ble_adv_data[11] = 0x72;

  // Data Header
  ble_adv_data[12] = 0x12; // Length
  ble_adv_data[13] = 0x16; // Sercice Data - 16 Bit UUID
  ble_adv_data[14] = 0xD2; // UUID BTHome
  ble_adv_data[15] = 0xFC;
  ble_adv_data[16] = 0x40; // BTHome V2, unencrypted.

  // Data
  ble_adv_data[17] = 0x01; // Battery Level in %
  ble_adv_data[18] = 0x00;  
  ble_adv_data[19] = 0x16; // Is battery charging
  ble_adv_data[20] = 0x00;  

  ble_adv_data[21] = 0x20; // Sensor 1 moisture 0 => no water, 1 water   
  ble_adv_data[22] = 0x00;

  ble_adv_data[23] = 0x20; // Sensor 2 moisture 0 => no water, 1 water   
  ble_adv_data[24] = 0x00;

  ble_adv_data[25] = 0x20; // Sensor 3 moisture 0 => no water, 1 water   
  ble_adv_data[26] = 0x00;

  ble_adv_data[27] = 0x20; // Sensor 4 moisture 0 => no water, 1 water   
  ble_adv_data[28] = 0x00;

  ble_adv_data[29] = 0x20; // Sensor 5 moisture 0 => no water, 1 water   
  ble_adv_data[30] = 0x00;
}

void SetupBleData2()
{
  // Header
  ble_adv_data2[0] = 0x02;
  ble_adv_data2[1] = 0x01;
  ble_adv_data2[2] = 0x06;
 
  // Sensor name
  ble_adv_data2[3] = 0x08; // Length
  ble_adv_data2[4] = 0x09; // Local name
  ble_adv_data2[5] = 0x50; // Planter : 50 6C 61 6E 74 65 72
  ble_adv_data2[6] = 0x6C;
  ble_adv_data2[7] = 0x61;
  ble_adv_data2[8] = 0x6E;
  ble_adv_data2[9] = 0x74;
  ble_adv_data2[10] = 0x65;
  ble_adv_data2[11] = 0x72;

  // Data Header
  ble_adv_data2[12] = 0x09; // Length
  ble_adv_data2[13] = 0x16; // Sercice Data - 16 Bit UUID
  ble_adv_data2[14] = 0xD2; // UUID BTHome
  ble_adv_data2[15] = 0xFC;

  ble_adv_data2[16] = 0x40; // BTHome V2, unencrypted.

  // Data
  ble_adv_data2[17] = 0x3E; // ADC
  ble_adv_data2[18] = 0x00;
  ble_adv_data2[19] = 0x00;
  ble_adv_data2[20] = 0x00;
  ble_adv_data2[21] = 0x00;
}

void SendBleData()
{
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.setData(ble_adv_data, sizeof(ble_adv_data));
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(160, 160);  // Interval in unit of 0.625 ms.
  Bluefruit.Advertising.setFastTimeout(30);    // Temp delay in sec.
  Bluefruit.Advertising.start(0);

  if (Calibration)
  {
    delay(10);

    Bluefruit.Advertising.clearData();
    Bluefruit.Advertising.setData(ble_adv_data2, sizeof(ble_adv_data2));
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(160, 160);  // Interval in unit of 0.625 ms.
    Bluefruit.Advertising.setFastTimeout(30);    // Temp delay in sec.
    Bluefruit.Advertising.start(0);
    delay(10);
  }
  else
  {
    delay(1000);
    Bluefruit.Advertising.stop();
  }
}

uint8_t IsCharging()
{
  return (digitalRead(PIN_CHG) == LOW);
}

uint8_t GetBatteryLvl()
{
  int rangeValue = MaxBatteryLvl - MinBatteryLvl;
  int valueInRange = AdcBatterySmoothedValue - MinBatteryLvl;
  if (AdcBatterySmoothedValue <= MinBatteryLvl)
  {
    return 0;
  }
  else if (AdcBatterySmoothedValue >= MaxBatteryLvl)
  {
    return 100;
  }
  else
  {
    return ((float) valueInRange / (float) rangeValue) * 100;
  }
}

// after a call AdcBatterySmoothedValue is updated
// return instant bat lvl for debug.
int CalculateAdcBatterySmoothedValue()
{
  digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
  delay(2);
  int vBatValue = analogRead(PIN_VBAT);
  digitalWrite(PIN_VBAT_ENABLE, HIGH); // VBAT read disable

  AdcBatterySmoothedValues[AdcBatterySmoothedValuesIndex] = vBatValue;
  AdcBatterySmoothedValuesIndex++;
  if (AdcBatterySmoothedValuesIndex >= samples)
  {
    AdcBatterySmoothedValuesIndex = 0;
  }

  double sum = 0;
  for (int i = 0; i < samples; i++)
  {
    sum = sum + AdcBatterySmoothedValues[i];
  }
  
  AdcBatterySmoothedValue = (sum / (double) (samples));

  return vBatValue;
}

void DebugDisplayData(int instantBatValue)
{
  Serial.print("Sensor 1: ");
  Serial.print(ble_adv_data[data_index_sensor_1]);
  Serial.print(" | Sensor 2: ");
  Serial.print(ble_adv_data[data_index_sensor_2]);
  Serial.print(" | Sensor 3: ");
  Serial.print(ble_adv_data[data_index_sensor_3]);
  Serial.print(" | Sensor 4: ");
  Serial.print(ble_adv_data[data_index_sensor_4]);
  Serial.print(" | Sensor 5: ");
  Serial.print(ble_adv_data[data_index_sensor_5]);
  Serial.print(" | IsCharging : ");
  Serial.print(ble_adv_data[data_index_charging]);
  Serial.print(" | GetBatteryLvl : ");
  Serial.print(ble_adv_data[data_index_batLvl]);

  Serial.print(" | SmoothAdcBattery : ");
  Serial.print(AdcBatterySmoothedValue);

  Serial.print(" | PIN_VBAT : ");
  Serial.print(instantBatValue);

  int8_t txPower = Bluefruit.getTxPower();
  Serial.print(" | TX Power : ");
  Serial.println(txPower);
}

void loop()
{
  // Sensors states
  ble_adv_data[data_index_sensor_1] = ReadValueOfSensor(Power_sensor_1, Value_sensor_1);
  ble_adv_data[data_index_sensor_2] = ReadValueOfSensor(Power_sensor_2, Value_sensor_2);
  ble_adv_data[data_index_sensor_3] = ReadValueOfSensor(Power_sensor_3, Value_sensor_3);
  ble_adv_data[data_index_sensor_4] = ReadValueOfSensor(Power_sensor_4, Value_sensor_4);
  ble_adv_data[data_index_sensor_5] = ReadValueOfSensor(Power_sensor_5, Value_sensor_5);

  // Battery state
  ble_adv_data[data_index_charging] = IsCharging();

  int vbatt = CalculateAdcBatterySmoothedValue();
  ble_adv_data[data_index_batLvl] = GetBatteryLvl();

  // Update Adc info in BTHome for battery range calibration.
  if (Calibration)
  {
    ble_adv_data2[18] = AdcBatterySmoothedValue & 0xFF;
    ble_adv_data2[19] = (AdcBatterySmoothedValue >> 8 ) & 0xFF;
    ble_adv_data2[20] = (AdcBatterySmoothedValue >> 8 >> 8) & 0xFF;
    ble_adv_data2[21] = (AdcBatterySmoothedValue >> 8 >> 8 >> 8) & 0xFF;
  }

  if (Serial)
  {
    DebugDisplayData(vbatt);
  }

  SendBleData();

  digitalWrite(LED_BLUE, HIGH); //LED off (active LOW)

  // Evaluation every 10 minutes.
  if ( ! Calibration)
  {
    delay(600000);
  }

  
}
