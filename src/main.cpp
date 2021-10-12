#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <Kaatru_sensor.h>
#include <Adafruit_GPS.h>
#include <ADS1115.h>
#include <ArduinoJson.h>
#include <secrets.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
void messageHandler(char* topic , byte* payload,unsigned int lenght) {
  Serial.println("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}
void connectAWS()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
   net.setCACert(AWS_CERT_CA);
   net.setCertificate(AWS_CERT_CRT);
   net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);

  Serial.print("Connecting to AWS IOT");

  while (!client.connect(DEVICE_ID)) {
    Serial.print(".");
    delay(10);
  }
  if(!client.connected()){
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}
ADS1115 adc0(ADS1115_DEFAULT_ADDRESS);
int alertReadyPin = 36;
#define GPSSerial Serial1
#define GPSECHO true


float temp,humidity;
int PM01Value=0;
int PM2_5Value=5;
int PM10Value=0;  
#define DHTPIN 19
#define DHTTYPE    DHT22 
float V_0 = 5.0; // supply voltage to the pressure sensor
float rho = 1.204; // density of air 

// parameters for averaging and offset
float offset = 0.00;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;

DHT_Unified dht(DHTPIN, DHTTYPE);
Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();

TaskHandle_t Task1, Task2;
SemaphoreHandle_t i2c_semaphore;
SemaphoreHandle_t serial_semaphore;

StaticJsonDocument<200> doc;
int m_counter = 0;

void pollAlertReadyPin() {
  for (uint32_t i = 0; i<100000; i++)
    if (!digitalRead(alertReadyPin)) return;
   Serial.println("Failed to wait for AlertReadyPin, it's stuck high!");
}
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void imu_task( void * parameter )
{
  struct Stack* imu_stack = create_imu_stack(25);
  struct imu_data imu_packet;
  for (;;) {
    imu_packet = getMotion(&Wire);
    if(!isFull(imu_stack))
    push(imu_stack,imu_packet);
    else 
    while(!isEmpty(imu_stack)){
      imu_packet = pop(imu_stack);
      // Serial.print(imu_packet.acx);
      // Serial.print("  ");
      // Serial.print(imu_packet.acy);
      // Serial.print("  ");
      // Serial.print(imu_packet.acz);
      // Serial.print("  ");
      // Serial.print(imu_packet.gcx);
      // Serial.print("  ");
      // Serial.print(imu_packet.gcy);
      // Serial.print("  ");
      // Serial.print(imu_packet.gcz);
      // Serial.print("  ");
      // Serial.print(imu_packet.tmp);
      // Serial.print("  ");
      // Serial.print(imu_packet.millis);
      // Serial.println();
    }
    vTaskDelay(10);
  }
}
void sensor_task( void * parameter )
{
  
  for(;;){
    client.loop();
  doc["did"] = DEVICE_ID;
  doc["mid"] = m_counter++;
    pinMode(2,OUTPUT);
    digitalWrite(2,HIGH);
  Serial.println("---------------------------------------------------------------------------");
  digitalWrite(2,LOW);
  sensors_event_t event;
  dht.temperature().getEvent(&event);

  temp =  event.temperature;
  // Serial.print("Temperature: ");
  // Serial.print(temp);
  // Serial.println(" *C");
  doc["tmp"] = temp;
  dht.humidity().getEvent(&event);
  humidity = event.relative_humidity;

  // Serial.print("Humidity: ");
  // Serial.print(humidity);
  // Serial.println("%");
  doc["hmdy"] = humidity;

  //Serial.println("---------------------------------------------------------------------------");
  adc0.setMultiplexer(ADS1115_MUX_P0_NG);
  adc0.triggerConversion();
  float veloc = 0.00;
  float rel_air = adc0.getMilliVolts();
  // Serial.print("Raw airspeed Sensor values:  ");
  // Serial.print(rel_air); Serial.print("mV\t");
  if(rel_air != 0)
  rel_air= mapf(rel_air-offset, 0.00,4932.00,0,1023) ;
  // Serial.print("mapped & offseted airspeed Sensor values:  ");
  // Serial.println(rel_air);
    if (rel_air>512-zero_span and rel_air<512+zero_span){
  } else{
    if (rel_air<512){
      veloc = -sqrt((-10000.0*((rel_air/1023.0)-0.5))/rho);
    } else{
      veloc = sqrt((10000.0*((rel_air/1023.0)-0.5))/rho);
    }
  }
  // Serial.print("Relative airspeed:  ");
  // Serial.println(veloc);
  doc["relair"] = veloc;
  //Serial.println("---------------------------------------------------------------------------");
  int n_uv = 131;//analogSample(10,5,);
  // Serial.print("UV Sensor Reading : ");
  // Serial.println(n_uv);
  doc["uv"] = n_uv;
  //Serial.println("---------------------------------------------------------------------------");
  n_uv = 121;//analogSample(10,5,2);
  // Serial.print("LIGHT Sensor Reading : ");
  // Serial.println(n_uv);
  // Serial.println("---------------------------------------------------------------------------");
  doc["light"] = n_uv;
  client.loop();
  getParticulateMatter(&Serial1,&PM01Value,&PM2_5Value,&PM10Value);
  // Serial.print("Pm 1 value : ");Serial.println(PM01Value);
  // Serial.print("Pm 2.5 value : ");Serial.println(PM2_5Value);
  // Serial.print("Pm 10 value : ");Serial.println(PM10Value);
  // Serial.println("---------------------------------------------------------------------------");
  doc["pm01"] = PM01Value;
  doc["pm2_5"] = PM2_5Value;
  doc["pm10"] = PM10Value;
  
  doc["lat"] = 12.9915;
  doc["Long"] = 80.2337;
  char jsonBuffer[512];
  serializeJson(doc,jsonBuffer);
  serializeJsonPretty(doc, Serial);
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
  client.loop();
  vTaskDelay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  connectAWS();
  Wire.begin();
  //MPU6050 Init
  mpuInit(&Wire);
  Serial.println("MPU6050 Initizialised");
  Serial.println("Testing ADS1115 connections...");
  Serial.println(adc0.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");
  adc0.initialize();
  adc0.setMode(ADS1115_MODE_SINGLESHOT);
  adc0.setRate(ADS1115_RATE_8);
  adc0.setGain(ADS1115_PGA_6P144);  
  pinMode(alertReadyPin,INPUT_PULLUP);
  adc0.setConversionReadyPinMode();
  #ifdef ADS1115_SERIAL_DEBUG
    adc0.showConfigRegister();
    Serial.print("HighThreshold="); Serial.println(adc0.getHighThreshold(),BIN);
    Serial.print("LowThreshold="); Serial.println(adc0.getLowThreshold(),BIN);
    #endif
  Serial.println("ADS1115 Initizialised");
  //dht22 Init
  dht.begin();
  Serial.println("DHT22 Initizialised");
  Serial.println("Calibrating Relative air speed Sensor");
  for (int ii=0;ii<offset_size;ii++){
    adc0.setMultiplexer(ADS1115_MUX_P0_NG);
  adc0.triggerConversion();
  //pollAlertReadyPin();
  offset += adc0.getMilliVolts() - 2500.00;
  Serial.println(offset);
  delay(10);
  }
  offset /= offset_size;
  Serial.print("Relative air speed Sensor offset value : "); Serial.println(offset);
  Serial1.begin(9600);
  Serial1.setTimeout(1000);

  GPS.begin(9600);
  //Serial1.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);

  xTaskCreatePinnedToCore(
    sensor_task,
    "Sensor_task",
    10000,
    NULL,
    1,
    &Task1,
    1);
    delay(500);
   xTaskCreatePinnedToCore(
     imu_task,
     "imu_task",
     100000,
     NULL,
     1,
     &Task2,
     0);
}

void loop() {
  delay(1);
}
