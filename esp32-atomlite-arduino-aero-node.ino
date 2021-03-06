/*
    esp32-atomlite-arduino-aero-node.ino

		Wind measurement node with
	  the IR sensor and 
	  Digital Compass (Adafruit LSM303AGR).

		Copyright (c) 2020,2021 keith maull
    Website    : 
    Author     : kmaull
    Create Time:
    Change Log :
*/
#include <WiFi.h>
#include <FastLED.h>                /// https://github.com/FastLED/FastLED
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include "IoTwx.h"                  /// https://github.com/iotwx
#include <Wire.h>
#include <Adafruit_Sensor.h>        /// https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_LSM303AGR_Mag.h> /// https://www.arduino.cc/reference//en/libraries/adafruit-lsm3

#define I2C_SDA        26
#define I2C_SCL        32
#define WATCH_INTERVAL 90

Adafruit_LSM303AGR_Mag_Unified lsm = Adafruit_LSM303AGR_Mag_Unified(12345);
TwoWire I2CLSM                     = TwoWire(0);

IoTwx           node;
unsigned long   lastMillis  = 0;
int             timezone;    
char*           sensor;
char*           topic; 
int             reset_interval; 
int             publish_interval; 
int             max_frequency       = 80;    
const byte      interruptPin        = 25;
volatile int    interruptCounter    = 0;
volatile unsigned long 
                millis_since_last_interrupt
                                    = 0;
volatile unsigned long 
                watch_millis
                                    = 0;

int             numberOfInterrupts  = 0;
portMUX_TYPE    mux                 = portMUX_INITIALIZER_UNLOCKED;
RTC_DATA_ATTR int          
                dataCount           = 0;
RTC_DATA_ATTR unsigned long 
                aData[54];          // 2 hours worth of dataM
int16_t         accel[3];           // we'll store the raw acceleration values here
int16_t         mag[3];             // raw magnetometer values stored here
float           realAccel[3];       // calculated acceleration values here
float           heading, titleHeading;
volatile int    headingCount        = 0;
volatile long   headingSum          = 0;
TaskHandle_t    Task1;
TaskHandle_t    Task2;



void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  interruptCounter++;   millis_since_last_interrupt = millis();
  portEXIT_CRITICAL_ISR(&mux);
}


void publish_measurements() {   
  char s[strlen(sensor)+64];

  node.establishCommunications();
  strcpy(s, sensor); strcat(s, "/ir/anemometer");
  node.publishMQTTMeasurement(topic, s, interruptCounter, 0);

  strcpy(s, sensor); strcat(s, "/lsm303d/windvane");
  node.publishMQTTMeasurement(topic, s, headingSum / headingCount, 0);

  WiFi.mode(WIFI_OFF); Serial.println("[] Wifi shut off for power reduction");  
}


void wait_and_watch()
{
  unsigned long watch_millis = 0;  
  
  /* while last interrupt count has not increased in some interval */
  Serial.print("[] wait_and_watch()");
    
  while ( millis() - millis_since_last_interrupt < WATCH_INTERVAL*1000 ) {
    // Serial.print("delta: "); Serial.println(millis() - millis_since_last_interrupt);
    if ( millis() - watch_millis >= 60*1000 && interruptCounter > 20 )
    {
      watch_millis = millis();
      Serial.print("interrupt count: "); Serial.println(interruptCounter); 
      publish_measurements();
      
      // TODO: THIS SHOULD BE IN A CRITICAL SECTION!
      interruptCounter = 0; 
    }
  }

  if ( interruptCounter > 20 ) publish_measurements();

  Serial.print("interrupt count: "); Serial.println(interruptCounter);
  interruptCounter = 0;
}


void setup() {
  File                      file;
  StaticJsonDocument<1024>  doc;
  char                      uuid[32];
  String                    mac = String((uint32_t)ESP.getEfuseMac(), HEX);

  Serial.begin(57600);
  Serial.println("[] This is the IoTwx AeroNode.  Initializing ...");

  // set up AtomLite LED
  init_led();

  node = IoTwx( wait_for_bluetooth_config(uuid, millis(), 1) ); // initializes config.json

  if (node.isConfigured())
  {
    Serial.println("[] deserializing to JSON");
    file = SPIFFS.open("/config.json", FILE_READ);
    deserializeJson(doc, file);
    file.close();

    Serial.println("[] reading from JSON doc (SPIFFS)");
    timezone         = atoi((const char*)doc["iotwx_timezone"]);
    sensor           = strdup((const char*)doc["iotwx_sensor"]);
    topic            = strdup((const char*)doc["iotwx_topic"]);  
    reset_interval   = atoi((const char*)doc["iotwx_reset_interval"]);  
    publish_interval = atoi((const char*)doc["iotwx_publish_interval"]);  
    max_frequency    = 240; // atoi((const char*)doc["iotwx_max_frequency"]);

    // begin shutdown sequence, downthrottle, shutdown wifi and BT
    btStop(); Serial.println("[] BT disconnected for power reduction");  
    setCpuFrequencyMhz(max_frequency); Serial.println(); Serial.print("[] CPU downthrottled to "); Serial.print(max_frequency); Serial.print("Mhz for power reduction");  
    watch_millis = millis();
    
    // core 0 handles the wind / compasss
    xTaskCreatePinnedToCore(
                      measureWindDir,   /* Task function. */
                      "winddir",     /* name of task. */
                      10000,       /* Stack size of task */
                      NULL,        /* parameter of the task */
                      1,           /* priority of the task */
                      &Task1,      /* Task handle to keep track of created task */
                      0);          /* pin task to core 0 */                  
    delay(500); 

    // core 1 handles the ir / windspeed
    xTaskCreatePinnedToCore(
                    measureWindSpeed,   /* Task function. */
                    "windspeed",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */                  
    delay(500); 

    Serial.println("[] intializing sleep interrupts");
    gpio_wakeup_enable(GPIO_NUM_25, GPIO_INTR_HIGH_LEVEL);
    I2CLSM.begin(I2C_SDA, I2C_SCL, 100000);

    /* Initialise the sensor */
    if(!lsm.begin(0x1e, &I2CLSM))
    {
      /* There was a problem detecting the LSM303 ... check your connections */
      Serial.println("[] no LSM303 detected ... Check your wiring!");
    }
    
    esp_sleep_enable_gpio_wakeup();
  } else
        Serial.println("[] halting: internal JSON configuration corrupt");
}


void measureWindSpeed( void * parameters ) {
  Serial.println("[] intializing anemometer interrupts");
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);

  while(true){
    vTaskDelay(100/portTICK_RATE_MS);

    wait_and_watch();
    Serial.println("[info]: sleeping"); delay(2000);
    esp_light_sleep_start();
    interruptCounter = 0;
  }
  vTaskDelete( NULL );
}


void measureWindDir(void * parameters) {
  while(true) {
    /* Get a new sensor event */ 
    sensors_event_t event; 
    lsm.getEvent(&event);
    
    float Pi = 3.14159;
    
    // Calculate the angle of the vector y,x
    heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi;
    
    // Normalize to 0-360
    if (heading < 0)
    {
      heading = 360 + heading;
    }
    headingSum = headingSum + heading;
    headingCount++;
  
    Serial.print("[] compass Heading: "); Serial.print(heading);
    Serial.println("");
  
    delay(6000); // polling interval
    vTaskDelay(100/portTICK_RATE_MS);
    }
  
    vTaskDelete( NULL );
}


void loop() 
{
}
