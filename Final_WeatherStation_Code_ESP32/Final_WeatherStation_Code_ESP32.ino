#include "2nd_Areeb_AWS_Keys.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"

#include <ModbusMaster.h>
#include <HardwareSerial.h>

#include "DHT.h"
#define DHTPIN 4
#define DHTTYPE DHT11   // DHT 11

#define AWS_IOT_PUBLISH_TOPIC   "ESP32_2_Arduino/Pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "ESP32_2_Arduino/Sub"

const int RO_PIN = 19;  // Receive (data in) pin
const int DI_PIN = 18;  // Transmit (data out) pin
const int RE_PIN = 22;
const int DE_PIN = 23;

DHT dht(DHTPIN, DHTTYPE);

int device_value = 1;

float windspeed;
float winddirection;
float solarradiation;
float rainsensor;
float airTemperature;
float airHumidity;

HardwareSerial swSerial(2);  // Use Serial2 for communication
ModbusMaster node;

WiFiClientSecure net = WiFiClientSecure(); // wifi function 
PubSubClient client(net);

void connectAWS()   // function start
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  Serial.println("Connecting to Wi-Fi");
 
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // Create a message handler
  client.setCallback(messageHandler);
 
  Serial.println("Connected to Wifi, Now Connecting to AWS IOT");
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(500);
  }
 Serial.print("Thing name detected finally!");
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }
 
  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("AWS IoT Connected!");
}                                            // function end 


void publishMessage()
{
  StaticJsonDocument<200> doc;
  doc["Wind Speed"] = windspeed;
  doc["Wind Direction"] = winddirection;
  doc["Solar Radiation"] = solarradiation;
  doc["Rain Sensor"] = rainsensor;
  doc["Air Temperature"] = airTemperature; 
  doc["Air Humidity"] = airHumidity;
  

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
 
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}
 
void messageHandler(char* topic, byte* payload, unsigned int length)
{
  Serial.print("incoming: ");
  Serial.println(topic);
 
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}


// Put the MAX485 into transmit mode
void preTransmission()
{
  digitalWrite(RE_PIN, HIGH);
  digitalWrite(DE_PIN, HIGH);
}

// Put the MAX485 into receive mode
void postTransmission()                 // actual code starts 
{
  digitalWrite(RE_PIN, LOW);
  digitalWrite(DE_PIN, LOW);
}

void setup()
{
  Serial.begin(9600);

  // Configure the MAX485 RE & DE control signals and enable receive mode
  dht.begin();
  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);

  // Modbus communication runs at 9600 baud
  swSerial.begin(9600, SERIAL_8N1, RO_PIN, DI_PIN);
 
  // Callbacks to allow us to set the RS485 Tx/Rx direction
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  delay(1000);
  connectAWS();
}

void loop()
{

  //--------------------------------------------------------//
   airHumidity = dht.readHumidity();
   airTemperature =  dht.readTemperature();
   if (isnan(airHumidity) || isnan(airTemperature)) {
         Serial.println("Failed to read from DHT sensor!");
         return;
      }
    Serial.print("Humidity: ");
    Serial.print(airHumidity);
    Serial.print("%  Temperature: ");
    Serial.print(airTemperature);
    Serial.print("°C ");
    Serial.println();
  uint8_t result;

  // Remove any characters from the receive buffer
  // Ask for 7x 16-bit words starting at register address 0x0000
  node.begin(device_value, swSerial);
  result = node.readHoldingRegisters(0x00, 2);

  if (result == node.ku8MBSuccess)
  {
    if (device_value == 1){
        Serial.print("Wind Speed: ");
        windspeed = (node.getResponseBuffer(0x00));
        Serial.print(windspeed);
        Serial.println(" m/s");
    }
    else if (device_value == 2){
        Serial.print("wind Direction: ");
        winddirection = (node.getResponseBuffer(0x00));
        Serial.print(winddirection);
        Serial.println(" Degrees");
    }
    else if (device_value == 3){
        Serial.print("Rain Sensor ");
        rainsensor = (node.getResponseBuffer(0x00));
        Serial.print(rainsensor);
        Serial.println(" mm");
    }
    else if (device_value == 4){
        Serial.print("Solar Radiation ");
        solarradiation = (node.getResponseBuffer(0x00));
        Serial.print(solarradiation);
        Serial.println(" W/m2");
    }
    
    
  }
  else
  {
    printModbusError(result);
  }

   if (device_value == 1){
      device_value = 2;
    }
  else if (device_value == 2){
      device_value = 3;
    }
  else if (device_value == 3){
      device_value = 4;
    }
  else if (device_value == 4){
      device_value = 1;
    }

  
  Serial.println();
  publishMessage();
  client.loop();
  delay(1000);
  //connectAWS();
}

// Print out the error received from the Modbus library
void printModbusError(uint8_t errNum)
{
  switch (errNum)
  {
    case node.ku8MBSuccess:
      Serial.println(F("Success"));
      break;
    case node.ku8MBIllegalFunction:
      Serial.println(F("Illegal Function Exception"));
      break;
    case node.ku8MBIllegalDataAddress:
      Serial.println(F("Illegal Data Address Exception"));
      break;
    case node.ku8MBIllegalDataValue:
      Serial.println(F("Illegal Data Value Exception"));
      break;
    case node.ku8MBSlaveDeviceFailure:
      Serial.println(F("Slave Device Failure"));
      break;
    case node.ku8MBInvalidSlaveID:
      Serial.println(F("Invalid Slave ID"));
      break;
    case node.ku8MBInvalidFunction:
      Serial.println(F("Invalid Function"));
      break;
    case node.ku8MBResponseTimedOut:
      Serial.println(F("Response Timed Out"));
      break;
    case node.ku8MBInvalidCRC:
      Serial.println(F("Invalid CRC"));
      break;
    default:
      Serial.println(F("Unknown Error"));
      break;
  }
}
