#include "Areeb_AWS_Keys.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <ModbusMaster.h>
#include <HardwareSerial.h>

//#include "DHT.h"
//#define DHTPIN 4     // Digital pin connected to the DHT sensor
//#define DHTTYPE DHT11   // DHT 11
 
#define AWS_IOT_PUBLISH_TOPIC   "ESP32_Arduino/Pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "ESP32_Arduino/Sub"
const int mq3= 36;
const int mq135 =39;
const int mq8=34;
const int mq7=35;
const int mq9=32;
const int mq4=33;
const int RO_PIN = 19;  // Receive (data in) pin
const int DI_PIN = 18;  // Transmit (data out) pin
const int RE_PIN = 22;
const int DE_PIN = 23;

HardwareSerial swSerial(2);  // Use Serial2 for communication
ModbusMaster node;

int mq3read;
int mq135read;
int mq8read;
int mq7read;
int mq9read;
int mq4read;
float Soil_Humidity;
float Soil_Temp;
float Soil_Conduc;
float Soil_N;
float Soil_P;
float Soil_K;
float Soil_PH;

int h;
int  t;
int id=0;
//DHT dht(DHTPIN, DHTTYPE);
 
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
 
void connectAWS()
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
}
 
void publishMessage()
{
  StaticJsonDocument<200> doc;
  doc["AirHumidity"] = h;
  doc["AirTemperature"] = t;
  //doc["AirEthanol"] = mq3read;
  //doc["MQ135"] = mq135read;
  doc["AirHydrogen"] = mq8read;
  doc["AirCarbonmonoxide"] = mq7read;
  //doc["MQ9"] = mq9read;
  doc["AirMethane"] = mq4read;
  doc["SoilHumidity"] = Soil_Humidity;
  doc["SoilTemperature"] = Soil_Temp ;
  doc["SoilConductivity"] = Soil_Conduc;
  doc["SoilPH"] = Soil_PH;
  doc["SoilNitrogen"] = Soil_N;
  doc["SoilPhosphorus"] = Soil_P;
  doc["SoilPotassium"] = Soil_K;
 
  //doc["Serial.no"] = id;
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
void postTransmission()
{
  digitalWrite(RE_PIN, LOW);
  digitalWrite(DE_PIN, LOW);
}


void setup()
{
  Serial.begin(4800);
   // Configure the MAX485 RE & DE control signals and enable receive mode
  pinMode(RE_PIN, OUTPUT);
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  digitalWrite(RE_PIN, LOW);

  // Modbus communication runs at 9600 baud
  swSerial.begin(4800, SERIAL_8N1, RO_PIN, DI_PIN);

  // Modbus slave ID of the NPK sensor is 2
  node.begin(1, swSerial);

  // Callbacks to allow us to set the RS485 Tx/Rx direction
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


  
  connectAWS();
  //dht.begin();
}
 
void loop()
{
 
  h=random(19,20);
  t=random(20,22);
  id=id+1;
  mq3read = analogRead(mq3); 
  mq135read = analogRead(mq135); 
  mq8read = analogRead(mq8); 
  mq7read = analogRead(mq7); 
  mq9read = analogRead(mq9); 
  mq4read = analogRead(mq4); 
  if (isnan(h) || isnan(t) )  // Check if any reads failed and exit early (to try again).
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print(id);
  Serial.print(F("   | Humidity: "));
  Serial.print(h);
  Serial.print(F("%  | Temperature: "));
  Serial.print(t);
  Serial.println(F("°C "));
  Serial.print(F("   | MQ3: "));
  Serial.print(mq3read);
  Serial.print(F("   | MQ135: "));
  Serial.print(mq135read);
  Serial.print(F("   | MQ18: "));
  Serial.print(mq8read);
  Serial.print(F("   | MQ7: "));
  Serial.print(mq7read);
  Serial.print(F("   | MQ9: "));
  Serial.print(mq9read);
  Serial.print(F("   | MQ4: "));
  Serial.print(mq4read);
  Serial.print("\n");
  Serial.print("\n");

  ///////////////////////////////////

  uint8_t result;

  // Remove any characters from the receive buffer
  // Ask for 7x 16-bit words starting at register address 0x0000
  result = node.readHoldingRegisters(0x0000, 7);

  if (result == node.ku8MBSuccess)
  {
    ///--------------------------------------------------------//
    Serial.print("   Humidity: ");
    Soil_Humidity = (node.getResponseBuffer(0x00))/10;
    Serial.print(Soil_Humidity);
    Serial.println(" %");

    Serial.print("  Temperature: ");
    Soil_Temp = (node.getResponseBuffer(0x01))/10;
    Serial.print(Soil_Temp);
    Serial.println(" C");
  
    Serial.print("  Conductivity: ");
    Soil_Conduc = node.getResponseBuffer(0x02);
    Serial.print(Soil_Conduc);
    Serial.println(" us/cm");

    Serial.print("  PH: ");
    Soil_PH = (node.getResponseBuffer(0x03))/10;
    Serial.print(Soil_PH);
    Serial.println("");

    Serial.print("  Nitrogen: ");
    Soil_N = node.getResponseBuffer(0x04);
    Serial.print(Soil_N);
    Serial.println(" mg/kg");

    Serial.print("  Phosphorus: ");
    Soil_P = node.getResponseBuffer(0x05);
    Serial.print(Soil_P);
    Serial.println(" mg/kg");

    Serial.print("  Potassium: ");
    Soil_K = node.getResponseBuffer(0x06);
    Serial.print(Soil_K);
    Serial.println(" mg/kg");


    ///--------------------------------------------///
  }
  else
  {
    printModbusError(result);
  }
  Serial.println();
  publishMessage();
  client.loop();
  delay(10000);
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
