#include <Arduino.h>

#define DEBUG
#define SEN

#include <PLG_datastring.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include <ModbusMaster.h>
#include <hardwareSerial.h>
#include <esp_task_wdt.h>
#define WDT_time 5

ModbusMaster relay_modbus;
#define ID_relay_modbus 1
// define the pins used by the transceiver module
#define ss 05
#define rst 04
#define dio0 14
#define led_connected 26
#define led_slave 33
#define led_master 25
#define TX 17
#define RX 16

/*-----------------nha so 2----------------*/
#define BOM "V1"
#define BOMCCON "V2"
#define CATNAG "V3"
#define DEN "V4"
#define PHUNSUONG "V5"
#define DAOKHI "V6"
#define QUATHUT "V7"
#define BONNUOC "V8"
#define ID_relay "PLG_relay"

// --------------nha so 4-------------------
#define BOM_NHA_4 "V9"
#define VAN_1 "V10"
#define VAN_2 "V11"
#define VAN_3 "V12"
#define VAN_4 "V13"

#define RELAY_BOM 0
#define RELAY_van1 1
#define RELAY_van2 2
#define RELAY_van3 3

String ID_control = "0";
String ID_CB = "1";
String ID_DD = "2";
String ID_master = "3";
String ID_control_DD = "4";
String ID_NHA_4 = "5";
String ID_relay_4 = "6";
unsigned long lastLoRaReceiveTime = 0; // thời điểm cuối nhận LoRa
enum control_soure
{
  dk_master = 0,
  dk_control = 1
};
unsigned long lastTime = 0;
float temp = 0.0;          // V10
float hum = 0.0;           // V11
float lux = 0.0;           // V12
float ph_nuoc = 0.0;       // V20
float ec_nuoc = 0.0;       // V19
float ph_dat = 0.0;        // V13
bool start_sensor = false; // Flag to indicate if sensor data should be processed
bool Auto_cool = false;
int couter = 0, mode = 0;
float temp_setpoint = 40.0;
// String receivedData = "";
String cmd = "";
#define BOARD_LORA

#if defined(BOARD_LORA)
void sen_lora_data_4()
{
  // Example of sending sensor data
  LoRa.beginPacket(); // Start a new packet
  LoRa.print(messages4);
  LoRa.endPacket();               // Finish the packet and send it
  digitalWrite(led_master, HIGH); // Turn off LED for slave status
  delay(20);                      // Delay to ensure the message is sent
  digitalWrite(led_master, LOW);  // Turn off LED for slave status
  esp_task_wdt_reset();
}

void setup()
{

  Serial.begin(115200);
  DEBUG_PRINTLN("LoRa Sender");

  // setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  esp_task_wdt_init(WDT_time, true);
  esp_task_wdt_add(NULL);
  // 433E6 for Asia
  while (!LoRa.begin(433E6))
  {
    Serial.print(".");
    delay(500);
  }
  DEBUG_PRINTLN("LoRa Initialized");
  pinMode(led_slave, OUTPUT);
  pinMode(led_master, OUTPUT);
  pinMode(led_connected, OUTPUT);
  digitalWrite(led_connected, LOW);
  digitalWrite(led_slave, HIGH);
  digitalWrite(led_master, LOW);
  DEBUG_PRINTLN("PLG RUNNING NHA 4");
  relay_modbus.begin(ID_relay_modbus, Serial2); // Cảm biến địa chỉ 2
  Serial2.begin(9600, SERIAL_8N1, RX, TX);
  PLG_write_4(ID_NHA_4, ID_master, ID_control, "reset");
  DEBUG_PRINTLN("PLG_board_reset");
  sen_lora_data_4();
  esp_task_wdt_reset();
}

void thucthilenh()
{
  if (address_slave.startsWith(ID_NHA_4) && address.startsWith(ID_master)) // nhan du lieu dieu khien tu master va du lieu cam bien
  {
    // Map tên thiết bị sang tên hiển thị debug
    struct Device
    {
      const char *name;
      const char *debugName;
      int thanhghi;
    };

    const Device devices[] = {
        {BOM_NHA_4, "BOM_NHA_4",RELAY_BOM},
        {VAN_1, "VAN_1",RELAY_van1},
        {VAN_2, "VAN_2 ",RELAY_van2},
        {VAN_3, "VAN_3 ",RELAY_van3}};

    bool commandHandled = false;

    for (const auto &device : devices)
    {
      if (namedata.startsWith(device.name))
      {
        if (data.startsWith("ok"))
        {
          // DEBUG_PRINTF("[ON]%s\n ", device.name);
          relay_modbus.writeSingleCoil(device.thanhghi, 1);
        }
        else if (data.startsWith("not ok"))
        {
          // DEBUG_PRINTF("[OFF]%s\n ", device.name);
          relay_modbus.writeSingleCoil(device.thanhghi, 0);
        }
        // SEN_PRINTLN(messages4);
         DEBUG_PRINTLN("ok");
        commandHandled = true;
        esp_task_wdt_reset();
        break;
      }
    }
    
    
  }
}

void loop()
{
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    receivedData = "";
    while (LoRa.available())
    {
      receivedData += (char)LoRa.read();
    }
    DEBUG_PRINTLN(receivedData);
    PLG_check_message(); // Check the received data
    thucthilenh();       // Execute the command

    //digitalWrite(led_connected, HIGH);
    //delay(20);
    //digitalWrite(led_connected, LOW);

    lastLoRaReceiveTime = millis(); // cập nhật thời gian nhận LoRa
    esp_task_wdt_reset();
  }

  // Kiểm tra 5 giây không nhận LoRa thì reset
//  if (millis() - lastLoRaReceiveTime > 5000)
 // {
   // ESP.restart();
 // }
  // relay_modbus.writeSingleCoil(1, 1);
   //delay(500);
   // relay_modbus.writeSingleCoil(0, 0);
   // delay(500);
  esp_task_wdt_reset();
}
#else

#define relay_1 32
#define relay_2 33
#define relay_3 25
#define relay_4 26
void thucthilenh()
{
  if (address_slave.startsWith(ID_relay_4) && namedata.startsWith(BOM_NHA_4))
  {

    if (data.startsWith("ON"))
    {

      DEBUG_PRINTLN("bom ON");
      digitalWrite(relay_1, 1);
      // bom = HIGH;
      // PLG_write_4("slave1", "PLG_relay", "bom", "ON");
      // SEN_PRINTLN(receivedData);
    }
    else if (data.startsWith("OFF"))
    {

      DEBUG_PRINTLN("bom OFF");
      digitalWrite(relay_1, 0);
      // bom = LOW;
      // PLG_write_4("slave1", "PLG_relay", "bom", "OFF");
      // SEN_PRINTLN(receivedData);
    }
  }
  if (address_slave.startsWith(ID_relay_4) && namedata.startsWith(VAN_1))
  {
    if (data.startsWith("ON"))
    {

      DEBUG_PRINTLN("quat ON");
      digitalWrite(relay_2, 1);
      // daokhi = HIGH;
      // PLG_write_4("slave1", "PLG_relay", "quat", "ON");
      // SEN_PRINTLN(receivedData);
    }
    else if (data.startsWith("OFF"))
    {

      DEBUG_PRINTLN("quat OFF");
      digitalWrite(relay_2, 0);
      // daokhi = LOW;
      // PLG_write_4("slave1", "PLG_relay", "quat", "OFF");
      // SEN_PRINTLN(receivedData);
    }
  }
  if (address_slave.startsWith(ID_relay_4) && namedata.startsWith(VAN_2))
  {
    if (data.startsWith("ON"))
    {

      DEBUG_PRINTLN("quat hut ON");
      digitalWrite(relay_3, 1);
      // quathut = HIGH;
      // PLG_write_4("slave1", "PLG_relay", "fanhut", "ON");
      // SEN_PRINTLN(receivedData);
    }
    else if (data.startsWith("OFF"))
    {

      DEBUG_PRINTLN("quat hut OFF");
      digitalWrite(relay_3, 0);
      // quathut = LOW;
      // PLG_write_4("slave1", "PLG_relay", "fanhut", "OFF");
      // SEN_PRINTLN(receivedData);
    }
  }
  if (address_slave.startsWith(ID_relay_4) && namedata.startsWith(VAN_3))
  {
    if (data.startsWith("ON"))
    {

      DEBUG_PRINTLN("cat nang ON");
      digitalWrite(relay_4, 1);
      // catnag = HIGH;
      // PLG_write_4("slave1","PLG_relay","catnag","ON");
      // SEN_PRINTLN(receivedData);
    }
    else if (data.startsWith("OFF"))
    {

      DEBUG_PRINTLN("cat nang OFF");
      digitalWrite(relay_4, 0);
      // catnag = LOW;
      // PLG_write_4("slave1","PLG_relay","catnag","OFF");
      // SEN_PRINTLN(receivedData);
    }
  }
}
void setup()
{
  Serial.begin(115200);
  // esp_task_wdt_init(WDT_time, true);
  // esp_task_wdt_add(NULL);
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);
  digitalWrite(relay_1, 1);
  digitalWrite(relay_2, 1);
  digitalWrite(relay_3, 1);
  digitalWrite(relay_4, 1);

  Serial2.begin(115200, SERIAL_8N1, RX, TX);
  // delay(1000);
  DEBUG_PRINTLN("PLG_readly_running");
  PLG_write_4(ID_relay_4, ID_NHA_4, "datarun", "running");
  SEN_PRINTLN(messages4);
  // esp_task_wdt_reset();
}

void loop()
{
  if (Serial2.available())
  {
    receivedData = Serial2.readStringUntil('\n'); // Ví dụ nhận chuỗi
    DEBUG_PRINTLN(receivedData);
    PLG_check_message(); // Gọi xử lý chuỗi
    thucthilenh();

    // Execute the command
  }
}

#endif
