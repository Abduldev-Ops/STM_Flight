#include <Bluepad32.h>

//UART ON esp to send to stm 32
//gpio_rx = 16, gpio_tx= 17
//wifi start
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid     = "WIFI-CE54";
const char* password = "bored8240county";

const char* GS_IP   = "255.255.255.255";  // broadcast
const int   GS_PORT = 5005;

WiFiUDP udp;
HardwareSerial STM32Serial(2);

struct __attribute__((packed)) TelemetryPacket {
    uint8_t  start;
    float    roll;
    float    pitch;
    float    altitude;
    float    latitude;
    float    longitude;
    float    throttle;
    float    m1, m2, m3, m4;
    uint8_t  armed;
    uint8_t  gps_fix;
    uint8_t  satellites;
    uint8_t  checksum;
};

struct __attribute__((packed)) RCPacket {
  uint8_t start;     //0xAB
  float throttle;    //0.0 to 100.0
  float roll;        // -45 to 45 deg
  float pitch;       // -45 to 45 deg
  float yaw;         //-100 to 100
  uint8_t armed;     //1 = armed
  uint8_t checksum; // XOR of bytes excpet start and cheksum
};

#define TELE_START   0xCD
#define TELE_SIZE    sizeof(TelemetryPacket)

uint8_t tele_buf[TELE_SIZE];
uint8_t tele_idx = 0;
uint8_t tele_synced = 0;

uint8_t calcTeleChecksum(TelemetryPacket *pckt)
{
    uint8_t *bytes = (uint8_t *)pckt;
    uint8_t chk = 0;
    for (int i = 1; i < (int)TELE_SIZE - 1; i++)
        chk ^= bytes[i];
    return chk;
}

//wifi end
GamepadPtr myGameP = nullptr;

#define PACKET_START 0XAB
#define PACKET_SIZE  23

uint8_t lastButtonState = 0;
uint8_t bstate = 0;

void onConnectGameP(GamepadPtr gp){
  myGameP = gp;
  Serial.println("PS Controller Connected");
}

void onDisconnectGameP(GamepadPtr gp){
  myGameP = nullptr;
  Serial.println("PS Controller Disconnected");
}

float mapAxis(int32_t raw, float out_min, float out_max){
  float normal = raw / 512.0f;
  normal = constrain(normal, -1.0f, 1.0f);
  return normal * (out_max - out_min) / 2.0f + (out_min + out_max) / 2.0f;
}

uint8_t calcCheckSum(RCPacket *pkt){
  uint8_t *bytes = (uint8_t *) pkt;
  uint8_t chk = 0;
  for (int i = 1; i < sizeof(RCPacket) -1; i++)
    chk ^= bytes[i];
  return chk;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  STM32Serial.begin(115200, SERIAL_8N1, 16, 17);

  BP32.setup(&onConnectGameP, &onDisconnectGameP);
  BP32.forgetBluetoothKeys();
  Serial.println("Waiting for controller");
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wifi");
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  udp.begin(GS_PORT);
}

void loop() {
  // put your main code here, to run repeatedly:
  BP32.update();

  if (myGameP && myGameP->isConnected())
  {
    RCPacket pkt;
    pkt.start = PACKET_START;

    //left stick y is throttle, negative because ps y is inverted
    float raw_throttle = (myGameP->throttle() / 1023.0f) * 100.0f;
    pkt.throttle = raw_throttle;
    
    //right stick x roll
    pkt.roll = mapAxis(myGameP->axisRX(), -45.0f, 45.0f);

    //right stick y pitch
    pkt.pitch = mapAxis(-myGameP->axisRY(), -45.0f, 45.0f);

    //left stick x = yaw arte
    pkt.yaw = mapAxis(myGameP->axisX(), -100.0f, 100.0f);

    pkt.armed = myGameP->a();
    if (pkt.armed == 1 && lastButtonState == 0){
      bstate ^= 1;
    }
    lastButtonState = pkt.armed;

    pkt.armed = bstate;
    if (pkt.armed == 1)
    {
      myGameP->setColorLED(0, 255, 0); //green
    } else {
      myGameP->setColorLED(255, 0, 0); //red
    }
    pkt.checksum = calcCheckSum(&pkt);

    STM32Serial.write((uint8_t *)&pkt, sizeof(RCPacket));

    Serial.printf("T:%.1f R:%.1f P:%.1f Y:%.1f ARM:%d\n", pkt.throttle, pkt.roll, pkt.pitch, pkt.yaw, pkt.armed);
  }

  while (STM32Serial.available()){
    uint8_t byte = STM32Serial.read();

    if  (!tele_synced){
      if (byte == TELE_START){
        tele_buf[0] = byte;
        tele_idx = 1;
        tele_synced = 1;
      }
    } else {
      tele_buf[tele_idx++] = byte;
      if (tele_idx >= TELE_SIZE){
        tele_synced = 0;
        tele_idx = 0;

        TelemetryPacket *pckt= (TelemetryPacket *) tele_buf;
        uint8_t expected = calcTeleChecksum(pckt);

        if (pckt->checksum == expected){
          udp.beginPacket(GS_IP, GS_PORT);
          udp.write(tele_buf, TELE_SIZE);
          udp.endPacket();

          Serial.printf("[TEL] R:%.1f P:%.1f Alt:%.1f ARM:%d\n", pckt->roll, pckt->pitch, pckt->altitude, pckt->armed);
        }
      }
    }
  }

  delay(20);
}
