#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <BluetoothSerial.h> // 블루투스 시리얼 라이브러리 추가

BluetoothSerial SerialBT;   // 블루투스 시리얼 객체 생성

// connection pins
const uint8_t PIN_SCK = 18;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_SS = 4;
const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17;

// messages used in the ranging protocol
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

// message flow state
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;
uint64_t timeComputedRange;

// last computed range/time
#define LEN_DATA 16
byte data[LEN_DATA];

// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

device_configuration_t DEFAULT_CONFIG = {
  false,
  true,
  true,
  true,
  false,
  SFDMode::STANDARD_SFD,
  Channel::CHANNEL_5,
  DataRate::RATE_850KBPS,
  PulseFrequency::FREQ_64MHZ,
  PreambleLength::LEN_256,
  PreambleCode::CODE_9
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
  true,
  true,
  true,
  false,
  true
};

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("### DW1000Ng-arduino-ranging-anchor ###"));
  
  // 블루투스 초기화
  SerialBT.begin("ESP32_UWB"); // 블루투스 장치 이름 설정
  Serial.println("Bluetooth Started");

  // initialize the driver
  DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
  Serial.println(F("DW1000Ng initialized ..."));
  
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
  DW1000Ng::setDeviceAddress(1);
  DW1000Ng::setAntennaDelay(16436);

  Serial.println(F("Committed configuration ..."));
  
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000Ng::getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000Ng::getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  
  // attach callback for (successfully) sent and received messages
  DW1000Ng::attachSentHandler(handleSent);
  DW1000Ng::attachReceivedHandler(handleReceived);

  receiver();
  noteActivity();
  
  // for first time ranging frequency computation
  rangingCountPeriod = millis();
}

void noteActivity() {
  lastActivity = millis();
}

void resetInactive() {
  expectedMsgId = POLL;
  receiver();
  noteActivity();
}

void handleSent() {
  sentAck = true;
}

void handleReceived() {
  receivedAck = true;
}

void transmitPollAck() {
  data[0] = POLL_ACK;
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitRangeReport(float curRange) {
  data[0] = RANGE_REPORT;
  memcpy(data + 1, &curRange, 4);
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void transmitRangeFailed() {
  data[0] = RANGE_FAILED;
  DW1000Ng::setTransmitData(data, LEN_DATA);
  DW1000Ng::startTransmit();
}

void receiver() {
  DW1000Ng::forceTRxOff();
  DW1000Ng::startReceive();
}

void loop() {
  int32_t curMillis = millis();
  if (!sentAck && !receivedAck) {
    if (curMillis - lastActivity > resetPeriod) {
      resetInactive();
    }
    return;
  }
  if (Serial.available()){
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()){
    SerialBT.write(SerialBT.read());
  }
  
  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];
    if (msgId == POLL_ACK) {
      timePollAckSent = DW1000Ng::getTransmitTimestamp();
      noteActivity();
    }
    DW1000Ng::startReceive();
  }
  
  if (receivedAck) {
    receivedAck = false;
    DW1000Ng::getReceivedData(data, LEN_DATA);
    byte msgId = data[0];
    if (msgId != expectedMsgId) {
      protocolFailed = true;
    }
    if (msgId == POLL) {
      protocolFailed = false;
      timePollReceived = DW1000Ng::getReceiveTimestamp();
      expectedMsgId = RANGE;
      transmitPollAck();
      noteActivity();
    }
    else if (msgId == RANGE) {
      timeRangeReceived = DW1000Ng::getReceiveTimestamp();
      expectedMsgId = POLL;
      if (!protocolFailed) {
        timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
        timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
        timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
        double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                          timePollReceived,
                          timePollAckSent,
                          timePollAckReceived,
                          timeRangeSent,
                          timeRangeReceived);
        distance = DW1000NgRanging::correctRange(distance);
        distance -= 0.40;
        
        
        // 시리얼 모니터에 출력
        Serial.println(distance, 2);
        SerialBT.println(distance, 2);
        delay(4000);
        /*// 거리 값에 따라 앱인벤터로 신호 전송
        if (distance <= 0.30) {
        SerialBT.println("1");  // 거리값이 0.30 이하이면 1 전송
        } else if (distance > 0.30 && distance <= 0.40) {
        SerialBT.println("2");  // 거리값이 0.30 초과 0.40 이하이면 2 전송
        } else if (distance > 0.40 && distance <= 0.50) {
        SerialBT.println("3");  // 거리값이 0.40 초과 0.50 이하이면 3 전송
        } else if (distance > 0.50 && distance <= 0.60) {
        SerialBT.println("4");  // 거리값이 0.50 초과 0.60 이하이면 4 전송
        } else if (distance > 0.60 && distance <= 0.70) {
        SerialBT.println("5");  // 거리값이 0.60 초과 0.70 이하이면 5 전송
        } //else {
        //SerialBT.println(String(distance, 2));  // 그 외의 거리값은 그대로 전송
        */
        transmitRangeReport(distance * DISTANCE_OF_RADIO_INV);
        successRangingCount++;
        if (curMillis - rangingCountPeriod > 1000) {
          samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
          rangingCountPeriod = curMillis;
          successRangingCount = 0;
        }
      }
      else {
        transmitRangeFailed();
      }
      noteActivity();
    }
  }
}
