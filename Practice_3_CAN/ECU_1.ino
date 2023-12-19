#include <SPI.h>
#include <mcp2515.h>
#include <LiquidCrystal_I2C.h>

#define LED_PIN 6
#define BUTTON_PIN 7
#define POT_PIN A0
int isLED_RED_ON = 0;

//CAN
struct can_frame canMsg_Rx;  //MSG to receive ECU2's message
struct can_frame canMsg1;
MCP2515 mcp2515(10);


void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  //CAN MESSAGE
  canMsg1.can_id = 0x036;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x8E;
  canMsg1.data[1] = 0x87;
  canMsg1.data[2] = 0x32;
  canMsg1.data[3] = 0xFA;
  canMsg1.data[4] = 0x26;
  canMsg1.data[5] = 0x8E;
  canMsg1.data[6] = 0xBE;
  canMsg1.data[7] = 0x86;

  while (!Serial)
    ;
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("Example: Write to CAN");
}



void loop() {
  //BUTTON
  // if (digitalRead(BUTTON_PIN) == LOW) {
  //   digitalWrite(LED_PIN, HIGH);
  //   isLED_RED_ON = 1;
  // }
  // else {
  //   digitalWrite(LED_PIN, LOW);
  //   isLED_RED_ON = 0;
  // }

  //POTENTIOMETER
  int potValue = analogRead(POT_PIN);
  int potValue8b = potValue / 4;
  analogWrite(LED_PIN, potValue8b);
  //Serial.println(potValue8b);

  if (potValue8b < 128) {
    digitalWrite(LED_PIN, LOW);
    isLED_RED_ON = 0;
  } else {
    digitalWrite(LED_PIN, HIGH);
    isLED_RED_ON = 1;
  }


  //read whatever is on the CAN BUS
  if (mcp2515.readMessage(&canMsg_Rx) == MCP2515::ERROR_OK) {
    // Serial.print(canMsg_Rx.can_id, HEX); // print ID
    // Serial.print(" ");
    // Serial.print(canMsg_Rx.can_dlc, HEX); // print DLC
    // Serial.print(" ");
    for (int i = 0; i < canMsg_Rx.can_dlc; i++) {  // print the data
      Serial.print(canMsg_Rx.data[7 - i], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }


  // calculo CRC
  int RxlistLength = sizeof(canMsg_Rx.data) / sizeof(canMsg_Rx.data[0]);
  uint8_t RxCRC = 0;                                  //In this protocol, you sum the 7 bytes of data and put it in the 8th byte, tossing out the overflowed bits
  for (uint8_t i = 0; i < (RxlistLength - 1); i++) {  //RxlistLength-1 as we don't add the last number
    RxCRC += canMsg_Rx.data[i];
  }  // Sum Rx data
  //Serial.print("Rx CRC check: ");
  //Serial.println(RxCRC);
  RxlistLength = 0;
  RxCRC = 0;

  if (RxCRC != canMsg_Rx.data[0]) {  //check ECU2 CRC
    Serial.print("ERROR: ECU2 MSG DOESN'T HAVE A MATCHING CRC");
  }

  //obtain time values
  uint32_t currTime = millis();
  uint8_t timeB0 = currTime;
  uint8_t timeB1 = (currTime >> 8);
  uint8_t timeB2 = (currTime >> 16);

  //calculate message and send it
  uint8_t myCRC = isLED_RED_ON + timeB0 + timeB1 + timeB2 + 0xFF + potValue8b + 0xFF;
  //Serial.print("myCRC: ");
  //Serial.println(myCRC, HEX);

  canMsg1.data[0] = myCRC;
  canMsg1.data[1] = isLED_RED_ON;
  canMsg1.data[2] = timeB0;
  canMsg1.data[3] = timeB1;
  canMsg1.data[4] = timeB2;
  canMsg1.data[5] = 0xFF;
  canMsg1.data[6] = potValue8b;
  canMsg1.data[7] = 0xFF;

  mcp2515.sendMessage(&canMsg1);

  delay(1000);
}
