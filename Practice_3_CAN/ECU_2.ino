
#include <mcp2515.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define LED_PIN 6
#define BUTTON_PIN 7

int isLedGreenOn = 0;
uint16_t LDR_value = 0;
uint32_t currTime = 0;
uint8_t TimeB0 = 0;
uint8_t TimeB1 = 0;
uint8_t TimeB2 = 0;
uint8_t RxlistLength = 0;
uint8_t RxCRC = 0;
uint8_t CRC_status = 0;


struct can_frame canMsg_Rx;
struct can_frame canMsg1;

MCP2515 mcp2515(10);

void setup() {


  lcd.init();  // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("I love EACE!");


  while (!Serial);
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //set the can speed and can clock
  mcp2515.setNormalMode();

  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {


   uint16_t LDR_value = analogRead(A0); // segona part
   LDR_value = map(LDR_value, 0, 1023, 0, 255);

  // if (digitalRead(BUTTON_PIN) == LOW){
  //   digitalWrite(LED_PIN, HIGH);
  //   isLedGreenOn = 1;
  // }
  // else {
  //   digitalWrite(LED_PIN, LOW);
  //   isLedGreenOn = 0;
  // }


  if (mcp2515.readMessage(&canMsg_Rx) == MCP2515::ERROR_OK) {

    if (canMsg1.can_id == 0x36) {

      mcp2515.sendMessage(&canMsg1);

      uint8_t RxlistLength = sizeof(canMsg_Rx.data) / sizeof(canMsg_Rx.data[0]);
      uint8_t RxCRC = 0;

      for (uint8_t i = 0; i < (RxlistLength - 1); i++) {
        RxCRC += canMsg_Rx.data[i];
      }  // Sum Rx data

      if (RxCRC = canMsg_Rx.data[0]) {
        CRC_status = 1;
      } else {
        CRC_status = 0;
      }

      RxlistLength = 0;
      RxCRC = 0;

      if (canMsg_Rx.data[6] < 128) {
        digitalWrite(LED_PIN, LOW);
        isLedGreenOn = 0;
      } else {
        digitalWrite(LED_PIN, HIGH);
        isLedGreenOn = 1;
      }

      for (int i = 0; i < canMsg_Rx.can_dlc; i++) {  // print the data
        Serial.print(canMsg_Rx.data[7-i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    currTime = millis();
    TimeB0 = currTime;
    TimeB1 = (currTime >> 8);
    TimeB2 = (currTime >> 16);

    int CRC = isLedGreenOn + TimeB0 + TimeB1 + TimeB2 + LDR_value;

    canMsg1.can_id = 0x36;
    canMsg1.can_dlc = 8;
    canMsg1.data[0] = CRC;  //CRC
    canMsg1.data[1] = isLedGreenOn;
    canMsg1.data[2] = TimeB0;
    canMsg1.data[3] = TimeB1;
    canMsg1.data[4] = TimeB2;
    canMsg1.data[5] = LDR_value;
    canMsg1.data[6] = 00;
    canMsg1.data[7] = 00;

    lcd.setCursor(0, 1);
    lcd.print("Pot=");
    lcd.setCursor(4, 1);
    lcd.print(canMsg_Rx.data[6]);
    lcd.setCursor(7, 1);
    lcd.print("LDR=");
    lcd.setCursor(11, 1);
    lcd.print(LDR_value);
  }
  //delay(500);
}
