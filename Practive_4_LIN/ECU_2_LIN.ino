
#include <SoftwareSerial.h>

#define LED_PIN 6
#define BUTTON_PIN 7
#define POT_PIN A0
#define EVENT_TRIGGERED_PID 0x32
#define UNCONDITIONAL_TRIGGERED_PID 0x35

int isLED_GREEN_ON = 0;

int isEventOrUnconditional = 1;  //empezamos como unconditional

const byte HEADER_ID = 0x55;
const byte PID_FIELD = 0x38;

SoftwareSerial mySerial(2, 3);  // RX, TX

// Initialize button state variable
int prevButtonState = HIGH;
unsigned int lastSentPotValue = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  //pinMode(BUTTON_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial)
    ;
  mySerial.begin(115200);

  //CAN MESSAGE
  uint8_t data[4];
  data[0] = 0x33;
  data[1] = 0x22;
  data[2] = 0x11;
  data[3] = 0x00;
  uint8_t ECU2_CRC = 0xCC;
} 



void loop() {
  // //BUTTON
  // int buttonState = digitalRead(BUTTON_PIN);
  // if (buttonState == LOW && prevButtonState == HIGH) {
  //   // Button has been pressed
  //   isLED_GREEN_ON = !isLED_GREEN_ON;                    // Toggle LED state
  //   digitalWrite(LED_PIN, isLED_GREEN_ON ? HIGH : LOW);  // Set LED state based on flag
  //   Serial.println("Button Pressed");
  // }
  // // Update previous button state
  // prevButtonState = buttonState;


  // Send data
  unsigned int time = millis();
  unsigned int potValue = analogRead(A0);
  uint8_t data[5];
  data[0] = (time >> 8) & 0xFF;      // D3
  data[1] = time & 0xFF;             // D2
  data[2] = (potValue >> 8) & 0xFF;  // D1
  data[3] = potValue & 0xFF;         // D0
  byte crc = 0;
  for (int i = 0; i < 4; i++) {
    crc += data[i];
  }
  crc = ~crc;     // Invert bits
  data[4] = crc;  // Add CRC byte
  // mySerial.write(data, 5);





  byte masterLINByte[3];
  //mySerial.listen();
  //wait for serial to accumulate 3 bytes, the bytes that the master sends.
  if (mySerial.available() > 2) {
    masterLINByte[0] = mySerial.read();
    masterLINByte[1] = mySerial.read();
    masterLINByte[2] = mySerial.read();
    Serial.print("Master Data: ");
    Serial.print(masterLINByte[0], HEX);
    Serial.print(", ");
    Serial.print(masterLINByte[1], HEX);
    Serial.print(", ");
    Serial.println(masterLINByte[2], HEX);

    if (masterLINByte[0] != 0x00) {
      Serial.println("Signal not detected");
    } else {
      if (masterLINByte[2] == 0x32) {  //EVENT_TRIGGERED_PID (0x32)
        isEventOrUnconditional = 0;
        Serial.println("SLAVE MODE: EVENT TRIGGERED");
      } else if (masterLINByte[2] == 0x75) {  //UNCONDITIONAL_TRIGGERED_PID (0x35)
        isEventOrUnconditional = 1;
        Serial.println("SLAVE MODE: UNCONDITIONAL");
      }

      if (masterLINByte[1] != HEADER_ID) {
        Serial.println("Signal not Synced");
      } else if (masterLINByte[2] == 0x78) {                                    //master requests data.
        if ((isEventOrUnconditional == 1) || (lastSentPotValue != potValue)) {  //if its on unconditional or the new pot value differs from the last sent one, we send new info.
          lastSentPotValue = potValue;
          for (int i = 0; i < 5; i++) {
            mySerial.write(data[i]);
          }
          int i = 0;
          //mySerial.write(data);
          Serial.print("Slave Data: ");
          //Wait for 1 byte of header data to be received
          for (int i = 0; i < 4; i++) {
            Serial.print(data[i], HEX);
            Serial.print(", ");
          }
          Serial.println(data[4], HEX);
          i = 0;
        }
      }
    }

    //delete remaining data if any is left since it shouldnt be there or hasnt been sent out correctly
    //if it is multiple of 3 it probably means it is another header
    while ((mySerial.available() % 3) != 0) {
      mySerial.read();
    }
  }
}