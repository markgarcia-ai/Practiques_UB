//LIN MASTER 
#include <SoftwareSerial.h>


#define rxPin 2
#define txPin 3

#define LED_PIN 6
#define BUTTON_PIN 5

const int redLED = 7;
const int greenLED = 6;
uint8_t buttonState = 0;

bool unconditionalOrEvent = 0;

int isLED_GREEN_ON = 0;

// Set up a new SoftwareSerial object
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

int prevButtonState = HIGH;

char Mymessage[3]; //String data

  void setup() {

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    // Define pin modes for TX and RX
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    // Define pin mode for red LED
    pinMode(redLED , OUTPUT);  
    // Define pin mode for green LED
    pinMode(greenLED , OUTPUT);  
    // initialize the pushbutton pin as an input:
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    // Set the baud rate for the Serial port
    Serial.begin(115200);
    //Set the baud rate for the SoftwareSerial object
    mySerial.begin(115200);

  }
  void loop() {
    //digitalWrite(LED_PIN, HIGH);

    send_header(0x38); //we send 0x38 every 1 second 
    
      //if we press the button we change the mode
    int buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW && prevButtonState == HIGH) {
      // Button has been pressed
      isLED_GREEN_ON = !isLED_GREEN_ON; // Toggle LED state
      unconditionalOrEvent = !unconditionalOrEvent; //change from inconditional to event trigger or viceversa
      digitalWrite(LED_PIN, isLED_GREEN_ON ? HIGH : LOW); // Set LED state based on flag
      Serial.println("Button Pressed");

    if (unconditionalOrEvent) { //activa el modo event trigger
      send_header(0x32);
    }
    else {
      send_header(0x35); //modo unconditional 
  }

    }
    // Update previous button state
    prevButtonState = buttonState;

    delay(1000);

  byte dataSlaveLin[5];
  //wait for serial to accumulate 3 bytes, the bytes that the master sends.
  if(mySerial.available()>4){

    for(int i=0; i <= 4; i++){
      dataSlaveLin[i] = mySerial.read(); // read the bytes from the buffer
      //Serial.println("CRC SLAVE = ", )
    }

  //calculate the crc of the received data  
  byte crc = 0;
    for (int i = 0; i < 4; i++) {
      crc += dataSlaveLin[i];
  }
  crc = ~crc; // Invert bits
  Serial.println(crc, HEX);
  if (crc != dataSlaveLin[4]) {
    digitalWrite(redLED , HIGH); //crc wrong -> RED LED high
  }
  else {
    digitalWrite(redLED , LOW);
  }
    //print slave data
    Serial.print("Slave Data: ");
    Serial.print(dataSlaveLin[0],HEX);
    Serial.print(", ");
    Serial.print(dataSlaveLin[1],HEX);
    Serial.print(", ");
    Serial.print(dataSlaveLin[2],HEX);
    Serial.print(", ");
    Serial.print(dataSlaveLin[3],HEX);
    Serial.print(", ");
    Serial.println(dataSlaveLin[4],HEX);
    }
    
  }
/*
We use this function to send the header to the slaves
*/
uint8_t send_header(uint8_t Frame_id){
    uint8_t P0 = (((Frame_id >> 0) & 0x01) ^ ((Frame_id >> 1) & 0x01) ^ ((Frame_id >> 2) & 0x01) ^ ((Frame_id >> 4) & 0x01));
    //Serial.println(P0, BIN);
    uint8_t P1 = !(((Frame_id >> 1) & 0x01) ^ ((Frame_id >> 3) & 0x01) ^ ((Frame_id >> 4) & 0x01) ^ ((Frame_id >> 5) & 0x01));

    mySerial.begin(115200);
    mySerial.write((byte) 0x00); //send the sync break field. We have to cast the 00 so the compiler couldn't interpret 0 as a pointer
    //we can change the baudrate here to send at least 13 bits but we get an error because of the start bit
    mySerial.begin(115200);
    Mymessage[0] =  (byte)0x55; //send sync field (auto baud detection slave) 
    Mymessage[1] = ((Frame_id & 0x3F) | (P0 << 6) | (P1 << 7));//Frame identifier bits & parity bits
    mySerial.write(Mymessage);
    Serial.print(Frame_id, HEX);
    Serial.print(" ");
    Serial.println(Mymessage[1], HEX);

    //This prints is to ensure that the PID is correct with different IDs
    Serial.print("P0 = ");
    Serial.println(P0, BIN);
    Serial.print("P1 = ");
    Serial.println(P1, BIN);
    Serial.print("ID = ");
    Serial.println(Frame_id, BIN);
    Serial.println(Frame_id >> 1, BIN);
    Serial.println(Frame_id >> 3, BIN);
    Serial.println(Frame_id >> 4, BIN);
    Serial.println(Frame_id >> 5, BIN);
    Serial.println((Frame_id >> 1) ^ (Frame_id >> 3), BIN);
    

}
