#include <RH_ASK.h>
#include <SPI.h>
#include <Servo.h>

RH_ASK rfDriver;

struct Packet{
  byte data[3];
  byte checksum;
};

struct Claw{
  byte button[1];
  byte checksum1;
};


int joy1PinX = A0; // Joystick 1 X-axis pin
int joy1PinY = A1; // Joystick 1 Y-axis pin
int buttonPin = 7; // Button pin
int SpeedMode = 6;
bool button2State = false;
int servo1Position = 90;
int servo2Position = 90;
bool servo3State = false;

void setup() {
  rfDriver.init();
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(SpeedMode, INPUT_PULLUP);

}

void loop() {

  Packet packet;
  Claw claw;
  // Read the joystick values and button state
  int joy1X = analogRead(joy1PinX);
  int joy1Y = analogRead(joy1PinY);
  bool buttonState = digitalRead(buttonPin);
  bool SpeedState = digitalRead(SpeedMode);

  // Map the joystick values to servo positions
  servo1Position = map(joy1X, 0, 1023, 0, 180);
  servo2Position = map(joy1Y, 0, 1023, 180, 0);

  // Send the servo data over RF
  packet.data[0] = servo1Position;
  packet.data[1] = servo2Position;

  if (SpeedState == LOW && !button2State) {
    button2State = true;
    packet.data[2] = 1;
  } else if (SpeedState == LOW && button2State) {
    button2State = false;
    packet.data[2] = 0;
  }
  
  packet.checksum = crc8(packet.data, 3);

  rfDriver.send((uint8_t *)&packet, sizeof(packet));
  rfDriver.waitPacketSent();
  Serial.println("Transmitted");

  
  // Send the button data over RF when the button is pressed
  if (buttonState == LOW && !servo3State) {
    servo3State = true;
    claw.button[0] = 1;
  } else if (buttonState == LOW && servo3State) {
    servo3State = false;
    claw.button[0] = 0;
  }
  if (buttonState == LOW) {
      claw.checksum1 = crc8(claw.button, 1);
      rfDriver.send((uint8_t *)&claw, sizeof(claw));
      rfDriver.waitPacketSent();
  }


  // Print debug information
  Serial.print("Servo1: ");
  Serial.print(servo1Position);
  Serial.print(" Servo2: ");
  Serial.print(servo2Position);
  Serial.print(" Button: ");
  Serial.println(buttonState);
  
  // Add a small delay to prevent sending data too quickly
  delay(10);
}
  
  byte crc8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte inbyte = *data++;
    for (byte i = 8; i; i--) {
      byte mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) {
        crc ^= 0x8C;
      }
      inbyte >>= 1;
    }
  }
  return crc;
  }

