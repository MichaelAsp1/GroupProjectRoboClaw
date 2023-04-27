#include <RH_ASK.h>
#include <SPI.h>
#include <Servo.h>

RH_ASK rfDriver;
Servo servo1;
Servo servo2;
Servo servo3;

struct Packet {
  byte data[3];
  byte checksum;
};

struct Claw {
  byte button[1];
  byte checksum1;
};

int servo1Pin = 12;
int servo2Pin = 13;
int servo3Pin = 9;
int servo1Position = 90;
int servo2Position = 90;
int servo3Position = 0;
bool servo3State = false;
int multiplier = 0;
bool SpeedState = false;

unsigned long lastPacketTime = 0;
unsigned long timeout = 500;

void Move(int rightwheel, int leftwheel, float multiplier);

void setup() {
  rfDriver.init();
  Serial.begin(9600);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
}

void loop() {
  //servo1.attach(servo1Pin);
  //servo2.attach(servo2Pin);
  // Check for incoming RF packets
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  
  if (rfDriver.recv(buf, &buflen)) {
    lastPacketTime = millis();
    if (buflen == sizeof(Packet)){
      Packet packet;
      memcpy(&packet, buf, sizeof(packet));
      
      byte expectedChecksum = crc8(packet.data, 3);

      if (packet.checksum == expectedChecksum) {
        // Packet received successfully
        Serial.println("Packet received successfully");
        Serial.print("Data: ");

        servo1Position = packet.data[0];
        servo2Position = packet.data[1];
        SpeedState = packet.data[2];
        if (SpeedState) {
          multiplier = 1;
        } else {
          multiplier = 0;
        }

        int left_wheel = map(servo1Position, 0, 180, 1300, 1700);
        int right_wheel = map(servo2Position, 0, 180, 1300, 1700);

        Serial.print("Received servo data: ");
        Serial.print(right_wheel);
        Serial.print(", ");
        Serial.println(left_wheel);
   
        Move(right_wheel, left_wheel, multiplier);

      }else{
        Serial.println("Checksum error");
      }
    }
    if (buflen == sizeof(Claw)){
      Claw claw;
      memcpy(&claw, buf, sizeof(claw));
       // Calculate CRC-8 checksum
      byte expectedChecksum = crc8(claw.button, 1);

      if (claw.checksum1 == expectedChecksum) {
        // Packet received successfully
        Serial.println("Packet received successfully");
        Serial.print("Data: ");
         // Update the state of the button servo based on the received data
        servo3State = claw.button[0];
        if (servo3State) {
          servo3Position = 105;
        } else {
          servo3Position = 45;
        }
        Serial.print(servo3Position);
        servo3.write(servo3Position);
      }

    }
  }
  
unsigned long elapsedTime = millis() - lastPacketTime;
if (elapsedTime > timeout) {
  // Stop the servo movement if no packets have been received recently
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  Serial.println("No data received. Stopping servo movement.");
  }
}


void Move(int rightwheel, int leftwheel, float multiplier) {

  if (((leftwheel >= 1490) && (leftwheel <= 1510)) && ((rightwheel >= 1490) && (rightwheel <= 1510))) {
    //STOP
    servo1.writeMicroseconds(1500);
    servo2.writeMicroseconds(1500);
  }
  if ((leftwheel >= 1690) && ((rightwheel >= 1300) && (rightwheel <= 1700))) {
    //forward FAST
    servo1.writeMicroseconds(1700 - ((multiplier)*170));
    servo2.writeMicroseconds(1300 + ((multiplier)*170));

  } else if (((rightwheel >= 1290) && (rightwheel <= 1310)) && ((leftwheel >= 1300) && (leftwheel <= 1700))) {
    //left FAST
    servo1.writeMicroseconds(1560 - ((multiplier)*30));
    servo2.writeMicroseconds(1560 - ((multiplier)*30));

  } else if (((leftwheel >= 1290) && (leftwheel <= 1310)) && ((rightwheel >= 1300) && (rightwheel <= 1700))) {
    //back FAST
    servo1.writeMicroseconds(1300 + ((multiplier)*170));
    servo2.writeMicroseconds(1700 - ((multiplier)*170));

  } else if ((rightwheel >= 1690) && ((leftwheel >= 1300) && (leftwheel <= 1700))) {
    //right
    servo1.writeMicroseconds(1440 + ((multiplier)*30));
    servo2.writeMicroseconds(1440 + ((multiplier)*30));
  }
  Serial.print("Multiplier: ");
  Serial.println(multiplier);
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
