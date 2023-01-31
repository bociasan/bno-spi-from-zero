// #include <Arduino.h>
// #include <SPI.h>

// #define BNO08X_CS D8
// #define BNO08X_INT D1
// #define BNO08X_RESET D2

// #define MAX_PACKET_SIZE 128 // Packets can be up to 32k but we don't have that much RAM.
// uint8_t shtpHeader[4];      // Each packet has a header of 4 bytes
// // uint8_t shtpData[MAX_PACKET_SIZE];
// uint8_t shtpDataOut[MAX_PACKET_SIZE];
// uint8_t shtpDataIn[MAX_PACKET_SIZE];

// void waitForSpi()
// {

//   // Serial.print("\n\nWaiting for SPI ");

//   // unsigned long startingTime = millis();

//   // for (int i = 0; i < 2000; i++)
//   // {
//   //   // Serial.print(".");
//   //   if (!digitalRead(BNO08X_INT))
//   //   {
//   //     // Serial.println("SPI SUCCESS!!");
//   //     break;
//   //   }
//   //   delay(1);
//   // }

//   while (digitalRead(BNO08X_INT))
//   {
//     // delay(1);
//   }
//   // unsigned long finishTime = millis();
//   // unsigned long duration = finishTime - startingTime;
//   // Serial.printf("SPI connected in %ld ms.", duration);
//   // // Serial.println();
// }

// void transferArrayOfBytes(int dataLength)
// {
//   digitalWrite(BNO08X_CS, LOW);
//   uint16_t dataSpot = 0;
//   while (dataLength > 0)
//   {
//     if (dataLength > 128)
//     {
//       dataSpot = 128;
//       dataLength -= dataSpot;
//     }
//     else
//     {
//       dataSpot = dataLength;
//       dataLength = 0;
//     }
//     memset(shtpDataOut, 0xFF, dataSpot);
//     SPI.transferBytes(shtpDataOut, shtpDataIn, dataSpot);
//   }
//   digitalWrite(BNO08X_CS, HIGH);
// }

// char buffer[128];
// int sequence = 0;
// void loopTransmission()
// {
//   waitForSpi();
//   digitalWrite(BNO08X_CS, LOW);
//   SPI.transfer(2);
//   SPI.transfer(0);
//   SPI.transfer(2);
//   SPI.transfer(sequence++);

//   SPI.transfer(0xf9);
//   SPI.transfer(0);

//   digitalWrite(BNO08X_CS, HIGH);
//   // transferByteByByte(dataLength);
//   waitForSpi();

//   digitalWrite(BNO08X_CS, LOW);

//   memset(shtpHeader, 0, 4);
//   SPI.transferBytes(shtpHeader, shtpHeader, 4);
//   for (int i = 0; i < (((uint16_t)shtpHeader[1]) << 8) | ((uint16_t)shtpHeader[0]); i++)
//   {
//     Serial.printf("%02X ", SPI.transfer(255));
//   }
//   Serial.println();

//   digitalWrite(BNO08X_CS, HIGH);
// }

// void setup()
// {
//   // pinMode(BNO08X_INT, INPUT_PULLUP);
//   pinMode(BNO08X_INT, INPUT);
//   pinMode(BNO08X_RESET, OUTPUT);
//   pinMode(BNO08X_CS, OUTPUT);
//   Serial.begin(115200);
//   digitalWrite(BNO08X_CS, HIGH);
//   digitalWrite(BNO08X_RESET, LOW);
//   delay(2);
//   digitalWrite(BNO08X_RESET, HIGH);
//   digitalWrite(BNO08X_CS, HIGH);

//   SPI.setDataMode(0x11);
//   SPI.setBitOrder(MSBFIRST);
//   SPI.setFrequency(1000000);
//   SPI.begin(); /* begin SPI */
//   // delay(500);

//   waitForSpi();

//   // loopTransmission();
//   // loopTransmission();
// }

// void loop()
// {
//   // // Serial.printf("%02X ", SPI.transfer(0));

//   loopTransmission();
//   delay(1000);
// }