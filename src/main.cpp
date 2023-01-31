#include <Arduino.h>
#include <SPI.h>

#define BNO08X_CS D8
#define BNO08X_INT D1
#define BNO08X_RESET D2

#define MAX_PACKET_SIZE 128 // Packets can be up to 32k but we don't have that much RAM.
uint8_t shtpHeader[4];      // Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];

void waitForSpi()
{

    Serial.print("\n\nWaiting for SPI ");
    unsigned long startingTime = millis();
    // for (int i = 0; i < 2000; i++)
    // {
    //   Serial.print(".");
    //   if (!digitalRead(BNO08X_INT))
    //   {
    //     Serial.println("SPI SUCCESS!!");
    //     break;
    //   }
    //   delay(1);
    // }

    while (digitalRead(BNO08X_INT))
    {
        // delay(1);
    }
    unsigned long finishTime = millis();
    unsigned long duration = finishTime - startingTime;
    Serial.printf("SPI connected in %ld ms.", duration);
    // Serial.println();
}

void transferByteByByte(int dataLength)
{
    for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++)
    {
        // uint8_t incoming = SPI.transfer(0xFF);
        Serial.printf("%02X ", SPI.transfer(0xFF));
    }
}

void printArrayOfHex(uint8_t *array, uint16_t length)
{
    for (int index = 0; index < length; index++)
    {
        Serial.printf("%02X ", array[index]);
        // Serial.write(lowByte(array[index]));
    }
    // Serial.println();
}

void transferArrayOfBytes(int dataLength)
{
    //   digitalWrite(BNO08X_CS, LOW);
    uint16_t dataSpot = 0;
    while (dataLength > 0)
    {
        if (dataLength > 128)
        {
            dataSpot = 128;
            dataLength -= dataSpot;
        }
        else
        {
            dataSpot = dataLength;
            dataLength = 0;
        }
        memset(shtpData, 0xFF, sizeof(dataSpot));
        SPI.transferBytes(shtpData, shtpData, sizeof(shtpData));
        printArrayOfHex(shtpData, dataSpot);
    }
    //   digitalWrite(BNO08X_CS, HIGH);
}

void print(char *message)
{
    Serial.print(message);
}
void println(char *message)
{
    Serial.println(message);
}

char buffer[128];

void loopTransmission()
{
    digitalWrite(BNO08X_CS, LOW);
    memset(shtpHeader, 0, sizeof(shtpHeader));
    SPI.transferBytes(shtpHeader, shtpHeader, sizeof(shtpHeader));

    snprintf(buffer, sizeof(buffer), "Header: 0x%02x 0x%02x 0x%02x 0x%02x  |  ", shtpHeader[0], shtpHeader[1], shtpHeader[2], shtpHeader[3]);
    print(buffer);

    uint16_t dataLength = (((uint16_t)shtpHeader[1]) << 8) | ((uint16_t)shtpHeader[0]);
    
    snprintf(buffer, sizeof(buffer), "Incoming data length: %d. Begin transmision!\n", dataLength);
    print(buffer);

    dataLength &= ~(1 << 15); // Clear the MSbit.

    snprintf(buffer, sizeof(buffer), "Data length after clearing the MSBIT: %d.\n", dataLength);
    print(buffer);
    //   digitalWrite(BNO08X_CS, HIGH);
    // transferByteByByte(dataLength);
    transferArrayOfBytes(dataLength);

    println("\nIncoming ended!");
    digitalWrite(BNO08X_CS, HIGH);
    waitForSpi();
}

void setup()
{
    Serial.begin(115200);
    // pinMode(BNO08X_INT, INPUT_PULLUP);
    pinMode(BNO08X_INT, INPUT);
    pinMode(BNO08X_RESET, OUTPUT);
    pinMode(BNO08X_CS, OUTPUT);

    digitalWrite(BNO08X_CS, HIGH);
    digitalWrite(BNO08X_RESET, LOW);
    delay(2);
    digitalWrite(BNO08X_RESET, HIGH);

    

    SPI.setDataMode(3);
    SPI.setBitOrder(MSBFIRST);
    SPI.setFrequency(1000000);
    SPI.begin(); /* begin SPI */
waitForSpi();

    loopTransmission();
    loopTransmission();
}

void loop()
{
    // Serial.printf("%02X ", SPI.transfer(0));
    //   loopTransmission();
    //   delay(1000);
}