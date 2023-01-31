#include <Arduino.h>
#include <SPI.h>

#define BNO08X_CS D8
#define BNO08X_INT D1
#define BNO08X_RESET D2

#define MAX_PACKET_SIZE 128 // Packets can be up to 32k but we don't have that much RAM.
uint8_t shtpHeader[4];      // Each packet has a header of 4 bytes
uint8_t shtpData[MAX_PACKET_SIZE];
bool debugFlag = true;
char buffer[128];
int16_t rotationVector_Q1 = 14;

uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0};

#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB

// Registers
const byte CHANNEL_COMMAND = 0;
const byte CHANNEL_EXECUTABLE = 1;
const byte CHANNEL_CONTROL = 2;
const byte CHANNEL_REPORTS = 3;
const byte CHANNEL_WAKE_REPORTS = 4;
const byte CHANNEL_GYRO = 5;

void print(char *message)
{
    if (debugFlag)
    {
        Serial.print(message);
    }
}

void print(uint8_t message, int mode = -1)
{
    if (debugFlag)
    {
        if (mode == -1)
        {
            Serial.print(message);
        }
        else
        {
            Serial.print(message, mode);
        }
    }
}

void println(char *message = "")
{
    if (debugFlag)
    {
        Serial.println(message);
    }
}

void waitForSpi()
{

    print("\n\nWaiting for SPI ");

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
    snprintf(buffer, sizeof(buffer), "SPI connected in %ld ms.", duration);
    print(buffer);
    // Serial.println();
}

void transferByteByByte(int dataLength)
{
    for (uint16_t dataSpot = 0; dataSpot < dataLength; dataSpot++)
    {
        // uint8_t incoming = SPI.transfer(0xFF);
        // Serial.printf("%02X ", SPI.transfer(0xFF));
        snprintf(buffer, sizeof(buffer), "%02X ", SPI.transfer(0xFF));
        print(buffer);
    }
}

void printArrayOfHex(uint8_t *array, uint16_t length)
{
    for (int index = 0; index < length; index++)
    {
        // Serial.printf("%02X ", array[index]);
        snprintf(buffer, sizeof(buffer), "%02X ", array[index]);
        print(buffer);
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

void loopTransmission()
{
    waitForSpi();

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
    // waitForSpi();
}

void sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
    uint8_t packetLength = dataLength + 4; // Add four bytes for the header

    // Wait for BNO080 to indicate it is available for communication
    waitForSpi();
    digitalWrite(BNO08X_CS, LOW);

    // Send the 4 byte packet header
    SPI.transfer(packetLength & 0xFF);             // Packet length LSB
    SPI.transfer(packetLength >> 8);               // Packet length MSB
    SPI.transfer(channelNumber);                   // Channel number
    SPI.transfer(sequenceNumber[channelNumber]++); // Send the sequence number, increments with each packet sent, different counter for each channel

    // Send the user's data packet
    for (uint8_t i = 0; i < dataLength; i++)
    {
        SPI.transfer(shtpData[i]);
    }

    digitalWrite(BNO08X_CS, HIGH);
}

void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
    long microsBetweenReports = (long)timeBetweenReports * 1000L;

    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;     // Set feature command. Reference page 55
    shtpData[1] = reportID;                            // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0;                                   // Feature flags
    shtpData[3] = 0;                                   // Change sensitivity (LSB)
    shtpData[4] = 0;                                   // Change sensitivity (MSB)
    shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  // Report interval (LSB) in microseconds. 0x7A120 = 500ms
    shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  // Report interval
    shtpData[7] = (microsBetweenReports >> 16) & 0xFF; // Report interval
    shtpData[8] = (microsBetweenReports >> 24) & 0xFF; // Report interval (MSB)
    shtpData[9] = 0;                                   // Batch Interval (LSB)
    shtpData[10] = 0;                                  // Batch Interval
    shtpData[11] = 0;                                  // Batch Interval
    shtpData[12] = 0;                                  // Batch Interval (MSB)
    shtpData[13] = (specificConfig >> 0) & 0xFF;       // Sensor-specific config (LSB)
    shtpData[14] = (specificConfig >> 8) & 0xFF;       // Sensor-specific config
    shtpData[15] = (specificConfig >> 16) & 0xFF;      // Sensor-specific config
    shtpData[16] = (specificConfig >> 24) & 0xFF;      // Sensor-specific config (MSB)

    // Transmit packet on channel 2, 17 bytes
    sendPacket(CHANNEL_CONTROL, 17);
}

float qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
    float qFloat = fixedPointValue;
    qFloat *= pow(2, qPoint * -1);
    return (qFloat);
}

float getRoll()
{
    float dqw = qToFloat(rawQuatReal, rotationVector_Q1);
    float dqx = qToFloat(rawQuatI, rotationVector_Q1);
    float dqy = qToFloat(rawQuatJ, rotationVector_Q1);
    float dqz = qToFloat(rawQuatK, rotationVector_Q1);

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // roll (x-axis rotation)
    float t0 = +2.0 * (dqw * dqx + dqy * dqz);
    float t1 = +1.0 - 2.0 * (dqx * dqx + ysqr);
    float roll = atan2(t0, t1);

    return (roll);
}

// Return the pitch (rotation around the y-axis) in Radians
float getPitch()
{
    float dqw = qToFloat(rawQuatReal, rotationVector_Q1);
    float dqx = qToFloat(rawQuatI, rotationVector_Q1);
    float dqy = qToFloat(rawQuatJ, rotationVector_Q1);
    float dqz = qToFloat(rawQuatK, rotationVector_Q1);

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // pitch (y-axis rotation)
    float t2 = +2.0 * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    float pitch = asin(t2);

    return (pitch);
}

// Return the yaw / heading (rotation around the z-axis) in Radians
float getYaw()
{
    float dqw = qToFloat(rawQuatReal, rotationVector_Q1);
    float dqx = qToFloat(rawQuatI, rotationVector_Q1);
    float dqy = qToFloat(rawQuatJ, rotationVector_Q1);
    float dqz = qToFloat(rawQuatK, rotationVector_Q1);

    float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
    dqw = dqw / norm;
    dqx = dqx / norm;
    dqy = dqy / norm;
    dqz = dqz / norm;

    float ysqr = dqy * dqy;

    // yaw (z-axis rotation)
    float t3 = +2.0 * (dqw * dqz + dqx * dqy);
    float t4 = +1.0 - 2.0 * (ysqr + dqz * dqz);
    float yaw = atan2(t3, t4);

    return (yaw);
}

void setup()
{
    Serial.begin(115200);
    pinMode(BNO08X_INT, INPUT_PULLUP);
    // pinMode(BNO08X_INT, INPUT);
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

    loopTransmission();
    loopTransmission();

    // enableRotationVector(,50,);

    shtpData[0] = 0xf9;
    shtpData[1] = 0;
    sendPacket(CHANNEL_CONTROL, 2);
    loopTransmission();
    if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
    {
        print("SW Version Major: 0x");
        print(shtpData[2], HEX);
        print(" SW Version Minor: 0x");
        print(shtpData[3], HEX);
        uint32_t SW_Part_Number = ((uint32_t)shtpData[7] << 24) | ((uint32_t)shtpData[6] << 16) | ((uint32_t)shtpData[5] << 8) | ((uint32_t)shtpData[4]);
        print(" SW Part Number: 0x");
        print(SW_Part_Number, HEX);
        uint32_t SW_Build_Number = ((uint32_t)shtpData[11] << 24) | ((uint32_t)shtpData[10] << 16) | ((uint32_t)shtpData[9] << 8) | ((uint32_t)shtpData[8]);
        print(" SW Build Number: 0x");
        print(SW_Build_Number, HEX);
        uint16_t SW_Version_Patch = ((uint16_t)shtpData[13] << 8) | ((uint16_t)shtpData[12]);
        print(" SW Version Patch: 0x");
        print(SW_Version_Patch, HEX);
        println();
    }
    else
    {
        println("Received packet is not product id response.");
    }

    setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, 50, 0);
    debugFlag = false;
}

void parseInputReport()
{
    uint8_t status = shtpData[5 + 2] & 0x03; // Get status bits
    uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
    uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
    uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
    uint16_t data4 = 0;
    uint16_t data5 = 0; // We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports
    quatAccuracy = status;
    rawQuatI = data1;
    rawQuatJ = data2;
    rawQuatK = data3;
    rawQuatReal = data4;

    // Only available on rotation vector and ar/vr stabilized rotation vector,
    //  not game rot vector and not ar/vr stabilized rotation vector
    rawQuatRadianAccuracy = data5;
}

void loop()
{
    loopTransmission();

    // if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
    if (shtpHeader[2] == CHANNEL_REPORTS)
    {
        parseInputReport(); // This will update the rawAccelX, etc variables depending on which feature report is found
        Serial.printf("Roll = %02f | Pitch = %02f | Yaw = %02f\n", getRoll(), getPitch(), getYaw());
    }
}