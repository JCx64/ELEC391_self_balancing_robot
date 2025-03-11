#include "myiic.h"
class myiic::myi2c(){

}

myi2c::I2C_Start() {
    pinMode(SDA_PIN, OUTPUT);
    pinMode(SCL_PIN, OUTPUT);
    digitalWrite(SDA_PIN, HIGH);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN, LOW);
}

void I2C_Stop() {
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(10);
}

void I2C_WriteBit(bool bit) {
    digitalWrite(SDA_PIN, bit);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN, LOW);
}

bool I2C_ReadBit() {
    pinMode(SDA_PIN, INPUT_PULLUP);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    bool bit = digitalRead(SDA_PIN);
    digitalWrite(SCL_PIN, LOW);
    pinMode(SDA_PIN, OUTPUT);
    return bit;
}

bool I2C_WriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        I2C_WriteBit(data & 0x80);
        data <<= 1;
    }
    return !I2C_ReadBit();  // 读取 ACK
}

uint8_t I2C_ReadByte(bool ack) {
    uint8_t data = 0;
    pinMode(SDA_PIN, INPUT_PULLUP);
    for (int i = 0; i < 8; i++) {
        data = (data << 1) | I2C_ReadBit();
    }
    pinMode(SDA_PIN, OUTPUT);
    I2C_WriteBit(!ack);  // 发送 ACK 或 NACK
    return data;
}

uint16_t readAS5600Angle() {
    I2C_Start();
    if (!I2C_WriteByte(AS5600_I2C_ADDRESS << 1)) {
        Serial.println("I2C Write Error");
        I2C_Stop();
        return 0;
    }
    I2C_WriteByte(RAW_ANGLE_REGISTER);
    I2C_Start();
    I2C_WriteByte((AS5600_I2C_ADDRESS << 1) | 1);
    uint8_t highByte = I2C_ReadByte(true);
    uint8_t lowByte = I2C_ReadByte(false);
    I2C_Stop();
    return (highByte << 8) | lowByte;
}

void setup() {
    Serial.begin(9600);
    pinMode(SDA_PIN, OUTPUT);
    pinMode(SCL_PIN, OUTPUT);
    digitalWrite(SDA_PIN, HIGH);
    digitalWrite(SCL_PIN, HIGH);
}

void loop() {
    uint16_t angleRaw = readAS5600Angle();
    float angle = (angleRaw * 360.0) / 4096.0;
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(100);
}
