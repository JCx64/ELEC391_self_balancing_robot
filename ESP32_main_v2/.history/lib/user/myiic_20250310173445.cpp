#include "myiic.h"

myiic :: myiic(){

}

void myiic::I2C_Start() {
    pinMode(SIM_SCL_PIN, OUTPUT);
    pinMode(SIM_SDA_PIN, OUTPUT);
    digitalWrite(SIM_SCL_PIN, HIGH);
    digitalWrite(SIM_SDA_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(SIM_SDA_PIN, LOW);
    delayMicroseconds(15);
    digitalWrite(SIM_SCL_PIN, LOW);
}

void myiic::I2C_Stop() {
    pinMode(SIM_SDA_PIN, OUTPUT);
    digitalWrite(SIM_SDA_PIN, LOW);
    digitalWrite(SIM_SCL_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(SIM_SDA_PIN, HIGH);
    delayMicroseconds(15);
}

void myiic::I2C_WriteBit(bool bit) {
    digitalWrite(SIM_SDA_PIN, bit);
    delayMicroseconds(15);
    digitalWrite(SIM_SCL_PIN, HIGH);
    delayMicroseconds(15);
    digitalWrite(SIM_SCL_PIN, LOW);
}

bool myiic::I2C_ReadBit() {
    pinMode(SIM_SDA_PIN, INPUT_PULLUP);
    delayMicroseconds(15);
    digitalWrite(SIM_SCL_PIN, HIGH);
    delayMicroseconds(15);
    bool bit = digitalRead(SIM_SDA_PIN);
    digitalWrite(SIM_SCL_PIN, LOW);
    pinMode(SIM_SDA_PIN, OUTPUT);
    return bit;
}

bool myiic::I2C_WriteByte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        I2C_WriteBit((bool)(data & 0x80));
        data <<= 1;
    }
    return !I2C_ReadBit();  // 读取 ACK
}

uint8_t myiic::I2C_ReadByte(bool ack) {
    uint8_t data = 0;
    pinMode(SIM_SDA_PIN, INPUT_PULLUP);
    for (int i = 0; i < 8; i++) {
        data = (data << 1) | I2C_ReadBit();
    }
    pinMode(SIM_SDA_PIN, OUTPUT);
    I2C_WriteBit(!ack);  // 发送 ACK 或 NACK
    return data;
}

uint16_t myiic::readAS5600Angle() {
    I2C_Start();
    if (!I2C_WriteByte(MAG_ENCODER_ADDR << 1)) {
        Serial.println("I2C Write Error");
        I2C_Stop();
        return 0;
    }
    I2C_WriteByte(RAW_ANGLE_REGISTER);
    I2C_Start();
    I2C_WriteByte((MAG_ENCODER_ADDR << 1) | 1);
    uint8_t highByte = I2C_ReadByte(true);
    uint8_t lowByte = I2C_ReadByte(false);
    I2C_Stop();
    return (highByte << 8) | lowByte;
}
