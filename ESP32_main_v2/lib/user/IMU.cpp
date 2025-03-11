#include "IMU.h"

IMU::IMU() : yaw_offset(0), pitch_offset(0), roll_offset(0), lastTime(0), dt(0), axo(0), ayo(0), azo(0), gxo(0), gyo(0), gzo(0), AcceRatio(16384.0), GyroRatio(131.0), 
             Px(1), Rx(0), Kx(0), Sx(0), Py(1), Ry(0), Ky(0), Sy(0), Pz(1), Rz(0), Kz(0), Sz(0), useDMP(false) {}

void IMU::begin(bool useDMP){
    this->useDMP = useDMP;
    Wire.begin(I2C_SDA, I2C_SCL, 100000);
    Serial.begin(115200);
    mpu.initialize();
    calibrate();
}

void IMU::calibrate(){
    if(useDMP){
        mpu.dmpInitialize();
        mpu.setDMPEnabled(true);

        if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw_offset = ypr[0];
            pitch_offset = ypr[1];
            roll_offset = ypr[2];
        }
    }
    else{
        unsigned short times = 200;
        axo = ayo = azo = gxo = gyo = gzo = 0;

        for(int i = 0; i < times; i++){
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            axo += ax; ayo += ay; azo += az;
            gxo += gx; gyo += gy; gzo += gz;
        }
        axo /= times; ayo /= times; azo /= times;
        gxo /= times; gyo /= times; gzo /= times;
    }
}

void IMU::update(){
    unsigned long now = millis();
    dt = (now - lastTime) / 1000.0;
    lastTime = now;

    if (useDMP){
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
    }
    else{
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        float accx = ax / AcceRatio;
        float accy = ay / AcceRatio;
        float accz = az / AcceRatio;

        aax = atan(accy / accz) * (-180) / pi;
        aay = atan(accx / accz) * 180 / pi;
        aaz = atan(accz / accy) * 180 / pi;

        // 对于加速度计原始数据的滑动加权滤波算法
        float aax_sum = 0, aay_sum = 0, aaz_sum = 0;
        static float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};

        for (int i = 1; i < 8; i++) {
            aaxs[i - 1] = aaxs[i];
            aax_sum += aaxs[i] * i;
            aays[i - 1] = aays[i];
            aay_sum += aays[i] * i;
            aazs[i - 1] = aazs[i];
            aaz_sum += aazs[i] * i;
        }

        aaxs[7] = aax;
        aax_sum += aax * 8;
        aax = (aax_sum / (11 * 8 / 2.0)) * 9 / 7.0;

        aays[7] = aay;
        aay_sum += aay * 8;
        aay = (aay_sum / (11 * 8 / 2.0)) * 9 / 7.0;

        aazs[7] = aaz;
        aaz_sum += aaz * 8;
        aaz = (aaz_sum / (11 * 8 / 2.0)) * 9 / 7.0;

        gyrox = - (gx - gxo) / GyroRatio * dt; //x轴角速度
        gyroy = - (gy - gyo) / GyroRatio * dt; //y轴角速度
        gyroz = - (gz - gzo) / GyroRatio * dt; //z轴角速度

        agx += gyrox; //x轴角速度积分
        agy += gyroy; //y轴角速度积分
        agz += gyroz; //z轴角速度积分

        static float a_x[10] = {0}, a_y[10] = {0}, a_z[10] = {0};

        for (int i = 1; i < 10; i++) {
            a_x[i - 1] = a_x[i];
            Sx += a_x[i];
            a_y[i - 1] = a_y[i];
            Sy += a_y[i];
            a_z[i - 1] = a_z[i];
            Sz += a_z[i];
        }

        a_x[9] = aax;
        Sx += aax;
        Sx /= 10;

        a_y[9] = aay;
        Sy += aay;
        Sy /= 10;

        a_z[9] = aaz;
        Sz += aaz;
        Sz /= 10;

        
        for (int i = 0; i < 10; i++) {
            Rx += sq(a_x[i] - Sx);
            Ry += sq(a_y[i] - Sy);
            Rz += sq(a_z[i] - Sz);
        }

        Rx = Rx / 9;
        Ry = Ry / 9;
        Rz = Rz / 9;

        Px += 0.0025;
        Kx = Px / (Px + Rx);
        agx = agx + Kx * (aax - agx);
        Px = (1 - Kx) * Px;

        Py += 0.0025;
        Ky = Py / (Py + Ry);
        agy = agy + Ky * (aay - agy);
        Py = (1 - Ky) * Py;

        Pz += 0.0025;
        Kz = Pz / (Pz + Rz);
        agz = agz + Kz * (aaz - agz);
        Pz = (1 - Kz) * Pz;

    }
}

void IMU::getEulerAngles(float &roll, float &pitch, float &yaw){
    if (useDMP){
        roll = (ypr[2] - roll_offset) * 180 / pi;
        pitch = (ypr[1] - pitch_offset) * 180 / pi;
        yaw = (ypr[0] - yaw_offset) * 180 / pi;
    }
    else{
        roll = agx;
        pitch = agy;
        yaw = agz;
    }
}

float IMU::getPitchAngularVelocity(){
    return useDMP ? (gy / GyroRatio) : gyroy;
}

float IMU::getXAcceleration(){
    return useDMP ? (ax / AcceRatio) : aax;
}

float IMU::getYawAngularVelocity(){
    return gyroz;
}
