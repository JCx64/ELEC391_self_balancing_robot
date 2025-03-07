/*
 * @Author: Su 2970989399@qq.com
 * @Date: 2024-02-21 21:13:58
 * @LastEditors: Su 2970989399@qq.com
 * @LastEditTime: 2024-03-17 22:33:25
 * @FilePath: \IMU\IMU_test\User\MahonyAHRS.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include "math.h"
class MahonyClass{
    public:
        void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
        void Mahony_computeAngles(void);
        void Mahony_Init(float sampleFrequency);
        float getRoll(void);
        float getPitch(void);
        float getYaw(void);
        float getRollRadians(void);
        float getPitchRadians(void);
        float getYawRadians(void);
        MahonyClass();
    private:
        void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
        void MahonyAHRSinit(float ax, float ay, float az, float mx, float my, float mz);
};
#endif
