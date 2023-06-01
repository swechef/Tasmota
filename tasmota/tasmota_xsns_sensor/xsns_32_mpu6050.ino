/*
  xsns_32_mpu6050.ino - MPU6050 gyroscope and temperature sensor support for Tasmota

  Copyright (C) 2021  Oliver Welter

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_I2C
#ifdef USE_MPU6050
/*********************************************************************************************\
 * MPU6050 3 axis gyroscope and temperature sensor
 *
 * Source: Oliver Welter, with special thanks to Jeff Rowberg
 *
 * I2C Address: 0x68 or 0x69 with AD0 HIGH
\*********************************************************************************************/

#define XSNS_32                          32
#define XI2C_25                          25  // See I2CDEVICES.md

#define D_SENSOR_MPU6050                 "MPU6050"

#define MPU_6050_ADDR_AD0_LOW            0x68
#define MPU_6050_ADDR_AD0_HIGH           0x69

#define MPU_6050_REPORT_TEMP
#define MPU_6050_REPORT_ACC
#define MPU_6050_REPORT_GYRO
#define MPU_6050_REPORT_YPR

uint8_t MPU_6050_address;
uint8_t MPU_6050_addresses[] = { MPU_6050_ADDR_AD0_LOW, MPU_6050_ADDR_AD0_HIGH };
uint8_t MPU_6050_found;

int16_t MPU_6050_ax = 0, MPU_6050_ay = 0, MPU_6050_az = 0;
int16_t MPU_6050_gx = 0, MPU_6050_gy = 0, MPU_6050_gz = 0;
int16_t MPU_6050_temperature = 0;

#ifdef USE_MPU6050_DMP
  #include "MPU6050_6Axis_MotionApps20.h"
  #include "I2Cdev.h"
  #include <helper_3dmath.h>
  typedef struct MPU6050_DMP{
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float yawPitchRoll[3];  // [yaw, pitch roll]    Yaw-pitch-roll container
  } MPU6050_DMP;

  MPU6050_DMP MPU6050_dmp;
#else
  #include <MPU6050.h>
#endif //USE_MPU6050_DMP
MPU6050 mpu6050;

void MPU_6050PerformReading(void)
{
#ifdef USE_MPU6050_DMP
    mpu6050.resetFIFO(); // with a default sampling rate of 200Hz, we create a delay of approx. 5ms with a complete read cycle
    MPU6050_dmp.fifoCount = mpu6050.getFIFOCount();
    while (MPU6050_dmp.fifoCount < MPU6050_dmp.packetSize) MPU6050_dmp.fifoCount = mpu6050.getFIFOCount();
    mpu6050.getFIFOBytes(MPU6050_dmp.fifoBuffer, MPU6050_dmp.packetSize);
    MPU6050_dmp.fifoCount -= MPU6050_dmp.packetSize;
    // calculate euler and acceleration with the DMP
    mpu6050.dmpGetQuaternion(&MPU6050_dmp.q, MPU6050_dmp.fifoBuffer);
    mpu6050.dmpGetEuler(MPU6050_dmp.euler, &MPU6050_dmp.q);
    mpu6050.dmpGetAccel(&MPU6050_dmp.aa, MPU6050_dmp.fifoBuffer);
    mpu6050.dmpGetGravity(&MPU6050_dmp.gravity, &MPU6050_dmp.q);
    mpu6050.dmpGetLinearAccel(&MPU6050_dmp.aaReal, &MPU6050_dmp.aa, &MPU6050_dmp.gravity);
    mpu6050.dmpGetYawPitchRoll(MPU6050_dmp.yawPitchRoll, &MPU6050_dmp.q, &MPU6050_dmp.gravity);
    MPU_6050_gx = MPU6050_dmp.euler[0] * 180/M_PI;
    MPU_6050_gy = MPU6050_dmp.euler[1] * 180/M_PI;
    MPU_6050_gz = MPU6050_dmp.euler[2] * 180/M_PI;
    MPU_6050_ax = MPU6050_dmp.aaReal.x;
    MPU_6050_ay = MPU6050_dmp.aaReal.y;
    MPU_6050_az = MPU6050_dmp.aaReal.z;
#else
  mpu6050.getMotion6(
    &MPU_6050_ax,
    &MPU_6050_ay,
    &MPU_6050_az,
    &MPU_6050_gx,
    &MPU_6050_gy,
    &MPU_6050_gz
  );
#endif //USE_MPU6050_DMP
  MPU_6050_temperature = mpu6050.getTemperature();
}

void MPU_6050SetGyroOffsets(int x, int y, int z)
{
  mpu6050.setXGyroOffset(x);
  mpu6050.setYGyroOffset(y);
  mpu6050.setZGyroOffset(z);
}

void MPU_6050SetAccelOffsets(int x, int y, int z)
{
  mpu6050.setXAccelOffset(x);
  mpu6050.setYAccelOffset(y);
  mpu6050.setZAccelOffset(z);
}

//calibration values
#define XG_OFFSET -72
#define YG_OFFSET 34
#define ZG_OFFSET 74
#define XA_OFFSET -3447
#define YA_OFFSET -1715
#define ZA_OFFSET 1184

void MPU_6050Detect(void)
{
  for (uint32_t i = 0; i < sizeof(MPU_6050_addresses); i++)
  {
    MPU_6050_address = MPU_6050_addresses[i];
    if (!I2cSetDevice(MPU_6050_address)) { break; }
    mpu6050.setAddr(MPU_6050_addresses[i]);

#ifdef USE_MPU6050_DMP
    MPU6050_dmp.devStatus = mpu6050.dmpInitialize();

    MPU_6050SetGyroOffsets(XG_OFFSET, YG_OFFSET, ZG_OFFSET);
    MPU_6050SetAccelOffsets(XA_OFFSET, YA_OFFSET, ZA_OFFSET);

    if (MPU6050_dmp.devStatus == 0) {
      mpu6050.setDMPEnabled(true);
      MPU6050_dmp.packetSize = mpu6050.dmpGetFIFOPacketSize();
      MPU_6050_found = true;
    }
#else
    mpu6050.initialize();
    MPU_6050_found = mpu6050.testConnection();
#endif //USE_MPU6050_DMP
    Settings->flag2.axis_resolution = 2;  // Need to be services by command Sensor32
  }

  if (MPU_6050_found) {
    I2cSetActiveFound(MPU_6050_address, D_SENSOR_MPU6050);
  }
}

#define D_YAW "Yaw"
#define D_PITCH "Pitch"
#define D_ROLL "Roll"

#ifdef USE_WEBSERVER
#ifdef MPU_6050_REPORT_ACC
const char HTTP_SNS_ACC[] PROGMEM = 
  "{s}" D_SENSOR_MPU6050 " " D_AX_AXIS "{m}%s{e}"                              // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
  "{s}" D_SENSOR_MPU6050 " " D_AY_AXIS "{m}%s{e}"                              // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
  "{s}" D_SENSOR_MPU6050 " " D_AZ_AXIS "{m}%s{e}";                             // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
#endif //MPU_6050_REPORT_ACC
#ifdef MPU_6050_REPORT_GYRO
const char HTTP_SNS_GYRO[] PROGMEM = 
  "{s}" D_SENSOR_MPU6050 " " D_GX_AXIS "{m}%s{e}"                              // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
  "{s}" D_SENSOR_MPU6050 " " D_GY_AXIS "{m}%s{e}"                              // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
  "{s}" D_SENSOR_MPU6050 " " D_GZ_AXIS "{m}%s{e}";                             // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
#endif //MPU_6050_REPORT_GYRO
#ifdef USE_MPU6050_DMP
#ifdef MPU_6050_REPORT_YPR
const char HTTP_SNS_YPR[] PROGMEM =
  "{s}" D_SENSOR_MPU6050 " " D_YAW "{m}%s{e}"                                  // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
  "{s}" D_SENSOR_MPU6050 " " D_PITCH "{m}%s{e}"                                // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
  "{s}" D_SENSOR_MPU6050 " " D_ROLL "{m}%s{e}";                                // {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>
#endif //MPU_6050_REPORT_YPR
#endif // USE_MPU6050_DMP
#endif // USE_WEBSERVER

#define D_JSON_AXIS_AX "AccelXAxis"
#define D_JSON_AXIS_AY "AccelYAxis"
#define D_JSON_AXIS_AZ "AccelZAxis"
#define D_JSON_AXIS_GX "GyroXAxis"
#define D_JSON_AXIS_GY "GyroYAxis"
#define D_JSON_AXIS_GZ "GyroZAxis"
#define D_JSON_YAW "Yaw"
#define D_JSON_PITCH "Pitch"
#define D_JSON_ROLL "Roll"

#define MPU_6050_REPORT_PROPERTY_LEN 33
#define MPU_6050_JSON_PROPERTY_LEN 25

struct MPU_6050TempReport
{
  String fmt = "%s";
  float fvalue;
  char value[MPU_6050_REPORT_PROPERTY_LEN]; // could probably be shorter
  char json[MPU_6050_JSON_PROPERTY_LEN]; // could probably be shorter
};

struct MPU_6050Measurement
{
  char value[MPU_6050_REPORT_PROPERTY_LEN];
  char json[MPU_6050_JSON_PROPERTY_LEN]; 
};

struct MPU_6050XyzReport
{
  String fmt = "%s,%s,%s";
  struct MPU_6050Measurement x;
  struct MPU_6050Measurement y;
  struct MPU_6050Measurement z;
};

struct MPU_6050YprReport
{
  String fmt = "%s,%s,%s";
  struct MPU_6050Measurement yaw;
  struct MPU_6050Measurement pitch;
  struct MPU_6050Measurement roll;
};

struct MPU_6050Report
{
#ifdef MPU_6050_REPORT_TEMP
  struct MPU_6050TempReport temp;
#endif //MPU_6050_REPORT_TEMP

#ifdef MPU_6050_REPORT_ACC
  struct MPU_6050XyzReport acc;
#endif //MPU_6050_REPORT_ACC

#ifdef MPU_6050_REPORT_GYRO
  struct MPU_6050XyzReport gyro;
#endif //MPU_6050_REPORT_GYRO

#if defined(USE_MPU6050_DMP) && defined(MPU_6050_REPORT_YPR)
struct MPU_6050YprReport ypr;
#endif //USE_MPU6050_DMP && MPU_6050_REPORT_YPR
};

#ifdef MPU_6050_REPORT_TEMP
void MPU_6050CreateTempReport(struct MPU_6050TempReport *temp)
{
  temp->fvalue = ConvertTemp(MPU_6050_temperature / 340.0 + 35.53);
  dtostrfd(temp->fvalue, Settings->flag2.temperature_resolution, temp->value);
  //snprintf_P(temp->json, sizeof(temp->json), PSTR("\"" D_JSON_TEMPERATURE "\":%*_f"), Settings->flag2.temperature_resolution, &temp->value);
  snprintf_P(temp->json, sizeof(temp->json), PSTR("\"" D_JSON_TEMPERATURE "\":%s"), temp->value);
}
#endif //MPU_6050_REPORT_TEMP

#ifdef MPU_6050_REPORT_ACC
void MPU_6050CreateAccReport(struct MPU_6050XyzReport *acc)
{
  dtostrfd(MPU_6050_ax, Settings->flag2.axis_resolution, acc->x.value);
  dtostrfd(MPU_6050_ay, Settings->flag2.axis_resolution, acc->y.value);
  dtostrfd(MPU_6050_az, Settings->flag2.axis_resolution, acc->z.value);
  snprintf_P(acc->x.json, sizeof(acc->x.json), PSTR("\"" D_JSON_AXIS_AX "\":%s"), acc->x.value);
  snprintf_P(acc->y.json, sizeof(acc->y.json), PSTR("\"" D_JSON_AXIS_AY "\":%s"), acc->y.value);
  snprintf_P(acc->z.json, sizeof(acc->z.json), PSTR("\"" D_JSON_AXIS_AZ "\":%s"), acc->y.value);

}
#endif //MPU_6050_REPORT_ACC

#ifdef MPU_6050_REPORT_GYRO
void MPU_6050CreateGyroReport(struct MPU_6050XyzReport *gyro)
{
  dtostrfd(MPU_6050_gx, Settings->flag2.axis_resolution, gyro->x.value);
  dtostrfd(MPU_6050_gy, Settings->flag2.axis_resolution, gyro->y.value);
  dtostrfd(MPU_6050_gz, Settings->flag2.axis_resolution, gyro->z.value);
  snprintf_P(gyro->x.json, sizeof(gyro->x.json), PSTR("\"" D_JSON_AXIS_GX "\":%s"), gyro->x.value);
  snprintf_P(gyro->y.json, sizeof(gyro->y.json), PSTR("\"" D_JSON_AXIS_GY "\":%s"), gyro->y.value);
  snprintf_P(gyro->z.json, sizeof(gyro->z.json), PSTR("\"" D_JSON_AXIS_GZ "\":%s"), gyro->y.value);

}
#endif //MPU_6050_REPORT_GYRO

#if defined(USE_MPU6050_DMP) && defined(MPU_6050_REPORT_YPR)
void MPU_6050CreateYprReport(struct MPU_6050YprReport *ypr)
{
  dtostrfd(MPU6050_dmp.yawPitchRoll[0] / PI * 180.0, Settings->flag2.axis_resolution, ypr->yaw.value);
  dtostrfd(MPU6050_dmp.yawPitchRoll[1] / PI * 180.0, Settings->flag2.axis_resolution, ypr->pitch.value);
  dtostrfd(MPU6050_dmp.yawPitchRoll[2] / PI * 180.0, Settings->flag2.axis_resolution, ypr->roll.value);
  snprintf_P(ypr->yaw.json, sizeof(ypr->yaw.json), PSTR("\"" D_JSON_YAW "\":%s"), ypr->yaw.value);
  snprintf_P(ypr->pitch.json, sizeof(ypr->pitch.json), PSTR("\"" D_JSON_PITCH "\":%s"), ypr->pitch.value);
  snprintf_P(ypr->roll.json, sizeof(ypr->roll.json), PSTR("\"" D_JSON_ROLL "\":%s"), ypr->roll.value);
}
#endif //USE_MPU6050_DMP && MPU_6050_REPORT_YPR

void MPU_6050CreateReport(struct MPU_6050Report *report)
{
#ifdef MPU_6050_REPORT_TEMP
  MPU_6050CreateTempReport(&report->temp);
#endif //MPU_6050_REPORT_TEMP
#ifdef MPU_6050_REPORT_ACC
  MPU_6050CreateAccReport(&report->acc);
#endif //MPU_6050_REPORT_ACC
#ifdef MPU_6050_REPORT_GYRO
  MPU_6050CreateGyroReport(&report->gyro);
#endif //MPU_6050_REPORT_GYRO
#if defined(USE_MPU6050_DMP) && defined(MPU_6050_REPORT_YPR)
  MPU_6050CreateYprReport(&report->ypr);
#endif //USE_MPU6050_DMP && MPU_6050_REPORT_YPR
}

void MPU_6050AppendFmt(bool *first, String *dst_fmt, const String fmt)
{
  if(!*first) // not first
  {
    *dst_fmt += ",";
  }
  *dst_fmt += fmt;
  *first = *first ? false : true; // if first, set first false
}

String MPU_6050CreateFmtJsonString(struct MPU_6050Report *report)
{
  bool first = true;
  String fmt = ",\"%s\":{";;

#ifdef MPU_6050_REPORT_TEMP
  MPU_6050AppendFmt(&first, &fmt, report->temp.fmt);
#endif //MPU_6050_REPORT_TEMP

#ifdef MPU_6050_REPORT_ACC
  MPU_6050AppendFmt(&first, &fmt, report->acc.fmt);
#endif //MPU_6050_REPORT_ACC

#ifdef MPU_6050_REPORT_GYRO
  MPU_6050AppendFmt(&first, &fmt, report->gyro.fmt);
#endif //MPU_6050_REPORT_GYRO

#if defined(USE_MPU6050_DMP) && defined(MPU_6050_REPORT_YPR)
  MPU_6050AppendFmt(&first, &fmt, report->ypr.fmt);
#endif //USE_MPU6050_DMP && MPU_6050_REPORT_YPR
  
  fmt += "}";
  return fmt;
}

void MPU_6050MqttReport(struct MPU_6050Report *report)
{
  String fmt = MPU_6050CreateFmtJsonString(report);

  ResponseAppend_P(fmt.c_str(), D_SENSOR_MPU6050 
#ifdef MPU_6050_REPORT_TEMP
                    ,report->temp.json
#endif //MPU_6050_REPORT_TEMP
#ifdef MPU_6050_REPORT_ACC
                    ,report->acc.x.json, report->acc.y.json, report->acc.z.json
#endif //MPU_6050_REPORT_ACC
#ifdef MPU_6050_REPORT_GYRO
                    ,report->gyro.x.json, report->gyro.y.json, report->gyro.z.json
#endif //MPU_6050_REPORT_GYRO
#if defined(USE_MPU6050_DMP) && defined(MPU_6050_REPORT_YPR)
                    ,report->ypr.yaw.json, report->ypr.pitch.json, report->ypr.roll.json
#endif //USE_MPU6050_DMP && MPU_6050_REPORT_YPR
  );
}

void MPU_6050WebReport(struct MPU_6050Report *report)
{
#ifdef MPU_6050_REPORT_TEMP
  WSContentSend_Temp(D_SENSOR_MPU6050, report->temp.fvalue);
#endif //MPU_6050_REPORT_TEMP
#ifdef MPU_6050_REPORT_ACC
  WSContentSend_PD(HTTP_SNS_ACC, report->acc.x.value, report->acc.y.value, report->acc.z.value);
#endif //MPU_6050_REPORT_ACC
#ifdef MPU_6050_REPORT_GYRO
  WSContentSend_PD(HTTP_SNS_GYRO, report->gyro.x.value, report->gyro.y.value, report->gyro.z.value);
#endif //MPU_6050_REPORT_GYRO
#if defined(USE_MPU6050_DMP) && defined(MPU_6050_REPORT_YPR)
  WSContentSend_PD(HTTP_SNS_YPR, report->ypr.pitch.value, report->ypr.yaw.value, report->ypr.roll.value);
#endif //USE_MPU6050_DMP && MPU_6050_REPORT_YPR
}

void MPU_6050Show(bool json)
{
  struct MPU_6050Report report;

  MPU_6050PerformReading();
  MPU_6050CreateReport(&report);

  if(json)
  {
    MPU_6050MqttReport(&report);

#ifdef USE_DOMOTICZ
  float tempConv = ConvertTemp(MPU_6050_temperature / 340.0 + 35.53);
  DomoticzFloatSensor(DZ_TEMP, tempConv);
#endif // USE_DOMOTICZ
  }
  else
  {
    MPU_6050WebReport(&report);
  }
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns32(uint32_t function)
{
  if (!I2cEnabled(XI2C_25)) { return false; }

  bool result = false;

  if (FUNC_INIT == function) {
    MPU_6050Detect();
  }
  else if (MPU_6050_found) {
    switch (function) {
      case FUNC_EVERY_SECOND:
        if (TasmotaGlobal.tele_period == Settings->tele_period -3) {
          MPU_6050PerformReading();
        }
        break;
      case FUNC_JSON_APPEND:
        MPU_6050Show(true);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        MPU_6050Show(false);
        MPU_6050PerformReading();
        break;
#endif // USE_WEBSERVER
    }
  }
  return result;
}

#endif // USE_MPU6050
#endif // USE_I2C
