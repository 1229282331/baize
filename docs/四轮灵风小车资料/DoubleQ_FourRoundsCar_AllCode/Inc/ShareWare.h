#ifndef __SHAREWARE_H
#define __SHAREWARE_H
#include "main.h"
#include <stdarg.h>
#include <math.h>
#include "adc1.h"
#include "uart.h"
#include "tim2.h"
#include "tim3.h"
#include "tim4.h"
#include "tim5.h"
#include "tim8.h"
#include "delayus.h"
#include "Encoder.h"
#include "Kinematics_4WD.h"
#include "LowPassFilter.h"
#include "BatteryInfor.h"
#include "IndicatorDevice.h"
#include "i2c.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define Constrain(AMT, MIN, MAX) ((AMT) < (MIN)? (MIN):( (AMT) > (MAX)?(MAX):(AMT) ))//约束函数

union floatHex_union
{
	uint8_t Hex[4];
	float floatValue;
};


extern LPF_Struct LPF_SupplyVoltage;//创建低通滤波电源电压值结构体参数
extern Battery_Struct Battery;//电池状态信息结构体参数



extern Encoder_Struct Encoder_M1;//创建M1编码器结构体参数
extern Encoder_Struct Encoder_M2;//创建M2编码器结构体参数
extern Encoder_Struct Encoder_M3;//创建M3编码器结构体参数
extern Encoder_Struct Encoder_M4;//创建M4编码器结构体参数


extern Kinematics_Struct Kinematics_Inverse;//创建车体运动学逆解结构体参数
extern Kinematics_Struct Kinematics_Forward;//创建车体运动学正解结构体参数

extern union floatHex_union floatHex;


#ifdef __cplusplus
}
#endif

#endif
