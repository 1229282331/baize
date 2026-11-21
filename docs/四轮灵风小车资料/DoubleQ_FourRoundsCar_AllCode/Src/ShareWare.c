#include "ShareWare.h"

Battery_Struct Battery;//电池状态信息结构体参数
LPF_Struct LPF_SupplyVoltage;//创建低通滤波电源电压值结构体参数


//Encoder_Struct Encoder_M1;//创建M1编码器结构体参数
//Encoder_Struct Encoder_M2;//创建M2编码器结构体参数
//Encoder_Struct Encoder_M3;//创建M3编码器结构体参数
//Encoder_Struct Encoder_M4;//创建M4编码器结构体参数


Kinematics_Struct Kinematics_Inverse;//创建车体运动学逆解结构体参数
Kinematics_Struct Kinematics_Forward;//创建车体运动学正解结构体参数


union floatHex_union floatHex;

