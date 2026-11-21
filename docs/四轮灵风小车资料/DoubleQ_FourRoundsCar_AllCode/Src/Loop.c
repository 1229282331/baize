#include "Loop.h"
#include "IndicatorDevice.h"

extern __IO uint16_t Loop_5msTime;//5ms轮询时间
extern __IO uint16_t Loop_10msTime;//10ms轮询时间
extern __IO uint16_t Loop_500msTime;//500ms轮询时间
extern __IO uint16_t HeartbeatTime;//心跳包时间
/*-----------------------------------------------------------
函数功能: 滴答定时器中断函数
输入参数: None
返 回 值: None
说    明: 滴答定时器中断周期为1ms，每隔1ms就会进入一次该函数
 -----------------------------------------------------------*/
void HAL_SYSTICK_Callback()//系统滴答定时器1ms中断函数
{
	if(Loop_5msTime) Loop_5msTime--;//5ms轮询时间自减
	if(Loop_10msTime) Loop_10msTime--;//10ms轮询时间自减
	if(Loop_500msTime) Loop_500msTime--;//500ms轮询时间自减
	if(HeartbeatTime) HeartbeatTime--;//心跳包时间自减
	IndicatorDeviceControl();//指示设备控制函数
}
