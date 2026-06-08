/*
 * Header: VZNL_ExtStrobeLaser.h
 * Description: C版激光器头文件
 * Sample:
 */

#ifndef __VIZUM_EXTSTROBELASER_HEADER__
#define __VIZUM_EXTSTROBELASER_HEADER__

#include "VZNL_Export.h"
#include "VZNL_Types.h"


 /**
  * @brief 是否支持C版功能
  * @param [in] hDevice 设备句柄
  * @param [in] pnErrorCode 错误码,为nullptr时不返回
  * @return 返回VzTrue表示支持，否则为不支持。
  */
VZNLAPI VzBool VzNL_IsSupportStrobe(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 开启/关闭C版功能
 * @param [in] hDevice 设备句柄
 * @param [in] bEnable VzTrue:开启，VzFalse:不开启
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_EnableStrobe(VZNLHANDLE hDevice, VzBool bEnable);

/**
 * @brief 激光器是否开启C版功能
 * @param [in] hDevice 设备句柄
 * @param [out] pnErrorCode 错误码,为nullptr时不返回
 * @return VzTrue:开启，VzFalse:未开启。
 */
VZNLAPI VzBool VzNL_IsEnableStrobe(VZNLHANDLE hDevice, int* pnErrorCode);


 /**
  * @brief 点亮激光器
  * @param [in] hDevice 设备句柄
  * @param [in] bOn VzTrue:点亮，VzFalse:关闭
  * @return 成功返回0，否则为错误码。
  */
VZNLAPI int VzNL_TurnOnStrobeLaser(VZNLHANDLE hDevice, VzBool bOn);

/**
 * @brief 激光器是否已点亮
 * @param [in] hDevice 设备句柄
 * @param [out] pnErrorCode 错误码,为nullptr时不返回
 * @return  VzTrue:点亮，VzFalse:关闭
 */
VZNLAPI VzBool VzNL_IsTurnOnStrobeLaser(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 配置C版激光器点亮时长（微妙）
 * @param [in] hDevice 设备句柄
 * @param [in] nTime   时长
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetStrobeTime(VZNLHANDLE hDevice, unsigned int nTime);
VZNLAPI unsigned int VzNL_GetStrobeTime(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 配置C版激光器点亮延迟时长（微妙）
 * @param [in] hDevice 设备句柄
 * @param [in] nTimeOffset   时长偏移
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetStrobeTimeOffset(VZNLHANDLE hDevice, int nTimeOffset);
VZNLAPI int VzNL_GetStrobeTimeOffset(VZNLHANDLE hDevice, int* pnErrorCode);

/**
* @brief 配置C版激光器亮度值(点亮时间与单帧采集时间的比例值：浮点百分比：[0,100])
* @param [in] hDevice 设备句柄
* @param [in] nTimeRatio: 点亮时间与单帧采集时间(1/nFrameRate)的比例值
* @return 成功返回0，否则为错误码，VzNL_GetStrobeTimeRatio() 函数返回错误可认为当前设备不支持此功能；
*/
VZNLAPI int VzNL_SetStrobeTimeRatio(VZNLHANDLE hDevice, float nTimeRatio);
VZNLAPI float VzNL_GetStrobeTimeRatio(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 是否支持激光器PWM调节功能
 * @param [in] hDevice 设备句柄
 * @param [in] pnErrorCode 错误码,为nullptr时不返回
 * @return VzTrue:支持，VzFalse:不支持
 */
VZNLAPI VzBool VzNL_IsSupportPwmControl(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 配置PWM频率
 * @param [in] hDevice 设备句柄
 * @param [in] nFreq 频率
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetPwmFreq(VZNLHANDLE hDevice, unsigned int nFreq);
VZNLAPI unsigned int VzNL_GetPwmFreq(VZNLHANDLE hDevice, int* pnErrorCode);


/**
 * @brief 配置PWM占空比
 * @param [in] hDevice 设备句柄
 * @param [in] nRatio 占空比 例如：50 ，代表占空比 50%
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetPwmRatio(VZNLHANDLE hDevice, unsigned int nRatio);
VZNLAPI unsigned int VzNL_GetPwmRatio(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 是否支持补光灯
 * @param [in] hDevice 设备句柄
 * @param [in] pnErrorCode 错误码,为nullptr时不返回
 * @return 返回VzTrue表示支持，否则为不支持。
 */
VZNLAPI VzBool VzNL_IsSupportExtLight(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 开启/关闭补光灯
 * @param [in] hDevice 设备句柄
 * @param [in] bOn VzTrue:开启，VzFalse:不开启
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_TurnOnExtLight(VZNLHANDLE hDevice, VzBool bOn);

/**
 * @brief 是否开启了补光灯
 * @param [in] hDevice 设备句柄
 * @param [out] pnErrorCode 错误码,为nullptr时不返回
 * @return VzTrue:开启，VzFalse:未开启。
 */
VZNLAPI VzBool VzNL_IsTurnOnExtLight(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置/获取抗反光增益
 * @param [in] hDevice 设备句柄
 * @param [out] eAntiReflectGainType 抗反光增益类型
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetAntiReflectGainType(VZNLHANDLE hDevice, EVzAntiReflectGainType eAntiReflectGainType);
VZNLAPI EVzAntiReflectGainType VzNL_GetAntiReflectGainType(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 开启/关闭补光灯自动控制
 * @param [in] hDevice 设备句柄
 * @param [in] bEnable VzTrue:开启，VzFalse:不开启
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_EnableAutoControlExtLight(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableAutoControlExtLight(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 配置自动控制补光灯开启后的等待时间
 * @param [in] hDevice 设备句柄
 * @param [in] nWaitTime  微秒
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetAutoControlWaitTime(VZNLHANDLE hDevice, unsigned int nWaitTime);
VZNLAPI unsigned int VzNL_GetAutoControlWaitTime(VZNLHANDLE hDevice, int* pnErrorCode);


/**
 * @brief 是否支持ch3功能
 * @param [in] hDevice 设备句柄
 * @param [in] pnErrorCode 错误码,为nullptr时不返回
 * @return VzTrue:支持，VzFalse:不支持
 */
VZNLAPI VzBool VzNL_IsSupportCh3(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 开启/关闭ch3功能
 * @param [in] hDevice 设备句柄
 * @param [in] bEnable VzTrue:开启，VzFalse:不开启
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_EnableCh3(VZNLHANDLE hDevice, VzBool bEnable);

/**
 * @brief 激光器是否ch3功能
 * @param [in] hDevice 设备句柄
 * @param [out] pnErrorCode 错误码,为nullptr时不返回
 * @return VzTrue:开启，VzFalse:未开启。
 */
VZNLAPI VzBool VzNL_IsEnableCh3(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 配置ch3频率
 * @param [in] hDevice 设备句柄
 * @param [in] nFreq 频率 10-50k
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetCh3Freq(VZNLHANDLE hDevice, unsigned int nFreq);
VZNLAPI unsigned int VzNL_GetCh3Freq(VZNLHANDLE hDevice, int* pnErrorCode);


/**
 * @brief 配置ch3占空比
 * @param [in] hDevice 设备句柄
 * @param [in] nRatio 占空比 例如：50 ，代表占空比 50%
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetCh3Ratio(VZNLHANDLE hDevice, unsigned int nRatio);
VZNLAPI unsigned int VzNL_GetCh3Ratio(VZNLHANDLE hDevice, int* pnErrorCode);


/**
 * @brief 配置C版激光自动关闭的时间
 * @param [in] hDevice 设备句柄
 * @param [in] nTime 时间，单位分钟，默认1分钟
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetAutoTurnOffStrobeLaserTime(VZNLHANDLE hDevice, unsigned int nTime);
VZNLAPI unsigned int VzNL_GetAutoTurnOffStrobeLaserTime(VZNLHANDLE hDevice, int* pnErrorCode);
#endif //__VIZUM_EXTSTROBELASER_HEADER__
