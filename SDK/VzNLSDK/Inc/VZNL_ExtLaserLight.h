/*
 * Header: VZNL_ExtLaserLight.h
 * Description: 外部激光器头文件
 * Sample:
 * Author: Mjw
 * Date:   2023/10/16
 */

#ifndef __VIZUM_EXTLASERLIGHT_HEADER__
#define __VIZUM_EXTLASERLIGHT_HEADER__

#include "VZNL_Export.h"
#include "VZNL_Types.h"


 /**
  * @brief 是否支持关闭激光器
  * @param [in] hDevice 设备句柄
  * @param [in] pnErrorCode 错误码,为nullptr时不返回
  * @return 返回VzTrue表示支持，否则为不支持。
  */
VZNLAPI VzBool VzNL_IsSupportTurnOffLaserLight(VZNLHANDLE hDevice, int* pnErrorCode);

 /**
  * @brief 关闭激光器
  * @param [in] hDevice 设备句柄
  * @param [in] bOff VzTrue:关闭，VzFalse:点亮
  * @return 成功返回0，否则为错误码。
  */
VZNLAPI int VzNL_TurnOffLaserLight(VZNLHANDLE hDevice, VzBool bOff);

/**
 * @brief 激光器是否已关闭
 * @param [in] hDevice 设备句柄
 * @param [out] pnErrorCode 错误码,为nullptr时不返回
 * @return VzTrue:关闭，VzFalse:点亮。
 */
VZNLAPI VzBool VzNL_IsTurnOffLaserLight(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 是否支持激光器亮度调节功能
 * @param [in] hDevice 设备句柄
 * @param [in] pnErrorCode 错误码,为nullptr时不返回
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI VzBool VzNL_IsSupportChangeLaserLightLevel(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 获取外部激光器功率控制范围
 * @param [in] hDevice 设备句柄
 * @param [out] pnErrorCode 错误码,为nullptr时不返回
 * @return 成功返回功率范围
 */
VZNLAPI SVzNLRange VzNL_GetExtLaserLightRange(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 配置功率等级
 * @param [in] hDevice 设备句柄
 * @param [in] nLevel 功率等级
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetLaserLightLevel(VZNLHANDLE hDevice, int nLevel);

/**
 * @brief 获取功率等级
 * @param [in] hDevice 设备句柄
 * @param [out] pnErrorCode 错误码,为nullptr时不返回
 * @return 成功返回当前激光功率等级
 */
VZNLAPI int VzNL_GetLaserLightLevel(VZNLHANDLE hDevice, int* pnErrorCode);

#endif //__VIZUM_EXTLASERLIGHT_HEADER__
