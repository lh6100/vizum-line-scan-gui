/*
 * Header: VZNL_DetectConfig.h
 * Description:相机参数配置相关的函数集合
 *
 * Author: Mjw
 * Date:   2018/08/28
 */

#ifndef __VIZUM_DETECTED_CONFIG_HEADER__
#define __VIZUM_DETECTED_CONFIG_HEADER__

#include "VZNL_Export.h"
#include "VZNL_Types.h"

/**
 * @brief 格式化ROI[可用于星光眼/智光眼/控制器]
 * @param [in] hDevice 设备句柄
 * @param [in] pLeftROI 左侧ROI
 * @param [in] pRightROI 右侧ROI
 * @param [in] bKeepTop 保持高度
 * @return 设置ROI成功返回0否则为错误码
 */
VZNLAPI int VzNL_FormatDetectROI(VZNLHANDLE hDevice, SVzNLROIRect* pLeftROI, SVzNLROIRect* pRightROI, VzBool bKeepTop);

/**
 * @brief 设置检测区域[可用于星光眼/智光眼/控制器]
 * @param [in] hDevice 设备句柄
 * @param [in] pLeftROIArea 左眼ROI区域
 * @param [in] pRightROI 右眼ROI区域
 * @return 返回错误码
 */
VZNLAPI int VzNL_ConfigDetectROI(VZNLHANDLE hDevice, const SVzNLROIRect* pLeftROIArea, const SVzNLROIRect* pRightROI);
VZNLAPI int VzNL_GetConfigDetectROI(VZNLHANDLE hDevice, SVzNLROIRect* pLeftROI, SVzNLROIRect* pRightROI);

/**
 * @brief 设置并格式化检测区域(自动格式化为支持的ROI)[可用于智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in/out] pLeftROI 左眼ROI区域
 * @param [in/out] pRightROI 右眼ROI区域
 * @return 返回错误码
 */
VZNLAPI int VzNL_ConfigDetectROIWithFormat(VZNLHANDLE hDevice, SVzNLROIRect* pLeftROI, SVzNLROIRect* pRightROI);

/**
 * @brief 自动求ROI的位置[可用于星光眼/智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] dObjectDistance	被测物最远距离
 * @param [in] dObjectMaxHeight	最高物体高度
 * @return 返回错误码
 * @note: 
 */
VZNLAPI int VzNL_AutoConfigDetectROI(VZNLHANDLE hDevice, double dObjectDistance, double dObjectMaxHeight, unsigned int* pnTop, unsigned int* pnBottom);
VZNLAPI int VzNL_CalcWorkRangeFromROI(VZNLHANDLE hDevice, double* pdNearDistance, double* pdFarDistance);

/**
 * @brief 设置/获取眼睛曝光值[可用于星光眼/智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] eExposeMode [keVzNLExposeMode_Fix=定值曝光 / keVzNLExposeMode_Auto=自动曝光]
 * @param [in] nExposeTime 曝光时间[0~65535]
 * @return 返回错误码
 * @note: 目前智光眼/星光眼仅支持定值曝光
 */
VZNLAPI int VzNL_ConfigEyeExpose(VZNLHANDLE hDevice, EVzNLExposeMode eExposeMode, unsigned int nExposeTime);
VZNLAPI int VzNL_GetConfigEyeExpose(VZNLHANDLE hDevice, EVzNLExposeMode* peExposeMode, unsigned int* pnExposeTime);

/**
* @brief 设置/获取相机增益[可用于星光眼/智光眼]
* @param [in] hDevice 设备句柄
* @param [in] eSensorType [keEyeSensorType_Left=左眼 / keEyeSensorType_Right=右眼]
* @param [in] nCameraGain 曝光时间[0~255]
* @return 返回错误码
* @note: 目前仅智光眼支持
*/
VZNLAPI int VzNL_SetCameraGain(VZNLHANDLE hDevice, EVzEyeSensorType eSensorType, unsigned short nCameraGain);
VZNLAPI int VzNL_GetCameraGain(VZNLHANDLE hDevice, EVzEyeSensorType eSensorType, unsigned short* pnCameraGain);

/**
 * @brief 设置帧速率[期望值][可用于星光眼/智光眼/控制器]
 * @param [in] hDevice 设备句柄
 * @param [in] bEnable 帧速率[0~2000]
 * @return 返回错误码
 */
VZNLAPI int VzNL_SetFrameRate(VZNLHANDLE hDevice, int nFPS);
VZNLAPI int VzNL_GetFrameRate(VZNLHANDLE hDevice, int* pnFPS);

/**
 * @brief 获取当前运行帧率
 * @param [in] hDevice 设备句柄
 * @param [in] pnErrorCode 错误码
 * @return 返回错误码
 * @note  当启动开流后，获取有效，当停流时获取无效。
 */
VZNLAPI int VzNL_QueryRunningFPS(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置Trigger模式[可用于智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] eTriggerMode Trigger模式[被动/主动/上升沿/下降沿]，Master模式为
 * @return 返回错误码
 */
VZNLAPI int VzNL_SetTriggerMode(VZNLHANDLE hDevice, EVzEyeTriggerMode eTriggerMode);
VZNLAPI EVzEyeTriggerMode VzNL_GetTriggerMode(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置触发极性模式[可用于智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] bNormal VzTrue为默认极性/VzFalse反向
 * @return 返回错误码
 */
VZNLAPI int VzNL_SetTriggerPolar(VZNLHANDLE hDevice, VzBool bNormal);
VZNLAPI VzBool VzNL_GetTriggerPolar(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置触发分频[可用于智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] nDivCoe 分频数
 * @return 返回错误码
 */
VZNLAPI int VzNL_SetTriggerDivCoe(VZNLHANDLE hDevice, unsigned int nDivCoe);
VZNLAPI unsigned int VzNL_GetTriggerDivCoe(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 生成软触发信号[可用于智光眼]
 * @detail 必须在keEyeTriggerMode_ManualTrigger触发模式下
 *         参考代码:VzNL_SetTriggerMode(hDevice, keEyeTriggerMode_ManualTrigger);
 * @param [in] hDevice 设备句柄
 * @param [in] nDivCoe 分频数
 * @return 返回错误码
 */
VZNLAPI int VzNL_GenerateTriggerSignal(VZNLHANDLE hDevice);

/**
 * @brief 是否忽略外部使能[可用于智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] bIgnore VzTrue忽略（默认)  VzFalse (关联外部使能信号)
 * @return 返回错误码
 */
VZNLAPI int VzNL_IgnoreTriggerExtSignal(VZNLHANDLE hDevice, VzBool bIgnore);
VZNLAPI VzBool VzNL_IsIgnoreTriggerExtSignal(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 启用外部信号使能[可用于智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] bHwExtEnable VzTrue为使用硬信号 VzFalse为使用软信号
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnableTriggerHwExtEn(VZNLHANDLE hDevice, VzBool bHwExtEnable);
VZNLAPI VzBool VzNL_IsEnableTriggerHwExtEn(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 生成软外部使能信号[可用于智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] bHighLevel 高电平信号
 * @return 返回错误码
 */
VZNLAPI int VzNL_GenerateTriggerSoftExtEn(VZNLHANDLE hDevice, VzBool bHighLevel);

/**
 * @brief 设置触发输出控制模式
 * @param hDevice			[in] 设备Handle
 * @param eTriggerOutMode	[in] 触发输出控制类型
 *								 keStrobeTriggerOutMode_None		默认不控制输出
 *								 keStrobeTriggerOutMode_CameraLink	当触发模式为keEyeTriggerMode_Master时,相机会跟随帧率进行一个信号的输出（此模式下，会导致帧率下降)
 *								 keStrobeTriggerOutMode_UserControl	用户控制，此模式下需要用VzNL_GenerateTriggerOutSignal生成一个控制信号
 * @return 返回错误码
 */
VZNLAPI int VzNL_SetStrobeTriggerOutMode(VZNLHANDLE hDevice, EVzStrobeTriggerOutMode eTriggerOutMode);
VZNLAPI EVzStrobeTriggerOutMode VzNL_GetStrobeTriggerOutMode(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 生成一个触发输出信号
 * @param hDevice [in] 设备Handle
 * @param bTriggerOn [in] VzTrue时为高电平，VzFalse时为低电平信号
 * @return 返回错误码
 */
VZNLAPI int VzNL_GenerateTriggerOutSignal(VZNLHANDLE hDevice, VzBool bTriggerOn);
VZNLAPI VzBool VzNL_IsTriggerOutOnSignal(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 是否启用加速[可用于星光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] bEnable 启用/禁用加速
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnableLaserAccelerator(VZNLHANDLE hDevice, VzBool bEnable);

/**
 * @brief 是否启用去躁[可用于星光眼/智光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] bEnable 启用/禁用去躁
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnableDenoiseMode(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableDenoiseMode(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置图像质量[仅用于控制器]
 * @param [in] hDevice 设备句柄
 * @param [in] nQualityLevel 图像压缩方式
 * @return 返回错误码
 */
VZNLAPI int VzNL_ConfigImageQuality(VZNLHANDLE hDevice, EVzNLImageCompress eQualityLevel);

/**
 * @brief 启用动态ROI[可用于星光眼]
 * @param [in] hDevice 设备句柄
 * @param [in] bEnableDynamicROI 启用/禁用 动态ROI
 * @return 返回
 */
VZNLAPI int VzNL_ConfigDynamicROI(VZNLHANDLE hDevice, VzBool bEnableDynamicROI);
VZNLAPI VzBool VzNL_IsConfigDynamicROI(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 配置曝光补偿[可用于智能眼]
 * @param [in] hDevice 设备句柄
 * @param [in] shExposureValue 曝光补偿[0~255]
 * @return 返回错误码
 */
VZNLAPI int VzNL_ConfigExposureValue(VZNLHANDLE hDevice, unsigned short shExposureValue);

/**
 * @brief 禁用自动白平衡,并设置定值增益[可用于智能眼]
 * @param [in] hDevice 设备句柄
 * @param [in] dFixGain 定值增益
 * @return 返回错误码
 */
VZNLAPI int VzNL_DisableAWBInfo(VZNLHANDLE hDevice, double dFixGain);

/**
 * @brief 启用自动白平衡[可用于智能眼]
 * @param [in] hDevice 设备句柄
 * @param [in] psAwbInfo 自动白平衡参数
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnableAWBInfo(VZNLHANDLE hDevice, const SVzNLAWBInfo* psAwbInfo);

/**
 * @brief 启用颜色校正[可用于智能眼]
 * @param [in] hDevice 设备句柄
 * @param [in] bEnable 启用/禁用 颜色矫正
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnableColorCortectMatrix(VZNLHANDLE hDevice, VzBool bEnable);

/**
 * @brief 配置颜色较正[可用于智能眼]
 * @param [in] hDevice 设备句柄
 * @param [in] shCCMVal 颜色矫正参数
 * @return 返回错误码
 */
VZNLAPI int VzNL_ConfigColorCortectMatrix(VZNLHANDLE hDevice, unsigned short shCCMVal[3][3]);

/*
 * @brief 获取参数设置范围
 * @param name="hDevice"[in]设备handle</param>
 * @param name="eType"[in]参数类型</param>
 * @param name="pnMinVal"[in]最小值</param>
 * @param name="pnMaxVal"[in]最大值</param>
 * @return 返回错误码
 */
VZNLAPI int VzNL_QueryParamRange(VZNLHANDLE hDevice, EVzDeviceParamType eType, unsigned int* pnMinVal, unsigned int* pnMaxVal);

/*
 * @brief 是否启用同步RGB设置
 * @param name="hDevice"[in]设备handle</param>
 * @param name="bEnable"[in]true表示同步设置RGB，false表示不进行同步</param>
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnableSyncRGBD(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableSyncRGBD(VZNLHANDLE hDevice, int* pnErrorCode);

/*
 * @brief 是否启用低功耗模式
 * @param name="hDevice"[in]设备handle</param>
 * @param name="bEnable"[in]true表示开启低功耗,false表示关闭低功耗</param>
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnableLowPowerMode(VZNLHANDLE hDevice, VzBool bEnable);

/*
 * @brief 停流后等待多长时间进入低功耗模式 单位：秒
 * @param name="hDevice"[in]设备handle</param>
 * @param name="dDuration"[in]时长：单位(秒)</param>
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnterLowPowerDuration(VZNLHANDLE hDevice, double dDuration);

/*
 * @brief 设置高斯模式
 * @param name="hDevice"[in]设备handle</param>
 * @param name="bHGauss"[in]横向高斯</param>
 * @param name="bVGauss"[in]纵向高斯</param>
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnableGauss(VZNLHANDLE hDevice, VzBool bHGauss, VzBool bVGauss);
VZNLAPI int VzNL_IsEnableGauss(VZNLHANDLE hDevice, VzBool* pbHGauss, VzBool* pbVGauss);

/*
 * @brief 设置高斯模式
 * @param name="hDevice"[in]设备handle</param>
 * @param name="bHGauss"[in]横向高斯</param>
 * @param name="bVGauss"[in]纵向高斯</param>
 * @return 返回错误码
 */
VZNLAPI int VzNL_EnableGauss(VZNLHANDLE hDevice, VzBool bHGauss, VzBool bVGauss);
VZNLAPI int VzNL_IsEnableGauss(VZNLHANDLE hDevice, VzBool* pbHGauss, VzBool* pbVGauss);

/*
 * @brief 设置/获取触发延迟时间
 * @param name="hDevice"[in]设备handle</param>
 * @param name="eTriggerMode"[in]触发模式</param>
 * @param name="nDelay"[in]延时时间ms</param>
 * @return 返回错误码
 */
VZNLAPI int VzNL_SetTriggerDelay(VZNLHANDLE hDevice, EVzEyeTriggerMode eTriggerMode, unsigned int nDelay);
VZNLAPI int VzNL_GetTriggerDelay(VZNLHANDLE hDevice, EVzEyeTriggerMode eTriggerMode, unsigned int* pnDelay);

/*
 * @brief 设置/获取触发消抖参数（单位: 10ns)
 * @param name="hDevice"[in]设备handle</param>
 * @param name="nDebounce"[in]消抖时间</param>
 * @return 返回错误码
 */
VZNLAPI int VzNL_SetTriggerDebounce(VZNLHANDLE hDevice, unsigned int nDebounce);
VZNLAPI unsigned int VzNL_GetTriggerDebounce(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 启用矫正ROI图像
 * @param hDevice [in] 设备Handle
 * @param bEnable [in] VzTrue启用/VzFalse禁用
 * @return 关闭成功返回0，否则为错误码
 */
VZNLAPI int VzNL_EnableCalibROI(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableCalibROI(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 是否支持高帧率下降沿模式
 * @param hDevice 当前设备句柄
 * @param pnErrorCode 错误信息，如果不需要可填NULL
 * @return VzTrue 表示支持 VzFalse 表示不支持
 */
VZNLAPI VzBool VzNL_IsSupportHighFrequenceFallingEdge(VZNLHANDLE hDevice, int* pnErrorCode);

/**
* @brief 激光检测全局降噪功能接口
* @param [in] hDevice	设备句柄
* @param [in] bEnable	VzTrue为使能全局降噪，VzFalse为不使能全局降噪
* @return 成功返回0，否则为其他错误码，返回错误可认为不支持此功能
*/
VZNLAPI int VzNL_EnableGlobalDenoise(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI int VzNL_IsEnableGlobalDenoise(VZNLHANDLE hDevice, VzBool* pbEnable);

/**
* @brief 激光检测是否启用数据压缩传输
* @param [in] hDevice	设备句柄
* @param [in] bEnable	VzTrue为使能压缩，VzFalse为不使能压缩
* @return 成功返回0，否则为其他错误码
*/
VZNLAPI int VzNL_EnableCompress(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableCompress(VZNLHANDLE hDevice, int* pnErrorCode);

/*
 * @brief 设置/获取扫描噪点过滤阈值
 * @param name="hDevice"[in]设备handle</param>
 * @param name="nDebounce"[in]噪点过滤阈值,取值范围0-255,0:关闭此功能</param>
 * @return 返回错误码
 */
VZNLAPI int VzNL_SetScanDenoiseTh(VZNLHANDLE hDevice, unsigned int nValue);
VZNLAPI int VzNL_GetScanDenoiseTh(VZNLHANDLE hDevice, unsigned int* pnValue);

/**
 * @brief 配置平滑门限
 * @param hDevice 当前设备句柄
 * @param nValue 门限值
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetSmoothingTh(VZNLHANDLE hDevice, unsigned int nValue);
VZNLAPI int VzNL_GetSmoothingTh(VZNLHANDLE hDevice, unsigned int* pnValue);

/**
 * @brief 配置激光线检测模式
 * @param hDevice 当前设备句柄
 * @param nValue 检测模式值
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetMLineMode(VZNLHANDLE hDevice, unsigned int nValue);
VZNLAPI int VzNL_GetMLineMode(VZNLHANDLE hDevice, unsigned int* pnValue);

#endif //__VIZUM_DETECTED_CONFIG_HEADER__
