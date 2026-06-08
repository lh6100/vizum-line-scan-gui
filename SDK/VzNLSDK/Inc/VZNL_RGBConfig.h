#ifndef __VIZUM_RGB_CONFIG_HEADER__
#define __VIZUM_RGB_CONFIG_HEADER__

#include "VZNL_Export.h"
#include "VZNL_Types.h"


/**
* @brief 输出RGB图像
* @param [out] pImage 返回的RGB 图像
* @param [out] pFrameProps 输出帧的参数
* @param [in] pParam 回调参数
* @return 成功返回0，否则为错误码。
*/
typedef void(*VzNL_OutputRGBImageCB)(SVzNLImageData* pImage, const SVzOutputFrameProps* pFrameProps, void* pParam);


/**
 * @brief 当前相机是否支持RGB
 * @param hDevice 当前设备句柄
 * @param pnErrorCode 错误信息，如果不需要可填NULL
 * @return VzTrue 表示支持 VzFalse 表示不支持
 */
VZNLAPI VzBool VzNL_IsSupportRGBCamera(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 获取原始分辨率
 * @param hDevice [in] 设备Handle
 * @return 关闭成功返回0，否则为错误码
 */
VZNLAPI int VzNL_GetRGBResolution(VZNLHANDLE hDevice, SVzVideoResolution* psVideoRes);

/**
 * @brief 当前相机是否支持RGB
 * @param hDevice 当前设备句柄
 * @param bEnable VzTrue启用 / VzFalse禁用
 * @return 正确返回0， 失败返回其他值
 */
/// @brief 启用/获取RGB功能
VZNLAPI int VzNL_EnableRGB(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableRGB(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 格式化彩色ROI
 * @param hDevice 当前设备句柄
 * @param psROI[in/out] 想要格式化的ROI大小
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_FormatRGBROI(VZNLHANDLE hDevice, SVzNLROIRect* psROI);

/**
 * @brief 配置RGB ROI
 * @param hDevice 当前设备句柄
 * @param psROI 想要设置的ROI大小
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetRGBROI(VZNLHANDLE hDevice, const SVzNLROIRect* psROI);
VZNLAPI int VzNL_GetRGBROI(VZNLHANDLE hDevice, SVzNLROIRect* psROI);

/**
 * @brief 启用/获取 自动白平衡
 * @param hDevice 当前设备句柄
 * @param bEnable 启用 VzTrue /禁用 VzFalse 白平衡
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_EnableRGBAWB(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableRGBAWB(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置/获取 自动曝光状态
 * @param hDevice 当前设备句柄
 * @param bEnable 启用 VzTrue / 禁用 VzFalse
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_EnableRGBAutoExpose(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableRGBAutoExpose(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置R Value
 * @param hDevice 当前设备句柄
 * @param fValue R分量值
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetRGBRValue(VZNLHANDLE hDevice, float fValue);
VZNLAPI float VzNL_GetRGBRValue(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置G Value
 * @param hDevice 当前设备句柄
 * @param fValue G分量值
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetRGBGValue(VZNLHANDLE hDevice, float fValue);
VZNLAPI float VzNL_GetRGBGValue(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置B Value
 * @param hDevice 当前设备句柄
 * @param fValue B分量值
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetRGBBValue(VZNLHANDLE hDevice, float fValue);
VZNLAPI float VzNL_GetRGBBValue(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置彩色Sensor帧率
 * @param hDevice 当前设备句柄
 * @param nValue 帧率
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetRGBFrameRate(VZNLHANDLE hDevice, unsigned int nValue);
VZNLAPI unsigned int VzNL_GetRGBFrameRate(VZNLHANDLE hDevice, int* pErrorCode);

/**
 * @brief 设置彩色Sensor曝光
 * @param hDevice 当前设备句柄
 * @param nValue 曝光值[20~100000]
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetRGBExpose(VZNLHANDLE hDevice, unsigned int nValue);
VZNLAPI unsigned int VzNL_GetRGBExpose(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置彩色sensor增益
 * @param hDevice 当前设备句柄
 * @param nValue 设置增益值
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetRGBGain(VZNLHANDLE hDevice, unsigned int nValue);
VZNLAPI unsigned int VzNL_GetRGBGain(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置RGBSensor触发方式
 * @param hDevice 当前设备句柄
 * @param eTriggerMode 触发模式
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetRGBTriggerMode(VZNLHANDLE hDevice, EVzEyeTriggerMode eTriggerMode);
VZNLAPI EVzEyeTriggerMode VzNL_GetRGBTriggerMode(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置彩色Sensor期望值
 * @param hDevice 当前设备句柄
 * @param fExposeThres 期望值[1~255]
 * @return 正确返回0， 失败返回其他值
 */
/// @brief 
VZNLAPI int VzNL_SetRGBAutoExposeThres(VZNLHANDLE hDevice, float fExposeThres);
VZNLAPI float VzNL_GetRGBAutoExposeThres(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 生成一个RGB信号
 * @param hDevice 当前设备句柄
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_GenRGBSoftSignal(VZNLHANDLE hDevice);

/**
 * @brief 设置CCM
 * @param hDevice 当前设备句柄
 * @param dCCMVal CCM值
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetRGBColorCortectMatrix(VZNLHANDLE hDevice, double dCCMVal[3][3]);
VZNLAPI int VzNL_GetRGBColorCortectMatrix(VZNLHANDLE hDevice, double dCCMVal[3][3]);

/**
 * @brief 自动计算RGBD异步参数
 * @param hDevice 当前设备句柄
 * @param dCCMVal CCM值
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_AutoCalcRGBAsyncParam(VZNLHANDLE hDevice);

/**
 * @brief 启用输出RGB 2D数据
 * @param hDevice [in] 设备Handle
 * @param bEnable [in] 启用为VzTrue,关闭为VzFalse
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_EnableOutputColor2D(VZNLHANDLE hDevice, VzBool bEnable);

/**
 * @brief 获取中间相机的RGB单帧图像
 * @param hDevice [in] 设备Handle
 * @param ppRGBImage [out] RGB图像
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_GetRGBImage(VZNLHANDLE hDevice, SVzNLImageData** ppRGBImage);


/**
 * @brief  通过3d点获取RGB图像的2d点坐标
 * @param hDevice [in] 设备Handle
 * @param s3DPoint [in] 3d点
 * @param ps2DPoint [in/out] 2d点
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_CalcColor2DFrom3D(VZNLHANDLE hDevice, const SVzNL3DPoint s3DPoint, SVzNL2DPoint* ps2DPoint);

/**
* @brief 获取RGB 相机内参及转换矩阵
* @detail 获取RGB相机内参数据及相机左目坐标系(3D坐标系)到RGB 图像坐标系的转换矩阵
* @param [Out] fIntrinsic: 输出RGB相机内参: 3*3 矩阵；
* @param [Out] fCalibMatrix: 输出转换矩阵: 4*4 矩阵；
* @param [Out] pResolution: 输出RGB图像分辨率，MapData 尺寸与此相同；
* @param [Out] ppMapData: 输出RGB图像Map数据，用于进行校正图与原图的转换；
* @return 成功返回0，否则为其他错误码
*/
VZNLAPI int VZNL_GetRGBCalibParam(VZNLHANDLE hDevice, float fIntrinsic[9], float fCalibMatrix[16],
	SVzVideoResolution* pResolution, SVzNL2DPointShortU** ppMapData);
/**
* @brief 释放获取的RGB Map数据
* @detail 释放由GetRGBCalibParam() 接口获取的Map数据；
* @param [In] ppMapData: 由GetRGBCalibParam() 接口输出的Map 数据；
* @return 成功返回0，否则为其他错误码
*/
VZNLAPI int VZNL_ReleaseRGBMapData(VZNLHANDLE hDevice, SVzNL2DPointShortU** ppMapData);

/**
* @brief 是否支持RGB HDR功能
* @param [in] hDevice 设备句柄
* @param [in] pnErrorCode 错误码,为nullptr时不返回
* @return 返回VzTrue表示支持，否则为不支持。
*/
VZNLAPI VzBool VzNL_IsSupportRGBHDR(VZNLHANDLE hDevice, int* pnErrorCode);

/**
* @brief 使能彩色sensor hdr功能
* @param hDevice 当前设备句柄
* @param bEnable VzTrue启用 / VzFalse禁用
* @return 正确返回0， 失败返回其他值
*/
VZNLAPI int VzNL_EnableRGBHDR(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableRGBHDR(VZNLHANDLE hDevice, int* pnErrorCode);

/**
* @brief 设置彩色Sensor hdr 高曝光
* @param hDevice 当前设备句柄
* @param nValue 曝光值[20~100000]
* @return 正确返回0， 失败返回其他值
*/
VZNLAPI int VzNL_SetRGBHDRHighExpose(VZNLHANDLE hDevice, unsigned int nValue);
VZNLAPI unsigned int VzNL_GetRGBHDRHighExpose(VZNLHANDLE hDevice, int* pnErrorCode);

/**
* @brief 设置彩色Sensor hdr 低曝光
* @param hDevice 当前设备句柄
* @param nValue 曝光值[20~100000]
* @return 正确返回0， 失败返回其他值
*/
VZNLAPI int VzNL_SetRGBHDRLowExpose(VZNLHANDLE hDevice, unsigned int nValue);
VZNLAPI unsigned int VzNL_GetRGBHDRLowExpose(VZNLHANDLE hDevice, int* pnErrorCode);

/**
* @brief 设置熔池检测区域
* @param [in] hDevice 设备句柄
* @param [in] pROIArea RGB ROI区域
* @return 返回错误码
*/
VZNLAPI int VzNL_ConfigMoltenPoolROI(VZNLHANDLE hDevice, const SVzNLROIRect* pROIArea);
VZNLAPI int VzNL_GetConfigMoltenPoolROI(VZNLHANDLE hDevice, SVzNLROIRect* pROIArea);


/**
* @brief 设置RGBHDR 出图模式
* @param hDevice 当前设备句柄
* @param nMode 输出图像模式：原图，HDR图，背景图，熔池图
* @return 正确返回0， 失败返回返回错误码
*/
VZNLAPI int VzNL_SetRGBHDRImageMode(VZNLHANDLE hDevice, EVzRGBHDROutImageMode eMode);
VZNLAPI EVzRGBHDROutImageMode VzNL_GetRGBHDRImageMode(VZNLHANDLE hDevice, int* pnErrorCode);

/*
* @brief 开始实时采集RGB图像
* @param [in] hDevice		设备Handle
* @param [in] pImageCB		图像回调接口
* @param [in] pCBParam		回调接口参数
* @return 调用接口成功返回0
*/
VZNLAPI int VzNL_StartRGBCapture(VZNLHANDLE hDevice, VzNL_OutputRGBImageCB pImageCB, void* pCBParam);

/**
* @brief 停止实时采集RGB图像
* @param [in] hDevice		设备Handle
* @return 调用接口成功返回0
*/
VZNLAPI int VzNL_StopRGBCapture(VZNLHANDLE hDevice);

#endif