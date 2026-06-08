/*
 * Header: VZNL_Graphics.h
 * Description:当前文件为伟景智能(Vizum)所研发的EyeCB板提供检测函数接口。
 *          
 * Sample:     
 *          VzNL_BeginDetectLaser();
 *  
 *          VzNL_SetLaserStandard();
 *  
 *          VzNL_EndDetectLaser();
 * Author: Mjw
 * Date:   2018/08/28
 */

#ifndef __VIZUM_DETECTED_LASER_HEADER__
#define __VIZUM_DETECTED_LASER_HEADER__

#include "VZNL_Export.h"
#include "VZNL_Types.h"

/**
 * @brief 开始激光检测
 * @param [in] hDevice 设备句柄
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI int VzNL_BeginDetectLaser(VZNLHANDLE hDevice);

/**
 * @brief 激光检测
 * @param [in] hDevice			设备句柄
 * @param [in] nPointInterval	点的间隔
 * @return 返回0为正确，失败返回错误码
 */
VZNLAPI int VzNL_DetectLaser(VZNLHANDLE hDevice, int nPointInterval);

/**
 * @brief 获取激光线结果点的个数
 * @param [in] hDevice			设备句柄
 * @return 返回点的个数
 */
VZNLAPI int VzNL_GetLaserResultPointCount(VZNLHANDLE hDevice);

/// @brief
/// 获取激光线2D结果
/// <param name = "hDevice">[in]设备句柄</param>
/// <param name = "p2DPoint">[out]2D点，内存空间由用户分配</param>
/// <param name = "pnCount">[in/out]传入:用户分配的SVzNL2DPoint的结构个数;传出:用户需要分配多少个SVzNL2DPoint</param>
/// @return 成功返回0，失败返回错误码。
VZNLAPI int VzNL_GetLaser2DResult(VZNLHANDLE hDevice, SVzNL2DPosition* p2DPoint, int* pnCount);

/// @brief
/// 获取激光线3D结果
/// <param name = "hDevice">[in]设备句柄</param>
/// <param name = "p3DPoint">[out]3D点，内存空间由用户分配</param>
/// <param name = "pnCount">[in/out]传入:用户分配的SVzNL3DPosition的结构个数;传出:用户需要分配多少个SVzNL3DPosition</param>
/// @return 成功返回0，失败返回错误码。
VZNLAPI int	VzNL_GetLaser3DResult(VZNLHANDLE hDevice, SVzNL3DPosition* p3DPoint, int* pnCount);

/// @brief
/// 获取结果图像
/// <param name = "hDevice">[in]设备句柄</param>
/// <param name = "ppIImageData">[in]图像数据</param>
VZNLAPI int VzNL_GetLaserImageResult(VZNLHANDLE hDevice, SVzNLImageData** ppLeftImageData, SVzNLImageData** ppRightImageData);

/// @brief
/// 检测激光凹槽数据
/// <param name = "hDevice">[in]设备句柄</param>
/// <param name = "dMinSlotDeep">[in]最小凹槽深度</param>
/// @return 返回0为正确，失败返回错误码
VZNLAPI int VzNL_SetLaserMinSlotDeep(VZNLHANDLE hDevice, double dMinSlotDeep);

/// @brief
/// 获取激光凹槽数据结果的波峰点的个数
/// <param name = "hDevice">[in]设备句柄</param>
VZNLAPI int VzNL_GetLaserSlotResultPeakPonitCount(VZNLHANDLE hDevice);

/// @brief
/// 获取激光凹槽数据结果的波谷点的个数
/// <param name = "hDevice">[in]设备句柄</param>
VZNLAPI int VzNL_GetLaserSlotResultValleyPonitCount(VZNLHANDLE hDevice);

/// @brief
/// 获取激光凹槽2D结果
/// <param name = "hDevice">[in]设备句柄</param>
/// <param name = "pPeakPoint">[out]波峰点，内存空间由用户分配</param>
/// <param name = "pnPeakPointCount">[in/out]传入:用户分配的SVzNL2DPoint的结构个数;传出:用户需要分配多少个SVzNL2DPoint</param>
/// <param name = "pValleyPoint">[out]波谷点，内存空间由用户分配</param>
/// <param name = "pnValleyPointCount">[in/out]传入:用户分配的SVzNL2DPoint的结构个数;传出:用户需要分配多少个SVzNL2DPoint</param>
/// @return 成功返回0，失败返回错误码。
VZNLAPI int VzNL_GetLaserSlot2DResult(VZNLHANDLE hDevice, SVzNL2DPosition* pPeakPoint, int* pnPeakPointCount, SVzNL2DPosition* pValleyPoint, int* pnValleyPointCount);

/// @brief
/// 获取激光凹槽3D结果
/// <param name = "hDevice">[in]设备句柄</param>
/// <param name = "pPeakPoint">[out]波峰点，内存空间由用户分配</param>
/// <param name = "pnPeakPointCount">[in/out]传入:用户分配的SVzNL3DPoint的结构个数;传出:用户需要分配多少个SVzNL3DPoint</param>
/// <param name = "pValleyPoint">[out]波谷点，内存空间由用户分配</param>
/// <param name = "pnValleyPointCount">[in/out]传入:用户分配的SVzNL3DPoint的结构个数;传出:用户需要分配多少个SVzNL3DPoint</param>
/// @return 成功返回0，失败返回错误码。
VZNLAPI int	VzNL_GetLaserSlot3DResult(VZNLHANDLE hDevice, SVzNL3DPosition* pPeakPoint, int* pnPeakPointCount, SVzNL3DPosition* pValleyPoint, int* pnValleyPointCount);

/// @brief
/// 启动自动检测，并且返回激光线
/// <param name = "hDevice">[in]设备句柄</param>
/// <param name = "eFlipType">[in]激光线方向,keFlipType_Vertical垂直翻转</param>
/// <param name = "pCB">[in]回调函数</param>
/// <param name = "pCBParam">[in]回调函数参数</param>
/// @return 成功返回0，失败返回错误码。
VZNLAPI int VzNL_StartAutoDetect(VZNLHANDLE hDevice, EVzFlipType eFlipType, VzNL_GetAutoDetectResultCB pCB, void* pCBParam);
VZNLAPI int VzNL_StartAutoDetectEx(VZNLHANDLE hDevice, EVzResultDataType eResultType, EVzFlipType eFlipType, VzNL_AutoOutputLaserLineExCB pCB, void* pCBParam);

VZNLAPI int VzNL_StopAutoDetect(VZNLHANDLE hDevice);

/// @brief
/// 是否在自动检测中
VZNLAPI VzBool VzNL_IsAutoDetecting(VZNLHANDLE hDevice, int* pnErrorCode);

 /// @brief
 /// 获取扫描后的RGB原图(见说明书3.5.31)
 /// <param name = "hDevice">[in]设备</param>
 /// <param name = "ppResultImage">[out]输出RGB图像，需要用户调用VzNL_ReleaseImage进行数据释放</param>
 /// @return 成功返回0，失败返回错误码。
VZNLAPI int VzNL_GetAutoDetectResultSurface(VZNLHANDLE hDevice, SVzNLImageData** ppResultImage);

/// @brief
/// 设置过滤高度
/// <param name = "hDevice">[in]设备</param>
/// <param name = "dFilterHeight">[in]过滤高度（未高度标定时，大于此高度的数据不输出，高度标定后，小于此数据的高度不输出）</param>
/// @return 成功返回0，失败返回错误码。
VZNLAPI int VzNL_ConfigLaserLineFilterHeight(VZNLHANDLE hDevice, double dFilterHeight);

/// @brief
/// 获取过滤高度
/// <param name = "hDevice">[in]设备</param>
/// <param name = "pdFilterHeight">[out]过滤高度</param>
/// @return 成功返回0，失败返回错误码。
VZNLAPI int VzNL_GetLaserLineFilterHeight(VZNLHANDLE hDevice, double* pdFilterHeight);

/// @brief
/// 配置激光线过滤范围
/// <param name = "hDevice">[in]设备</param>
/// <param name = "eFilterType">[in]过滤类型</param>
/// <param name = "dRange">[in]过滤范围</param>
/// @return 成功返回0，失败返回错误码
VZNLAPI int VzNL_SetLaserLineFilterRange(VZNLHANDLE hDevice, EVzLaserPointFilterType eFilterType, double dRange[2]);

/// @brief
/// 获取激光线过滤范围
/// <param name = "hDevice">[in]设备</param>
/// <param name = "eFilterType">[in]过滤类型</param>
/// <param name = "dRange">[in]过滤范围</param>
/// @return 成功返回0，失败返回错误码
VZNLAPI int VzNL_GetLaserLineFilterRange(VZNLHANDLE hDevice, EVzLaserPointFilterType eFilterType, double dRange[2]);

/// @brief
/// 启用/禁用激光过滤
/// <param name = "hDevice">[in]设备</param>
/// <param name = "eFilterType">[in]过滤类型</param>
/// <param name = "dRange">[in]过滤范围</param>
/// @return 成功返回0，失败返回错误码
VZNLAPI int VzNL_EnableLaserLineFilter(VZNLHANDLE hDevice, EVzLaserPointFilterType eFilterType, VzBool bEnable);

/// @brief
/// 是否启用激光过滤
/// <param name = "hDevice">[in]设备</param>
/// <param name = "pnErrorCode">[out]错误码</param>
/// @return 启用返回VzTrue，未启用返回VzFalse
VZNLAPI VzBool VzNL_IsEnableLaserLineFilter(VZNLHANDLE hDevice, EVzLaserPointFilterType eFilterType, int* pnErrorCode);


/// @brief
/// 启用/禁用过滤高度
/// <param name = "hDevice">[in]设备</param>
/// <param name = "bEnable">[in]启用过滤高度</param>
/// @return 成功返回0，失败返回错误码
VZNLAPI int VzNL_EnableLaserLineFilterHeight(VZNLHANDLE hDevice, VzBool bEnable);

/// @brief
/// 是否启用过滤高度
/// <param name = "hDevice">[in]设备</param>
/// <param name = "pnErrorCode">[out]错误码</param>
/// @return 启用返回VzTrue，未启用返回VzFalse
VZNLAPI VzBool VzNL_IsEnableLaserLineFilterHeight(VZNLHANDLE hDevice, int* pnErrorCode);

/// @brief
/// 设置杂点过滤阈值
/// <param name = "hDevice">[in]设备句柄</param>
/// <param name = "dMaxDeviation">[in]过滤值</param>
VZNLAPI int VzNL_ConfigLaserLineMaxDeviation(VZNLHANDLE hDevice, double dMaxDeviation);

/// @brief
/// 设置偏移量
/// <param name = "hDevice">[in]设备句柄</param>
/// <param name = "dOffsetValue">[in]偏移量</param>
VZNLAPI int VzNL_ConfigLaserBeginOffsetValue(VZNLHANDLE hDevice, double dOffsetValue);

/**
 * @brief 设置点云处理模式
 * @param [in] hDevice					设备句柄
 * @param [in] ePointCloudProcMode		kePointCloudProcMode_Speed 速度计算模式, kePointCloudProcMode_Encoder 编码器模式 kePointCloudProcMode_FixedStep 固定步长模式
 * @return 成功返回0，否则为其他错误码
 * Old Interface int VzNL_EnableLaserRollerMode(VZNLHANDLE hDevice, VzBool bEnable);
 */
VZNLAPI int VzNL_SetPointCloudProcMode(VZNLHANDLE hDevice, EVzPointCloudProcMode ePointCloudProcMode);
VZNLAPI int VzNL_GetPointCloudProcMode(VZNLHANDLE hDevice, EVzPointCloudProcMode* pePointCloudProcMode);

/** @brief 设置物体运动方向
 * <param name = "hDevice">[in]设备句柄</param>
 * <param name = "eDirect">[in]运行方向[keObjRunDirect_Plus:偏移递增 keObjRunDirect_Minus:偏移递减]</param> 
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI int VzNL_SetLaserObjRunDirection(VZNLHANDLE hDevice, EVzObjRunDirect eDirect);
VZNLAPI int VzNL_GetLaserObjRunDirection(VZNLHANDLE hDevice, EVzObjRunDirect* peDirect);

/** @brief 设置传送带速度值 
 * 需要配置 VzNL_SetPointCloudProcMode(hDevice, kePointCloudProcMode_Speed) 速度计算模式
 * <param name = "hDevice">[in]设备句柄</param>
 * <param name = "dSpeed">[in]速度</param>
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI int VzNL_ConfigLaserObjRunSpeedValue(VZNLHANDLE hDevice, double dSpeed);
VZNLAPI int VzNL_GetLaserObjRunSpeedValue(VZNLHANDLE hDevice, double* pdSpeed);

/**
 * @brief 设置固定步长
 * @detail 需要配置 VzNL_SetPointCloudProcMode(hDevice, kePointCloudProcMode_FixedStep) 固定步长计算模式
 * @param [in] hDevice			设备句柄
 * @param [in] dStep			
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI int VzNL_SetLaserLineFixedStep(VZNLHANDLE hDevice, double dStep); 
VZNLAPI int VzNL_GetLaserLineFixedStep(VZNLHANDLE hDevice, double* pdStep);

/*
 * @brief 设置/获取编码器参数类型
 * @detail 需要配置 VzNL_SetPointCloudProcMode(hDevice, kePointCloudProcMode_Encoder) 编码器计算模式
 * @param[in] hDevice			设备句柄
 * @param[in] eParamType		编码器参数类型
 *                              使用 编码器分辨率 + 编码器半径参数 时配置 keEncodeParamType_RollerResolution
 *								使用 编码器每两个脉冲的间距 时配置		keEncodeParamType_DistancePerPulse
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI int VzNL_SetLaserEncoderParamType(VZNLHANDLE hDevice, EVzEncodeParamType eParamType);
VZNLAPI int VzNL_GetLaserEncoderParamType(VZNLHANDLE hDevice, EVzEncodeParamType* peParamType);

/**
 * @brief 配置转动轴半径
 * @detail 需要配置 VzNL_SetPointCloudProcMode(hDevice, kePointCloudProcMode_Encoder) 编码器计算模式
 *         需要配置 VzNL_SetLaserEncoderParamType(hDevice, keEncodeParamType_RollerResolution) 参数使用模式
 * @param [in] hDevice			设备句柄
 * @param [in] dRadius			转动轴半径
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI int VzNL_ConfigLaserRollerRadius(VZNLHANDLE hDevice, double dRadius);
VZNLAPI int VzNL_GetLaserRollerRadius(VZNLHANDLE hDevice, double* pdRadius);

/**
 * @brief 设置编码器脉冲精度
 * @detail 需要配置 VzNL_SetPointCloudProcMode(hDevice, kePointCloudProcMode_Encoder) 编码器计算模式
 *         需要配置 VzNL_SetLaserEncoderParamType(hDevice, keEncodeParamType_RollerResolution) 参数使用模式
 * @param [in] hDevice			设备句柄
 * @param [in] nPulsePerRound	脉冲个数
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI int VzNL_SetLaserEncoderResolution(VZNLHANDLE hDevice, unsigned int nPulsePerRound);
VZNLAPI int VzNL_GetLaserEncoderResolution(VZNLHANDLE hDevice, unsigned int* pnPulsePerRound);

/**
 * @brief 配置编码器间距
 * @detail 需要配置 VzNL_SetPointCloudProcMode(hDevice, kePointCloudProcMode_Encoder) 编码器计算模式
 *         需要配置 VzNL_SetLaserEncoderParamType(hDevice, keEncodeParamType_DistancePerPulse) 参数使用模式
 * @param [in] hDevice				设备句柄
 * @param [in] dDistancePerPulse	编码器间距
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI int VzNL_SetLaserEncoderDistancePerPulse(VZNLHANDLE hDevice, double dDistancePerPulse);
VZNLAPI int VzNL_GetLaserEncoderDistancePerPulse(VZNLHANDLE hDevice, double* pdDistancePerPulse);

/**
 * @brief 计算平均高度/最低高度/最高高度
 * @param [in] p3DPoint			3D点【Array】
 * @param [in] nCount	        点个数
 * @param [out] pdAvgHeight	    平均高度
 * @param [out] pdMinHeight	    最小高度
 * @param [out] pdMaxHeight	    最大高度
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI void VzNL_CalcLaserZHeight(SVzNL3DPosition* p3DPoint, int nCount, double* pdAvgHeight, double* pdMinHeight, double* pdMaxHeight);

/**
 * @brief 激光检测填充点模式
 * @param [in] hDevice			设备句柄
 * @param [in] bFillPoint		VzTrue为填充，VzFalse为不填充(默认为VzFalse)
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI int VzNL_EnableFillLaserPoint(VZNLHANDLE hDevice, VzBool bFillPoint);
VZNLAPI int VzNL_IsEnableFillLaserPoint(VZNLHANDLE hDevice, VzBool* pbFillPoint);

/**
 * @brief 设置每次回调返回多少帧数据的数量
 * @param [in] hDevice	设备句柄
 * @param [in] nCount	多少帧回调一次数，参数范围0-10000
 * @return 成功返回0，否则为其他错误码
 */
VZNLAPI int VzNL_SetOutputLaserLinesEachTime(VZNLHANDLE hDevice, unsigned int nNum);
VZNLAPI int VzNL_GetOutputLaserLinesEachTime(VZNLHANDLE hDevice, unsigned int* nNum);


/**
* @brief 使能/禁止点云数据缓存功能
* @param [in] bEnable 使能/禁用，使能后扫描点云过程中会将数据缓存到本地，可进行后续的2D点到3D点查找
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_EnablePointCloudCache(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnablePointCloudCache(VZNLHANDLE hDevice, int* pnErrorCode);

/**
* @brief 查找输入2D点对应的3D 点坐标，需使能点云数据缓存功能，并完成扫描。
* @param [in] s2DPos: 输入从图上选中的2D点坐标，RGBD扫描：RGB图像的坐标；双目RGBD扫描，左目图像坐标；
* @param [in] p3DPoint: 输出匹配的3D 点坐标；
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_GetCached3DPointBy2DPos(VZNLHANDLE hDevice, SVzNL2DPoint s2DPos, SVzNL3DPoint* p3DPoint);

/**
* @brief 执行静态扫描点云拍照(阻塞执行)，输出完整扫描点云，如果为RGBD 扫描则同步返回RGB图像；
* @param [in] hDevice			设备句柄
* @param [in] eResultType		输出点云数据类型
* @param [in] eFlipType			点云是否进行Y轴镜像
* @param [in] bDoPointCloudMap	是否将点云数据映射到RGB图像分辨率
* @param [in] bEnableClip		是否按点云实际对应图像区域裁剪RGB图像，true：输出裁剪后的图像；false：输出原始图像；
* @param [out] ppImage			RGBD 扫描时输出RGB图像(参考bEnableClip标志)，非RGBD 扫描时输出为nullptr
* @param [out] pPointCnt		输出点云数据点个数
* @param [out] ppPointList		输出点云数据，数据类型为eResultType
* @return 返回0表示正确，非0 表示失败,可以使用VzNL_GetErrorInfo获取错误信息
*/
VZNLAPI int VzNL_PointCloudSnapShort(VZNLHANDLE hDevice, EVzResultDataType eResultType, EVzFlipType eFlipType, VzBool bDoPointCloudMap,
									 VzBool bEnableClip, SVzNLImageData** ppImage, unsigned int* pPointCnt, void** ppPointList);
/**
* @brief 释放VzNL_PointCloudSnapShort输出的点云和图像数据
*/
VZNLAPI int VzNL_ReleasePointCloudSnapShortResult(SVzNLImageData** ppImage, void** ppPointList);

/**
* @brief 执行焊缝跟踪点云扫描
* @detail 保存当前工作模式及配置参数->设置相机为焊缝跟踪模式(预先设置的焊缝跟踪参数)->开始点云扫描
* @param [in] hDevice		设备句柄
* @param [in] sPara			焊缝跟踪功能需要的可配参数，保存到相机中。
* @return 成功返回0，否则为其他错误码
*/
VZNLAPI int VzNL_StartAutoDetectForTracking(VZNLHANDLE hDevice, EVzResultDataType eResultType, EVzFlipType eFlipType, VzNL_AutoOutputLaserLineExCB pCB, void* pCBParam);

/**
* @brief 停止焊缝跟踪点云扫描
* @detail 停止点云扫描->恢复相机参数及工作模式
* @param [in] hDevice		设备句柄
* @return 成功返回0，否则为其他错误码
*/
VZNLAPI int VzNL_StopAutoDetectForTracking(VZNLHANDLE hDevice);

/**
 * @brief 结束激光检测
 * @param [in] hDevice			设备句柄
 * @return 返回点的个数
 */
VZNLAPI void VzNL_EndDetectLaser(VZNLHANDLE hDevice);
#endif //__VIZUM_DETECTED_LASER_HEADER__
