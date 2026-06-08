/*
 * Header: VzNL_Common.h
 * Description:当前文件为伟景智能(Vizum)所研发的EyeCB板提供公用函数接口。在使用此套SDK时必须调用VzNL_Init接口，
 *             在不需要的时候调用VzNL_Destroy接口进行销毁。
 * Sample:
 *             VzNL_Init();
 *             xxxxxxxxxxx
 *             VzNL_Destroy();
 * Author: Mjw
 * Date:   2018/08/28
 */

#ifndef __VIZUM_COMMON_HEADER__
#define __VIZUM_COMMON_HEADER__

#include "VZNL_Export.h"
#include "VZNL_Types.h"

/**
 * @brief 输出图像
 * @param [out] pLeftImage 左图
 * @param [out] pRightImage 右图
 * @param [out] pFrameProps 输出帧的参数
 * @param [in] pParam 回调参数
 * @return 成功返回0，否则为错误码。
 */
typedef void(*VzNL_OutputImageCB)(SVzNLImageData* pLeftImage, SVzNLImageData* pRightImage, SVzNLImageData* pCenterImage, const SVzOutputFrameProps* pFrameProps, void* pParam);



/**
 * @brief 融合设备状态改变CallBack函数
 * @param eStatus [out] 状态
 * @param pInfoParam [out] 状态参数
 */
typedef void(*VzNL_OnMulDevNotifyStatusCB)(EVzMulDevWorkStatus eStatus, void* pInfoParam);

/**
 * @brief 设备状态改变CallBack函数
 * @param eStatus [out] 状态
 * @param pInfoParam [out] 状态参数
 */
typedef void(*VzNL_OnNotifyStatusCB)(EVzDeviceWorkStatus eStatus, void* pInfoParam);
typedef void(*VzNL_OnNotifyStatusCBEx)(EVzDeviceWorkStatus eStatus, void* pExtData, unsigned int nDataLength, void* pInfoParam);

/**
 * @brief 初始化SDK
 * @param [in] pConfigParam 系统参数
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_Init(const SVzNLConfigParam* pConfigParam);

/**
 * @brief 添加设备IP列表
 * @param [in] 设备IP列表
 * @param [in] 个数
 * @return 添加成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetDeviceIpList(SVzNLDeviceIp* pDeviceList,unsigned int nCount);

/**
 * @brief 重新搜索设备
 * @param [in] nFindDevFlag 
 * keSearchDeviceFlag_EyeCB				搜索控制器
 * keSearchDeviceFlag_USBLargeEye		搜索USB极光眼
 * keSearchDeviceFlag_EthLargeEye		搜索网络星光眼:激光
 * keSearchDeviceFlag_EthSmallEye		搜索网络星光眼:智能
 * keSearchDeviceFlag_EthLaserRobotEye	搜索网络智光眼:激光
 * keSearchDeviceFlagAll				搜索全部支持的设备
 * @return 搜索成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_ResearchDevice(EVzSearchDeviceFlag nFindDevFlag);

/**
 * @brief 获取所有搜索到的设备。
 * @param [in] 设备信息，这个Buffer需要用户根据Count值来自己malloc一个大小，获取大小时可填入NULL 用户分配空间
 * @param [in/out] in:用户申请info结构的个数，out:实际获取到的设备个数
 * @return 获取成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_GetEyeCBDeviceInfo(SVzNLEyeCBInfo* pInfo, int* pnCount);

/**
 * @brief 获取所有打开的设备。
 * @param [in] pInfo 用户分配空间
 * @param [in/out] in:用户申请的info的个数，out:实际获取的设备个数
 * @return 获取成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_GetAllOpenDevice(SVzNLEyeCBInfo* pInfo, int* pnCount);


/**
 * @brief 查询设备是否被占用。
 * @param [in] 设备信息
 * @param [out] pErrorCode			返回错误码, 为 NULL时，不返回
 * @return VzTrue 被占用，否则未被占用
 */
VZNLAPI VzBool VzNL_IsDeviceInUse(const SVzNLEyeCBInfo* pInfo, int* pnErrorCode);

/**
 * @brief 打开设备。
 * @param [in] hDevice 设备句柄
 * @param [out] pOpenDeviceParam	设备打开信息
 * @param [out] pErrorCode			返回错误码, 为 NULL时，不返回
 * @return 获取成功返回0，否则为错误码。
 */
VZNLAPI VZNLHANDLE VzNL_OpenDevice(const SVzNLEyeCBInfo* pInfo, const SVzNLOpenDeviceParam* pOpenDeviceParam, int* pErrorCode);

/**
 * @brief 获取当前的SDK版本信息。
 * @param [in] hDevice 设备句柄
 * @param [out] pVersionInfo 版本信息
 * @return 获取成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_GetVersion(VZNLHANDLE hDevice, SVzNLVersionInfo* pVersionInfo);

/**
 * @brief 获取设备信息
 * @param [in] hDevice 设备句柄
 * @param [in] pInfo 设备信息
 * @return 获取成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_GetDeviceInfo(VZNLHANDLE hDevice, SVzNLEyeCBInfo* pInfo);

/**
 * @brief 设置图像输出格式
 * @param [in] eOutputImageType 当值为keVzNLImageType_None时，表示输出原图格式，值为其他时则返回其他格式
 */
VZNLAPI void VzNL_SetOutputImageFormat(EVzNLImageType eOutputImageType);

/**
 * @brief 获取眼睛视图
 * 如果不需要左眼图，或者不需要右眼图，那么可以设置其变量为null
 * @param [in] hDevice 设备句柄
 * @param [out] ppLeftEye 左眼视图
 * @param [out] ppRightEye 右眼视图
 * @param [in] nTimeOut 超时时间
 * @return 成功释放返回0，否则为错误码。
 */
VZNLAPI int VzNL_GetEyeImage(VZNLHANDLE hDevice, SVzNLImageData** ppLeftEye, SVzNLImageData** ppRightEye, unsigned int nTimeOut);

/**
 * @brief 释放图像
 * @param [in] ppImageData 图像指针
 * @return 成功释放返回0，否则为错误码。
 */
VZNLAPI int VzNL_ReleaseImage(SVzNLImageData** ppImageData);

/**
 * @brief 连续取图
 * @param [in] hDevice		设备Handle
 * @param [in] pImageCB		图像回调接口
 * @param [in] pCBParam		回调接口参数
 * @return 调用接口成功返回0
 */
VZNLAPI int VzNL_StartCapture(VZNLHANDLE hDevice, VzNL_OutputImageCB pImageCB, void* pCBParam);

/** 
 * @brief 停止采集图像
 * @param [in] hDevice		设备Handle
 * @return 调用接口成功返回0
 */
VZNLAPI int VzNL_StopCapture(VZNLHANDLE hDevice);

/**
 * @brief 是否处于取图中
 * @param [in]  hDevice		设备Handle
 * @param [out] pnErrorCode 当pnErrorCode不为nullptr时,输出错误码
 * @return 返回VzTrue,表示正在取图,返回VzFalse表示不在取图
 */
VZNLAPI VzBool VzNL_IsCapturing(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 关闭设备
 * @param [in] hDevice		设备Handle
 * @return 关闭成功返回0，否则为错误码
 */
VZNLAPI int VzNL_CloseDevice(VZNLHANDLE hDevice);

/**
 * @brief 系统Idle时执行此函数
 * @return 关闭成功返回0，否则为错误码
 */
VZNLAPI int VzNL_SysIdleProc();

/**
 * @brief 设置Log等级
 * @param eLevel [in] Log等级
 * @param eLogType [in] Log类型
 * @return 关闭成功返回0，否则为错误码
 */
VZNLAPI void VzNL_SetLogLevel(EVzNLLogLevel eLevel, EVzNLLogType eLogType);

/**
 * @brief 获取错误信息
 * @param [in] nErrorCode 错误码
 * @param [out] szError 错误信息
 * @return 关闭成功返回0，否则为错误码
 */
VZNLAPI int VzNL_GetErrorInfo(int nErrorCode, char szError[256]);

/**
 * @brief 设备状态提醒
 * @param [in] hDevice	 设备Handle
 * @param [in] pNotifyCB 返回函数
 * @param [in] pCBParam  回调函数返回参数
 * @return 关闭成功返回0，否则为错误码
 */
VZNLAPI int VzNL_SetDeviceStatusNotify(VZNLHANDLE hDevice, VzNL_OnNotifyStatusCB pNotifyCB, void* pCBParam);
VZNLAPI int VzNL_SetDeviceStatusNotifyEx(VZNLHANDLE hDevice, VzNL_OnNotifyStatusCBEx pNotifyCB, void* pCBParam);

/**
 * @brief 重连控制器
 * @param [in] hDevice		设备Handle
 * @return 关闭成功返回0，否则为错误码
 */
VZNLAPI int VzNL_ReConnectDevice(VZNLHANDLE hDevice);

/** @brief 重启相机
 *  @param[in] hDevice		设备Handle
 */
VZNLAPI int VzNL_RebootDevice(VZNLHANDLE hDevice);

/** @brief 获取图像标定矩阵
 *  @param[in] hDevice			设备Handle
 *  @param[out] dQMatrixData	图像矩阵
 */
VZNLAPI int VzNL_GetImageCalibMatrix(VZNLHANDLE hDevice, double dQMatrixData[16]);

/** @brief 获取双目固定视差
*  @param[in] hDevice			设备Handle
*  @param[out] pFixDisparity    返回固定视差
*/
VZNLAPI int VzNL_GetFixDisparity(VZNLHANDLE hDevice, int* pFixDisparity);

/** @brief 获取SDK内存占用大小
 *  @param [out] pnErrorCode 当pnErrorCode不为nullptr时,输出错误码
 */
VZNLAPI unsigned int VzNL_GetUsingMemorySize(int* pnErrorCode);

/**
 * @brief 获取相机瞳距信息
 * @param hDevice 当前设备句柄
 * @param [out] pnErrorCode 当pnErrorCode不为nullptr时,输出错误码
 * @return 瞳距
 */
VZNLAPI unsigned int VzNL_GetPupilDistance(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 开启/关闭快速采图模式
 * @param hDevice 当前设备句柄
 * @param [in] bEnableQuickMode 是否启用
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_EnableQuickMode(VZNLHANDLE hDevice, VzBool bEnableQuickMode);
VZNLAPI VzBool VzNL_IsEnableQuickMode(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 获取左右目分辨率
 * @param hDevice [in] 设备Handle
 * @param psVideoRes [in] 输出的分辨率
 * @return 关闭成功返回0，否则为错误码
 */
VZNLAPI int VzNL_GetResolution(VZNLHANDLE hDevice, SVzVideoResolution* psVideoRes);

/**
 * @brief 开启/关闭以左目坐标为原点来计算点云方式
 * @param hDevice 当前设备句柄
 * @param [in] bEnable 是否启用
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_EnableLeftEyeOrigin(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableLeftEyeOrigin(VZNLHANDLE hDevice, int* pnErrorCode);


/**
 * @brief 获取左目内参
 * @param hDevice 当前设备句柄
 * @param [in] dMatrix 左目内参矩阵 3*3 + 5*1
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_QueryLeftEyeCalibData(VZNLHANDLE hDevice, double dMatrix[14]);

/**
 * @brief 获取左目到3D点云的转换矩阵
 * @param hDevice 当前设备句柄
 * @param [in] dMatrix 左目到3D的转换矩阵 4*4
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_QueryLeftEye2StereoMatrix(VZNLHANDLE hDevice, double dMatrix[16]);

/**
 * @brief 获取3D点云到左目的转换矩阵
 * @param hDevice 当前设备句柄
 * @param [in] dMatrix 3D到左目的转换矩阵 4*4
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_QueryStereo2LeftEyeMatrix(VZNLHANDLE hDevice, double dMatrix[16]);


/**
 * @brief 配置检测物体环境参数
 * @param hDevice 当前设备句柄
 * @param eEnvType 环境类型
 * @param nLevel 参数等级
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_ConfigDetectEnvLevel(VZNLHANDLE hDevice, EVzEnvType eEnvType,unsigned int nLevel);
VZNLAPI SVzEnvLevel VzNL_GetConfigDetectEnvLevel(VZNLHANDLE hDevice, int* pnErrorCode);
/**
 * @brief 获取配置检测物体环境参数范围
 * @param hDevice 当前设备句柄
 * @param eEnvType 环境类型
 * @param [out] pnErrorCode 当pnErrorCode不为nullptr时,输出错误码
 * @return 参数等级范围
 */
VZNLAPI SVzNLRange VzNL_GetDetectEnvLevelRange(VZNLHANDLE hDevice, EVzEnvType eEnvType, int* pnErrorCode);


/*
 * @brief 获取温度信息
 * @param hDevice [in] 设备句柄
 * @param eChipInfo [in] 芯片信息
 * @param pfMin [out] 最低温度
 * @param pfMax [out] 最高温度
 * @param pfCur [out] 当前温度
 * @return 失败返回非0
 */
VZNLAPI int VzNL_QueryChipInfo(VZNLHANDLE hDevice, EVzDevChipInfo eChipInfo, float* pfMin, float* pfMax, float* pfCur);

/**
 * @brief 启用无限制帧率
 * @param hDevice 当前设备句柄
 * @param [in] bEnable 是否启用
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_EnableFreeFrameRate(VZNLHANDLE hDevice, VzBool bEnable);


/**
 * @brief 启用传图模式
 * @param hDevice 当前设备句柄
 * @param [in] bEnable 是否启用
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_EnableTransPicMode(VZNLHANDLE hDevice, VzBool bEnable);

/**
 * @brief 设置是否输出ROI图像
 * @param [in] bOutputROI 是否输出ROI图像
 */
VZNLAPI void VzNL_SetOutputROIImage(VzBool bOutputROI);

/**
* @brief 使能双目/RGB Map本地缓存/加载功能，提升设备连接速度
* @param hDevice 当前设备句柄
* @param [in] bEnable 是否启用
* @return 成功返回0，否则为错误码。
*/
VZNLAPI int VzNL_EnableLocalMapDataLoad(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableLocalMapDataLoad(VZNLHANDLE hDevice, int* pnErrorCode);


/**
 * @brief 配置实时采图的模式
 * @param hDevice 当前设备句柄
 * @param eCaptureType 采图模式
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_SetCaptureMode(VZNLHANDLE hDevice, EVzCaptureMode eCaptureMode);
VZNLAPI EVzCaptureMode VzNL_GetCaptureMode(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 是否支持设置采图模式
 * @param hDevice 当前设备句柄
 * @param [out] pnErrorCode 当pnErrorCode不为nullptr时,输出错误码
 * @return 返回VzTrue,表示支持,返回VzFalse表示不支持
 */
VZNLAPI VzBool VzNL_IsSupportCaptureMode(VZNLHANDLE hDevice, int* pnErrorCode);


/**
* 焊缝跟踪参数操作建议
* 1、配置信息时：先将相机当前参数调试到满足要求，然后调用：VzNL_SetTrackingScanPara(hDevice, nullptr): 记录当前配置参数为焊缝跟踪参数；
* 2、获取信息时：VzNL_GetTrackingScanPara(hdevice, pPara) 只用于查看焊缝跟踪参数值信息(可配参数+固定参数)，并不会更改当前相机参数。
* 3、想要查看焊缝跟踪参数效果：直接调用VzNL_SetToTrackingScanMode(hDevice, pPara)配置相机参数后执行扫描即可：函数内主动获取焊缝跟踪并进行配置，同时项pPara 返回实际的配置信息；
* 4、VzNL_SetToTrackingScanMode() 后使用返回的pPara 进行软件端的参数更新显示。
*/
/**
* @brief 设置焊缝跟踪功能需要的相机配置参数信息（只保存到相机中，不做配置）
* @param [in] hDevice		设备句柄
* @param [in] pPara			输入焊缝跟踪功能配置参数：(pPara== nullptr)：函数内部自动获取当前相机配置参数作为焊缝跟踪配置参数进行保存；
* @param [in] nOutputFrameRate	输入输出帧率参数：(pPara== nullptr)：使用此参数作为输出帧率；(pPara !=nullptr)：使用pPara 中的参数作为输出帧率；
* @description: 参数限制：帧率大于等于1000，禁用图像校正，ROI尺寸固定为：64*256；输出帧率小于采集帧率；
* @return 成功返回0，否则为其他错误码
*/
VZNLAPI int VzNL_SetTrackingScanPara(VZNLHANDLE hDevice, SVzTrackingScanPara* pPara, unsigned int nOutputFrameRate);

/**
* @brief 获取焊缝跟踪功能的相机配置参数信息，只获取信息，不会进行配置
* @param [in] hDevice		设备句柄
* @param [out] pPara		返回焊缝跟踪功能需要的配置参数；
* @return 成功返回0，否则为其他错误码
*/
VZNLAPI int VzNL_GetTrackingScanPara(VZNLHANDLE hDevice, SVzTrackingScanPara* pPara);

/**
* @brief 配置当前相机为焊缝跟踪状态：配置焊缝跟踪参数 + 配置固定的工作模式(固定参数部分)
* @detail 主动读取SDK内存储的配置参数，并使用焊缝跟踪参数配置相机，同时设置相机固定的工作模式，返回当前实际的配置参数
* @param [in] hDevice	设备句柄
* @param [out] pPara	(pPara!= nullptr)：返回实际的焊缝跟踪配置参数，外部可用于更新本地参数及软件显示；(pPara== nullptr)：不返回参数；
* @return 成功返回0，否则为其他错误码
*/
VZNLAPI int VzNL_SetToTrackingScanMode(VZNLHANDLE hDevice, SVzTrackingScanPara* pPara);

/**
* @brief 恢复当前相机为焊缝跟踪前的初始状态：按备份的参数恢复相机状态
* @detail 主动读取SDK内存储的备份参数，并使用备份参数配置相机
* @param [in] hDevice	设备句柄
* @param [out] pPara	(pPara!= nullptr)：返回操作后实际的配置参数，外部可用于更新本地参数及软件显示；(pPara== nullptr)：不返回参数；
* @return 成功返回0，否则为其他错误码
*/
VZNLAPI int VzNL_RestoreToNonTrackingScanMode(VZNLHANDLE hDevice, SVzTrackingScanPara* pPara);

/**
* @brief 获取相机当前时间戳(单位 微妙)
* @param [in] hDevice		设备句柄
* @param [out] pnErrorCode 当pnErrorCode不为nullptr时,输出错误码
* @return 成功返回时间戳
*/
VZNLAPI unsigned long long VzNL_GetTimeStamp(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 结束SDK生命周期
 */
VZNLAPI void VzNL_Destroy();

#endif //__VIZUM_COMMON_HEADER__
