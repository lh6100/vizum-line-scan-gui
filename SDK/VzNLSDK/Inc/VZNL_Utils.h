#ifndef __VIZUM_UTILS_HEADER__
#define __VIZUM_UTILS_HEADER__

/*****************************************************************************
*  Vizum Detect SDK Utils                                                    *
*  Copyright (C) 2022 Vizum Technology.										 *
*                                                                            *
*  This file is part of VizumSDK.                                            *
*                                                                            *                                                                           *
*                                                                            *
*  @file     VZNL_Utils.h                                                    *
*  @brief    包含部分图像生成的方式                                             *
*  Details.                                                                  *
*                                                                            *
*  @author   Vizum                                                           *
*  @email                                                                    *
*  @date     2022/07/03			                                             *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2022/07/03 | 3.6.17.10 | Vizum          | 增加了2D图生成工具                *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/

#include "VZNL_Export.h"
#include "VZNL_Types.h"

typedef struct
{
	union {
		struct
		{
			unsigned int bNoFirewallToken : 1;		//< 是否有防火墙权限
			unsigned int bNetWork100M : 1;			//< 设备中含有 100M 网络
			unsigned int bSameMACDevice : 1;		//< 有相同MAC的设备
			unsigned int bVLANError : 1;			//< IP不在同网段
			unsigned int bIsAdmin : 1;				//< 是否管理员权限
			unsigned int bOpenJumboFrame : 1;		//< 是否开启了巨型帧
			unsigned int bIPConflict : 1;			//< 是否存在IP地址冲突
		};

		unsigned int nCheckMask;						//< 当为0时表示都符合要求
	};
	unsigned int nHostNetCardCnt;					//本机网卡数量
	SVzNetCardInfo* sHostNetCard;					//本机网卡列表
	unsigned int nSameMACDeviceCnt;					//有相同MAC的设备数量
	SVzNLEyeCBInfo* sSameMACDevice;					//相同MAC的设备IP列表
	unsigned int nVLANErrorDeviceCnt;				//IP不在同网段的设备数量
	SVzNLEyeCBInfo* sVLANErrorDevice;				//IP不在同网段的设备IP列表
	unsigned int nIPConflictDeviceCnt;				//IP地址冲突的设备数量
	SVzNLEyeCBInfo* sIPConflictDevice;				//IP地址冲突的设备IP列表
} SVzCheckPlatformInfo;

/**
 * @brief 检测运行环境
 * @param psPlatformInfo 环境信息
 * @return 返回VzTrue说明一切正常，返回VzFalse表示有信息不匹配的地方
 */
VZNLAPI int VzNL_CheckPlatform(SVzCheckPlatformInfo* psPlatformInfo);

/// @name 静态相机图像生成工具集
/// @{
/**
 * @brief 使用固定点云数据生成深度图
 * @param eDataType         数据类型
 * @param p3DPoint          3D点云数组
 * @param nPointCount       点的个数
 * @param sDepthMapParam    深度图参数
 * @param ppImageData 图像
 * @return 返回0表示正常
 */
VZNLAPI int VzNL_CreateDepthMapImage(EVzResultDataType eDataType, void* p3DPoint, unsigned int nPointCount, SVzDepthMapImgCtlPara* sDepthMapParam, SVzNLImageData** ppImageData);

/**
 * @brief 使用固定点云生成灰度图
 * @param eDataType         数据类型
 * @param p3DPoint          3D点云数组
 * @param nPointCount       点的个数
 * @param fXYScale			XY比例	
 * @param ppImageData		图像
 * @return 返回0表示正常
 */
VZNLAPI int VzNL_CreateGrayImage(EVzResultDataType eDataType, void* p3DPoint, unsigned int nPointCount, float fXYScale, SVzNLImageData** ppImageData);
/// @}

/**
 * @brief 读取图像
 * @param szFileName [in] 要读取的文件全路径
 * @param ppImageData [out] 无压缩图像实例
 * @return 返回0表明正确
 */
VZNLAPI int VzNL_LoadImageFromFile(const char* szFileName, SVzNLImageData** ppImageData);

/**
 * @brief 从tif文件中加载3D点云
 * @param lpszFile			文件全路径
 * @param nImageWidth       图像宽
 * @param nImageHeight      图像高
 * @param eDataType			数据类型
 * @param p3DPoint			3D点指针
 * @return 返回0表示正常
 */
VZNLAPI int VzNL_LoadDepthMapTiffImage(const char* lpszFile, unsigned int* nImageWidth, unsigned int* nImageHeight, SVzNL3DPointF* p3DPoint, unsigned int* nPointCount);

/**
 * @brief 将3D点云数据存储为Tif深度图
 * @param lpszFile			文件全路径
 * @param nImageWidth       图像宽
 * @param nImageHeight      图像高
 * @param eDataType			数据类型
 * @param p3DPoint			3D点指针
 * @return 返回0表示正常
 */
VZNLAPI int VzNL_SaveDepthMapTiffImage(const char* lpszFile, unsigned int nImageWidth, unsigned int nImageHeight, EVzResultDataType eDataType, void* p3DPoint);

/// @name 点云后处理工具
/// @{

/** 
 * @brief 深度图图像回调
 * @param pImageData    传出处理后的图像  [输出参数]
 * @param pParam        回调传递的数据    [输出参数]
 */
typedef void(*VzNL_OnOutputCloudPointAPData)(const SVzNLImageData* psImageData, const SVz3DPointMapTable* psPointMapTable, void* pParam);

/** 
 * @brief 创建点云后处理工具Handle
 * @return 返回深度图的句柄
 */
VZNLAPI VZNLPOINTCLOUDHANDLE VzNL_CreateCloudPointAPTool();

/**
 * @brief 当配置自适应参数时,选择绑定设备Handle
 */
VZNLAPI int VzNL_AttachHandleForCloudPointAP(VZNLPOINTCLOUDHANDLE hCloudPointTool, VZNLHANDLE hDevice);

/**
 * @brief 当配置自适应参数时,可以选择绑定检测工具Handle
 */
VZNLAPI int VzNL_AttachDetectToolForCloudPointAP(VZNLPOINTCLOUDHANDLE hCloudPointTool, VZNLDETECTHANDLE hDetectHandle);

/**
 * @brief 当配置自适应参数时,绑定设备Handle
 */
VZNLAPI int VzNL_DetachHandleForCloudPointAP(VZNLPOINTCLOUDHANDLE hCloudPointTool);

/**
 * @brief 设置3D点云基准激光线(当生成灰度/深度图时需要配置), 配置后内部的图像参数会进行改变,若查看参数信息请调用 VzNL_GetCloudPointAPParam
 * @param hCloudPointTool 点云工具句柄
 * @param p3DPosition		3D数据
 * @param nPointCount		3D点的个数
 */
VZNLAPI int VzNL_SetCloudPointAPBaseLine(VZNLPOINTCLOUDHANDLE hCloudPointTool, SVzNL3DPosition* p3DPosition, unsigned int nPointCount);

/**
 * @brief 设置深度图/灰度图配置参数(当生成灰度/深度图时需要配置)
 * @param hCloudPointTool 点云工具句柄
 * @param psDepthMapParam	深度图配置参数
 */
VZNLAPI int VzNL_SetCloudPointAPParam(VZNLPOINTCLOUDHANDLE hCloudPointTool, SVzDepthMapImgCtlPara* psDepthMapParam);

/**
 * @brief 启用基准线深度值计算模式(当生成灰度/深度图时需要配置)
 * @param hCloudPointTool 点云工具句柄
 * @param bEnable			VzTrue 启用 / VzFalse 禁用
 * @note  基准线模式:bEnable = VzTrue 当深度计算需要用当前一条线作为高度基准时,配置为此模式,例如激光线照射U型皮带，会形成高低不等的一条激光线，这个时候
 *					U型皮带上的各个点均为自身位置的0点位置                     
 */
VZNLAPI int VzNL_EnableCloudPointAPBaseLineMode(VZNLPOINTCLOUDHANDLE hCloudPointTool, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableCloudPointAPBaseLineMode(VZNLPOINTCLOUDHANDLE hCloudPointTool);

/**
 * @brief 设置输出图像的最小宽度(当生成灰度 / 深度图时需要配置)
 * @paramhCloudPointTool 点云工具句柄
 * @paramnMinImageWidth  输出图像的最小宽度
 */
VZNLAPI int VzNL_SetCloudPointAPImageWidth(VZNLPOINTCLOUDHANDLE hCloudPointTool, unsigned int nMinImageWidth);

/**
 * @brief 获取点云配置
 * @paramhCloudPointTool 点云工具句柄
 * @psDepthMapParam [out] 点云配置  
 */
VZNLAPI int VzNL_GetCloudPointAPParam(VZNLPOINTCLOUDHANDLE hCloudPointTool, SVzDepthMapImgCtlPara* psDepthMapParam);

/**
 * @brief 获取输出图像的最小宽度(当生成灰度 / 深度图时需要配置)
 * @paramhCloudPointTool 点云工具句柄
 * @pnImageHeight [out] 输出图像高度
 */
VZNLAPI int VzNL_GetCloudPointAPImageHeight(VZNLPOINTCLOUDHANDLE hCloudPointTool, int* pnImageHeight);

/**
 * @brief 开始点云处理(目前仅支持 keResultDataType_Position)
 * @param hCloudPointTool 点云工具句柄
 * @param eAfterProcType 图像后处理类型 kePointAfterProcType_DepthMap(深度图) kePointAfterProcType_GrayPic(灰度图)
 * @param eDataType		激光线数据 SVzLaserLineData 的数据类型
 * @param eDirect			激光线扫描方向
 * @param pDepthMap		深度图的回调
 * @param pParam			回调函数的参数
 * @return 返回0则为正确，返回非0为错误，可通过VzNL_GetErrorInfo 获取错误信息
 */
VZNLAPI int VzNL_BeginCloudPointAfterProc(VZNLPOINTCLOUDHANDLE hCloudPointTool, EVzPointAfterProcType eAfterProcType, EVzResultDataType eDataType, EVzObjRunDirect eDirect, VzNL_OnOutputCloudPointAPData pDepthMap, void*  pParam);

/**
 * @brief 点云处理(目前仅支持 keResultDataType_Position)
 * @param hCloudPointTool 点云工具句柄
 * @param psLineData		激光线数据 
 */
VZNLAPI int VzNL_CloudPointAfterProc(VZNLPOINTCLOUDHANDLE hCloudPointTool, SVzLaserLineData* psLineData);

/**
 * @brief 结束点云处理
 * @param hCloudPointTool 点云工具句柄
 */
VZNLAPI int VzNL_EndCloudPointAfterProc(VZNLPOINTCLOUDHANDLE hCloudPointTool);

/** 
 * @brief 销毁点云处理工具
 * @param hCloudPointTool 点云工具句柄
 */
VZNLAPI int VzNL_DestroyCloudPointAPTool(VZNLPOINTCLOUDHANDLE hCloudPointTool);
/// @}

/// @name 图像生成工具
/// @{

/**
 * @brief 创建图像生成工具
 * @param hDevice 设备句柄
 */
VZNLAPI VZNLIMAGEGENERATOR VzNL_CreateImageGenerator(VZNLHANDLE hDevice);

/**
 * @brief 开始送入点云数据
 * @param hImageGenerator   图像生成工具
 * @param eDataType         送入数据的类型,目前仅支持 keResultDataType_PointXYZRGBA 与 keResultDataType_PointGray
 *                          keResultDataType_PointGray 输出图像为 8  bit 灰度图
 *                          keResultDataType_PointGray 输出图像为 24 bit 彩色图
 */
VZNLAPI int VzNL_BeginPushPointToImageGenerator(VZNLIMAGEGENERATOR hImageGenerator, EVzResultDataType eDataType);

/**
 * @brief 送入点云数据进行处理
 * @param hImageGenerator  图像生成工具
 * @param psLineData		激光线数据
 */
VZNLAPI int VzNL_PushPointToImageGenerator(VZNLIMAGEGENERATOR hImageGenerator, SVzLaserLineData* psLineData);

/**
 * @brief 结束点云传送
 * @param hImageGenerator 图像生成工具
 */
VZNLAPI int VzNL_EndPushPointToImageGenerator(VZNLIMAGEGENERATOR hImageGenerator);

/**
 * @brief 通过图像生成工具，生成图像
 * @param hImageGenerator   图像生成工具
 * @param psLineData		图像数据
 */
VZNLAPI int VzNL_GetImageFromImageGenerator(VZNLIMAGEGENERATOR hImageGenerator, SVzNLImageData** ppImageData);

/**
 * @brief 通过图像生成工具，生成图像
 * @param hImageGenerator   图像生成工具
 * @param lpszFile			文件名 tif
 */
VZNLAPI int VzNL_SaveDepthMapImageFromImageGenerator(VZNLIMAGEGENERATOR hImageGenerator, const char* lpszFile);

/**
 * @brief 通过2D位置找3D点
 * @param hImageGenerator  图像生成工具
 * @param psLineData		3D数据
 */
VZNLAPI int VzNL_Find3DPointFrom2DPosForImageGenerator(VZNLIMAGEGENERATOR hImageGenerator, int nX, int nY, SVzNLPointXYZ* p3DPoint);

/**
 * @brief 销毁图像创建工具
 * @param hImageGenerator  图像生成工具
 */
VZNLAPI int VzNL_DestroyImageGenerator(VZNLIMAGEGENERATOR hImageGenerator);


/*
* 功能：将输入的点云数据转换为指定分辨率的 2D 伪彩图像输出
* 参数说明：
*   EVzResultDataType eDataType: 输入点数据类型，内部根据数据类型解析数据；
*   unsigned int nPointCnt: 输入 pPointList 中点数据个数；
*   void* pPointList: 输入点数据列表。
*   unsigned int nImageWidth: 输入希望输出的图像宽度；
*   unsigned int nImageHeight: 输入希望输出的图像高度；
*   unsigned int nColorGradient: 输出点数据颜色梯度
*   unsigned int nEdgeMargin: 输入图像边缘的留白宽度(像素值，四边留白);
*   SVzNLImageData* pImage: 输出的图像数据；
*   SVzPColorImageConvertInfo* pConvertInfo: 转换信息，用于从图像回算到点云；
*/
VZNLAPI int VzNL_PointCloudTo2DFixedSizeImage(EVzResultDataType eDataType,
	unsigned int nPointCnt,
	void* pPointList,
	unsigned int nImageWidth,
	unsigned int nImageHeight,
	unsigned int nColorGradient,
	unsigned int nEdgeMargin,
	SVzNLImageData* pImage,
	SVzPColorImageConvertInfo* pConvertInfo);

/*
* 功能：将输入的点云数据转换为指定像素精度的 2D 伪彩图像输出
* 参数说明：
*   EVzResultDataType eDataType: 输入点数据类型，内部根据数据类型解析数据；
*   unsigned int nPointCnt: 输入 pPointList 中点数据个数；
*   void* pPointList: 输入点数据列表。
*   float fWHPixelResolution: 输入图像像素精度，代表每个像素对应的实际物理距离，例如：0.1 表示像素间隔为0.1毫米；
*   unsigned int nColorGradient: 输出点数据颜色梯度
*   unsigned int nEdgeMargin: 输入图像边缘的留白宽度(像素值，四边留白);
*   SVzNLImageData* pImage: 输出的图像数据；
*   SVzPColorImageConvertInfo* pConvertInfo: 转换信息，用于从图像回算到点云；
*/
VZNLAPI int VzNL_PointCloudTo2DFixedResolutionImage(EVzResultDataType eDataType,
	unsigned int nPointCnt,
	void* pPointList,
	float fWHPixelResolution,
	unsigned int nColorGradient,
	unsigned int nEdgeMargin,
	SVzNLImageData* pImage,
	SVzPColorImageConvertInfo* pConvertInfo);


/*
* 释放输出的图像数据
*/
VZNLAPI int VzNL_PointCloudTo2DImageRelease(SVzNLImageData* pImage);

/*
* 功能：基于伪彩图像上的点坐标回算3D点云点坐标(输出点云中距离最近的点坐标)
* 参数说明：
*   SVzNLImageData* pImage: 输入由点云生成的伪彩图像；
*   EVzResultDataType eDataType: 输入点数据类型，内部根据数据类型解析数据；
*   unsigned int nPointCntIn: 输入 pPointList 中点数据个数；
*   void* pPointListIn: 输入点数据列表。
*   SVzPColorImageConvertInfo* pConvertInfo: 转换信息，生成伪彩图时的信息
*   SVzPColorImageTo3DPointMethod eConvertMethod: 回算点输出方法
*   unsigned int n2DPointCntIn: 输入伪彩图像上的2D点坐标个数
*   SVzNL2DPoint* p2DPointListIn: 输入伪彩图上的2D 点坐标列表
*   double dSearchDis: 从点云中搜索目标点的搜索范围
*	SVzNL3DPoint* p3DPointList: 输出2D坐标列表回算到3D 点的坐标列表，外部分配好空间
*/
VZNLAPI int VzNL_Get3DPointsFrom2DPseudoColorImagePoints(SVzNLImageData* pImage,
											EVzResultDataType eDataType,
											unsigned int nPointCntIn,
											void* pPointListIn,
											SVzPColorImageConvertInfo* pConvertInfo,
											SVzPColorImageTo3DPointMethod eConvertMethod,
											unsigned int n2DPointCntIn,
											SVzNL2DPoint* p2DPointListIn,
											double dSearchDis,
											SVzNL3DPoint* p3DPointList);

/*
* 功能：基于伪彩图像上的ROI区域获取对应区域的点云数据（点数据类型与输入一致）
* 参数说明：
*   SVzNLImageData* pImage: 输入由点云生成的伪彩图像；
*   EVzResultDataType eDataType: 输入点数据类型，内部根据数据类型解析数据；
*   unsigned int nPointCntIn: 输入 pPointList 中点数据个数；
*   void* pPointListIn: 输入点数据列表。
*   SVzPColorImageConvertInfo* pConvertInfo: 转换信息，生成伪彩图时的信息
*   SVzNL2DRect s2DRect: 输入伪彩图上的目标区域；
*   unsigned int* pPointCntOut: 输出目标区域内的3D 点云数据点个数；
*	void** ppPointListOut: 输出目标区域内的3D 点云数据列表，点数据类型与输入一致
*/
VZNLAPI int VzNL_Get3DPointListFrom2DPseudoColorImageRect(SVzNLImageData* pImage,
											EVzResultDataType eDataType,
											unsigned int nPointCntIn,
											void* pPointListIn,
											SVzPColorImageConvertInfo* pConvertInfo,
											SVzNL2DRect s2DRect,
											unsigned int* pPointCntOut,
											void** ppPointListOut);

/*
* 释放从2D 伪彩图回算得到的3D 点坐标列表，适用于如下函数生成的点列表：
*   VzNL_Get3DPointListFrom2DPseudoColorImageRect()
*/
VZNLAPI int VzNL_2DPseudoColorImage3DPointListRelease(unsigned int* pPointCnt, void** ppPointList);


/**
 * @brief 创建RGB点云处理工具
 * @param hDevice   设备句柄
 */
VZNLAPI VZNLRGBCLOUDPOINTTOOL VzNL_CreateRGBCloudPointTool(VZNLHANDLE hDevice);


/**
 * @brief 开始送入点云数据
 * @param hRGBCloudPointTool	RGB点云处理工具
 * @param eDataType				送入数据的类型,目前仅支持 keResultDataType_PointXYZRGBA 与keResultDataType_PointRGBA_D
 */
VZNLAPI int VzNL_BeginPushPointToRGBCloudPointTool(VZNLRGBCLOUDPOINTTOOL hRGBCloudPointTool, EVzResultDataType eDataType);

/**
 * @brief 送入点云数据进行处理
 * @param hRGBCloudPointTool	RGB点云处理工具
 * @param psLineData			激光线数据
 */
VZNLAPI int VzNL_PushPointToRGBCloudPointTool(VZNLRGBCLOUDPOINTTOOL hRGBCloudPointTool, SVzLaserLineData* psLineData);

/**
 * @brief 通过2D位置找3D点
 * @param hRGBCloudPointTool	RGB点云工处理工具
 * @param s2DPoint				2D数据
 * @param p3DPoint				3D数据
 */
VZNLAPI int VzNL_Find3DPointFrom2DPosForRGBCloudPointTool(VZNLRGBCLOUDPOINTTOOL hRGBCloudPointTool, const SVzNL2DPoint s2DPoint, SVzNL3DPoint* p3DPoint);


/**
 * @brief 通过3D点找2D位置
 * @param hRGBCloudPointTool	RGB点云工处理工具
 * @param s3DPoint				3D数据
 * @param p2DPoint				2D数据
 */
VZNLAPI int VzNL_Find2DPosFrom3DPointForRGBCloudPointTool(VZNLRGBCLOUDPOINTTOOL hRGBCloudPointTool, const SVzNL3DPoint s3DPoint, SVzNL2DPoint* p2DPoint);

/**
 * @brief 结束RGB点云处理
 * @param hRGBCloudPointTool RGB点云工处理工具
 */
VZNLAPI int VzNL_EndPushPointToRGBCloudPointTool(VZNLRGBCLOUDPOINTTOOL hRGBCloudPointTool);

/**
 * @brief 销毁RGB点云工处理工具
 * @param hRGBCloudPointTool  RGB点云工处理工具
 */
VZNLAPI int VzNL_DestroyRGBCloudPointTool(VZNLRGBCLOUDPOINTTOOL hRGBCloudPointTool);

/**
* @brief 通过双目校正图2D 坐标计算3D 点坐标，根据当前设备内部参数计算
* @param hDevice		输入设备句柄
* @param nPointCnt		输入点个数
* @param p2DPointList   输入双目校正图2D 点坐标列表
* @param p3DPointList   输出计算得到的3D 点列表
*/
VZNLAPI int VzNL_Cal3DPointsFromStereo2DPoints(VZNLHANDLE hDevice, unsigned int nPointCnt, SVzNL2DLRPoint* p2DPointList, SVzNL3DPoint* p3DPointList);

/**
* @brief 输入2D RGB图像数据 + 扫描点云数据(静态RGBD扫描)，获取映射后的2D RGB图像数据和同分辨率的点云序列
* @param hDevice		输入设备句柄
* @param pRGBImage		输入与点云匹配的RGB 图像
* @param eDataType		输入点云数据类型，同样也为输出的点云数据类型
* @param nLineCnt		输入激光线数据条数
* @param pLineDataList  输入激光线数据列表
* @param bEnableClip    输入是否进行有效图像裁剪，true：输出点云实际对应的有效图像区域及映射点云；false：输出原始图像及映射点云；
* @param ppOutImage     输出处理后的图像(参考 bEnableClip 标志决定是否裁剪图像)
* @param pPointCnt      输出点云数据中的点个数
* @param ppMappedPointList  输出映射后与ppOutImage 相同分辨率的点云数据，点数据类型为eDataType
*/
VZNLAPI int VzNL_GetMappedRGBImageAndPointCloud(VZNLHANDLE hDevice, 
												SVzNLImageData* pRGBImage, 
												EVzResultDataType eDataType,
												unsigned int nLineCnt, 
												SVzLaserLineData* pLineDataList,
												bool bEnableClip,
	                                            SVzNLImageData** ppOutImage,
											    unsigned int* pPointCnt,
	                                            void** ppMappedPointList);

/**
* @brief 输入左目或右目图像数据 + 扫描点云数据(静态非RGBD扫描)，获取映射后的2D 左目或右目图像数据和同分辨率的点云序列
* @param hDevice		输入设备句柄
* @param bIsRightImage	输入当前图像是否为右目图像
* @param pImage			输入与点云匹配的左目或右目图像数据
* @param eDataType		输入点云数据类型，同样也为输出的点云数据类型
* @param nLineCnt		输入激光线数据条数
* @param pLineDataList  输入激光线数据列表
* @param bEnableClip    输入是否进行有效图像裁剪，true：输出点云实际对应的有效图像区域及映射点云；false：输出原始图像及映射点云;
* @param ppOutImage     输出处理后的图像(参考 bEnableClip 标志决定是否裁剪图像)
* @param pPointCnt      输出点云数据中的点个数
* @param ppMappedPointList  输出映射后与ppOutImage 相同分辨率的点云数据，点数据类型为eDataType
*/
VZNLAPI int VzNL_GetMappedLRImageAndPointCloud(VZNLHANDLE hDevice,
												bool bIsRightImage,
												SVzNLImageData* pImage,
												EVzResultDataType eDataType,
												unsigned int nLineCnt,
												SVzLaserLineData* pLineDataList,
												bool bEnableClip,
												SVzNLImageData** ppOutImage,
												unsigned int* pPointCnt,
												void** ppMappedPointList);

/**
* @brief 释放VzNL_GetMappedRGBImageAndPointCloud()/VzNL_GetMappedLRImageAndPointCloud() 函数返回的图像和3D 点数据
*/
VZNLAPI int VzNL_ReleaseImageAndPointCloudMappedResult(SVzNLImageData** ppImage, void** ppPointList);


/**
* @brief USB 串口继电器设备搜索
* @param pDeviceNum				返回搜索到的设备个数
* @param ppDeviceInfoList		返回搜索到的设备名称列表		
*/
VZNLAPI int VzNL_SearchUSBSerialRelay(unsigned int* pDeviceNum, SVzSerialPortInfo** ppDeviceInfoList);

/**
* @brief 获取指定USB 串口继电器设备的配置信息
* @param pDeviceName		输入设备名称(搜索函数返回)
* @param pCfgPara			返回设备配置信息
*/
VZNLAPI int VzNL_GetUSBSerialRelayCfgPara(char* pDeviceName, SVzUSBSerialRelayCfgPara* pCfgPara);

/**
* @brief 查看指定USB 串口继电器设备是否打开
* @param pDeviceName		输入设备名称(搜索函数返回)
*/
VZNLAPI bool VzNL_IsUSBSerialRelayOpen(char* pDeviceName);

/**
* @brief 打开指定端口的USB 串口继电器设备，按配置参数打开指定设备
* @param pDeviceName		输入设备名称(搜索函数返回)
* @param sCfgPara			输入设备配置信息
*/
VZNLAPI int VzNL_OpenUSBSerialRelay(char* pDeviceName, SVzUSBSerialRelayCfgPara sCfgPara);

/**
* @brief 关闭指定端口的USB 串口继电器设备
* @param pDeviceName		输入设备名称(搜索函数返回)
*/
VZNLAPI int VzNL_CloseUSBSerialRelay(char* pDeviceName);

/**
* @brief 获取所有打开的USB 串口继电器设备端口列表
* @param pDeviceName		输入设备名称(搜索函数返回)
* @param ppDeviceInfoList	返回已打开设备的名称列表
*/
VZNLAPI int VzNL_GetOpenedUSBSerialRelayList(unsigned int* pDeviceNum, SVzSerialPortInfo** ppDeviceInfoList);

/**
* @brief 获取指定USB继电器通道当前开关状态
* @param pDeviceName		输入设备名称(搜索函数返回)
* @param nChannelIdx		输入继电器通道索引
* @param pState				返回该通道当前开关状态
*/
VZNLAPI int VzNL_GetUSBSerialRelayChannelOnOffState(char* pDeviceName, unsigned int nChannelIdx, bool* pState);

/**
* @brief 打开/关闭指定继电器通道，pState 返回操作后该通道状态
* @param pDeviceName		输入设备名称(搜索函数返回)
* @param bOn				输入需要设置的开关状态
* @param nChannelIdx		输入继电器通道索引
* @param pState				不为null时返回该通道当前开关状态
*/
VZNLAPI int VzNL_SetUSBSerialRelayChannelOnOff(char* pDeviceName, bool bOn, unsigned int nChannelIdx, bool* pState);

/**
* @brief 获取所有继电器通道开关状态：unsigned int* pState: 低位开始，每位代表一个通道；
* @param pDeviceName		输入设备名称(搜索函数返回)
* @param pState				返回该通道当前开关状态
*/
VZNLAPI int VzNL_GetUSBSerialRelayAllChannelOnOffState(char* pDeviceName, unsigned int* pState);

/**
* @brief 打开/关闭所有继电器通道，pState 返回操作后所有继电器通道状态(低位开始，每位代表一个通道)
* @param pDeviceName		输入设备名称(搜索函数返回)
* @param bOn				输入需要设置的开关状态
* @param pState				不为null时返回所有通道当前开关状态
*/
VZNLAPI int VzNL_SetUSBSerialRelayAllChannelOnOff(char* pDeviceName, bool bOn, unsigned int* pState);


#endif