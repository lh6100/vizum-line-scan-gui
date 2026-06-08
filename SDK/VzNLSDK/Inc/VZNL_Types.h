/*
 * Header: vizum_type.h
 * Description:当前文件为双目相机(Vizum)研发的EyeCB板提供所有类型的定义。
 * Author: Mjw
 * Date:   2018/08/28
 * Update: 2023/04/27
 */

#ifndef __VIZUM_TYPES_HEADER__
#define __VIZUM_TYPES_HEADER__

/// @name 网络相关定义
/// @{
/// @brief
/// 网络IP长度
#define VZNL_SDK_NETWORK_IPv4_LENGTH 16

/// @brief
/// 网络MAC长度
#define VZNL_SDK_NETWORK_MAC_LENGTH 18
/// @}

/// @breif
/// 版本字符串长度
#define VZNL_VERSION_LENGTH 64

/// @brief
/// 设备名称长度。
#define VZNL_DEVICE_NAME_LENGTH 64

/// @brief
/// 唯一标识符大小
#define VZNL_GUID_LENGTH 64

/// @brief
/// 设备名称长度
#define VZ_DEVICE_NAME 64

/// @brief
/// 设备描述长度
#define VZ_DEVICE_DESC 256

/// @brief
/// 设备ID长度
#define VZNL_DEVICEID_LENGTH 64

/// @brief
/// 设备地址
#define VZNL_DEVICE_ADDR_LENGTH 64

/// @brief
/// File Path Length
#define VZ_MAXPATH 256

/// @brief UserProfile MAX Count
#define VZ_USERPROFILE_MAX_COUNT 10

#define VZ_USERCONFIG_MAX_COUNT 200

/// @brief UserProfile Name Length
#define VZ_USERPROFILE_NAME_LENGTH 64

#define VZ_USERCONFIG_NAME_LENGTH 64

#define VZ_USERCONFIG_DESC_LENGTH 256


/// @brief
/// 设备句柄
typedef void* VZNLHANDLE;

/// @brief
/// 设备句柄
typedef void* VZNLFILE;

/// @brief
/// 检测工具句柄 SDK 2.2.0开始支持
typedef void* VZNLDETECTHANDLE;

/// @brief
/// 数据处理句柄
typedef void* VZNLPOINTCLOUDHANDLE;

/// @brief
/// 数据处理句柄
typedef void* VZNLIMAGEGENERATOR;

/// @brief
/// RGB点云处理工具句柄
typedef void* VZNLRGBCLOUDPOINTTOOL;

enum
{
	VzFalse = 0,
	VzTrue = 1,
};
typedef unsigned char VzBool;

/// @brief
/// 颜色定义
typedef unsigned int VzColorRGB;	//RGB

/// @brief
/// Data Type
#ifdef _WIN64
	typedef unsigned long long VzParam;
#else
	typedef unsigned long VzParam;
#endif

typedef struct
{
	unsigned short	shYear;
	unsigned short	shMonth;
	unsigned short	shDay;
	unsigned short	shHour;
	unsigned short	shMinute;
	unsigned short	shSecond;
	unsigned short	shWeek;
} SVzDateTime;

// 设备搜索标志
enum
{
	keSearchDeviceFlagNone				= 0x0000,		//< 不搜索
	keSearchDeviceFlag_EyeCB			= 0x0001,		//< 搜索控制器
	keSearchDeviceFlag_USBLargeEye		= 0x0002,		//< 搜索USB极光眼
	keSearchDeviceFlag_EthLargeEye		= 0x0004,		//< 搜索网络星光眼:激光
	keSearchDeviceFlag_EthSmallEye		= 0x0008,		//< 搜索网络星光眼:智能
	keSearchDeviceFlag_EthLaserRobotEye	= 0x0010,		//< 搜索网络智光眼:激光
	keSearchDeviceFlagAll				= keSearchDeviceFlag_EthLaserRobotEye,		//< 搜索全部支持的设备
};
typedef unsigned int EVzSearchDeviceFlag;

/// @brief
/// 眼睛分辨率枚举值
typedef enum
{
    keVzNLEyeResolution_Unknown = 0,
    keVzNLEyeResolution_130WH,           ///< 130W像素的眼睛 1280 * 960
	keVzNLEyeResolution_130WV,           ///< 130W像素的眼睛 960 * 1280
    keVzNLEyeResolution_300WH,           ///< 300W像素的眼睛 2048 * 1520
	keVzNLEyeResolution_300WV,			 ///< 300W像素的眼睛 1536 * 2048
    keVzNLEyeResolution_1300W,           ///< 1300W像素的眼睛
	keVzNLEyeResolutionLast
} EVzNLEyeResolution;

/// @brief
/// 设备类型
typedef enum
{
	keDeviceType_Unknown = 0,
	keDeviceType_EyeCB,             // EyeCB控制器
	keDeviceType_LaserEye,			// 星光眼:激光
	keDeviceType_SmartEye,			// 星光眼:智能
	keDeviceType_LaserRobotEye,		// 智光眼:激光
	keDeviceType_SmartRobotEye,		// 智光眼:智能
} EVzDeviceType;

/// @brief
/// 眼睛类型标志
enum 
{
	keEyeTypeFlag_Unknown = 0x00,
	keEyeTypeFlag_USB = 0x01,		//< USB
	keEyeTypeFlag_Eth = 0x02,		//< 网络

	keEyeTypeFlag_Color = 0x10,		//< 彩色
	keEyeTypeFlag_Gray = 0x20,		//< 黑白

	keEyeTypeFlag_Laser = 0x100,		//< 极光眼
	keEyeTypeFlag_Smart = 0x200,		//< 智能眼

	keEyeTypeFlag_Product1 = 0x1000,		//< 小眼睛
	keEyeTypeFlag_Product2 = 0x2000,		//< 大眼睛
	keEyeTypeFlag_Product3 = 0x3000,		//< 大眼睛
};
typedef unsigned int EVzEyeTypeFlag;

enum
{
	keEyeSensorType_Left	= 0x01,
	keEyeSensorType_Right	= 0x02,
};
typedef unsigned int EVzEyeSensorType;

/// @bried
/// 眼睛出图方式
typedef enum 
{
	keEyeImgMode_Low	= 1,
	keEyeImgMode_Middle,
	keEyeImgMode_High,
	keEyeImgMode_Higher,
	keEyeImgMode_Hights,
	keEyeImgMode_Algo
}EVzEyeImgBitMode;

/// @brief
/// 运动方向
typedef enum
{
	keObjRunDirect_Plus = 0,		//< 正方向
	keObjRunDirect_Minus = 1,		//< 反方向
} EVzObjRunDirect;

/// @brief
/// 电动机运行方向
typedef enum
{
	keEncodeMotorRunDirectNone = 0,				// 未知
	keEncodeMotorRunDirect_ClockWise,			// 顺时针
	keEncodeMotorRunDirect_CounterClockWise,	// 逆时针
} EVzEncodeMotorRunDirect;

/// @brief
/// 外部触发模式
enum 
{
	keEyeTriggerMode_Invalid = -1,		// 无效值
	keEyeTriggerMode_Slave = 0,		// 被动触发
	keEyeTriggerMode_Master = 1,		// 主动模式
	keEyeTriggerMode_FallingEdge = 2,		// 下降沿触发
	keEyeTriggerMode_RisingEdge = 3,		// 上升沿触发
	keEyeTriggerMode_LowLevelEnable = 4,		// 低电平使能数据输出
	keEyeTriggerMode_ManualTrigger = 6,		// 手动模式
	keEyeTriggerMode_Package = 9,		// 物流包裹触发模式
	keEyeTriggerMode_HighFrequenceFallingEdge			= 10,		// 高帧率下降沿触发
};
typedef unsigned int EVzEyeTriggerMode;

/// @brief
/// 设置点云计算模式
typedef enum 
{
	kePointCloudProcMode_Invalid = 0,	//< 无效
	kePointCloudProcMode_Speed,			//< 速度偏移模式
	kePointCloudProcMode_Encoder,		//< 编码器模式
	kePointCloudProcMode_FixedStep,		//< 固定间隔模式
} EVzPointCloudProcMode;

/// @brief
/// 设备状态
typedef enum 
{
	keDeviceWorkStatusUnknown = 0,				//< 未知
	keDeviceWorkStatus_Eye_Comming = 1,			//< 眼睛连接
	keDeviceWorkStatus_Eye_Reconnect,			//< 眼睛重连
	keDeviceWorkStatus_Working,					//< 工作中
	keDeviceWorkStatus_Offline,					//< 离线
	keDeviceWorkStatus_Eye_TimeOut,				//< 眼睛超时
	keDeviceWorkStatus_Eye_UnStability,			//< 眼睛不稳定
	keDeviceWorkStatus_EyeCB_Mem,				//< 内存
	keDeviceWorkStatus_EyeCB_TimeStampDiffErr,	//< 时间戳错误
	keDeviceWorkStatus_Device_Swing_Finish,		//< 摆动模块已完成
	keDeviceWorkStatus_Device_Auto_Stop,		//< 设备自动停止
	keDeviceWorkStatus_EyeCB_TaskFull,			//< 任务饱和
	keDeviceWorkStatus_EyeCB_GetImageErr,		//< 获取图像错误

	keDeviceWorkStatus_EyeCB_ObjectCome,		//< 物体来了
	keDeviceWorkStatus_EyeCB_ObjectLeave,		//< 物体走了

	keDeviceWorkStatus_Trigger_Coming,			//< Trigger信号来了
	keDeviceWorkStatus_Trigger_Leave,			//< Trigger信号结束了

	keDeviceWorkStatus_Eye_StreamResume,		//< 数据恢复
	keDeviceWorkStatus_Eye_SwingNoResponse,		//< 静态模块无响应

	keDeviceWorkStatus_PetaMsg,					//< Peta 消息
	keDeviceWorkStatus_PSMsg,					//< PS	 消息
	keDeviceWorkStatus_Eye_LaserlineErr,		//< 
	keDeviceWorkStatus_Device_Swing_Running,	//< 开流后摆动模块开始运动时给出信号

	keDeviceWorkStatus_Eye_SwingEncodeNoChange,	//< 摆动编码器值不改变
	keDeviceWorkStatus_Eye_SwingUpdateStatus,	//< 发现了摆动模块

	keDeviceWorkStatus_Eye_New_Client_Connected,	//< 有新客户端连接

	keDeviceWorkStatus_Eye_LinearActuator_Warning,	//< 推杆电机关闭点可能存在异常
} EVzDeviceWorkStatus;

/// @brief
/// 设备状态
typedef enum
{
	keMulDevWorkStatusUnknown = 0,				//< 未知
	keMulDevWorkStatus_Auto_Stop,				//< 自动停止
} EVzMulDevWorkStatus;

/// @brief
/// Laser FrameRate
typedef enum
{
	keLaserFrameRate_30 = 0,
	keLaserFrameRate_100,
	keLaserFrameRate_220,
	keLaserFrameRate_400,
	keLaserFrameRate_630,
	keLaserFrameRate_50,
	keLaserFrameRate_120,
	keLaserFrameRate_320,
	keLaserFrameRate_550,
	keLaserFrameRate_830,
	keLaserFrameRate_80,
	keLaserFrameRate_210,
	keLaserFrameRate_530,
	keLaserFrameRate_840,
	keLaserFrameRate_1200,
	keLaserFrameRateLast
} EVzLaserFrameRate;

/// @brief
/// Laser FrameRate
typedef enum
{
	keLaserCaptureMode_LastFrame,
	keLaserCaptureMode_AllFrame,
	keLaserCaptureMode_CalcAllFrameWhenStop,
} EVzLaserCaptureMode;

/// @brief
/// 产品类型
enum
{
	keProjectType_None = 0,
	keProjectType_Coal,
	keProjectType_Logistics,
	keProjectType_InfraredMark,
	keProjectType_DivRebar,
	keProjectTypeLast,
};
typedef unsigned int EVzProjectType;

/// @brief
/// 定义检测物体类型
enum
{
	keVzNLDetectObjType_Unknown = 0,
	keVzNLDetectObjType_Line = 1,				// 直线
	keVzNLDetectObjType_CoplaneCircle = 2,		// 圆/椭圆(共面)
	keVzNLDetectObjType_Triangle = 3,
	keVzNLDetectObjType_Rect = 4,				// 矩形
	keVzNLDetectObjType_Cube = 5,				// 立方体
	keVzNLDetectObjType_StickWidth = 6,			// 杆状物
	keVzNLDetectObjType_StickLength = 7,
	keVzNLDetectObjType_StickVolume = 8,
	keVzNLDetectObjType_StickApex = 9,
	keVzNLDetectObjType_Laser = 10,				// 激光
	keVzNLDetectObjType_Slot = 11,				// 凹槽
	keVzNLDetectObjType_SameSizeCircle = 12,    // 同大小
	keVzNLDetectObjType_3DModel = 14,			// 匹配三维模型
};
typedef unsigned int EVzNLDetectObjType;

enum
{
	keSDKType_Unknown = 0,
	keSDKType_DetectShape,
	keSDKType_DetectLaser,
	keSDKType_DetectCircle,
	keSDKType_DetectColorMark,
	keSDKType_DetectGrains,
};
typedef int EVzSDKType;

typedef enum
{
	keVzColorType_None = 0,
	keVzColorType_Red,
	keVzColorType_Green,
	keVzColorType_Blue,
	keVzColorType_Last,
} EVzColorType;

/// @brief
/// Log Level
enum
{
	keNLLogLevel_None = 0,
	keNLLogLevel_Error,
	keNLLogLevel_Warning,
	keNLLogLevel_Debug,
	keNLLogLevel_Information,
};
typedef int EVzNLLogLevel;

/// @brief
/// Log Type
enum
{
	keNLLogType_None = 0,
	keNLLogType_Console = 0x01,
	keNLLogType_File	= 0x02,
	keNLLogType_LocalPgm= 0x04,
	keNLLogType_Remote	= 0x08,
};
typedef int EVzNLLogType;

/*
* @brief
* 眼睛配置信息
*/
typedef enum
{
	keNLEthernetEyeIPType_Unknown = 0,	//< 未知
	keNLEthernetEyeIPType_StaticIP,		//< 静态IP
	keNLEthernetEyeIPType_DHCP,			//< DHCP
} EVzNLEthernetEyeIPType;

/// @brief
/// 爆光模式
typedef enum
{
	keVzNLExposeMode_Fix = 0,		///< 定值曝光
	keVzNLExposeMode_Auto = 1,		///< 自动曝光
} EVzNLExposeMode;

/// @brief
/// Expose Type
typedef enum
{
	keExposeThresType_Average = 0,
	keExposeThresType_Max = 1,
} EVzExposeThresType;

/// @brief
/// 图像压缩格式
typedef enum
{
	keVzNLImageCompress_Unknown = 0,
	keVzNLImageCompress_Jpeg = 1,        // Jpeg格式
	keVzNLImageCompress_Png = 2,         // png格式
	keVzNLImageCompress_Bmp = 3         // bmp格式
} EVzNLImageCompress;

/// @brief
/// 图像类型
typedef enum
{
	keVzNLImageType_None = 0,
	keVzNLImageType_RGB888,		// 3 channel 24bit
	keVzNLImageType_BGR888,		// 3 channel 24bit
	keVzNLImageType_BAYERGR,	// 1 channel 
	keVzNLImageType_BAYERRG,	// 1 channel 
	keVzNLImageType_BAYERBG,	// 1 channel 
	keVzNLImageType_BAYERGB,	// 1 channel 
	keVzNLImageType_GRAY,		// 1 channel
	keVzNLImageType_YUV420,
	keVzNLImageType_BGRA8888,	// 4 channel 32bit
	keVzNLImageType_YUYV_YUV422,// 
	keVzNLImageType_UNKNOWN = 0xffff,
} EVzNLImageType;

/// @brief
/// 圆形定义
typedef enum
{
	keNL3DCircleTypeUnknown = 0,
	keNL3DCircleType_Circle = 3,				// 单圆
	keNL3DCircleType_ConcentricCircle = 4,		// 同心圆
	keNL3DCircleType_SteelTube = 5,				// 钢管
} EVzNL3DCircleType;

/// @brief
/// 眼睛数据工作模式
typedef enum 
{
	keNLEyeDataWorkMode_Data = 0,				// 数据模式
	keNLEyeDataWorkMode_ImageData = 1,			// 图像数据模式
	keNLEyeDataWorkMode_Image = 2,
} EVzNLEyeDataWorkMode;

/// @breif
/// 数据翻转
enum
{
	keFlipType_None = 0x00,						///< 无翻转
	keFlipType_Vertical = 0x01,					///< 垂直翻转
};
typedef unsigned int EVzFlipType;

/// @brief 
/// 激光点过滤类型
enum
{
	keLaserPointFilterType_None				= 0,		///< 不过滤
	keLaserPointFilterType_BetweenYMinMax	= 0x01,		///< 保留Y轴某范围内的点
	keLaserPointFilterType_ExceptYRange		= 0x02,		///< 排除Y轴某范围
	keLaserPointFilterType_BetweenZMinMax	= 0x10,		///< 保留Z轴某范围的点
	keLaserPointFilterType_ExceptZRange		= 0x20,		///< 排除Z轴某范围的点
	keLaserPointFilterType_BetweenXMinMax	= 0x100,	///< 保留X轴某范围的点
	keLaserPointFilterType_ExceptXRange		= 0x200		///< 排除X轴某范围的点
};

typedef unsigned int EVzLaserPointFilterType;

// @brief
// 摆动机构扫描模式
typedef enum
{
	keSwingMotorScanModeNone = 0,
	keSwingMotorScanMode_Once,		//<	一次
	keSwingMotorScanMode_Loop,		//< 轮询
} EVzSwingMotorScanMode;

// @brief
// 灰度值来源
typedef enum
{
	keGrayValueSource_Left = 0,		//< 左目
	keGrayValueSource_Right,		//<	右目
	keGrayValueSource_Both,			//< 左右
} EVzGrayValueSource;

// @brief
// 摆动机构速度
const float keSwingMotorSpeedMin = 1.f;
const float keSwingMotorSpeed_12 = 12.f;
const float keSwingMotorSpeed_24 = 24.f;
const float keSwingMotorSpeed_36 = 36.f;
const float keSwingMotorSpeed_48 = 48.f;
const float keSwingMotorSpeed_60 = 60.f;
const float keSwingMotorSpeed_72 = 72.f;
const float keSwingMotorSpeed_84 = 84.f;
const float keSwingMotorSpeed_96 = 96.f;
const float keSwingMotorSpeedMax = keSwingMotorSpeed_96;

// @brief
// 参考ROI大小
typedef enum
{
	keDeviceROIInvalid = 0,
	keDeviceROI_128_1024,		//< ROI 128 * 1024
	keDeviceROI_256_1024,		//< ROI 256 * 1024
	keDeviceROI_Full,			//< ROI Full Resolution
	keDeviceROILast				//< 
} EVzDeviceROI;

/// @brief
/// 摆动机构旋转方向
typedef enum
{
	keSwingRotateDirect_Clockwise,				//< 顺时针旋转
	keSwingRotateDirect_Counterclockwise		//< 逆时针旋转
} EVzSwingRotateDirect;

/// @brief
/// 图像分辨率
typedef struct
{
	unsigned int nFrameWidth;
	unsigned int nFrameHeight;
} SVzVideoResolution;

/// @brief
/// 点类型
typedef enum
{
	keResultDataType_Invalid = -1,	//							3D数据类型			2D数据类型
	keResultDataType_Position,		//< Position数据类型		使用SVzNL3DPosition		SVzNL2DPosition
	keResultDataType_PointF,		//< PointF数据类型		使用SVzNL3DPointF		SVzNL2DLRPoint
	keResultDataType_PointXYZ,		//< PointXYZ数据类型		使用SVzNLPointXYZ		SVzNL2DLRPoint
	keResultDataType_PointXYZRGBA,	//< PointXYZRGB数据		使用SVzNLPointXYZRGBA	SVzNL2DLRPoint
	keResultDataType_PointGray,		//< PointXYZGray数据		使用SVzNLPointXYZGray	SVzNL2DLRPoint
	keResultDataType_PointXYZI,		//< PointXYZI数据		使用SVzNLPointXYZI		SVzNL2DLRPoint
	keResultDataType_PointRGBA_D,	//< PointRGBA_D			使用SVzNLPointRGBA_D		SVzNL2DLRPoint
	keResultDataType_PositionF,		//< Position数据类型		使用SVzNL3DPosition		SVzNL2DPositionF
} EVzResultDataType;

/// @brief 设备参数类型
typedef enum
{
	keDeviceParamType_Invalid = 0,				//< 无效
	keDeviceParamType_FrameRate,				//< 帧率参数
	keDeviceParamType_Expose,					//< 曝光参数
	keDeviceParamType_Gain,						//< 增益参数
	keDeviceParamType_LaserLight,				//< 激光亮度参数
	keDeviceParamType_FrameRateWithoutColorData,//< 不携带颜色数据的帧率
	keDeviceParamType_FrameRateWithColorData,	//< 携带颜色数据的帧率
	keDeviceParamType_StrobeTimeRatio,			//< C版激光器亮度值
	keDeviceParamType_RGBExpose,				//< RGB曝光参数
	keDeviceParamType_RGBGain,					//< RGB增益参数
} EVzDeviceParamType;

/// @brief 设备参数类型
typedef enum
{
	keDevChipInfoInvalid = 0,
	keDevChipInfo_Temperature,
	keDevChipInfo_VccPin,
	keDevChipInfo_VccPaux,
	keDevChipInfo_VccPdro,
} EVzDevChipInfo;

/// @brief
/// 网络信息
typedef struct
{
	unsigned int	nAdapterIndex;					//< 网络适配器索引
	char			szNetCardName[VZ_MAXPATH];		//< 网卡名称
	unsigned char	byLocalIP[4];					//< 本机网卡IP
	unsigned char	byMacAddress[6];				//< 本机网卡MAC
	unsigned char	bySubMask[4];					//< 子网掩码
	unsigned char	byGetWay[4];					//< 网关
	unsigned int	dwSpeed;						//< 网络速度 Mbps
} SVzNetCardInfo;

/// @brief
/// 眼睛控制板的设备信息
typedef struct
{
	unsigned int		nSize;										// 结构大小
	EVzDeviceType		eDeviceType;								// 设备类型	
	char				szDeviceName[VZNL_DEVICE_NAME_LENGTH];		// 名称
    char				byServerIP[VZNL_SDK_NETWORK_IPv4_LENGTH];   // 服务端IP
    char				byMAC[VZNL_SDK_NETWORK_MAC_LENGTH];         // MAC地址
	char				szGUID[VZNL_GUID_LENGTH];					// 唯一标识
	char				szDeviceID[VZNL_GUID_LENGTH];				// 设备ID
	char				szDesc[VZ_DEVICE_DESC];						// 设备描述
	VzBool				bValidDevice;								// Device是否有效
} SVzNLEyeCBInfo;

/// @brief
/// 控制板信息
typedef struct
{
	SVzNLEyeCBInfo		sEyeCBInfo;								//< Base Info
	unsigned int		nHardwareVersion;						//< Device HW Version
	unsigned int		chEyeVersion;							//< Device Eye Version
	char				chEyeName[VZ_DEVICE_NAME];				//< Device Eye Name
	char				chEyeIPv4[VZNL_SDK_NETWORK_IPv4_LENGTH];//< Device IP Address
	char				chEyeMAC[VZNL_SDK_NETWORK_MAC_LENGTH];	//< Device Eye MAC
	char				szDeviceAddr[VZNL_DEVICE_ADDR_LENGTH];	//< Device Address
	SVzVideoResolution	sVideoRes;								//< Device Video Resolution
	char				chHwSerialID[VZNL_DEVICEID_LENGTH];		//< Device ID Length

} SVzNLEyeCBDeviceInfoEx;

/// @brief 设备能力描述
typedef union 
{
	unsigned int nCapabilityMask;
	struct
	{
		unsigned int bIsSupportRGBSensor : 1;		//< 是否支持Center Sensor
		unsigned int bSupportSwingMotor  : 1;		//< 是否支持摆动机构
		unsigned int bIsSupportAsyncRGBD : 1;		//< 是否支持异步RGBD
	};

} SVzEthLaserEyeCapability;

/// @brief
/// 眼睛信息
typedef struct
{
	SVzNLEyeCBInfo				sEyeCBInfo;
	EVzEyeTypeFlag				eEyeTypeFlag;							//< 眼睛类型标志
	SVzVideoResolution			sVideoRes;								//< 眼睛分辨率
	int							nFixDisparity;							//< 偏心
	SVzEthLaserEyeCapability	sCapability;
	unsigned int				nMaxFrameIdx;							//< 帧号最大值
	unsigned int				nMaxTimeStamp;							//< 时间戳最大值
	int							nHardwareVersion;						//< 硬件版本
	char						szDeviceAddr[VZNL_DEVICE_ADDR_LENGTH];	//< Device Address	
	char						szVersion[VZNL_VERSION_LENGTH];			//< Version

	// Ethernet Info
	EVzNLEthernetEyeIPType		eEthernetIPType;						//< 以太网IP类型
	char						chEyeIP[VZNL_SDK_NETWORK_IPv4_LENGTH];	//< 眼睛IP
	char						chEyeMac[VZNL_SDK_NETWORK_MAC_LENGTH];	//< 当前眼睛MAC
	char						chLocalIP[VZNL_SDK_NETWORK_IPv4_LENGTH];//< 网卡IP
	char						chLocalMac[VZNL_SDK_NETWORK_MAC_LENGTH];//< 网卡MAC
	char						vendor[8];								//< 
	char						productName[8];							//< 产品名
	char						serialID[8];							//< 序列号

	// Ext Data	
	unsigned int				nSDKClientCnt;							//< SDK Link Count
	unsigned int				nModbusClientCnt;						//< Modbus Link Count
	unsigned int				nEthernetSpeed;							//< xxx Mbps
	char						szHardwareVersion[32];					//< HardWareVersion
	unsigned int                nUdpFlag;                               //< 用于判断是否可以使用UDP广播修改IP，0表示不可以　1表示可以
	unsigned int				nPupilDistance;							//< 瞳距

	// NetCard Info
	SVzNetCardInfo				sNetCardInfo;							//< 网卡信息
} SVzNLEyeDeviceInfoEx;

/// @brief 系统配置参数
typedef struct
{
    unsigned int        nDeviceTimeOut;     // 设备超时设置	
} SVzNLConfigParam;

/// @brief 打开设备类型
typedef struct _tagSVzNLOpenDeviceParam
{
	EVzNLEyeResolution  eEyeResolution;		// 眼睛的分辨率
	EVzSDKType			eSDKType;
	VzBool				bStopStream;
	_tagSVzNLOpenDeviceParam()
	{
		eEyeResolution = keVzNLEyeResolution_Unknown;
		eSDKType = keSDKType_Unknown;
		bStopStream = VzTrue;
	}
} SVzNLOpenDeviceParam;

typedef struct
{
	char	szPath[256];
} SVzNLTestOpenDeviceParam;

/// @brief
/// 更新工程信息
typedef struct
{
	char szProject[32];
	char szVersion[32];
} SVzSystemProjectInfo;

/// @breif
/// 版本信息。
typedef struct
{
	long bValidSDKVersion: 1;		///< SDK版本是否可用
	long bValidAppVersion: 1;		///< EyeCB版本是否可用
	long bValidCameraVersion: 1;	///< 眼睛库版本是否可用
	long bValidAlgoVersion: 1;		///< 算法版本是否可用
	long bValidApkVersion: 1;		///< 安卓版本是否可用
	long bValidDevIDVersion: 1;		///< DEVID版本是否可用
	long bValidWifiVersion: 1;		///< WIFI版本是否可用
	long bValidHDMIVersion: 1;		///< HDMI版本是否可用
	long bValidVizumEyeVersion : 1;	///< 硬件版本是否可用
	long bValidFwVersion : 1;	///< 硬件版本是否可用
} SVzNLVersionCapGroup;

typedef union
{
	SVzNLVersionCapGroup sVersionCap;
	long				 nMaskCap;
} SVzNLDevVersionCap;

/// @brief
/// 版本信息
typedef struct 
{
	SVzNLDevVersionCap sCap;
	char szSDKVersion[VZNL_VERSION_LENGTH];		///< SDK版本
	char szAppVersion[VZNL_VERSION_LENGTH];		///< App版本
	char szCameraVersion[VZNL_VERSION_LENGTH];	///< 眼睛版本
	char szAlgoVersion[VZNL_VERSION_LENGTH];	///< 算法版本	
	char szApkVersion[VZNL_VERSION_LENGTH];		///< 安卓版本
	char szDevIDVersion[VZNL_VERSION_LENGTH];	///< 设备ID版本
	char szWifiVersion[VZNL_VERSION_LENGTH];	///< WIFI版本
	char szHDMIVersion[VZNL_VERSION_LENGTH];	///< HDMI版本
	char szVizumEyeVersion[VZNL_VERSION_LENGTH];///< 硬件版本
	char szFwVersion[VZNL_VERSION_LENGTH];		///< 硬件版本
} SVzNLVersionInfo;

/// @brief
/// 图像信息
typedef struct
{
	unsigned char*      pBuffer;     // 图像buffer信息
	unsigned int        nBufferSize; // 图像buffer的大小

	unsigned int        nWidth;      // 图像宽度
	unsigned int        nHeight;     // 图像高度
	unsigned char		nChannels;	 // 通道数
	EVzNLImageCompress	eImageCompress;
} SVzNLCompressData;

/// @brief
/// 点坐标
typedef struct
{
	int  x;
	int  y;
} SVzNL2DPoint;

/// @brief
/// 点坐标
typedef struct
{
	unsigned short x;
	unsigned short y;
} SVzNL2DPointShortU;

/// @brief
/// 点坐标
typedef struct
{
	SVzNL2DPoint  sLeft;
	SVzNL2DPoint  sRight;
} SVzNL2DLRPoint;

/// @brief
/// 点坐标
typedef struct
{
	float  x;
	float  y;
} SVzNL2DPointF;

typedef struct
{
	double  x;
	double  y;
} SVzNL2DPointD;

typedef struct
{
    double x;
    double y;
    double z;
} SVzNL3DPoint;

typedef struct 
{
	float x;
	float y;
	float z;
} SVzNL3DPointF;

// 3D Point
typedef union
{
	float fData[4];
	struct
	{
		float x;
		float y;
		float z;
	};
} SVzNLPointXYZ;

// 3D Point + RGB
typedef union
{
	float fData[4];
	struct
	{
		float		x;
		float		y;
		float		z;
		VzColorRGB  nRGB;
	};
} SVzNLPointXYZRGBA;

// 3D Point + Gray
typedef union
{
	float fData[4];
	struct
	{
		float		x;
		float		y;
		float		z;
		int			nGray;
	};
} SVzNLPointXYZGray;

// 3D Point + Gray
typedef struct
{
	union
	{
		float fData[4];
		struct
		{
			float		x;
			float		y;
			float		z;
		};
	};

	union
	{
		float fData_c[4];
		struct
		{
			float		intensity;
		};
	};
} SVzNLPointXYZI;

// 3D Point + Gray
typedef struct
{
	union
	{
		double dData[4];
		struct
		{
			double			x;
			double			y;
			double			z;
			VzColorRGB		nRGB;
		};
	};
} SVzNLPointRGBA_D;

typedef struct
{
	int	x;
	int y;
	int z;
	int flag;
} SVzNLLogisticsPoint;

typedef struct
{
	float x;
	float y;
	float z;
	float reserve;
} SVzNLAlign3DPointF;

typedef struct
{
	int x;
	int y;
	int z;
	int reserve;
} SVzNLAlign3DPointI;

struct SVz3DPointFWithPixel
{
	float			x;
	float			y;
	float			z;
	unsigned char	byPixelL;
	unsigned char	byPixelR;
	unsigned char	byReserve[2];
};

/// @brief 范围类型 double
typedef struct
{
	float min;	//< 最小值
	float max; //< 最大值
} SVzNLRangeF;

/// @brief 范围类型 double
typedef struct
{
	double min;	//< 最小值
	double max; //< 最大值
} SVzNLRangeD;

/// @brief 范围类型 int
typedef struct
{
	unsigned int nMin;	//< 最小值
	unsigned int nMax;	//< 最大值
} SVzNLRangeU;

/// @brief 范围类型 int
typedef struct
{
	int nMin;	//< 最小值
	int nMax;	//< 最大值
} SVzNLRange;

/// @brief 3D值范围
typedef struct
{
	SVzNLRangeD xRange;	//< X范围
	SVzNLRangeD yRange; //< Y范围
	SVzNLRangeD zRange; //< Z范围
} SVzNL3DRangeD;

/// @brief 线的2D信息
typedef struct
{
	SVzNL2DPoint  ptStart2D;	// 2D起始坐标
	SVzNL2DPoint  ptEnd2D;		// 2D结束坐标
} SVzNL2DLine;

typedef struct
{
	SVzNL2DPointF  ptStart2D;	// 2D起始坐标
	SVzNL2DPointF  ptEnd2D;		// 2D结束坐标
} SVzNL2DLineF;

/// @brief 线的3D信息
typedef struct
{
	SVzNL3DPoint  ptStart3D;	// 3D起始坐标
	SVzNL3DPoint  ptEnd3D;		// 3D结束坐标
} SVzNL3DLine;

/// @brief 2D矩形
typedef struct 
{
	SVzNL2DPoint	ptLT;		//< 左上
	SVzNL2DPoint	ptRT;		//< 右上
	SVzNL2DPoint	ptRB;		//< 右下
	SVzNL2DPoint	ptLB;		//< 左下
} SVzNL2DRect;

/// @brief 3D矩形
typedef struct 
{
	SVzNL3DPoint	ptLT;		//< 左上
	SVzNL3DPoint	ptRT;		//< 右上
	SVzNL3DPoint	ptRB;		//< 右下
	SVzNL3DPoint	ptLB;		//< 左下
} SVzNL3DRect;

/// @brief 三角形
typedef struct
{
	SVzNL3DPoint	ptVertexA;
	SVzNL3DPoint	ptVertexB;
	SVzNL3DPoint	ptVertexC;
} SVzNL3DTriangle;

/// @brief 三角形
typedef struct
{
	SVzNL2DPoint	ptVertexA;
	SVzNL2DPoint	ptVertexB;
	SVzNL2DPoint	ptVertexC;
} SVzNL2DTriangle;

/// @brief 点的坐标
typedef struct
{
	int			  nPointIdx;  // 点序
	SVzNL2DPoint  ptLeft2D;   // 2D左图坐标
	SVzNL2DPoint  ptRight2D;  // 2D右图坐标
	SVzNL3DPoint  pt3D;       // 3D坐标
} SVzNLPosition;

/// @brief 点坐标 (double)
typedef struct
{
	int				nPointIdx;  // 点序
	SVzNL2DPointD	ptLeft2D;   // 2D左图坐标
	SVzNL2DPointD	ptRight2D;  // 2D右图坐标
	SVzNL3DPoint	pt3D;       // 3D坐标
} SVzNLPositionD;

/// @brief 2D点坐标
typedef struct
{
	int			  nPointIdx;  // 点序
	SVzNL2DPoint  ptLeft2D;   // 2D左图坐标
	SVzNL2DPoint  ptRight2D;  // 2D右图坐标
} SVzNL2DPosition;

/// @brief 2D点坐标
typedef struct
{
	int			  nPointIdx;  // 点序
	SVzNL2DPointF  ptLeft2D;   // 2D左图坐标
	SVzNL2DPointF  ptRight2D;  // 2D右图坐标
} SVzNL2DPositionF;


/// @brief 3D点坐标
typedef struct
{
	int			  nPointIdx;  // 点序
	SVzNL3DPoint  pt3D;       // 3D坐标
} SVzNL3DPosition;

/// @brief 3D激光线
typedef struct
{
	SVzNL3DPosition*		p3DPosition;		//< 3D点(数组)
	int						nPositionCnt;		//< 3D点个数
	unsigned long long		nTimeStamp;			//< 时间戳
} SVzNL3DLaserLine;

/// @brief 3D激光线
typedef struct
{
	SVzNLPointXYZRGBA*		p3DPoint;		//< 3D点(数组)
	int						nPointCnt;		//< 3D点个数
	unsigned long long		nTimeStamp;		//< 时间戳
} SVzNLXYZRGBDLaserLine;

/// @brief 3D点坐标(float)
typedef struct
{
	int				nPointIdx;	//< 点索引
	SVzNL3DPointF	pt3D;		//< 点值
} SVzNL3DPositionF;

/// @brief 矩形
typedef struct
{
    int left;		//< 左
    int right;		//< 右
    int top;		//< 上
    int bottom;		//< 下
} SVzNLRect;

/// @brief 大小
typedef struct
{
	int nWidth;		//< 宽
	int nHeight;	//< 高
} SVzNLSize;

/// @brief 大小
typedef struct
{
	short shWidth;		//< 宽
	short shHeight;	//< 高
} SVzNLSizeSh;

/// @brief 大小
typedef struct
{
	float fWidth;		//< 宽
	float fHeight;	//< 高
} SVzNLSizeF;

/// @brief 大小(double)
typedef struct
{
	double dWidth;	//< 宽
	double dHeight; //< 高
} SVzNLSizeD;

/// @brief 三角形的类型
typedef struct  
{
	double dSideA;
	double dSideB;
	double dSideC;
} SVzNLTriangleInfo;

/// @brief
/// ROI区域
/// o————————————————o
/// |     ↑  ↑       |
/// |     |T |B      |
/// |     ↓  |       |
/// |  L  o——|——o    |
/// |←———→|  |  |    |
/// |  R  |  |  |    |
/// |←————|——↓—→|    |
/// |     o—————o    |
/// o————————————————o
typedef struct			
{						
	int left;		// 左
	int right;		// 右
	int top;		// 上
	int bottom;		// 下
} SVzNLROIRect;			

/// @brief 2D 位置
typedef struct
{
	double x;		//< X
	double y;		//< Y
	double value;	//< None
} SVzNL2DValueD;

/// @brief 设备范围数据
typedef struct
{
	SVzNLRangeU	sFrameRate;		//< 帧率范围
	SVzNLRangeU	sExpose;		//< 曝光范围
	SVzNLRangeU	sGain;			//< 增益范围
} SVzDeviceLimitInfo;

/// @brief 图像信息
typedef struct
{
	unsigned char*      pBuffer;     // 图像buffer信息
	unsigned int        nBufferSize; // 图像buffer的大小

	SVzNL2DPoint		ptOriPos;	 // 原始位置(顶点)
	unsigned int        nWidth;      // 图像宽度
	unsigned int        nHeight;     // 图像高度
	EVzNLImageType		eImageType;  // 图像格式
	unsigned char       byBitDepth;  // 位图深度
	unsigned char		nChannels;	 // 通道数
} SVzNLImageData;

//---------------------------------------------------------------
// @brief 3D数据二维索引表
//---------------------------------------------------------------
typedef struct
{
	EVzResultDataType	ePointType;		// 3D点类型
	void*				pPointPtr;		// 3D点指针
	unsigned int		nWidth;			// 二维表宽
	unsigned int		nHeight;		// 二维表高
} SVz3DPointMapTable;

/// @breif 各检测类型的结构体
typedef struct
{
    EVzNLDetectObjType eDetectType;    
    int				   nReserve[2];//< 保留指针，可能是内部的一个对象？    
} SVzNLDetectObjectResult;

/// @breif 线的2D结果
typedef struct  
{
	SVzNL2DLine	sLeftLine;
	SVzNL2DLine	sRightLine;
	double		dLineLength;	// 线长
} SVzNL2DLineResult;

/// @breif 线的3D结果
typedef struct  
{
	SVzNL3DLine	sLine;
	double		dLineLength;	// 线长
} SVzNL3DLineResult;

/// @breif 线的2D结果
typedef struct  
{
	SVzNL2DRect		sLeftRect;
	SVzNL2DRect		sRightRect;
	
	double			dTopDistance;
	double			dRightDistance;
	double			dBottomDistance;
	double			dLeftDistance;
} SVzNL2DRectResult;

/// @breif 2D结果
typedef struct  
{
	SVzNL3DRect		sRect;
	double			dTopDistance;
	double			dRightDistance;
	double			dBottomDistance;
	double			dLeftDistance;
} SVzNL3DRectResult;

/// @breif 三角形结果
typedef struct
{
	SVzNL2DTriangle	sLeftTriangle;
	SVzNL2DTriangle	sRightTriangle;

	double			dABDistance;
	double			dBCDistance;
	double			dACDistance;
} SVzNL2DTriangleResult;

typedef struct
{
	SVzNL3DTriangle	sTriangle;

	double			dABDistance;
	double			dBCDistance;
	double			dACDistance;
} SVzNL3DTriangleResult;

/// @brief 椭圆检测结果
typedef struct
{
    SVzNLDetectObjectResult   sDetectObj;
    SVzNL2DPoint              s2DPoint;
} SVzNLDetectEllipseResult;

typedef struct
{
    int	nSize;
} SVzNLDetectCubeParam;

typedef struct
{
    SVzNLDetectObjectResult   sDetectObj;
} SVzNLDetectCubeResult;

/// @brief 3D物体的连接线
typedef struct 
{
	int nLineStart;
	int nLineEnd;
} SVzNL3DObjectLine;

/// @brief 像素深度值归一化模式
typedef enum
{
	keDepthNormalizingMode_None			= 0, //不使用归一化
	keDepthNormalizingMode_Indicated	= 1, //指定固定归一化范围
	keDepthNormalizingMode_Auto			= 2  //自动检测归一化范围
} EVzDepthNormalizingMode;

/// @brief 图像位深
typedef enum
{
	keImageBitDepthTypeNone = 0,
	keImageBitDepthType_8bit = 8,
	keImageBitDepthType_16bit = 16,
} EVzImageBitDepthType;

/// 深度图存储配置参数 
typedef struct
{
	EVzDepthNormalizingMode	eDepthNormalizingMode;		// 像素深度值归一化模式
	unsigned int			nImageMargin;				// 输出深度图边缘预留空白区域宽度（单位：像素值）
	EVzImageBitDepthType	eBitDepth;					// 输出深度图的位深度：
	float					fDepthResolution;			// 深度图中深度值精度，例如：0.1 表示1个像素值对应0.1mm 实际距离

	float					fWHPixelResolution;			// 深度图中每个像素的精度，例如：0.1 表示1个像素点对应0.1mm实际距离，eDepthNormalizingMode = eDepthNormalizingMode_Indicated 时有效
	float					fNormalizingStartVal;		// 客户指定的归一化范围的起始值(mm)：eDepthNormalizingMode = keDepthNormalizingMode_Indicated 时有效
	float					fNormalizingEndVal;			// 客户指定的归一化范围的结束值(mm)：eDepthNormalizingMode = keDepthNormalizingMode_Indicated  时有效

	float					fNormalizingMargin;			// eDepthNormalizingMode = keDepthNormalizingMode_Auto 时，在自动检测到的归一化区间上下预留的空白空间高度（单位mm）上下各预留指定高度空间

} SVzDepthMapImgCtlPara;

/// @brief 3D物体面信息
typedef struct  
{
	int				nPointCount;
	unsigned int*	pArryPointIdx;
} SVzNL3DObjectPanel;

/// @brief 三维物体描述
typedef struct  
{
	int					nType;
	SVzNL3DPoint*		pGrabPoint;			///< 物体抓取点
	int					nGrabPointCount;
	SVzNL3DPoint*		p3DPoint;			///< 物体各顶点
	int					n3DPointCount;		///< 物体上点的个数
	SVzNL3DObjectLine*	p3DObjLine;			///< 物体上的线
	int					n3DObjLineCount;	///< 物体上线的个数
	SVzNL3DObjectPanel* p3DObjPanel;
	int					n3DObjPanelCount;
} SVzNL3DObjectDesc;

/// @brief 三维物体描述
typedef struct  
{
	SVzNL2DPoint*		pLeftGrabPoint;		///< 物体抓取点
	SVzNL2DPoint*		pRightGrabPoint;	///< 物体抓取点
	int					nGrabPointCount;
	SVzNL2DPoint*		pLeft2DPoint;		///< 物体各顶点
	SVzNL2DPoint*		pRight2DPoint;		///< 物体各顶点
	int					n2DPointCount;		///< 物体上点的个数
	SVzNL3DObjectLine*	p3DObjLine;			///< 物体上的线
	int					n3DObjLineCount;	///< 物体上线的个数
	SVzNL3DObjectPanel* p3DObjPanel;
	int					n3DObjPanelCount;
} SVzNL3DObject2DResult;

/// @brief ColorMark 形状
typedef enum
{
	keVzNLColorMarkType_Unknown = 0,
	keVzNLColorMarkType_Rect,
	keVzNLColorMarkType_Triangle,
	keVzNLColorMarkType_Circle,
} EVzNLColorMarkType;

/// @brief Color Mark
typedef struct
{
	int			  nPointIdx;  // 点序
	SVzNL3DPoint  pt3D;       // 3D坐标
} SVzNLColorMark3DResult;

/// @brief ROI
typedef struct
{
	int				nPointIdx;  // 点序
	SVzNL2DPoint	ptLeft2D;   // 2D左图坐标
	SVzNL2DPoint	ptRight2D;  // 2D右图坐标
	SVzNLRect		sLeftBox;
	SVzNLRect		sRightBox;
} SVzNLColorMark2DResult;

/// @brief ROI
typedef struct
{
	int						nPointIdx;  // 点序
	EVzNLColorMarkType		eType;
	SVzNL3DPoint			pt3D;
	SVzNL2DPoint			ptLeft2D;   // 2D左图坐标
	SVzNL2DPoint			ptRight2D;  // 2D右图坐标
	SVzNLRect				sLeftBox;
	SVzNLRect				sRightBox;
} SVzNLColorMarkResult;

typedef struct
{
	SVzNL3DPosition* p3DPoint;
	int				 nCount;
}SVzDetectGrainsLaserResult;

typedef struct
{
	unsigned int nIndex;
	float fLength;
	float fWidth;
	float fHeight;

	float fVolume;				//until mm
	SVzNL3DPoint sObjCenter;
	SVzNL3DPoint sBoxGoundTop[4];
	SVzNL3DPoint sBoxGoundBottom[4];
} SVzLaserGrainsData;

typedef struct
{
	SVzLaserGrainsData* pGrainData;
	int					nCount;
} SVzDetectGrainResult;

typedef enum 
{
	keGrainLaserStatus_Unknown = 0,
	keGrainLaserStatus_BeginWork,
	keGrainLaserStatus_Working,
	keGrainLaserStatus_EndWork,
} EVzGrainLaserStatus;

///< 每个石子的信息
typedef struct
{
	int   index_;
	char  lithology[128];	//岩性
	char  origin_[128];		//产地
	double length_;			//长
	double width_;			//宽
	double height_;			//高
	double vol_;			//体积
	double area_;			//面积
	double perimeter_;		//周长
	float  A_;				//Axial factor 轴向系数
	float  CR_;				//Convexity 凸度
	float  AP_;				//棱角参数
	VzBool isNeedle_;		//是否针片状
	SVzNLRect rcPos;
} SVzLaserAggregateData;

///< 级配信息
typedef struct 
{
	int index_;      // 级配级别 6 7 8 9 10 11 12 13
	int num_;        // 数量
	float pi_;       // pi
	float passRate_; // 通过率
	float top_;      // 上限
	float bottom_;   // 下限
} SVzLaserAggregateLevelData;

///< 石子检测综合信息
typedef struct
{
	char	chLithology[128];			///< 岩性
	char	chOrigin[128];				///< 产地
	float	fNeedleRate;				///< 针片状含量
	float	fMeanAP;					///< 综合棱角值
	float	fMeanA;						///< 综合轴向系数
	float	fMeanCR;					///< 综合凸度
} SVzLaserAggregateMeanInfo;

///< 石子检测状态
typedef enum
{
	keAggerateLaserStatus_Unknown = 0,
	keAggerateLaserStatus_BeginWork,	///< 开始工作
	keAggerateLaserStatus_Working,		///< 正在工作
	keAggerateLaserStatus_EndWork,		///< 结束工作
} EVzAggerateLaserStatus;

/// @brief
/// 钢筋信息
typedef struct
{
	int				nFlag;
	int				nRadius;
	float			fPhyRadius;
	SVzNL2DPoint	nPoint;
	int				nLineIndex;
	unsigned int	nTimeStamp;
	SVzNL3DPoint	n3DPoint;
}SVzNLCounterItem;

/// @brief
/// 钢筋信息
typedef struct
{
	int					nNum;			// 总数
	SVzNLCounterItem*	pPointArr;		// 二维扫描图坐标
	unsigned long long	nStartTimeStamp;// 起始线的时间戳
	SVzNLPosition		sTopPoint;		// 最突出点数据
	SVzNLCounterItem	sMarkPoint;		// mark信息
	SVzNLImageData		sImage;			// 生成的二维图像
} SVzNLCounter;

/// @brief
typedef struct
{
	unsigned int	nSize;
	char			szName[64];
	char			szVersion[64];
	void*			pVoid;
	int				nVersionCode;
} SVzAICreateParam;

/**
 * @brief 
 * 输出帧参数
 */ 
typedef struct
{
	unsigned long long	nFrameIdx;	//< 帧号
	unsigned long long	nTimeStamp;	//< 时间戳
} SVzOutputFrameProps;

/**
 * @brief 
 * Circle Mark Information
 */
typedef struct  
{
	int				nMarkID;			//< Mark 标志
	SVzNL3DPosition	ptCenterPos;		//< 中心点
	SVzNL3DPosition	ptLeftPos;			//< 左
	SVzNL3DPosition	ptTopPos;			//< 上
	SVzNL3DPosition	ptRightPos;			//< 右
	SVzNL3DPosition	ptBottomPos;		//< 下
	unsigned int	nGray;
} SVzCircleMarkPosInfo;

/// @brief
/// 3D圆形
typedef struct 
{
	float						fR;
	SVzNL3DPointF				ptCenter;
	SVzNL3DPointF				ptWordCenter;
	SVzNL3DPointF				ptNormalVector;
	SVzNL3DPointF*				pArray3DPoint;
	SVzNL3DPointF*				pArrayWorld3DPoint;
	int							nPointCount;
} SVzNL3DCircleResult;

/// @brief
/// 3D圆形物体
typedef struct
{
	EVzNL3DCircleType			eType;		// 圆类型

	SVzNLROIRect				box2dLeft;	// 包围盒左图
	SVzNLROIRect				box2dRight; // 包围盒右图

	SVzNL3DCircleResult*		pArrayCircle;
	int							nCircleCount;
} SVzNL3DCircleObjectResult;

/// @brief
/// 3D圆形
typedef struct
{
	double						dR;
	SVzNL2DPoint				ptLeftCenter;
	SVzNL2DPoint				ptRightCenter;
	SVzNL2DPoint*				pArray2DLeftPoint;
	SVzNL2DPoint*				pArray2DRightPoint;
	int							nPointCount;
} SVzNL2DCircleResult;

/// @brief
/// 3D圆形物体
typedef struct
{
	EVzNL3DCircleType			eType;		// 圆类型

	SVzNLROIRect				box2dLeft;	// 包围盒左图
	SVzNLROIRect				box2dRight; // 包围盒右图

	SVzNL2DCircleResult*		pArrayCircle;
	int							nCircleCount;
} SVzNL2DCircleObjectResult;

/*
 * @brief 眼睛配置信息
 */
typedef struct
{
	EVzNLEthernetEyeIPType	eEthernetIPType;
	unsigned char			byEyeIP[4];
	unsigned char			byEyeMac[6];
	unsigned char			byLocalIP[4];
	unsigned char			byLocalMac[6];
	char					vendor[8];
	char					productName[8];
	char					serialID[8];
	VzBool					bIsSupportUDPFilter;

	SVzNetCardInfo			sNetCardInfo;
} SVzNLEthernetEyeConfigInfo;

typedef struct 
{
	double			dWhiteCheckPercent;
	unsigned char	chRgbMax[3];
	double			dBlockEffectiveTh;
	short			shMinBlockNumber;
	double			dGainRange[2];
	unsigned char	chHistStartAddr;
	int				nBlockWidth;
	int				nBlockHeight;
} SVzNLAWBInfo;

// 钢筋检测模式
enum
{
	keEyeCBDetectCounterMode_All = 1,
	keEyeCBDetectCounterMode_Center = 2,
};
typedef unsigned int EVzEyeCBDetectMode;

// @brief 摆动机构信息
struct SVzSwingMotorDevInfo
{
	int				nMotorMaxAngle;		//< 电机最大旋转角度
	int				nMaxAngle;			//< 最大旋转角度
	int				nMinSpeed;			//< 最小角速度
	int				nMaxSpeed;			//< 最大角速度
	int				nStartAngle;		//< 起始位置
	int				nEndAngle;			//< 结束位置
	int				nSpeed;				//< 速度
	int				nStopAngle;			//< 停止角度
	float			fEndAngleThres;		//< 结束角度判断阈值
};

/// @brief 
typedef struct
{
	unsigned int nAirforceCount;	//< 气枪数
	unsigned int nStartAddress;		//< 起始位置
	double dVolumeThres;			//< 体积阈值
	unsigned int nMaxAirTimes;		//< 最大喷气次数
	unsigned int nTimePerAir;		//< 单次喷气时间 ms
} SVzDetectGangueParam;

/// @brief
typedef struct  
{
	double			dArea;
	SVzNL3DPoint	ptPos[8];
} SVzGangueObjectResult;

/// @brief
/// Dump文件回调
/// @param [out] szCrashFile Dump全路径
typedef void(*VzNL_CrashCallBack)(char szCrashFile[VZ_MAXPATH]);

/// @name 自动检测回调函数
/// @{
/**
 * @brief 获取自动检测结果
 * @param [out] p3DPoint 3D数据
 * @param [out] p2DPoint 2D数据
 * @param [out] nCount 数据个数
 * @param [out] nTimeStamp 时间戳
 * @param [out] dTotleOffset 总偏移
 * @param [out] dStep 单次偏移
 * @param [out] pLeftImage 左图
 * @param [out] pRightImage 右图
 * @param [out] pParam 回调参数
 */
typedef int(*VzNL_GetAutoDetectResultCB)(SVzNL3DPosition* p3DPoint, SVzNL2DPosition* p2DPoint, int nCount, unsigned long long nTimeStamp, double dTotleOffset, double dStep, SVzNLImageData* pLeftImage, SVzNLImageData* pRightImage, void* pParam);

/// @brief 激光线数据
typedef struct
{
	void*				p3DPoint;		//< 3D点 根据ResultDataType使用不同的数据结构
	void*				p2DPoint;		//< 2D点 根据ResultDataType使用不同的数据结构
	int					nPointCount;	//< 2D 3D 数据个数
	double				dTotleOffset;	//< 总偏移
	double				dStep;			//< 距离上一条线的偏移
	unsigned long long	llFrameIdx;		//< 帧号
	unsigned long long	llTimeStamp;	//< 时间戳
	unsigned int		nEncodeNo;		//< 编码器值
	float				fSwingAngle;	//< 当前摆动角度
	VzBool				bEndOnceScan;	//< 是否已完成一次扫描
} SVzLaserLineData;

typedef void(*VzNL_AutoOutputLaserLineExCB)(EVzResultDataType eDataType, SVzLaserLineData* pLaserLinePoint, void* pParam);

typedef struct
{
	unsigned int nCount;
	EVzResultDataType eDataType;
	SVzLaserLineData* pLaserLineData;
}SVzLaserLineCacheData;

/// @brief 激光线数据
typedef struct
{
	SVzLaserLineData	sLineData;		//<

	SVzNLImageData*		pImageData[3];	//< 图像数据
	void*				pUnknownData;	//< 未知数据
} SVzLaserLineDataEx;

/// @}

/// @brief 检测码类型
enum
{
	keLogisticsDetectCodeTypeNone		= 0,	//< 什么都不检测
	keLogisticsDetectCodeType_QRCode	= 0x01,	//< 二维码
	keLogisticsDetectCodeType_BarCode	= 0x02,	//< 条形码
};
typedef unsigned int EVzLogisticsDetectCodeType;

/// @brief 扣图类型
enum
{
	keLogisticsClipObjModeNone		= 0,		//< 不做抠图操作
	keLogisticsClipObjMode_Object	= 0x01,		//< 抠物体图
	keLogisticsClipObjMode_QRCode	= 0x02,		//< 抠二维码图
	keLogisticsClipObjMode_BarCode	= 0x04,		//< 抠条形码图
};
typedef unsigned int EVzLogisticsClipObjMode;

// 物流触发位置
enum
{
	keLogisticsPackageSyncTypeInvalid = 0,		//< 不触发
	keLogisticsPackageSyncType_In,				//< 物体进入时触发
	keLogisticsPackageSyncType_Out,				//< 物体出来时触发
};
typedef unsigned int EVzLogisticsPackageSyncType;

// 物流相机运行模式
typedef enum
{
	keLogisticsRunMode_None = 0,				//< 无
	keLogisticsRunMode_SoftSignal = 1,			//< 软件使能开始检测
	keLogisticsRunMode_ExtSignal,				//< 外部使能开始检测
	keLogisticsRunMode_AutoSignal,				//< 自动检测
} EVzLogisticsRunMode;

// 物流结果状态
typedef enum 
{
	keLogisticsResultStatusInvalid = 0,	//< 无效
	keLogisticsResultStatus_Begin,		//< 物体开始
	keLogisticsResultStatus_Work,		//< 物体运动中    返回点云数据
	keLogisticsResultStatus_End			//< 当前物体结束  返回物体体积
} EVzLogisticsResultStatus;

// @brief 结果信息掩码
enum 
{
	keLogisticsResultMask_PointCloud	= 0x01,		//< 点云数据
	keLogisticsResultMask_ObjectInfo	= 0x02,		//< 物体结果数据
	keLogisticsResultMask_ColorImage	= 0x04,		//< 彩色图像
};
typedef unsigned int EVzLogisticsResultMask;

// @brief 硬件使能线信号类型
enum
{
	keEnLineInPoint_None			= 0x00,		//< 无效
	keEnLineInPoint_EnSignal		= 0x01,		//< 使能信号
	keEnLineInPoint_BiPhaseEncoder	= 0x02,		//< 双向编码器
};
typedef unsigned int EVzEnLineInPoint;

// @brief 
typedef struct
{
	char			szCode[VZ_MAXPATH];		//< 码值
	VzBool			bValidImage;
	SVzNLImageData	sCodeImageData;			//< 码图像
	void*			pVoid;
} SVzLogisticsCodeInfo;

// @brief 物体结果描述
typedef struct  
{
	float						fVolume;			//< 体积
	float						fLength;			//<	长
	float						fWidth;				//< 宽
	float						fHeight;			//< 高

	// ①------------②
	// /|            /|
	//③------------④ |
	//| |			| |
	//| |			| |
	//| |			| |
	//|⑤------------⑥
	// /            / 
	//⑦------------⑧
	SVzNL3DPoint				sConnerPoint[8];	//< 8角点
	SVzNL3DPoint*				pOutlinePoint;		//< 轮廓点
	unsigned int				nOutlinePointCount;	//< 轮廓点个数
		
	SVzLogisticsCodeInfo*		psQRCodeInfo;		//< 二维码信息
	unsigned int				nQRCodeCount;
	SVzLogisticsCodeInfo*		psBarCodeInfo;		//< 条形码信息
	unsigned int				nBarCodeCount;

	VzBool						bValidObjImage;		//< 物体图像有效
	SVzNLImageData				sObjImageData;		//< 物体图像
	void*						pVoidObjData;
} SVzLogisticsObjInfo;

// 物流结果信息
typedef struct  
{
	EVzLogisticsResultMask		eMask;				//< 结果标志

	// @name PointClound
	// @{
	unsigned long long			ullTimeStamp;		//< 时间戳
	unsigned long long		    ullEncodeNo;		//< 编码器值
	SVzNL3DPosition*			p3DPoint;			//< 3D点
	unsigned int				nPointCount;		//< 点数
	double						dOffset;			//< 偏移
	// @}

	// @name ObjectInfo
	// @{
	unsigned int				nObjectCount;		//< 物体个数
	SVzLogisticsObjInfo*		pArrayObjectInfo;	//< 物体数组
	
	VzBool						bIsResetOffset;		//< 当前帧开始X值被重置
	// @}

	// @name ColorImage
	// @{
	SVzNLImageData*				psCenterImage;			//< 图像
	unsigned int				nCenterImageTimeStamp;	//< 图像时间
	// @}
} SVzLogisticsResultData;

typedef enum
{
	keLaserFileType_Invalid = 0,
	keLaserFileType_PureTxt = 1,	//< 纯文本
	keLaserFileType_Txt,			//< 本公司自用
	keLaserFileType_Pcd,			//< PCD
	keLaserFileType_Las,			//< LAS
	keLaserFileType_VzBinaryData,	//< Binary File
	keLaserFileType_Ply,
} EVzLaserFileType;

/// @brief 文件模式
enum
{
	keFileModeInvalid = 0,
	keFileModeRead = 0x01,
	keFileModeWrite = 0x02,
};
typedef unsigned int EVzFileMode;

/// @brief 点云后处理图像类型定义
typedef enum
{
	kePointAfterProcTypeInvalid = -1,
	kePointAfterProcType_DepthMap,			// 深度图
	kePointAfterProcType_GrayPic,			// 灰度图
	kePointAfterProcType_Intensity,			// 光强图
} EVzPointAfterProcType;

/*
 * 机械臂点数据结构定义
 */
typedef struct
{
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
} SVzNLRobotPointF;

typedef struct
{
	SVzNLRobotPointF sPoint;			// 坐标系内的点坐标
	SVzNL3DPointF	 sRX;				// 坐标系 X方向
	SVzNL3DPointF	 sRY;				// 坐标系 Y方向
	SVzNL3DPointF	 sRZ;				// 坐标系 Z方向
} SVzNLRobotPointWithPoseF;

typedef struct
{
	SVzNL3DPointF sPoint;				// 坐标系内的点坐标
	SVzNL3DPointF sRX;					// 坐标系 X方向
	SVzNL3DPointF sRY;					// 坐标系 Y方向
	SVzNL3DPointF sRZ;					// 坐标系 Z方向
} SVzNL3DPosePointF;

//
typedef struct
{
	SVzNL3DPointF	 sPoint;			// 点位置
	SVzNL3DPointF	 sNormal;			// 法向量
} SVzNL3DTracePointF;

/// @brief 激光线数据
typedef struct
{
	SVzLaserLineData	sLineData;

	unsigned int		nLeftPointCount;
	unsigned int		nRightPointCount;

	SVzNLPointXYZ*		pLeftTearData;
	SVzNLPointXYZ*		pRightTearData;
	unsigned int		nLeftTearPointCount;
	unsigned int		nRightTearPointCount;
} SVzLaserTearDataEx;

/// @brief 
typedef enum
{
	keLaserObjectDetectPresetProfile_Custom = -1,
	keLaserObjectDetectPresetProfile_General,		//<	 通用
	keLaserObjectDetectPresetProfile_ThinWire,		//<  细铁丝
	keLaserObjectDetectPresetProfile_BigSence,		//<  大景深
	keLaserObjectDetectPresetProfile_ReBar,			//<  钢筋计数
	keLaserObjectDetectPresetProfileLast
} EVzLaserObjectDetectPresetProfile;


/// @brief
/// 激光线文件配置参数
typedef enum
{
	keLaserFileParamStart = 0,
	keLaserFileParam_Speed = keLaserFileParamStart,	//<	速度值 类型 double 当前用于 binarydata 和 txt 两种文件存储
	keLaserFileParam_DataType,						//<	数据类型	类型 unsigned int 当值为 0时表示Ascii模式，当为1时表示Binary模式，目前用于pcd的配置
	keLaserFileParam_SavePointCnt,					//< 是否存储point个数 类型 unsigned int ,当值为0时表示不存储，为1表示存储，目前用于pure txt的配置
	keLaserFileParam_SaveSwingAngle,
	keLaserFileParamLast
} EVzLaserFileParam;

/// @brief 
/// 触发输出模式
typedef enum 
{
	keStrobeTriggerOutMode_None = 0,	//< 不控制触发输出 ： 默认
	keStrobeTriggerOutMode_CameraLink,	//< 跟随主模式输出
	keStrobeTriggerOutMode_UserControl,	//< 用户控制
	keStrobeTriggerOutMode_LoopBack,	//< 回环控制
	keStrobeTriggerOutMode_ServoControl,//< 舵机控制
} EVzStrobeTriggerOutMode;

/// @brief 
/// Sensor 是否为长焦镜头
enum EVzSensorLenType
{
	keSensorLenType_Invalid = 0,
	keSensorLenType_3mm,
	keSensorLenType_6mm,
};

/// @brief 
/// 镜头盖住的状态类型
enum EVzServoStatusType
{
	keServoStatusType_Invalid = -1,
	keServoStatusType_Cover,
	keServoStatusType_UnCover,
};

/// @brief 
/// 编码器参数类型
enum EVzEncodeParamType
{
	keEncodeParamType_RollerResolution = 0,		//< 使用 编码器分辨率 + 编码器半径参数 模式
	keEncodeParamType_DistancePerPulse,			//< 使用 编码器每两个脉冲的间距
};

/// @brief
/// 配置多相机触发模式
enum EVzMultiDevWorkMode
{
	keMultiDevWorkMode_MasterSlave					= 0,		///< 主从触发
	keMultiDevWorkMode_Ext_FallingEdge				= 1,		///< 外部下降沿触发
	keMultiDevWorkMode_Ext_HighFrequenceFallingEdge = 2,		///< 外部高帧率下降沿触发
};

/// @brief
/// 配置融合的相机类型
enum EVzMultiDevType
{
	keMultiDev_Dynanmic = 0,		///< 动态相机融合
	keMultiDev_Static = 1,			///< 静态相机融合
};

/// @brief
/// 配置静态相机融合扫描模式
enum EVzMultiStaticDevScanMode
{
	keMultiStaticDevScanMode_OneByOne = 0,		///< 一个接一个的扫描
	keMultiStaticDevScanMode_SameTime = 1,		///< 同时扫描
};

/// @brief
/// 编码器方向
enum EVzEncoderRollingDirection
{
	keEncoderRollingDirection_Forward = 0,
	keEncoderRollingDirection_Backward
};


/// @brief
/// 旋转图像类型
enum EVzImageRotateType
{
	keImageRotateType_Invalid = -1,
	keImageRotateType_FlipY_Rotate = 0,
	keImageRotateType_ClockWise = 1,
	keImageRotateType_CounterClockWise = 2,
	keImageRotateType_90ClockWise = 3,
	keImageRotateType_FlipX = 4,
};

typedef struct
{
	unsigned int nEncodeNo[2];
	EVzEncoderRollingDirection eDirection;
	unsigned int nFrequency[2];
} SVzEncoderData;

enum
{
	keEmulationDetectType_AutoDetect = 0,
	keEmulationDetectType_Detect,
};
using EVzEmulationDetectType = unsigned int;

/// @brief
/// 旋转图像类型
enum EVzDustCoverConfigType
{
	keDustCoverConfigType_Large = 0,
	keDustCoverConfigType_Medium,
	keDustCoverConfigType_Small,
};

/// @brief
//环境类型
typedef enum
{
	keEnvType_Invalid = -1,
	keEnvType_InDoor = 0,
	keEnvType_OutDoor = 1,
} EVzEnvType;

/// @brief 范围类型 int
typedef struct
{
	EVzEnvType eEnvType;
	unsigned int nLevel;
} SVzEnvLevel;

/// @brief
//抗反光增益
typedef enum
{
	keAntiReflectGainType_Close = 1,
	keAntiReflectGainType_Double = 2,
	keAntiReflectGainType_Quadruple = 4,
	keAntiReflectGainType_Sextuple = 6,
	keAntiReflectGainType_Octuple = 8,
} EVzAntiReflectGainType;

typedef enum
{
	keAntiReflectCmpsDirType_LeftToRight = 0,
	keAntiReflectCmpsDirType_RightToLeft = 1,
} EVzAntiReflectCmpsDirType;

typedef struct
{
	unsigned int		m_nCheckTimeInterval;
	unsigned int		m_nCheckTimeOut;
	unsigned int		m_nCheckExposeRatio;
}SVzAutoCoverControlParam;

typedef struct
{
	char chIP[VZNL_SDK_NETWORK_IPv4_LENGTH];
}SVzNLDeviceIp;

typedef enum
{
	keLRSensorType_Gray = 0,
	keLRSensorType_Color = 1,
} EVzLRSensorType;

typedef enum
{
	keRGBOutputChannelType_R = 0,
	keRGBOutputChannelType_G = 1,
	keRGBOutputChannelType_B = 2,
	keRGBOutputChannelType_All = 3,
} EVzRGBOutputChannelType;

/// @brief
// 多相机融合时各子设备融合配置参数
typedef struct
{
	unsigned int nDevIdx;       // 融合索引：0：主设备；>0：从设备；
	double dFusionMatrix[3][4]; // 融合矩阵
	double dYRange[2];          // 当前设备融合时Y轴过滤范围
}SVzNLDeviceFusionData;

// RGBHDR 出图模式
typedef enum
{
	keRGBHDROutImageMode_Ori = 0,		// 输出原始图像：背景图 + 熔池图
	keRGBHDROutImageMode_HDR = 1,		// 输出HDR 叠加后的图像：HDR图
	keRGBHDROutImageMode_MPool = 2,		// 只输出熔池图像
	keRGBHDROutImageMode_BG = 3,		// 只输出背景图
	keRGBHDROutImageMode_Num			// 模式个数
} EVzRGBHDROutImageMode;

/**
* @brief 点云<->2D伪彩图转换参数结构体
*/
typedef struct
{
	unsigned int nEdgeMargin;
	double d3DXMin;
	double d3DXMax;
	double d3DYMin;
	double d3DYMax;
	double dPixelResolution;
}SVzPColorImageConvertInfo;

/**
* @brief 2D伪彩图坐标<->3D坐标选取方法枚举
*/
typedef enum
{
	kePColorImageTo3DPoint_NearestPoint = 0,	// 输出距离回算点X/Y 最近的原始点云3D 点坐标。
	kePColorImageTo3DPoint_RealPoint			// 输出实际回算点X/Y坐标 + 最邻近原始点的Z坐标。
}SVzPColorImageTo3DPointMethod;

/// @brief
/// 采集图像模式
typedef enum
{
	keCaptureMode_LR_Image = 0,			// 输出双目图像
	keCaptureMode_C_Image,				// 输出RGB图像
	keCaptureMode_LRC_Image,			// 输出双目+RGB图像
}EVzCaptureMode;

/// @brief 焊缝跟踪点云扫描配置参数（存储到相机内）
typedef struct
{
	// 可配参数部分
	bool bIsEnableCalibROI;				// 是否启用校正图
	EVzNLExposeMode eExposeMode;		// 曝光模式
	unsigned int nExposeTime;			// 曝光值
	unsigned short nLeftCameraGain;		// 左目增益
	unsigned short nRightCameraGain;	// 右目增益
	unsigned short nDynamicScanAngle;	// 动态扫描时摆动机构角度
	unsigned short nFltrRgnTh;			// 小信号过滤值
	SVzNLROIRect sLeftROI;				// 左目ROI
	SVzNLROIRect sRightROI;				// 右目ROI
	unsigned int nFrameRate;			// 采集帧率
	unsigned int nOutputFrameRate;		// 输出帧率：对接收到的激光线进一步抽样输出(0：不进行抽帧处理)
	// 固定参数部分(SDK 内部自动设置)，无需修改
	bool bIsSwingEnable;				// 是否使能摆动机构，固定为false
	bool bIsRGBEnable;					// 是否使能RGB，固定为false
	bool bIsStrobeEnable;				// 是否使能C版功能，固定为true
	bool bIsAntiReflectGainEnable;		// 是否使能抗强光增益，固定为true
} SVzTrackingScanPara;


typedef struct{
	int denoiseStrength;		// 去噪强度
	int sharpenStrength;		// 锐化强度
	int brightness;				// 亮度
	int contrast;				// 对比度
	bool useAdvancedDenoise;	// 使用高级去噪
}AlgorithmParams;

/**
* @brief 多相机融合操作，重叠区数据处理方法
*/
typedef enum
{
	keMulDevOverlayProcessType_SeqFilter = 0,		// 按Y轴顺序过滤重叠的点云
	keMulDevOverlayProcessType_PointsFilter,		// 按重叠区域内点数据个数过滤数据少的相机的点云
	keMulDevOverlayProcessType_DataFusion,			// 对重叠区域内的点数据进行数据融合，补齐缺失的数据
}EVzMulDevOverlayProcessType;


// Feature标志位定义
typedef enum 
{
	KE_FEATURE_RGB_CAMERA = 0x01,			// RGB相机
	KE_FEATURE_SWING_MECHANISM = 0x02,		// 摆动机构
	KE_FEATURE_DUAL_OUTPUT = 0x04,			// 双端出图
	KE_FEATURE_STEREO_COLOR = 0x08,			// 双目彩色
	KE_FEATURE_WELDING_CAMERA = 0x10			// 焊接相机
}EVzFeatureFlags;

// Feature数据结构
typedef struct 
{
	unsigned short	flags;			// 特性标志位
	unsigned short	pupilDistance;	// 相机瞳距配置
}SVzFeatureData;

/*
* 命令操作类型定义
*/
enum EVzUSBSerialRelayCmdType
{
	keUSBSerialRelayCmdType_CloseNoAck = 0,       // 关闭不反馈
	keUSBSerialRelayCmdType_OpenNoAck,           // 打开不反馈
	keUSBSerialRelayCmdType_CloseWithAck,        // 关闭并反馈
	keUSBSerialRelayCmdType_OpenWithACck,        // 打开并反馈
	keUSBSerialRelayCmdType_ReverseWithAck,       // 取反并反馈
	keUSBSerialRelayCmdType_CheckState,          // 查询状态
	keUSBSerialRelayCmdType_FlashBreakWithAck,   // 闪断并反馈
	keUSBSerialRelayCmdType_Num
};

/*
* 串口波特率枚举
*/
enum EVzSerialPortBaudRate
{
	keSerialPortBaudRate_1200 = 0,
	keSerialPortBaudRate_2400,
	keSerialPortBaudRate_4800,
	keSerialPortBaudRate_9600,
	keSerialPortBaudRate_19200,
	keSerialPortBaudRate_38400,
	keSerialPortBaudRate_57600,
	keSerialPortBaudRate_115200,
	keSerialPortBaudRate_Num
};

/*
* 串口数据长度枚举(4~8bits)
*/
enum EVzSerialPortByteSize
{
	keSerialPortByteSize_4 = 0,
	keSerialPortByteSize_5,
	keSerialPortByteSize_6,
	keSerialPortByteSize_7,
	keSerialPortByteSize_8,
	keSerialPortByteSize_Num
};

/*
* 串口数据校验位枚举(0-4=None,Odd,Even,Mark,Space)
*/
enum EVzSerialPortParity
{
	keSerialPortParity_None = 0,
	keSerialPortParity_Odd,
	keSerialPortParity_Even,
	keSerialPortParity_Mark,
	keSerialPortParity_Space,
	keSerialPortParity_Num
};

/*
* 串口数据停止位枚举(0,1,2 = 1, 1.5, 2)
*/
enum EVzSerialPortStopBits
{
	keSerialPortStopBits_1 = 0,
	keSerialPortStopBits_1_5,
	keSerialPortStopBits_2,
	keSerialPortStopBits_Num
};

/*
* 串口配置参数结构
*/
typedef struct
{
	EVzSerialPortBaudRate eBaudRate;	// 串口波特率
	EVzSerialPortByteSize eByteSize;	// 串口波特率
	EVzSerialPortParity eParity;		// 串口校验位
	EVzSerialPortStopBits eStopBits;	// 串口停止位
}SVzSerialPortCfgPara;

/*
* USB 继电器设备参数
*/
typedef struct
{
	unsigned int nChannelNum;					// 模块控制的通道个数
	SVzSerialPortCfgPara sSerialPortCfgPara;	// 串口控制参数
}SVzUSBSerialRelayCfgPara;

/*
* 搜索输出的信息结构，目前只包括设备端口名称
*/
typedef struct
{
	char sPortName[VZNL_DEVICE_NAME_LENGTH];
}SVzSerialPortInfo;

/*
* 
*/
typedef struct
{
	char szName[VZ_USERCONFIG_NAME_LENGTH];
	char szDesc[VZ_USERCONFIG_DESC_LENGTH];
}SVzUserConfigInfo;


#endif //__VIZUM_TYPES_HEADER_
