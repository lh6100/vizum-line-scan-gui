#pragma once

enum
{
	keErrorCode_Success										= 0,							//< 正常
	keErrorCode_Unknown										= -1,							//< 未知错误

	keErrorCode_AppStart									= -60000,						//< 应用层错误起始码
	keErrorCode_App_NoCreate								= -60000,						//< 没有创建Application
	keErrorCode_App_Not_Support_Any_Instance				= -59999,						//< 不支持多个实例
	keErrorCode_App_InvalidParam							= -59998,						//< 无效的接口参数
	keErrorCode_App_ConvertDataFailed						= -59997,						//< 转换数据失败
	keErrorCode_App_NoDetectObject							= -59996,						//< 没检测物体
	keErrorCode_App_InvalidHandle							= -59995,						//< 无效的句柄
	keErrorCode_App_NoneAlgo								= -59994,						//< 无算法	
	keErrorCode_App_nullptr									= -59993,						//< 空指针
	keErrorCode_App_NoBaseLine								= -59992,						//< 没有基准线
	keErrorCode_App_Lock_Failed								= -59991,						//< 锁失败
	keErrorCode_App_No_BeginWork							= -59990,						//< 开始工作
	keErrorCode_App_Invalid_3DPoint							= -59989,						//< 无效的3D数据
	keErrorCode_App_NoAttach_Device							= -59988,						//< 没有绑定设备
	keErrorCode_App_CRC_No_Match							= -59987,						//< CRC校验失败
	keErrorCode_App_Data_Length_No_Match					= -59986,						//< 数据长度不一致
	keErrorCode_App_Param_Is_Null							= -59985,						//< 参数为空
	keErrorCode_App_Invalid_FrameRate						= -59984,						//< 无效帧率
	keErrorCode_AppEnd										= -50000,						//< 应用层错误结束码

	keErrorCode_NetworkStart								= -70000,						//< 网络错误码
	keErrorCode_NetworkConnectFailed						= -69999,						//< 网络连接失败
	keErrorCode_Network_Init_Failed							= -69998,						//< 初始化网络失败
	keErrorCode_Network_ARP_Bind_Failed						= -69997,						//< 绑定失败
	keErrorCode_Network_EmptySendDataBuffer					= -69996,						//< 空的发送数据
	keErrorCode_Network_NetworkAbnormal						= -69995,						//< 网络异常
	keErrorCode_Network_SendDataFailed						= -69994,						//< 发送数据失败
	keErrorCode_Network_TimeOut								= -69993,						//< 网络超时
	keErrorCode_Network_Invalid_SubMask						= -69992,						//< 无效的子网掩码
	keErrorCode_Network_Invalid_IPAddress					= -69991,						//< 无效的IP地址
	keErrorCode_Network_Invalid_GW							= -69990,						//< 无效的网关地址
	keErrorCode_Network_ChangeIp_Need_Same_Ethernet			= -69989,						//< 当前网络内存在多台智光眼设备，请将以太网IP改为同一网段后配置
	keErrorCode_Network_Ip_Conflict							= -69988,						//< IP 冲突
	keErrorCode_Network_Speed_Warning						= -69987,						//< 不是千兆网速
	keErrorCode_NetworkEnd									= -60000,						//< 网络错误码

	keErrorCode_DeviceStart									= -80000,						//< 设备错误码
	keErrorCode_Device_NoSupport							= -80000,						//< 不支持的设备/功能
	keErrorCode_Device_InitError							= -79999,						//< 初始化错误
	keErrorCode_Device_NotFoundOpenDevice					= -79998,						//< 没有找到打开的设备
	keErrorCode_Device_TimeOut								= -79997,						//< 超时
	keErrorCode_Device_Offline								= -79996,						//< 设备离线
	keErrorCode_Device_Invalid								= -79995,						//< 无效设备
	keErrorCode_Device_EyeResError							= -79994,						//< 设备眼睛的分辨率不对
	keErrorCode_Device_Param_Invalid						= -79993,						//< 参数无效
	keErrorCode_Device_No_MemSpace							= -79992,						//< 没有合适的buffer供我返回
	keErrorCode_Device_ParseData_Failed						= -79991,						//< 数据解析失败
	keErrorCode_Device_Invalid_Detect_Type					= -79990,						//< 无效的检测类型
	keErrorCode_Device_Result_Empty							= -79989,						//< 结果为空
	keErrorCode_Device_CoreHandle_Empty						= -79988,						//< 激光眼为空	
	keErrorCode_Device_AutoDetecting						= -79987,						//< 正在检测中
	keErrorCode_Device_StandardFailed						= -79986,						//< 标定失败
	keErrorCode_Device_NoStandardData						= -79985,						//< 没有标定数据
	keErrorCode_Device_SetHostDataFailed					= -79984,						//< 设置Host数据失败
	keErrorCode_Device_CreateDevice							= -79983,						//< 创建设备失败
	keErrorCode_Device_InvalidData							= -79982,						//< 无效数据
	keErrorCode_Device_InvalidDetectTool					= -79980,						//< 无效的检测工具
	keErrorCode_Device_CaptureResultEmpty					= -79979,						//< 采集结果为空
	keErrorCode_Device_CallBackEmpty						= -79978,						//< 没有注册Callback
	keErrorCode_Device_DataError							= -79977,						//< 设备数据错误
	keErrorCode_Device_Version								= -79976,						//< 设备版本错误
	keErrorCode_Device_DeviceDontEnough						= -79975,						//< 设备不可用
	keErrorCode_Device_ResultNotEnough						= -79974,						//< 结果不可用
	keErrorCode_Device_NotFoundHost							= -79973,						//< 没有找到
	keErrorCode_Device_SameDevice							= -79972,						//< 设备相同
	keErrorCode_Device_Capturing							= -79971,						//< 正在采集中
	keErrorCode_Device_NotStartCapture						= -79970,						//< 没有开始采集
	keErrorCode_Device_DisableReq							= -79969,						//< 禁止请求
	keErrorCode_Device_Busy									= -79968,						//< 设备忙
	keErrorCode_Device_NotFoundCmd							= -79967,						//< 没有找到命令
	keErrorCode_Device_RecvCmdDataErr						= -79966,						//< 接收命令数据错误
	keErrorCode_Device_ImageLevelErr						= -79965,						//< 图像等级错误
	keErrorCode_Device_NotFound								= -79964,						//< 没有找到设备
	keErrorCode_Device_GetResultFailed						= -79963,						//< 获取结果失败
	keErrorCode_Device_No_Device							= -79962,						//< 没有设备
	keErrorCode_Device_Query_Thres_Failed					= -79961,						//< 获取门限值失败
	keErrorCode_Device_NotSupportParam						= -79960,						//< 不支持的参数
	keErrorCode_Device_QuerySurfaceFailed					= -79959,						//< 获取图像失败
	keErrorCode_Device_SetGapFailed							= -79958,						//< 设置Gap值失败
	keErrorCode_Device_Invalid_Type							= -79957,						//< 无效的类型
	keErrorCode_Device_Compare_Value						= -79956,						//< 比较值
	keErrorCode_Device_Invalid_ParseData					= -79955,						//< 无效的数据
	keErrorCode_Device_Invalid_ParseDataLengthError			= -79954,						//< 数据长度错误
	keErrorCode_Device_QueryResultEmpty						= -79953,						//< 获取结果为空
	keErrorCode_Device_ParseDataFailed						= -79952,						//< 解析数据失败
	keErrorCode_Device_DataMatchFailed						= -79951,						//< 数据匹配错误
	keErrorCode_Device_RequestMemFailed						= -79950,						//< 请求内存失败
	keErrorCode_Device_Invalid_DeviceInfo					= -79949,						//< 无效的设备信息
	keErrorCode_Device_Invalid_NetCardIndex					= -79948,						//< 无效的网卡索引
	keErrorCode_Device_Invalid_Param_Range					= -79945,						//< 无效的参数范围
	keErrorCode_Device_Invalid_Buffer_Size_Not_Enough		= -79944,						//< Buffer大小不够
	keErrorCode_Device_SwingMotor_Err						= -79943,						//< 摆动机构错误
	keErrorCode_Device_SwingMotor_Busy						= -79942,						//< 摆动机构忙碌
	keErrorCode_Device_SwingMotor_Encoder_Err				= -79941,						//< 编码器错误
	keErrorCode_Device_SwingMotor_Motor_Driver_Protect_Err	= -79940,						//< 摆动机构触发电机保护
	keErrorCode_Device_Invalid_Device_Info					= -79939,						//< 无效设备信息
	keErrorCode_Device_Not_Found_NetCard_Info				= -79938,						//< 没有找到网卡
	keErrorCode_Device_SwingMotor_NoCalib_Err				= -79936,						//< 摆动机构没有标定
	keErrorCode_Device_Not_Found_Inner_Lib					= -79935,						//< 没有找到内部库
	keErrorCode_Device_No_Function							= -79934,						//< 无此功能
	keErrorCode_Device_KernelLengthNotMatch					= -79933,						//< 长度不匹配
	keErrorCode_Device_DataLengthNotEnough					= -79932,						//< 数据长度不正常
	keErrorCode_Device_Invalid_Param						= -79931,						//< 无效的参数
	keErrorCode_Device_Not_Setting							= -79930,						//< 没有设置
	keErrorCode_Device_Data_Length_NoMatch					= -79929,						//< 数据长度不匹配
	keErrorCode_Device_Recv_Empty_Result					= -79928,
	keErrorCode_Device_Recv_Lable_Failed					= -79927,						//< 接收标签错误
	keErrorCode_Device_Recv_Image_Failed					= -79926,						//< 图像接收错误
	keErrorCode_Device_ConfigIP_Failed						= -79925,						//< 配置IP错误
	keErrorCode_Device_Invalid_ROI_Range					= -79924,						//< ROI范围无效
	keErrorCode_Device_NoSupport_Current_ROI				= -79923,						//< 不支持当前ROI
	keErrorCode_Device_NoCreate_TCP_Execute					= -79922,						//< 创建TCP执行器失败
	keErrorCode_Device_No_Write								= -79921,						//< 当前设备不能为写
	keErrorCode_Device_Cancel_Calibration					= -79920,						//< 取消标定
	keErrorCode_Device_Err_MapData							= -79919,						//< 错误的Map数据
	keErrorCode_Device_SwingMotor_Update_Err				= -79918,	
	keErrorCode_Device_NoSupport_RGB                        = -79917,                       //< 没有RGB设备
	keErrorCode_Device_Swing_SPI_Broken						= -79916,                       //< 摆动模块 SPI通信异常
	keErrorCode_Device_Param_dynamic_Failed					= -79915,
	keErrorCode_Device_Need_Reboot							= -79914,
	keErrorCode_Device_RGBDisable							= -79913,						//< RGB 相机未使能
	keErrorCode_Device_TrackingScan_CalibROI			    = -79912,						//< 焊缝跟踪参数：使能图像校正错误
	keErrorCode_Device_TrackingScan_ROISize					= -79911,						//< 焊缝跟踪参数：ROI尺寸错误
	keErrorCode_Device_TrackingScan_FrameRate				= -79910,						//< 焊缝跟踪参数：采集帧率错误
	keErrorCode_Device_TrackingScan_OutputFrameRate			= -79909,						//< 焊缝跟踪参数：输出帧率错误
	keErrorCode_Device_SwingMotor_Encoder_Exception			= -79908,						//< 编码器异常
	keErrorCode_Device_SwingMotor_NoSupport_Cover			= -79907,						//< 未开启翻盖功能
	keErrorCode_Device_SwingMotor_Swing_Runing				= -79906,						//< 自摆动正在运行，不能控制翻盖
	keErrorCode_Device_SwingMotor_Opened					= -79905,						//< 已经在打开状态
	keErrorCode_Device_SwingMotor_Closed					= -79904,						//< 已经在关闭状态
	keErrorCode_Device_SwingMotor_Cover_Runing				= -79903,						//< 翻盖电机正在运行
	keErrorCode_Device_SwingMotor_Cover_Err					= -79902,						//< 翻盖电机错误
	keErrorCode_Device_SwingMotor_Cover_OverSpeed_Err		= -79901,						//< 过速报警
	keErrorCode_Device_SwingMotor_Cover_AbnormalCurrent_Err = -79900,						//< 电流异常
	keErrorCode_Device_SwingMotor_Cover_CriticalCurrent_Err = -79899,						//< 电流严重异常
	keErrorCode_Device_SwingMotor_Cover_Chip_Err			= -79898,						//< 芯片异常错误
	keErrorCode_Device_Over_Limit							= -79897,						//< 超出限制
	keErrorCode_DeviceEnd									= -70000,						//< 设备错误码

	keErrorCode_OtherStart									= -90000,
	keErrorCode_Invalid_File_Param							= -89999,						// 无效文件参数
	keErrorCode_Match_Obj_Failed							= -89998,						// 文件对比失败
	keErrorCode_Invalid_File								= -89997,						// 无效文件
	keErrorCode_Create_Detect_Tool_Failed					= -89996,						// 创建检测工具失败
	keErrorCode_Query_Detect_Tool_Failed					= -89995,						// 获取检测工具失败
	keErrorCode_QueryConfigObjectFailed						= -89994,						// 获取配置对象失败
	keErrorCode_NoSupportDataType							= -89799,						// 不支持的数据类型
	
	keErrorCode_NoSupport_Image_Compress					= -89988,						// 不支持的图像压缩方式
	keErrorCode_Image_Invalid_Graphics						= -89987,						// 无效的绘图工具
	keErrorCode_Image_SaveFile_Failed						= -89986,						// 图像文件存储失败
	keErrorCode_Image_Convert_Image_Failed					= -89985,						// 图像转换失败
	keErrorCode_Image_FillImageFailed						= -89984,						// 填充图像失败
	keErrorCode_Image_CompressImageFailed					= -89983,						// 图像压缩失败

	keErrorCode_File_NoAttach								= -89982,						// 没有Attach文件
	keErrorCode_File_CanNotFind								= -89981,						// 文件没有找到
	keErrorCode_File_MD5_NoMatch							= -89980,						// MD5不匹配
	keErrorCode_File_EOF									= -89979,						// 到了文件末尾
	keErrorCode_File_Err_Oper								= -89978,						// 错误操作
	keErrorCode_File_Can_Not_Found_Flag						= -89977,
	keErrorCode_File_Error_Line_Flag						= -89976,
	keErrorCode_File_Open_File_Failed						= -89975,
	keErrorCode_File_Handle_Is_Null							= -89974,
	keErrorCode_File_Invalid_Param							= -89973,
	keErrorCode_File_ReadLen_NoMatch						= -89972,
	keErrorCode_File_ReadPos_Err							= -89971,
	keErrorCode_File_Create_File_Failed						= -89970,
	keErrorCode_File_Data_Head_Err							= -89969,
	keErrorCode_File_File_Guid_Err							= -89968,
	keErrorCode_File_No_Support								= -89967,
	keErrorCode_File_No_Support_Param						= -89966,

	keErrorCode_EncoderStart								= -82000,						// 
	keErrorCode_EncoderStop									= -81000,
	
	keErrorCode_Hardware_Calib_MapData_Err					= -80040,						// 矫正数据错误
	keErrorCode_Hardware_FIFO_CAN_NOT_WRITE					= -80039,						// FIFO 不可写
	keErrorCode_Hardware_Ethernet_Busy						= -80038,						// 网络忙碌
	keErrorCode_Hardware_Swing_Interrupt_Busy				= -80037,						// 摆动模块中断忙碌
	keErrorCode_Hardware_Secure_Failed						= -80036,						//
	keErrorCode_Hardware_IsCapturing						= -80035,						// 当前在开流过程
	keErrorCode_Hardware_Sensor_Current_TriggerMode_NoSupport= -80034,						// 当前触发模式不支持
	keErrorCode_Hardware_Sensor_Command_Not_Found			= -80033,						// 命令未找到
	keErrorCode_Hardware_Sensor_SetROI_Map_Failed			= -80032,						// 设置ROI MAP失败
	keErrorCode_Hardware_Sensor_SetROI_Failed				= -80031,						// 设置ROI失败
	keErrorCode_Hardware_Sensor_SetSensor_Failed			= -80030,						// 设置Sensor失败
	keErrorCode_Hardware_EMMC_Address_Align_Failed			= -80029,						// EMMC 地址对齐错误
	keErrorCode_Hardware_EMMC_Read_Failed					= -80028,						// EMMC 读取失败
	keErrorCode_Hardware_EMMC_Write_Failed					= -80027,						// EMMC 写入失败
	keErrorCode_Hardware_EMMC_Init_Failed					= -80026,						// EMMC 初始化失败
	keErrorCode_Hardware_ReadData_Version_Failed			= -80025,						// 读取数据版本失败
	keErrorCode_Hardware_ReadData_Failed					= -80024,						// 读取数据失败
	keErrorCode_Hardware_Check_RW_Data_NoMatch				= -80023,						// 
	keErrorCode_Hardware_TimeOut							= -80022,						// 硬件没有回应
	keErrorCode_Hardware_Not_Found_Register					= -80021,						// 没有找到寄存器
	keErrorCode_Hardware_Check_CRC_Failed					= -80020,						// CRC 校验失败
	keErrorCode_Hardware_ROI_Incorrect						= -80019,						// ROI 错误
	keErrorCode_Hardware_Trans_Unknown_Upload_Failed		= -80018,						// 传输未知错误
	keErrorCode_Hardware_Trans_Offset_Failed				= -80017,						// 传输偏移错误
	keErrorCode_Hardware_Lock_Failed						= -80016,						// 锁失败
	keErrorCode_Hardware_File_Seek_Failed					= -80015,						// Seek文件失败
	keErrorCode_Hardware_File_Open_Failed					= -80014,						// 打开文件失败
	keErrorCode_Hardware_File_Not_Begin						= -80013,						// 文件没有开始
	keErrorCode_Hardware_Write_File_Failed					= -80012,						// 写文件失败
	keErrorCode_Hardware_Close_File_Failed					= -80011,						// 关闭文件失败
	keErrorCode_Hardware_Query_FileInfo_Failed				= -80010,						// 获取文件信息失败
	keErrorCode_Hardware_FileSize_No_Match					= -80009,						// 文件长度不匹配
	keErrorCode_Hardware_MD5_No_Match						= -80008,						// MD5不匹配
	keErrorCode_Hardware_Out_Of_Memory						= -80007,						// 没有空间
	keErrorCode_Hardware_File_Oper_Err						= -80006,						// 文件操作失败
	keErrorCode_Hardware_Param_Invalid						= -80005,						// 无效的参数
	keErrorCode_Hardware_ReadOnly							= -80004,						// 只读 
	keErrorCode_Hardware_ParamLength_NoMatch				= -80003,						// 参数长度不匹配
	keErrorCode_Hardware_NoSupport							= -80002,						// 硬件不支持此功能
	keErrorCode_HardwareStart								= -81000,
	keErrorCode_HardWareEnd									= -80000,						// 结束

	keErrorCode_OtherEnd									= -80000,

	keErrorCode_CustomStart									= -100000,

	keErrorCode_POINTCLOUDTO2DIMAGE_INPUTPARAISNULL			= -99999,   // 输入数据指针为空
	keErrorCode_POINTCLOUDTO2DIMAGE_IMAGESIZENOTCORRECT		= -99998,	// 图像尺寸不正确
	keErrorCode_POINTCLOUDTO2DIMAGE_IMAGEEDGEMARGIN			= -99997,	// 图像边缘留白值不正确
	keErrorCode_POINTCLOUDTO2DIMAGE_INPUTDATANOTENOUGH		= -99996,	// 输入数据不足
	keErrorCode_POINTCLOUDTO2DIMAGE_NOMEM					= -99995,	// 分配内存失败
	keErrorCode_POINTCLOUDTO2DIMAGE_POINTFORMATISNOTSUPPORT = -99994,   // 点数据格式不支持
	keErrorCode_POINTCLOUDTO2DIMAGE_PIXELRESOLUTIONNOTCORRECT = -99993, // 输入的fWHPixelResolution 太小了
    keErrorCode_Utils_DataNotMatch							= -99992,   // 数据不匹配
	keErrorCode_Utils_2DPointNotInImage						= -99991,   // 输入的2D 点作标不在图像内
	keErrorCode_Utils_NotFindMatched3DPoint					= -99991,   // 没在点云指定范围内找到匹配的3D 点坐标
	keErrorCode_CustomEnd									= -90000,
};

/*
* 定义USB 串口继电器操作错误码
*/
enum EVzUSBSerialRelayOpErrCode
{
	keUSBSerialRelayOpErr_Ok = 0,					// 没有错误
	keUSBSerialRelayOpErr_ParaNull,					// 参数为空
	keUSBSerialRelayOpErr_OpenSerial,				// 打开串口失败
	keUSBSerialRelayOpErr_CloseSerial,				// 关闭串口失败
	keUSBSerialRelayOpErr_NotDevice,				// 没有找到设备
	keUSBSerialRelayOpErr_NotOpen,					// 串口未打开
	keUSBSerialRelayOpErr_NoRcvData,				// 没有接收到数据
	keUSBSerialRelayOpErr_WaitRcvTimeOut,			// 等待接收数据超时
	keUSBSerialRelayOpErr_SendData,					// 发送数据失败
	keUSBSerialRelayOpErr_CmdAckNotValid,			// 接收命令反馈无效
	keUSBSerialRelayOpErr_ChNotSupport,				// 输入通道不知道
};
