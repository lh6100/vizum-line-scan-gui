#ifndef __VIZUM_SWING_MOTOR_HEADER__
#define __VIZUM_SWING_MOTOR_HEADER__

#include "VZNL_Export.h"
#include "VZNL_Types.h"

/**
 * @brief 是否支持摆动机构
 * @param [in] hDevice		设备句柄
 * @param [in] pnErrorCode	错误码
 * @return 支持摆动机构返回VzTrue, 否则返回VzFalse
 */
VZNLAPI VzBool VzNL_IsSupportSwingMotor(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置工作范围
 * @param [in] hDevice			设备句柄
 * @param [in] dNearDistance	近距离
 * @param [in] dFarDistance		远距离
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI int VzNL_SetSwingMotorWorkRange(VZNLHANDLE hDevice, double dNearDistance, double dFarDistance);
VZNLAPI int VzNL_GetSwingMotorWorkRange(VZNLHANDLE hDevice, double* pdNearDistance, double* pdFarDistance);

/**
 * @brief 是否启用摆动机构
 * @param [in] hDevice 设备句柄
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI int VzNL_EnableSwingMotor(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableSwingMotor(VZNLHANDLE hDevice, int* pnErrorCode);

/**
* @brief 打开/关闭激光器
* @param [in] hDevice 设备句柄
* @param [in] bEnable 启用/禁用
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_EnableLaserLight(VZNLHANDLE hDevice, VzBool bEnable);
VZNLAPI VzBool VzNL_IsEnableLaserLight(VZNLHANDLE hDevice);

/**
* @brief 调节激光器亮度
* @param [in] hDevice 设备句柄
* @param [in] bEnable 启用/禁用
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_SetLaserLight(VZNLHANDLE hDevice, unsigned int nLight);
VZNLAPI unsigned int VzNL_GetLaserLight(VZNLHANDLE hDevice, int* pnErrorCode);

/**
* @brief 设置/获取角速度
* @param [in] hDevice 设备句柄
* @param [in] bEnable 启用/禁用
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_SetSwingAngleSpeed(VZNLHANDLE hDevice, int nAngleSpeed);
VZNLAPI int VzNL_GetSwingAngleSpeed(VZNLHANDLE hDevice, int* pnAngleSpeed);
VZNLAPI int VzNL_GetSwingAngleSpeedRange(VZNLHANDLE hDevice, int* pnMinAngleSpeed, int* pnMaxAngleSpeed);

/**
* @brief 设置/获取角速度
* @param [in] hDevice 设备句柄
* @param [in] bEnable 启用/禁用
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_SetSwingMotorAngle(VZNLHANDLE hDevice, int nStartPos, int nEndPos);
VZNLAPI int VzNL_GetSwingMotorAngle(VZNLHANDLE hDevice, int* pnStartPos, int* pnEndPos);

/**
* @brief 设置当前位置为开始/结束位置
* @param [in] hDevice 设备句柄
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_SetSwingCurPosToMotorStartPos(VZNLHANDLE hDevice);
VZNLAPI int VzNL_SetSwingCurPosToMotorEndPos(VZNLHANDLE hDevice);

/**
* @brief 旋转到某个角度
* @param [in] hDevice 设备句柄
* @param [in] nAngle 角度
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_RotateSwing(VZNLHANDLE hDevice, float fAngle);

/**
* @brief 旋转到开始位置
* @param [in] hDevice 设备句柄
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_RotateSwingToStartPos(VZNLHANDLE hDevice);

/**
* @brief 设置扫描模式
* @param [in] hDevice 设备句柄
* @param [in] eScanMode 扫描模式
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_SetSwingScanMode(VZNLHANDLE hDevice, EVzSwingMotorScanMode eScanMode);
VZNLAPI EVzSwingMotorScanMode VzNL_GetSwingScanMode(VZNLHANDLE hDevice);

/**
* @brief 获取摆动机构参数
* @param [in] hDevice 设备句柄
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_GetSwingMotorInfo(VZNLHANDLE hDevice, SVzSwingMotorDevInfo* psSwingMotorInfo);

/**
* @brief 获取当前角度
* @param [in] hDevice 设备句柄
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI float VzNL_GetSwingCurAngle(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置停顿时间
 * @param [in] hDevice 设备句柄
 * @param [in] nMillionSecond 停顿时间 范围 [500 ~ 32767]
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI int VzNL_SetSwingStopTime(VZNLHANDLE hDevice, unsigned int nMillionSecond);

/**
 * @brief 左移
 * @param [in] hDevice 设备句柄
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI int VzNL_SwingMoveLeft(VZNLHANDLE hDevice);

/**
 * @brief 右移
 * @param [in] hDevice 设备句柄
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI int VzNL_SwingMoveRight(VZNLHANDLE hDevice);

/**
 * @brief 设置相机摆动方向
 * @param [in] hDevice 设备句柄
 * @param [in] eRotateDirect 摆动方向[keSwingRotateDirect_Clockwise 顺时针方向摆动, keSwingRotateDirect_Counterclockwise 逆时针方向摆动]
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI int VzNL_SetSwingRotateDirect(VZNLHANDLE hDevice, EVzSwingRotateDirect eRotateDirect);
VZNLAPI int VzNL_GetSwingRotateDirect(VZNLHANDLE hDevice, EVzSwingRotateDirect* peRotateDirect);


/**
 * @brief 获取版本
 * @param [in] hDevice 设备句柄
 * @return 返回版本号
 */
VZNLAPI unsigned int VzNL_GetSwingVersionCode(VZNLHANDLE hDevice);

/**
 * @brief 设置相机激光器停止位置
 * @param [in] hDevice 设备句柄
 * @param [in] fAngle 停止角度
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI int VzNL_SetSwingStopAngle(VZNLHANDLE hDevice, int nAngle);
VZNLAPI int VzNL_GetSwingStopAngle(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 设置判断完成了一次扫描的角度阈值
 * @param [in] hDevice 设备句柄
 * @param [in] fAngle 角度阈值
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI int VzNL_SetSwingEndAngleThres(VZNLHANDLE hDevice, float fAngle);
VZNLAPI float VzNL_GetSwingEndAngleThres(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 是否支持静态相机输出激光线时告知是否完成了一次扫描
 * @param [in] hDevice 设备句柄
 * @return 返回0表示正确
 * @retval 0 表示成功
 * @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
 */
VZNLAPI VzBool VzNL_IsSupportEndAngleThres(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 查找摆动机构模块
 * @param [in] hDevice 设备句柄
 * @param [in] pnErrorCode	错误码
 * @return 找到摆动机构返回VzTrue, 否则返回VzFalse
 */
VZNLAPI VzBool VzNL_FindSwing(VZNLHANDLE hDevice, int* pnErrorCode);

/// @brief 重启摆动模块
VZNLAPI int VzNL_RebootSwing(VZNLHANDLE hDevice);

#endif