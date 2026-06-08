#ifndef __VIZUM_DUST_COVER_HEADER__
#define __VIZUM_DUST_COVER_HEADER__

#include "VZNL_Export.h"
#include "VZNL_Types.h"

/**
 * @brief 是否支持防飞溅翻盖控制的功能
 * @param hDevice 当前设备句柄
 * @param pnErrorCode 错误信息，如果不需要可填NULL
 * @return VzTrue 表示支持 VzFalse 表示不支持
 */
VZNLAPI VzBool VzNL_IsSupportCoverCamera(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 防飞溅翻盖控制
 * @param hDevice		[in] 设备Handle
 * * @param bCover		[in] 是否开关盖
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_CoverCamera(VZNLHANDLE hDevice, VzBool bCover);

/**
 * @brief
 * 开关盖状态
 * VzTrue 表示关闭 VzFalse 表示打开
 */
VZNLAPI VzBool VzNL_IsCoverCamera(VZNLHANDLE hDevice, int* pnErrorCode);


/** @brief 开启/关闭防飞溅翻盖自动控制功能
 *  @param hDevice		[in] 设备Handle
 *  @param bEnable		[in] 是否自动控制防飞溅翻盖
 *  @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_EnableAutoCoverControl(VZNLHANDLE hDevice, VzBool bEnable);

/**
 * @brief 是否开启了防飞溅翻盖控制的功能
 * @param hDevice 当前设备句柄
 * @param pnErrorCode 错误信息，如果不需要可填NULL
 * @return VzTrue 表示支持 VzFalse 表示不支持
 */
VZNLAPI VzBool VzNL_IsEnableAutoCoverControl(VZNLHANDLE hDevice, int* pnErrorCode);


/**
 * @brief 是否支持IO控制防飞溅翻盖的功能
 * @param hDevice 当前设备句柄
 * @param pnErrorCode 错误信息，如果不需要可填NULL
 * @return VzTrue 表示支持 VzFalse 表示不支持
 */
VZNLAPI VzBool VzNL_IsSupportIOControl(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 开启/关闭IO控制防飞溅翻盖的功能
 * @param hDevice 当前设备句柄
 * @param bEnable VzTrue 表示开启 VzFalse 表示关闭
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_EnableIOControl(VZNLHANDLE hDevice, VzBool bEnable);

/**
 * @brief 是否开启了IO控制防飞溅翻盖的功能
 * @param hDevice 当前设备句柄
 * @param pnErrorCode 错误信息，如果不需要可填NULL
 * @return VzTrue VzTrue 表示开启 VzFalse 表示关闭
 */
VZNLAPI VzBool VzNL_IsEnableIOControl(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 配置/获取IO控制消抖参数
 * @param hDevice 当前设备句柄
 * @param nDbcTh  消抖参数
 * @return 正确返回0， 失败返回其他值
 */
VZNLAPI int VzNL_SetIOControlDbcTh(VZNLHANDLE hDevice, unsigned int nDbcTh);
VZNLAPI unsigned int VzNL_GetIOControlDbcTh(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 是否支持应用防飞溅翻盖大中小配置的功能
 * @param hDevice 当前设备句柄
 * @param pnErrorCode 错误信息，如果不需要可填NULL
 * @return VzTrue 表示支持 VzFalse 表示不支持
 */
VZNLAPI VzBool VzNL_IsSupportApplyDustCoverConfig(VZNLHANDLE hDevice, int* pnErrorCode);

/**
 * @brief 根据类型使用防飞溅翻盖配置
 * @param hDevice 当前设备句柄
 * @param [in] eConfigType 保存的配置类型
 * @return 成功返回0，否则为错误码。
 */
VZNLAPI int VzNL_ApplyDustCoverConfigType(VZNLHANDLE hDevice, EVzDustCoverConfigType eConfigType);
/**
 * @brief 获取当前使用防飞溅翻盖配置的类型
 * @param hDevice 当前设备句柄
 * @param [out] pnErrorCode 成功返回0，否则为错误码。
 * @return 当前使用的配置类型
 */
VZNLAPI EVzDustCoverConfigType VzNL_QueryDustCoverConfigType(VZNLHANDLE hDevice, int* pnErrorCode);

#endif