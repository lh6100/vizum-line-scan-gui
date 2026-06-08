#ifndef __VIZUM_APP_UTILS_HEADER__
#define __VIZUM_APP_UTILS_HEADER__

#include "VZNL_Export.h"
#include "VZNL_Types.h"

/**
 * @brief 读取用户文件列表 当前仅支持智光眼
 * @param hDevice 设备Handle
 * @param szUserProfileName [out] 获取用户配置名
 * @param nProfileCount [out] 获取文件个数
 * @return 返回名称个数
 */
VZNLAPI int VzNL_ReadProfileNameList(VZNLHANDLE hDevice, char szUserProfileName[VZ_USERPROFILE_MAX_COUNT][VZ_USERPROFILE_NAME_LENGTH]);

/**
 * @brief 保存用户配置(共10个) 当前仅支持智光眼
 * @param hDevice 设备Handle
 * @param nProfileIdx 文件Index
 * @param lpszName 名称
 */
VZNLAPI int VzNL_SaveUserSetting(VZNLHANDLE hDevice, unsigned int nProfileIdx, const char* lpszName);

/**
 * @brief 保存用户配置到文件 当前仅支持智光眼
 * @param hDevice 设备Handle
 * @param lpszFile 文件
 */
VZNLAPI int VzNL_SaveUserSettingToFile(VZNLHANDLE hDevice, const char* lpszFile);

/**
 * @brief 恢复用户配置(共10个) 当前仅支持智光眼
 * @param hDevice 设备Handle
 * @param nProfileIdx 文件Index
 */
VZNLAPI int VzNL_RestoreUserSettingFromIndex(VZNLHANDLE hDevice, unsigned int nProfileIdx);

/**
 * @brief 恢复用户配置(共10个) 当前仅支持智光眼
 * @param hDevice 设备Handle
 * @param lpszName 名称
 */
VZNLAPI int VzNL_RestoreUserSettingFromName(VZNLHANDLE hDevice, const char* lpszName);

/**
 * @brief 恢复用户配置(共10个) 当前仅支持智光眼
 * @param hDevice 设备Handle
 * @param lpszFile 文件全路径
 */
VZNLAPI int VzNL_RestoreUserSettingFromFile(VZNLHANDLE hDevice, const char* lpszFile);

/**
 * @brief 恢复系统默认配置
 * @param hDevice 设备Handle
 */
VZNLAPI int VzNL_RestoreDefaultSetting(VZNLHANDLE hDevice);

#endif //__VIZUM_APP_UTILS_HEADER__