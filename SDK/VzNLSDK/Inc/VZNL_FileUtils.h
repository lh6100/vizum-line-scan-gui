#pragma once

#include "VZNL_Types.h"
#include "VZNL_Export.h"

/**
 * @brief 创建文件工具
 * @return 文件类型
 */
VZNLAPI EVzLaserFileType VzNL_IsSupportLaserType(const char* lpszFile);

/**
 * @brief 创建文件工具
 * @param [in] eFileType 文件类型
 * keLaserFileType_PureTxt 		纯文本
 * keLaserFileType_Txt			本公司自用
 * keLaserFileType_Pcd			PCD
 * keLaserFileType_Las			LAS
 * keLaserFileType_VzBinaryData Binary File
 * @return 返回文件句柄
 */
VZNLAPI VZNLFILE VzNL_CreateLaserFile(EVzLaserFileType eFileType);

/**
* @brief 文件参数类型
* @param [in] eLaserFileParam 文件参数
* @param [in] pData 数据
* @param [in] nDataLength 数据长度
* @return 返回0表示正确
* @retval 0 表示成功
* @retval 非0 表示失败,可以使用VzNL_GetErrorInfo获取
*/
VZNLAPI int VzNL_SetLaserFileParam(VZNLFILE hFile, EVzLaserFileParam eLaserFileParam, const unsigned char* pData, unsigned int nDataLength);

/**
 * @brief 打开文件
 * @param hFile		[in] 文件句柄
 * @param lpszFile	[in] 文件名
 * @param eFileMode [in] 文件打开模式
 * @param eDataType [in] 数据类型
 * @param dSpeed	[in] 速度 mm/s
 * @return 返回值为0表示正确
 */ 
VZNLAPI int VzNL_OpenLaserFile(VZNLFILE hFile, const char* lpszFile, EVzFileMode eFileMode, EVzResultDataType eDataType);

/**
 * @brief 写入文件
 * @param hFile		[in] 文件句柄
 * @param psLaserLinePoint	[in] 激光线数据
 * @return 返回值为0表示正确
 */
VZNLAPI int VzNL_WriteLaserFile(VZNLFILE hFile, const SVzLaserLineData* psLaserLinePoint);

/**
 * @brief 读取文件
 * @param hFile		[in] 文件句柄
 * @param pnReadLineCnt	[in] 读取的激光线个数
 * @return 返回激光线数据
 */
VZNLAPI SVzLaserLineData* VzNL_ReadLaserFile(VZNLFILE hFile, int* pnReadLineCnt);

/**
 * @brief 关闭文件
 * @param hFile		[in] 文件句柄
 * @return 返回值为0表示正确
 */
VZNLAPI int VzNL_CloseLaserFile(VZNLFILE hFile);