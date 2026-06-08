/*
 * Header: VZNL_Graphics.h
 * Description:当前文件为伟景智能(Vizum)所研发的EyeCB板提供绘图函数接口。
 *             目前提供了绘制矩形，圆形，线等方法
 *          
 * Sample:     // 在图像左上角绘制一个长宽各100像素的空心红色矩形。
 *             VZNLHANDLE hGraphics = VzNL_BeginPaint();
 * 
 *             SVzNLRect sRect = {0, 0, 100, 100};
 *             VzNL_DrawRect(hGraphics, &sRect, 0xffff0000， 0, 1);
 * 
 *             VzNL_EndPaint(hGraphics);
 * Author: Mjw
 * Date:   2018/08/28
 */

#ifndef __VIZUM_GRAPHICS_HEADER__
#define __VIZUM_GRAPHICS_HEADER__

#include "VZNL_Types.h"

#if defined(_WIN32)
	#ifdef VZNL_GRAPHICS_EXPORTS
		#ifdef __cplusplus
			#define VZNL_GRAPHICS_API extern "C" __declspec(dllexport)
		#else
			#define VZNL_GRAPHICS_API __declspec(dllexport)
		#endif
	#else
		#ifdef __cplusplus
			#define VZNL_GRAPHICS_API extern "C" __declspec(dllimport)
		#else
			#define VZNL_GRAPHICS_API __declspec(dllimport)			
		#endif
		#ifdef _DEBUG
			#pragma comment(lib, "VzNLGraphics.lib")
		#else
			#pragma comment(lib, "VzNLGraphics.lib")
		#endif
	#endif
#else
	#ifndef VZNL_GRAPHICS_API
            #ifdef __cplusplus
                #define VZNL_GRAPHICS_API extern "C" __attribute__((visibility("default")))
            #else
                #define VZNL_GRAPHICS_API __attribute__((visibility("default")))
            #endif
	#endif
#endif

/// @brief
/// 开始绘图
/// <param name = "pImageData">[in]传入数据buffer</param>
/// @return 返回绘图工具handle
VZNL_GRAPHICS_API VZNLHANDLE VzNL_BeginPaint(SVzNLImageData* pImageData);

/// @brief
/// 绘制矩形
/// <param name = "handle">[in]绘图工具handle。</param>
/// <param name = "pRect">[in]要绘制的位置。</param>
/// <param name = "dwFrameColor">[in]边框颜色</param>
/// <param name = "dwFillColor">[in]填充色</param>
/// <param name = "nBorder">[in]边框宽度</param>
/// @return 绘制完成返回true
VZNL_GRAPHICS_API int VzNL_DrawRect(VZNLHANDLE hHandle, const SVzNL2DPoint sPoint[4], unsigned long dwFrameColor, unsigned long dwFillColor, unsigned int nBorder);

/// @brief
/// 绘制多边形
/// <param name = "handle">[in]绘图工具handle。</param>
/// <param name = "pRect">[in]要绘制的位置。</param>
/// <param name = "dwFrameColor">[in]边框颜色</param>
/// <param name = "dwFillColor">[in]填充色</param>
/// <param name = "nBorder">[in]边框宽度</param>
/// @return 绘制完成返回true
VZNL_GRAPHICS_API int VzNL_DrawPolygon(VZNLHANDLE hHandle, const SVzNL2DPoint* psPoint, const unsigned int nPtCount, unsigned long dwFrameColor, unsigned long dwFillColor, unsigned int nBorder);

/// @brief
/// 绘制圆形
/// <param name = "handle">[in]绘图工具handle。</param>
/// <param name = "pPoint">[in]中点位置。</param>
/// <param name = "nRadius">[in]半径。</param>
/// <param name = "dwFrameColor">[in]边框颜色</param>
/// <param name = "dwFillColor">[in]填充色</param>
/// <param name = "nBorder">[in]边框宽度</param>
/// @return 绘制完成返回true
VZNL_GRAPHICS_API int VzNL_DrawCircle(VZNLHANDLE hHandle, const SVzNL2DPoint sPoint, int nRadius, unsigned long dwFrameColor, unsigned long dwFillColor, unsigned int nBorder);


/// @brief
/// 绘制椭圆
/// <param name = "handle">[in]绘图工具handle。</param>
/// <param name = "pPoint">[in]中点位置。</param>
/// <param name = "nRadius">[in]半径。</param>
/// <param name = "dwFrameColor">[in]边框颜色</param>
/// <param name = "dwFillColor">[in]填充色</param>
/// <param name = "nBorder">[in]边框宽度</param>
/// @return 绘制完成返回true
VZNL_GRAPHICS_API int VzNL_DrawEllipe(VZNLHANDLE hHandle, const SVzNL2DPoint sPoint, unsigned long dwFrameColor, unsigned long dwFillColor, unsigned int nLongAxes, unsigned int nShortAxes, double angle, unsigned int nBorder);

/// @brief
/// 绘制直线
/// <param name = "handle">[in]绘图工具handle。</param>
/// <param name = "pPoint[2]">[in]直线的两个点位置。</param>
/// @return 绘制完成返回true
VZNL_GRAPHICS_API int VzNL_DrawLine(VZNLHANDLE hHandle, unsigned long dwColor, const SVzNL2DPoint sPoint[2], unsigned int nBorder);

/// @brief
/// 绘制文字
/// <parma name = "text">[in]文字内容</param>
/// <parma name = "palce">[in]坐上起点</param>
/// <parma name = "fontSize">[in]字体大小</param>
/// <parma name = "clrTxt">[in]字体颜色</param>
VZNL_GRAPHICS_API int VzNL_DrawText(VZNLHANDLE hHandle, const char* text, SVzNL2DPoint& palce, int fontSize, unsigned long clrTxt);


/// @brief
/// 结束绘图
/// <param name = "handle">[in]绘图工具handle。</param>
VZNL_GRAPHICS_API void VzNL_EndPaint(VZNLHANDLE hHandle);

/// @brief
/// 显示一个图像（linux因为版本比较繁杂，支持的界面库不同，所以这个函数linux目前不可用）
/// <param name = "szWindowName">[in]窗口名</param>
/// <param name = "pImageData">[in]压缩图像数据结构</param>
VZNL_GRAPHICS_API void VzNL_ShowImage(const char* szWindowName, const SVzNLImageData* pImageData);

/// @brief
/// 存储图像。
/// <param name = "szFileName">[in]文件全路径</param>
/// <param name = "pImageData">[in]压缩图像数据结构</param>
/// @return 成功返回0，失败返回错误码
VZNL_GRAPHICS_API int VzNL_SaveImage(const char* szFileName, const SVzNLImageData* pImageData);

/// @brief
/// 存储图像。
/// <param name = "szFileName">[in]文件全路径</param>
/// <param name = "pImageData">[in]压缩图像数据结构</param>
/// @return 成功返回0，失败返回错误码
VZNL_GRAPHICS_API int VzNL_SaveScaleImage(const char* szFileName, SVzNLImageData* pImageData, SVzVideoResolution* psResolution);

/// @brief
/// 压缩图片
/// <param name = "handle">[in]绘图工具handle。</param>
/// <param name = "eCompress">[in]压缩类型。</param>
/// <param name = "pSrc">[in]数据源（无压缩）。</param>
/// <param name = "pDst">[out]压缩后的数据。</param>
/// @return 成功返回0，失败返回错误码
VZNL_GRAPHICS_API int VzNL_CompressImage(EVzNLImageCompress eCompress, SVzNLImageData* pSrc, SVzNLCompressData** pDst);

/// @brief
/// 释放压缩图片
VZNL_GRAPHICS_API void VzNL_ReleaseCompressImage(SVzNLCompressData** pDst);

#endif // __VIZUM_GRAPHICS_HEADER__