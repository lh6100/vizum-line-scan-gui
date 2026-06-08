#ifndef __VIZUM_EXPORT_HEADER__
#define __VIZUM_EXPORT_HEADER__

#if defined(_WIN32)
	#ifdef VIZUMSDK_EXPORTS
		#ifdef __cplusplus
			#define VZNLAPI extern "C" __declspec(dllexport)
		#else
			#define VZNLAPI __declspec(dllexport)
		#endif
	#else
		#ifdef __cplusplus
			#define VZNLAPI extern "C" __declspec(dllimport)
		#else
			#define VZNLAPI __declspec(dllimport)			
		#endif
		#ifdef _DEBUG
			#pragma comment(lib, "VzNLDetect.lib")
		#else
			#pragma comment(lib, "VzNLDetect.lib")
		#endif
	#endif
#else
	#ifndef VZNLAPI
            #ifdef __cplusplus
                #define VZNLAPI extern "C" __attribute__((visibility("default")))
            #else
                #define VZNLAPI __attribute__((visibility("default")))
            #endif
	#endif
#endif

#endif
