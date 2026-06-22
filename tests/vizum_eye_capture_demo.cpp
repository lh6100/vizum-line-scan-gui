#include "VZNL_Common.h"
#include "VZNL_DetectConfig.h"
#include "VZNL_DustCover.h"
#include "VZNL_EyeConfig.h"
#include "VZNL_ExtStrobeLaser.h"
#include "VZNL_Graphics.h"
#include "VZNL_Types.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

namespace {

std::string errString(int code) {
    char buf[256] = {0};
    VzNL_GetErrorInfo(code, buf);
    std::ostringstream out;
    out << "[" << code << "] " << buf;
    return out.str();
}

bool ensureDir(const std::string& dir) {
    if (dir.empty()) {
        return false;
    }
    if (::mkdir(dir.c_str(), 0775) == 0 || errno == EEXIST) {
        return true;
    }
    return false;
}

std::string nowStamp() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return buf;
}

cv::Mat grayMatFromVzImage(const SVzNLImageData* image) {
    if (!image || !image->pBuffer || image->nWidth == 0 || image->nHeight == 0) {
        return {};
    }
    const int width = static_cast<int>(image->nWidth);
    const int height = static_cast<int>(image->nHeight);
    const int channels = static_cast<int>(image->nChannels);
    if (channels == 1) {
        return cv::Mat(height, width, CV_8UC1, image->pBuffer).clone();
    }
    if (channels == 3) {
        cv::Mat bgr(height, width, CV_8UC3, image->pBuffer);
        cv::Mat gray;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        return gray;
    }
    return {};
}

std::string statsString(const cv::Mat& gray) {
    if (gray.empty()) {
        return "empty";
    }
    double minVal = 0.0;
    double maxVal = 0.0;
    cv::minMaxLoc(gray, &minVal, &maxVal);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(gray, mean, stddev);

    int histSize = 256;
    float range[] = {0.0f, 256.0f};
    const float* ranges[] = {range};
    cv::Mat hist;
    cv::calcHist(&gray, 1, 0, cv::Mat(), hist, 1, &histSize, ranges);
    const double total = static_cast<double>(gray.total());
    double acc = 0.0;
    int p99 = 0;
    for (int i = 0; i < 256; ++i) {
        acc += hist.at<float>(i);
        if (acc >= total * 0.99) {
            p99 = i;
            break;
        }
    }
    const int nonzero = cv::countNonZero(gray);
    std::ostringstream out;
    out << "min=" << minVal
        << " max=" << maxVal
        << " mean=" << std::fixed << std::setprecision(2) << mean[0]
        << " std=" << stddev[0]
        << " p99=" << p99
        << " nonzero=" << nonzero << "/" << gray.total();
    return out.str();
}

void saveImageSet(const std::string& prefix, const SVzNLImageData* image) {
    if (!image) {
        std::cout << prefix << ": null image" << std::endl;
        return;
    }
    const std::string rawPath = prefix + "_raw.png";
    int rc = VzNL_SaveImage(rawPath.c_str(), image);
    std::cout << prefix << ": save raw " << rawPath << " rc=" << errString(rc) << std::endl;

    cv::Mat gray = grayMatFromVzImage(image);
    std::cout << prefix << ": "
              << image->nWidth << "x" << image->nHeight
              << " type=" << static_cast<int>(image->eImageType)
              << " bitDepth=" << static_cast<int>(image->byBitDepth)
              << " channels=" << static_cast<int>(image->nChannels)
              << " " << statsString(gray) << std::endl;
    if (!gray.empty()) {
        const std::string grayPath = prefix + "_gray.png";
        cv::imwrite(grayPath, gray);
        cv::Mat enhanced;
        cv::normalize(gray, enhanced, 0, 255, cv::NORM_MINMAX);
        const std::string enhancedPath = prefix + "_enhanced.png";
        cv::imwrite(enhancedPath, enhanced);
    }
}

bool ipEquals(const char* sdkIp, const std::string& ip) {
    return ip.empty() || ip == sdkIp;
}

SVzNLEyeCBInfo* findDevice(std::vector<SVzNLEyeCBInfo>& devices, const std::string& ip) {
    for (auto& d : devices) {
        if (d.bValidDevice && ipEquals(d.byServerIP, ip)) {
            return &d;
        }
    }
    for (auto& d : devices) {
        if (ipEquals(d.byServerIP, ip)) {
            return &d;
        }
    }
    return nullptr;
}

bool searchDevices(std::vector<SVzNLEyeCBInfo>* devices) {
    devices->clear();
    bool keep = true;
    int loop = 0;
    while (keep && loop++ < 5) {
        keep = false;
        int rc = VzNL_ResearchDevice(keSearchDeviceFlag_EthLaserRobotEye);
        std::cout << "ResearchDevice: " << errString(rc) << std::endl;
        int count = 0;
        rc = VzNL_GetEyeCBDeviceInfo(nullptr, &count);
        std::cout << "GetEyeCBDeviceInfo count rc=" << errString(rc) << " count=" << count << std::endl;
        if (count <= 0) {
            continue;
        }
        devices->assign(static_cast<size_t>(count), SVzNLEyeCBInfo{});
        rc = VzNL_GetEyeCBDeviceInfo(devices->data(), &count);
        std::cout << "GetEyeCBDeviceInfo data rc=" << errString(rc) << " count=" << count << std::endl;
        devices->resize(static_cast<size_t>(count));

        for (auto& d : *devices) {
            std::cout << "  found ip=" << d.byServerIP << " valid=" << static_cast<int>(d.bValidDevice)
                      << " name=" << d.szDeviceName << std::endl;
            if (d.bValidDevice) {
                continue;
            }
            SVzNLEthernetEyeConfigInfo eth{};
            rc = VzNL_GetEthernetEyeConfigInfo(&d, &eth);
            if (rc != 0) {
                std::cout << "  GetEthernetEyeConfigInfo: " << errString(rc) << std::endl;
                continue;
            }
            if (std::memcmp(eth.byLocalIP, eth.sNetCardInfo.byLocalIP, 4) == 0) {
                std::cout << "  skip bind: local IP matches NIC IP" << std::endl;
                continue;
            }
            rc = VzNL_BindEthernetEye(&d);
            std::cout << "  BindEthernetEye: " << errString(rc) << std::endl;
            if (rc == 0) {
                keep = true;
            }
        }
    }
    return !devices->empty();
}

void configureFullRoi(VZNLHANDLE h) {
    VzNL_SetOutputROIImage(VzFalse);
#ifdef VIZUM_HAS_TRANSPICMODE
    int rc = VzNL_EnableTransPicMode(h, VzTrue);
    std::cout << "EnableTransPicMode(true): " << errString(rc) << std::endl;
#else
    int rc = 0;
    std::cout << "EnableTransPicMode: unavailable in this SDK" << std::endl;
#endif

    int supportRc = 0;
    if (VzNL_IsSupportCaptureMode(h, &supportRc) == VzTrue) {
        rc = VzNL_SetCaptureMode(h, keCaptureMode_LR_Image);
        std::cout << "SetCaptureMode(LR): " << errString(rc) << std::endl;
        int modeRc = 0;
        EVzCaptureMode mode = VzNL_GetCaptureMode(h, &modeRc);
        std::cout << "GetCaptureMode: " << errString(modeRc) << " mode=" << static_cast<int>(mode) << std::endl;
    } else {
        std::cout << "SetCaptureMode unsupported: " << errString(supportRc) << std::endl;
    }

    rc = VzNL_EnableCalibROI(h, VzFalse);
    std::cout << "EnableCalibROI(false): " << errString(rc) << std::endl;
    rc = VzNL_ConfigDynamicROI(h, VzFalse);
    std::cout << "ConfigDynamicROI(false): " << errString(rc) << std::endl;

    SVzVideoResolution res{};
    rc = VzNL_GetResolution(h, &res);
    std::cout << "GetResolution: " << errString(rc) << " " << res.nFrameWidth << "x" << res.nFrameHeight << std::endl;
    if (rc == 0 && res.nFrameWidth > 0 && res.nFrameHeight > 0) {
        SVzNLROIRect left{0, static_cast<int>(res.nFrameWidth), 0, static_cast<int>(res.nFrameHeight)};
        SVzNLROIRect right = left;
        rc = VzNL_ConfigDetectROIWithFormat(h, &left, &right);
        std::cout << "ConfigDetectROIWithFormat(full): " << errString(rc) << std::endl;
        SVzNLROIRect actualL{};
        SVzNLROIRect actualR{};
        int getRc = VzNL_GetConfigDetectROI(h, &actualL, &actualR);
        std::cout << "GetConfigDetectROI: " << errString(getRc)
                  << " L=[" << actualL.left << "," << actualL.right << "," << actualL.top << "," << actualL.bottom << "]"
                  << " R=[" << actualR.left << "," << actualR.right << "," << actualR.top << "," << actualR.bottom << "]"
                  << std::endl;
    }
}

void configureWindowsLikeMode(VZNLHANDLE h) {
    int rc = VzNL_SetTriggerMode(h, keEyeTriggerMode_Master);
    std::cout << "SetTriggerMode(Master): " << errString(rc) << std::endl;
    int modeRc = 0;
    const EVzEyeTriggerMode triggerMode = VzNL_GetTriggerMode(h, &modeRc);
    std::cout << "GetTriggerMode: " << errString(modeRc)
              << " mode=" << static_cast<int>(triggerMode) << std::endl;

    rc = VzNL_IgnoreTriggerExtSignal(h, VzTrue);
    std::cout << "IgnoreTriggerExtSignal(true): " << errString(rc) << std::endl;
    int ignoreRc = 0;
    const VzBool ignoreExt = VzNL_IsIgnoreTriggerExtSignal(h, &ignoreRc);
    std::cout << "IsIgnoreTriggerExtSignal: " << errString(ignoreRc)
              << " ignore=" << static_cast<int>(ignoreExt) << std::endl;

    rc = VzNL_EnableTriggerHwExtEn(h, VzFalse);
    std::cout << "EnableTriggerHwExtEn(false): " << errString(rc) << std::endl;
    int extRc = 0;
    const VzBool hwExt = VzNL_IsEnableTriggerHwExtEn(h, &extRc);
    std::cout << "IsEnableTriggerHwExtEn: " << errString(extRc)
              << " enabled=" << static_cast<int>(hwExt) << std::endl;

    rc = VzNL_SetStrobeTriggerOutMode(h, keStrobeTriggerOutMode_None);
    std::cout << "SetStrobeTriggerOutMode(None): " << errString(rc) << std::endl;
    int strobeRc = 0;
    const EVzStrobeTriggerOutMode strobeMode = VzNL_GetStrobeTriggerOutMode(h, &strobeRc);
    std::cout << "GetStrobeTriggerOutMode: " << errString(strobeRc)
              << " mode=" << static_cast<int>(strobeMode) << std::endl;

    rc = VzNL_SetAntiReflectGainType(h, keAntiReflectGainType_Close);
    std::cout << "SetAntiReflectGainType(Close): " << errString(rc) << std::endl;
    int antiRc = 0;
    const EVzAntiReflectGainType anti = VzNL_GetAntiReflectGainType(h, &antiRc);
    std::cout << "GetAntiReflectGainType: " << errString(antiRc)
              << " type=" << static_cast<int>(anti) << std::endl;
}

void configureExposure(VZNLHANDLE h, int fps, int exposure, int gain) {
    unsigned int minFps = 0;
    unsigned int maxFps = 0;
    VzNL_QueryParamRange(h, keDeviceParamType_FrameRate, &minFps, &maxFps);
    int rc = VzNL_EnableFreeFrameRate(h, VzTrue);
    std::cout << "EnableFreeFrameRate(true): " << errString(rc) << std::endl;
    rc = VzNL_SetFrameRate(h, fps);
    std::cout << "SetFrameRate(" << fps << ") range=[" << minFps << "," << maxFps << "]: " << errString(rc) << std::endl;
    int actualFps = 0;
    rc = VzNL_GetFrameRate(h, &actualFps);
    std::cout << "GetFrameRate: " << errString(rc) << " fps=" << actualFps << std::endl;

    rc = VzNL_ConfigEyeExpose(h, keVzNLExposeMode_Fix, static_cast<unsigned int>(exposure));
    std::cout << "ConfigEyeExpose(fix," << exposure << "): " << errString(rc) << std::endl;
    EVzNLExposeMode mode = keVzNLExposeMode_Fix;
    unsigned int actualExposure = 0;
    rc = VzNL_GetConfigEyeExpose(h, &mode, &actualExposure);
    std::cout << "GetConfigEyeExpose: " << errString(rc)
              << " mode=" << static_cast<int>(mode) << " exposure=" << actualExposure << std::endl;

    rc = VzNL_SetCameraGain(h, keEyeSensorType_Left, static_cast<unsigned short>(gain));
    std::cout << "SetCameraGain(left," << gain << "): " << errString(rc) << std::endl;
    unsigned short actualGain = 0;
    rc = VzNL_GetCameraGain(h, keEyeSensorType_Left, &actualGain);
    std::cout << "GetCameraGain(left): " << errString(rc) << " gain=" << actualGain << std::endl;

    rc = VzNL_SetCameraGain(h, keEyeSensorType_Right, static_cast<unsigned short>(gain));
    std::cout << "SetCameraGain(right," << gain << "): " << errString(rc) << std::endl;
    actualGain = 0;
    rc = VzNL_GetCameraGain(h, keEyeSensorType_Right, &actualGain);
    std::cout << "GetCameraGain(right): " << errString(rc) << " gain=" << actualGain << std::endl;
}

void logLeftEyeCalibration(VZNLHANDLE h) {
    double calib[14] = {};
    int rc = VzNL_QueryLeftEyeCalibData(h, calib);
    std::cout << "QueryLeftEyeCalibData: " << errString(rc);
    if (rc == 0) {
        std::cout << " K=["
                  << calib[0] << "," << calib[1] << "," << calib[2] << ";"
                  << calib[3] << "," << calib[4] << "," << calib[5] << ";"
                  << calib[6] << "," << calib[7] << "," << calib[8] << "]"
                  << " D=[" << calib[9] << "," << calib[10] << "," << calib[11]
                  << "," << calib[12] << "," << calib[13] << "]";
    }
    std::cout << std::endl;
}

void logImageCalibMatrix(VZNLHANDLE h) {
    double q[16] = {};
    int rc = VzNL_GetImageCalibMatrix(h, q);
    std::cout << "GetImageCalibMatrix: " << errString(rc);
    if (rc == 0) {
        const double cx = -q[3];
        const double cy = -q[7];
        const double f = q[11];
        std::cout << " Q=[";
        for (int r = 0; r < 4; ++r) {
            if (r > 0) {
                std::cout << ";";
            }
            for (int c = 0; c < 4; ++c) {
                if (c > 0) {
                    std::cout << ",";
                }
                std::cout << q[r * 4 + c];
            }
        }
        std::cout << "] K_from_Q=[" << f << ",0," << cx << ";0," << f << "," << cy << ";0,0,1]"
                  << " D=[0,0,0,0,0 for rectified image]";
    }
    std::cout << std::endl;
}

bool turnOnExtLightIfSupported(VZNLHANDLE h) {
    int rc = 0;
    if (VzNL_IsSupportExtLight(h, &rc) != VzTrue) {
        std::cout << "ExtLight unsupported: " << errString(rc) << std::endl;
        return false;
    }

    rc = VzNL_EnableAutoControlExtLight(h, VzFalse);
    std::cout << "EnableAutoControlExtLight(false): " << errString(rc) << std::endl;

    rc = VzNL_TurnOnExtLight(h, VzTrue);
    std::cout << "TurnOnExtLight(true): " << errString(rc) << std::endl;

    int stateRc = 0;
    const VzBool on = VzNL_IsTurnOnExtLight(h, &stateRc);
    std::cout << "IsTurnOnExtLight: " << errString(stateRc)
              << " on=" << static_cast<int>(on) << std::endl;
    return rc == 0 || on == VzTrue;
}

void turnOffExtLight(VZNLHANDLE h) {
    int rc = 0;
    if (VzNL_IsSupportExtLight(h, &rc) != VzTrue) {
        return;
    }
    rc = VzNL_TurnOnExtLight(h, VzFalse);
    std::cout << "TurnOnExtLight(false): " << errString(rc) << std::endl;
}

struct CaptureContext {
    std::string prefix;
    std::atomic<int> count{0};
    int maxCount{3};
};

void onImage(SVzNLImageData* left, SVzNLImageData* right, SVzNLImageData* center,
             const SVzOutputFrameProps* props, void* param) {
    (void)center;
    (void)props;
    auto* ctx = static_cast<CaptureContext*>(param);
    const int index = ctx->count.fetch_add(1);
    if (index >= ctx->maxCount) {
        return;
    }
    std::ostringstream leftPrefix;
    leftPrefix << ctx->prefix << "_capture_" << index << "_left";
    std::ostringstream rightPrefix;
    rightPrefix << ctx->prefix << "_capture_" << index << "_right";
    saveImageSet(leftPrefix.str(), left);
    saveImageSet(rightPrefix.str(), right);
}

} // namespace

int main(int argc, char** argv) {
    std::string ip = "192.168.1.105";
    std::string outDir = "data";
    int fps = 1;
    int exposure = 12000;
    int gain = 1;
    int samples = 3;
    bool runCapture = true;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--ip" && i + 1 < argc) {
            ip = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            outDir = argv[++i];
        } else if (arg == "--fps" && i + 1 < argc) {
            fps = std::atoi(argv[++i]);
        } else if (arg == "--exposure" && i + 1 < argc) {
            exposure = std::atoi(argv[++i]);
        } else if (arg == "--gain" && i + 1 < argc) {
            gain = std::atoi(argv[++i]);
        } else if (arg == "--samples" && i + 1 < argc) {
            samples = std::max(1, std::atoi(argv[++i]));
        } else if (arg == "--capture" && i + 1 < argc) {
            runCapture = std::atoi(argv[++i]) != 0;
        } else {
            std::cout << "Usage: " << argv[0]
                      << " [--ip 192.168.1.105] [--fps 1] [--exposure 12000] [--gain 1]"
                      << " [--samples 3] [--capture 1] [--out data]\n";
            return 1;
        }
    }

    ensureDir(outDir);
    const std::string prefix = outDir + "/eye_demo_" + nowStamp();
    std::cout << "Output prefix: " << prefix << std::endl;

    SVzNLConfigParam cfg{};
    cfg.nDeviceTimeOut = 0;
    int rc = VzNL_Init(&cfg);
    std::cout << "VzNL_Init: " << errString(rc) << std::endl;
    if (rc != 0) {
        return 2;
    }
    VzNL_SetLogLevel(keNLLogLevel_Information, keNLLogType_File);

    std::vector<SVzNLEyeCBInfo> devices;
    if (!searchDevices(&devices)) {
        std::cout << "No devices found." << std::endl;
        VzNL_Destroy();
        return 3;
    }
    SVzNLEyeCBInfo* picked = findDevice(devices, ip);
    if (!picked) {
        std::cout << "Device IP not found: " << ip << std::endl;
        VzNL_Destroy();
        return 4;
    }
    std::cout << "Opening device ip=" << picked->byServerIP << std::endl;
    SVzNLOpenDeviceParam openParam{};
    int openRc = 0;
    VZNLHANDLE h = VzNL_OpenDevice(picked, &openParam, &openRc);
    std::cout << "OpenDevice: " << errString(openRc) << " handle=" << h << std::endl;
    if (openRc != 0 || !h) {
        VzNL_Destroy();
        return 5;
    }

    if (VzNL_IsSupportCoverCamera(h, nullptr) == VzTrue) {
        rc = VzNL_CoverCamera(h, VzFalse);
        std::cout << "CoverCamera(open): " << errString(rc) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        int stateRc = 0;
        VzBool covered = VzNL_IsCoverCamera(h, &stateRc);
        std::cout << "IsCoverCamera: " << errString(stateRc)
                  << " covered=" << static_cast<int>(covered) << std::endl;
    } else {
        std::cout << "CoverCamera unsupported" << std::endl;
    }

    configureFullRoi(h);
    configureWindowsLikeMode(h);
    configureExposure(h, fps, exposure, gain);
    logLeftEyeCalibration(h);
    logImageCalibMatrix(h);
    const bool extLightOn = turnOnExtLightIfSupported(h);
    if (extLightOn) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    VzNL_SetOutputImageFormat(keVzNLImageType_GRAY);
    std::cout << "SetOutputImageFormat(GRAY)" << std::endl;
    const unsigned int timeoutMs = static_cast<unsigned int>(std::max(2500, static_cast<int>(std::ceil(4000.0 / fps))));
    for (int i = 0; i < samples; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(std::ceil(1000.0 / fps))));
        SVzNLImageData* left = nullptr;
        SVzNLImageData* right = nullptr;
        rc = VzNL_GetEyeImage(h, &left, &right, timeoutMs);
        std::cout << "GetEyeImage #" << i << ": " << errString(rc) << std::endl;
        if (rc == 0) {
            std::ostringstream leftPrefix;
            leftPrefix << prefix << "_get_" << i << "_left";
            std::ostringstream rightPrefix;
            rightPrefix << prefix << "_get_" << i << "_right";
            saveImageSet(leftPrefix.str(), left);
            saveImageSet(rightPrefix.str(), right);
        }
        VzNL_ReleaseImage(&left);
        VzNL_ReleaseImage(&right);
    }

    if (runCapture) {
        CaptureContext ctx;
        ctx.prefix = prefix;
        ctx.maxCount = 3;
        rc = VzNL_StartCapture(h, &onImage, &ctx);
        std::cout << "StartCapture: " << errString(rc) << std::endl;
        if (rc == 0) {
            const int waitMs = std::max(5000, static_cast<int>(std::ceil(5000.0 / fps)));
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
            rc = VzNL_StopCapture(h);
            std::cout << "StopCapture: " << errString(rc) << " callbackCount=" << ctx.count.load() << std::endl;
        }
    }

    VzNL_SetOutputImageFormat(keVzNLImageType_BGR888);
    turnOffExtLight(h);
    VzNL_CloseDevice(h);
    VzNL_Destroy();
    std::cout << "Done." << std::endl;
    return 0;
}
