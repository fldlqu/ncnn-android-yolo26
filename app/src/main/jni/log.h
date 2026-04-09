//
// Created by xianwei.yan on 2026/4/9.
//

#ifndef NCNN_ANDROID_YOLO26_LOG_H
#define NCNN_ANDROID_YOLO26_LOG_H


#include <android/log.h>

#define TAG "YOLO26"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, TAG, __VA_ARGS__)


#endif //NCNN_ANDROID_YOLO26_LOG_H
