//
// Created by xianwei.yan on 2026/4/9.
//


#ifndef NCNN_ANDROID_YOLO26_DETECTOR_H
#define NCNN_ANDROID_YOLO26_DETECTOR_H

#include <opencv2/core/types.hpp>

struct Object {
    cv::Rect_<float> rect;
    int label;
    float prob;
};


// Bounding box structure
struct BBox {
    float x;       // Top-left x
    float y;       // Top-left y
    float width;
    float height;

    BBox() : x(0), y(0), width(0), height(0) {}
    BBox(float x_, float y_, float w_, float h_) : x(x_), y(y_), width(w_), height(h_) {}
};

// Tracked object (vehicle with optional plate)
struct TrackedObject {
    int trackId;
    float score;
    BBox bbox;
    int classId;
    int framesSinceUpdate;


    TrackedObject()
            : trackId(-1), score(0), classId(0),framesSinceUpdate(0) {}
};

#endif //NCNN_ANDROID_YOLO26_DETECTOR_H
