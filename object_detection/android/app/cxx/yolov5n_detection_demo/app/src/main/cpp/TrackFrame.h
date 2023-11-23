#pragma once
#include <opencv2/opencv.hpp>
//
// Created by 25360 on 2023-11-22.
//

using namespace std;

class TrackFrame {
public:
    TrackFrame(vector<float> tlwh_, long long timeStamp_);
    ~TrackFrame();

public:
        std::vector<float> tlwh;
        long long timeStamp = 0;
};


