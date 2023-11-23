//
// Created by 25360 on 2023-11-22.
//

#include "TrackFrame.h"
TrackFrame::TrackFrame(vector<float> tlwh_, long long timeStamp_) {
    tlwh = tlwh_;
    timeStamp = timeStamp_;
}
TrackFrame::~TrackFrame()
{
}