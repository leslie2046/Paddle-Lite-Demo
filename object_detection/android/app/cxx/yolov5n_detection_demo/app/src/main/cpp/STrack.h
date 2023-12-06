#pragma once

#include <opencv2/opencv.hpp>
#include "kalmanFilter.h"
#include "TrackFrame.h"

using namespace cv;
using namespace std;

enum TrackState { New = 0, Tracked, Lost, Removed };
enum DirectionX { StationaryX, Left, Right };
enum DirectionZ { StationaryZ, Close, Away };
class STrack
{
public:
	STrack(vector<float> tlwh_, float score);
	~STrack();

	vector<float> static tlbr_to_tlwh(vector<float> &tlbr);
	void static multi_predict(vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter);
	void static_tlwh();
	void static_tlbr();
	vector<float> tlwh_to_xyah(vector<float> tlwh_tmp);
	vector<float> to_xyah();
	void mark_lost();
	void mark_removed();
	int next_id();
	int end_frame();
	
	void activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id);
	void re_activate(STrack &new_track, int frame_id, bool new_id = false);
	void update(STrack &new_track, int frame_id);
	void updateHistory(int inputW,int inputH);
	void updateAreaStateAndAction(const std::vector<cv::Point2f> area);
	void updateLineStateAndAction(const std::vector<cv::Point2f> lineOut,const std::vector<cv::Point2f> lineIn);

public:
	bool is_activated;
	int track_id;
	int state;

	vector<float> _tlwh;
	vector<float> tlwh;
	vector<float> tlbr;
	int frame_id;
	int tracklet_len;
	int start_frame;

	KAL_MEAN mean;
	KAL_COVA covariance;
	float score;
	std::deque<TrackFrame> history;  // 存储历史轨迹
    int areaState = 0;//0:功能未启用  1:in   2:out
	int areaAction = 0;//0:功能未启用 1:not change 2:in to out 3:out to in
	int lineState = 0;//0:功能未启用 1:no change 2:跨越了out  3:跨越了in
	int lineAction = 0; //0:功能未启用 1:not change 2:in to out 3:out to in
	int input_w;
	int input_h;
private:
	byte_kalman::KalmanFilter kalman_filter;
};