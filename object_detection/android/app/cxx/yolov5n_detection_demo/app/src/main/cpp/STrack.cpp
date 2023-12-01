#include "STrack.h"
#include "Utils.h"
const int N = 7;  // 考虑最近N帧

STrack::STrack(vector<float> tlwh_, float score)
{
	_tlwh.resize(4);
	_tlwh.assign(tlwh_.begin(), tlwh_.end());

	is_activated = false;
	track_id = 0;
	state = TrackState::New;
	
	tlwh.resize(4);
	tlbr.resize(4);

	static_tlwh();
	static_tlbr();
	frame_id = 0;
	tracklet_len = 0;
	this->score = score;
	start_frame = 0;
}

STrack::~STrack()
{
}

void STrack::activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id)
{
	this->kalman_filter = kalman_filter;
	this->track_id = this->next_id();

	vector<float> _tlwh_tmp(4);
	_tlwh_tmp[0] = this->_tlwh[0];
	_tlwh_tmp[1] = this->_tlwh[1];
	_tlwh_tmp[2] = this->_tlwh[2];
	_tlwh_tmp[3] = this->_tlwh[3];
	vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.initiate(xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	if (frame_id == 1)
	{
		this->is_activated = true;
	}
	//this->is_activated = true;
	this->frame_id = frame_id;
	this->start_frame = frame_id;
}

void STrack::re_activate(STrack &new_track, int frame_id, bool new_id)
{
	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];
	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->tracklet_len = 0;
	this->state = TrackState::Tracked;
	this->is_activated = true;
	this->frame_id = frame_id;
	this->score = new_track.score;
	if (new_id)
		this->track_id = next_id();
}

void STrack::update(STrack &new_track, int frame_id)
{
	this->frame_id = frame_id;
	this->tracklet_len++;

	vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
	DETECTBOX xyah_box;
	xyah_box[0] = xyah[0];
	xyah_box[1] = xyah[1];
	xyah_box[2] = xyah[2];
	xyah_box[3] = xyah[3];

	auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
	this->mean = mc.first;
	this->covariance = mc.second;

	static_tlwh();
	static_tlbr();

	this->state = TrackState::Tracked;
	this->is_activated = true;

	this->score = new_track.score;
}

void STrack::static_tlwh()
{
	if (this->state == TrackState::New)
	{
		tlwh[0] = _tlwh[0];
		tlwh[1] = _tlwh[1];
		tlwh[2] = _tlwh[2];
		tlwh[3] = _tlwh[3];
		return;
	}

	tlwh[0] = mean[0];
	tlwh[1] = mean[1];
	tlwh[2] = mean[2];
	tlwh[3] = mean[3];

	tlwh[2] *= tlwh[3];
	tlwh[0] -= tlwh[2] / 2;
	tlwh[1] -= tlwh[3] / 2;
}

void STrack::static_tlbr()
{
	tlbr.clear();
	tlbr.assign(tlwh.begin(), tlwh.end());
	tlbr[2] += tlbr[0];
	tlbr[3] += tlbr[1];
}

vector<float> STrack::tlwh_to_xyah(vector<float> tlwh_tmp)
{
	vector<float> tlwh_output = tlwh_tmp;
	tlwh_output[0] += tlwh_output[2] / 2;
	tlwh_output[1] += tlwh_output[3] / 2;
	tlwh_output[2] /= tlwh_output[3];
	return tlwh_output;
}

vector<float> STrack::to_xyah()
{
	return tlwh_to_xyah(tlwh);
}

vector<float> STrack::tlbr_to_tlwh(vector<float> &tlbr)
{
	tlbr[2] -= tlbr[0];
	tlbr[3] -= tlbr[1];
	return tlbr;
}

void STrack::mark_lost()
{
	state = TrackState::Lost;
}

void STrack::mark_removed()
{
	state = TrackState::Removed;
}

int STrack::next_id()
{
	static int _count = 0;
	_count++;
	return _count;
}

int STrack::end_frame()
{
	return this->frame_id;
}

void STrack::multi_predict(vector<STrack*> &stracks, byte_kalman::KalmanFilter &kalman_filter)
{
	for (int i = 0; i < stracks.size(); i++)
	{
		if (stracks[i]->state != TrackState::Tracked)
		{
			stracks[i]->mean[7] = 0;
		}
		kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
		stracks[i]->static_tlwh();
		stracks[i]->static_tlbr();
	}
}

// 判断数组趋势的函数
DirectionZ determine_trend(float* array, int size, float delta) {
	int up=0;//up
	int down=0;//down
	if (size < 2) {
		return StationaryZ; // 数组元素不足以判断趋势
	}
	DirectionZ trend = StationaryZ;
	for (int i = 1; i < size; ++i) {
		if (array[i] > array[i - 1] + delta) { // 明确上升
			if (trend == Close) {
				return StationaryZ; // 已经有下降趋势，现在上升，所以不确定
			}
			trend = Away;
			up++;
		} else if (array[i] < array[i - 1] - delta) { // 明确下降
			if (trend == Away) {
				return StationaryZ; // 已经有上升趋势，现在下降，所以不确定
			}
			trend = Close;
			down++;
		}
		// 如果两个数在delta范围内，我们认为它们相等，趋势不变
	}
	if(up>=7){
		trend = Away;
	}else if(up<=3){
		trend = Close;
	}else{
		trend = StationaryZ;
	}
	return trend; // 返回最终的趋势
}

float calculateEMA(float newData, float previousEMA, float alpha) {
	return alpha * newData + (1 - alpha) * previousEMA;
}

void STrack:: updateAreaStateAndAction(const std::vector<cv::Point2f> area) {
	int areaState_ = 0;
	int areaAction_ = 0;
	if (area.empty()) {
		areaState = 0;
		areaAction = 0;
	} else {
		cv::Point2f point2F;
		point2F.x = tlwh[0] + tlwh[2] / 2;
		point2F.y = tlwh[1] + tlwh[3];
		//point2F.y = (tlwh[1]+tlwh[3])<stracks[i].input_h?tlwh[1]+tlwh[3]:stracks[i].input_h-1;
		double distance = cv::pointPolygonTest(area, point2F, true);
		if (distance < 0) {
			areaState_ = 1;
		} else {
			areaState_ = 2;
		}
		if (areaState_ == 1) {
			if (areaState == 1) {
				areaAction = 1;
			} else if(areaState == 2) {
				areaAction = 3;
			}else{
				//TODO
			}
		} else if (areaState_ == 2) {
			if (areaState == 2) {
				areaAction = 1;
			} else if(areaState == 1) {
				areaAction = 2;
			}else{
				//TODO
			}
		} else {
			//TODO
		}
		areaState = areaState_;
		LOGD("updateAreaStateAndAction  (%d,%d)", areaState, areaAction);
	}
}
void STrack::updateHistoryAndDirection(int inputW,int inputH) {
	// 更新历史轨迹
	input_w = inputW;
	input_h = inputH;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	long long milliseconds = (long long)tv.tv_sec * 1000 + tv.tv_usec / 1000;
    TrackFrame trackFrame = TrackFrame(tlwh,milliseconds);
	if(history.size()==1){
		history.push_back(trackFrame);
		history.push_back(trackFrame);
		history.pop_front();//丢弃第一个
	}else{
		history.push_back(trackFrame);
	}
	if (history.size() > N) {
		history.pop_front();
	}
	// 如果历史轨迹不足N帧，跳过此次判断
	if (history.size() < N) {
		return;
	}
	// 计算平均位置和尺寸变化
    float emaX0 =history[0].tlwh[0]+history[0].tlwh[2]/2;
	float emaX = emaX0;  // 初始EMA值，可以设置为第一个数据点的值
	double alpha = 0.3;  // 指数加权移动平均的权重
    float emaZH0 = history[0].tlwh[1];
    float emaZF0 = history[0].tlwh[1]+history[0].tlwh[3];
    float emaZH = emaZH0;
    float emaZF = emaZF0;
	for (int i = 0; i < history.size(); ++i) {
        emaX = calculateEMA(history[i].tlwh[0]+ history[i].tlwh[2]/2, emaX, alpha);
        emaZH = calculateEMA(history[i].tlwh[1], emaZH, alpha);
        emaZF = calculateEMA(history[i].tlwh[1]+history[i].tlwh[3], emaZF, alpha);
	}
	float timeMS = history[N-1].timeStamp-history[0].timeStamp;
	float timeSF = (float)timeMS/1000;
//    speedX = (emaX-emaX0)/timeSF;
	speedX =( (history[N-1].tlwh[0]+ history[N-1].tlwh[2]/2)-(history[0].tlwh[0]+ history[0].tlwh[2]/2))/timeSF;
//    float height1 = history[N-1].tlwh[1] - history[0].tlwh[1];//上边位移
//    float height2 = (history[N-1].tlwh[1]+history[N-1].tlwh[3])-(history[0].tlwh[1]+history[0].tlwh[3]);//下边位移
	float height1 = emaZH - emaZH0;//上边位移
	float height2 = emaZF - emaZF0;//下边位移
	speedZ = ((std::abs(height1)>std::abs(height2))?height1:-height2)/timeSF;
//    speedZ = ((history[N-1].tlwh[1] - history[0].tlwh[1])/std::abs(history[0].tlwh[1]-240))/timeS;
	LOGD("speedZ %f ",speedZ);
	float threshholdX1=0.02343f*inputW;//判定为静止的阈值
	float threshholdX2=0.03515f*inputW;//判定为运动的阈值
//	float threshholdZ1=0.02083f*inputH;//判定为静止的阈值
//	float threshholdZ2=0.04166f*inputH;//判定为运动的阈值
//	float threshholdX1=15.0f;//判定为静止的阈值
//	float threshholdX2=22.5f;//判定为运动的阈值
	float threshholdZ1=10.0f;//判定为静止的阈值
	float threshholdZ2=30.0f;//判定为运动的阈值
	float  abs_speedX = std::abs(speedX);
	if (abs_speedX < threshholdX1) {
		directionX = StationaryX;
	} else if (abs_speedX < threshholdX2) {
	}else {
		directionX = (speedX < 0) ? Left : Right;
	}
	// 判断z轴运动方向
	float  abs_speedZ = std::abs(speedZ);
	if(abs_speedZ<threshholdZ1){
		directionZ = StationaryZ;
	}else if(abs_speedZ<threshholdZ2){
	}else{
		directionZ = (speedZ < 0) ? Close : Away;
	}
}