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

double distance_to_line(double x, double y, double x1, double y1, double x2, double y2) {
	double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);

	if (cross <= 0) {
		return std::sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	}

	double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

	if (cross >= d2) {
		return std::sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));
	}

	double r = cross / d2;
	double px = x1 + (x2 - x1) * r;
	double py = y1 + (y2 - y1) * r;

	return std::sqrt((x - px) * (x - px) + (py - y1) * (py - y1));
}

double vector_angle(const cv::Point2f& midpoint, const cv::Point2f& previous_midpoint) {
	cv::Point2f diff = midpoint - previous_midpoint;
	double angle = atan2(diff.y, diff.x) * 180.0 / CV_PI;
	return angle;
}
bool onSegment(cv::Point2f p, cv::Point2f q, cv::Point2f r) {
	return (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
			q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y));
}
int orientation(cv::Point2f p, cv::Point2f q, cv::Point2f r) {
	float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
	if (val == 0) return 0;
	return (val > 0) ? 1 : 2;
}

bool doIntersect(cv::Point2f p1, cv::Point2f q1, cv::Point2f p2, cv::Point2f q2) {
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	if (o1 != o2 && o3 != o4)
		return true;

	if (o1 == 0 && onSegment(p1, p2, q1)) return true;
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false;
}
void STrack::updateLineStateAndAction(const std::vector<cv::Point2f> lineOut,const std::vector<cv::Point2f> lineIn){
	if(lineOut.size()<2||lineIn.size()<2){
		lineState = 0;
		lineAction = 0;
		return;
	}
	if(history.size()<2){
		lineState = 1;
		return;
	}
	cv::Point2f point2F;
	point2F.x = tlwh[0] + tlwh[2] / 2;
	point2F.y = tlwh[1] + tlwh[3];
	cv::Point2f prePoint2F;
	prePoint2F.x = history[history.size()-2].tlwh[0] + history[history.size()-2].tlwh[2] / 2;
	prePoint2F.y = history[history.size()-2].tlwh[1] + history[history.size()-2].tlwh[3];
	bool isIntersectOut = doIntersect(lineOut[0],lineOut[1],prePoint2F,point2F);
	bool isIntersectIn = doIntersect(lineIn[0],lineIn[1],prePoint2F,point2F);
	if(!isIntersectOut&&!isIntersectIn){
		lineAction = 1;
		return;
	}

	if(isIntersectOut&&isIntersectIn){
		double distanceOut = distance_to_line(prePoint2F.x,prePoint2F.y,lineOut[0].x,lineOut[0].y,lineOut[1].x,lineOut[1].y);
		double distanceIn = distance_to_line(prePoint2F.x,prePoint2F.y,lineIn[0].x,lineIn[0].y,lineIn[1].x,lineIn[1].y);
		//一次跨越两条线
		if(distanceOut < distanceIn){
			lineAction = 3;
			lineState = 3;
		}else{
			lineAction = 2;
			lineState = 2;
		}
		return;
	}
	if(isIntersectOut){
		if(lineState == 0||lineState == 1){
			lineState = 2;
			lineAction = 1;
		}else if(lineState == 2){
			lineState = 2;
			lineAction = 1;
		}else if(lineState == 3){
			lineState = 2;
			lineAction = 2;
		}else{
			//TODO
		}
	}else{
		if(lineState == 0||lineState == 1){
			lineState = 3;
			lineAction = 1;
		}else if(lineState == 2){
			lineState = 3;
			lineAction = 3;
		}else if(lineState == 3){
			lineState = 3;
			lineAction = 1;
		}else{
			//TODO
		}
	}
}

void STrack:: updateAreaStateAndAction(const std::vector<cv::Point2f> area) {
	int areaState_ = 0;
	if (area.size()<3) {
		areaState = 0;
		areaAction = 0;
	} else {
		cv::Point2f point2F;
		point2F.x = tlwh[0] + tlwh[2] / 2;
		point2F.y = tlwh[1] + tlwh[3];
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


void STrack::updateHistory(int inputW,int inputH) {
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
}
