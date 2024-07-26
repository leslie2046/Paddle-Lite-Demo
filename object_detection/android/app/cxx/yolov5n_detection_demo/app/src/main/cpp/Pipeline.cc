// Copyright (c) 2019 PaddlePaddle Authors. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Pipeline.h"
#include "BYTETracker.h"
#include <algorithm>
#include <map>
#include <utility>
BYTETracker tracker(10, 30);
int trackingClassId = -1;
Detector::Detector(const std::string &modelDir, const std::string &labelPath,
                   const int cpuThreadNum, const std::string &cpuPowerMode,
                   int inputWidth, int inputHeight,
                   const std::vector<float> &inputMean,
                   const std::vector<float> &inputStd, float scoreThreshold)
    : inputWidth_(inputWidth), inputHeight_(inputHeight), inputMean_(inputMean),
      inputStd_(inputStd), scoreThreshold_(scoreThreshold) {
  paddle::lite_api::MobileConfig config;
  config.set_model_from_file(modelDir + "/model.nb");

  LOGD("--->model path: %s", modelDir.c_str());

  config.set_threads(cpuThreadNum);
  config.set_power_mode(ParsePowerMode(cpuPowerMode));
  predictor_ =
      paddle::lite_api::CreatePaddlePredictor<paddle::lite_api::MobileConfig>(
          config);
  labelList_ = LoadLabelList(labelPath);
  colorMap_ = GenerateColorMap(labelList_.size());
  channelLength_ = inputWidth_ * inputHeight_;
  isInited_ = false;
}

void Detector::InitParams(const int &width, const int &height) {
  if (isInited_)
    return;

  float r_w = inputWidth_ / (width * 1.0);
  float r_h = inputHeight_ / (height * 1.0);
  if (r_h > r_w) {
    inputW = inputWidth_;
    inputH = r_w * height;
    inputX = 0;
    inputY = (inputHeight_ - inputH) / 2;
    ratio_ = r_w;
  } else {
    inputW = r_h * width;
    inputH = inputHeight_;
    inputX = (inputWidth_ - inputW) / 2;
    inputY = 0;
    ratio_ = r_h;
  }
  isInited_ = true;
}

std::vector<std::string> Detector::LoadLabelList(const std::string &labelPath) {
  std::ifstream file;
  std::vector<std::string> labels;
  file.open(labelPath);
  while (file) {
    std::string line;
    std::getline(file, line);
    labels.push_back(line);
  }
  file.clear();
  file.close();
  return labels;
}

std::vector<cv::Scalar> Detector::GenerateColorMap(int numOfClasses) {
  std::vector<cv::Scalar> colorMap = std::vector<cv::Scalar>(numOfClasses);
  for (int i = 0; i < numOfClasses; i++) {
    int j = 0;
    int label = i;
    int R = 0, G = 0, B = 0;
    while (label) {
      R |= (((label >> 0) & 1) << (7 - j));
      G |= (((label >> 1) & 1) << (7 - j));
      B |= (((label >> 2) & 1) << (7 - j));
      j++;
      label >>= 3;
    }
    colorMap[i] = cv::Scalar(R, G, B);
  }
  return colorMap;
}

void Detector::Preprocess(const cv::Mat &rgbaImage) {
  InitParams(rgbaImage.cols, rgbaImage.rows);

  // Feed the input tensor with the data of the preprocessed image
  auto inputTensor = predictor_->GetInput(0);
  std::vector<int64_t> inputShape = {1, 3, inputHeight_, inputWidth_};
  inputTensor->Resize(inputShape);
  auto inputData = inputTensor->mutable_data<float>();

  cv::Mat img;
  cv::cvtColor(rgbaImage, img, cv::COLOR_BGRA2RGB);
  cv::Mat re(inputH, inputW, CV_8UC3);
  cv::resize(img, re, cv::Size(inputW, inputH));
  cv::Mat out(inputHeight_, inputWidth_, CV_8UC3, cv::Scalar(128, 128, 128));
  re.copyTo(out(cv::Rect(inputX, inputY, re.cols, re.rows)));

  // split channels
  out.convertTo(out, CV_32FC3, 1. / 255.);
  cv::Mat input_channels[3];
  cv::split(out, input_channels);
  for (int j = 0; j < 3; j++) {
    memcpy(inputData + channelLength_ * j, input_channels[j].data,
           channelLength_ * sizeof(float));
  }
}

void Detector::ExtractBoxes(int seq_id, const float *in,
                            std::map<int, std::vector<Object>> *outs,
                            const std::vector<int64_t> &shape) {
  int cls_num = shape[3] - 5;
  int xdim = static_cast<int>(inputWidth_ / strides_[seq_id]);
  for (int c = 0; c < shape[1]; c++) {
    int step = c * shape[2] * shape[3];
    for (int r = 0; r < shape[2]; r++) {
      int offset = step + r * shape[3];
      float score = in[offset + 4];
      if (score < confThresh_)
        continue;

      int max_cls_id = 0;
      float max_cls_val = 0;
      for (int i = 0; i < cls_num; i++) {
        if (in[offset + 5 + i] > max_cls_val) {
          max_cls_val = in[offset + 5 + i];
          max_cls_id = i;
        }
      }

      score *= max_cls_val;
      if (score < confThresh_)
        continue;

      Object obj;
      int y = static_cast<int>(r / xdim);
      int x = static_cast<int>(r % xdim);
      int cx = static_cast<int>(
          ((in[offset] * 2 - 0.5 + x) * strides_[seq_id] - inputX) / ratio_);
      int cy = static_cast<int>(
          ((in[offset + 1] * 2 - 0.5 + y) * strides_[seq_id] - inputY) /
          ratio_);
      int w = static_cast<int>(pow(in[offset + 2] * 2, 2) *
                               anchors_[seq_id][2 * c] / ratio_);
      int h = static_cast<int>(pow(in[offset + 3] * 2, 2) *
                               anchors_[seq_id][2 * c + 1] / ratio_);
      int left = cx - w / 2.0;
      int top = cy - h / 2.0;

      obj.rect = cv::Rect(left, top, w, h);
      obj.prob = score;
      obj.class_id = max_cls_id;

      if (outs->count(obj.class_id) == 0)
        outs->emplace(obj.class_id, std::vector<Object>());
      (*outs)[obj.class_id].emplace_back(obj);
    }
  }
}

static float iou_calc(const cv::Rect &rec_a, const cv::Rect &rec_b) {
  cv::Rect u = rec_a | rec_b;
  cv::Rect s = rec_a & rec_b;
  float s_area = s.area();
  if (s_area < 20)
    return 0.f;
  return u.area() * 1.0 / s_area;
}

static bool cmp(const Object &a, const Object &b) { return a.prob > b.prob; }

void Detector::Nms(const std::map<int, std::vector<Object>> &src,
                   std::vector<Object> *res) {
  for (auto it = src.begin(); it != src.end(); it++) {
    auto dets = it->second;
    std::sort(dets.begin(), dets.end(), cmp);
    for (size_t m = 0; m < dets.size(); ++m) {
      auto &item = dets[m];
      item.class_name = item.class_id >= 0 && item.class_id < labelList_.size()
                            ? labelList_[item.class_id]
                            : "Unknow";
      item.fill_color = item.class_id >= 0 && item.class_id < colorMap_.size()
                            ? colorMap_[item.class_id]
                            : cv::Scalar(0, 0, 0);
      res->push_back(item);
      for (size_t n = m + 1; n < dets.size(); ++n) {
        if (iou_calc(item.rect, dets[n].rect) > nmsThresh_) {
          dets.erase(dets.begin() + n);
          --n;
        }
      }
    }
  }
}

void Detector::Postprocess(std::vector<Object> *results) {
  std::map<int, std::vector<Object>> raw_outputs;
  for (int k = 0; k < 3; k++) {
    std::unique_ptr<const paddle::lite_api::Tensor> output_tensor(
        std::move(predictor_->GetOutput(k)));
    auto *outptr = output_tensor->data<float>();
    auto shape_out = output_tensor->shape();
    ExtractBoxes(k, outptr, &raw_outputs, shape_out);
  }
  vector<Object> tmp_results;
  Nms(raw_outputs, &tmp_results);
  for(int i=0;i<tmp_results.size();i++){
    if(trackingClassId<0){
      results->push_back(tmp_results[i]);
    }else if(tmp_results[i].class_id==trackingClassId){
      results->push_back(tmp_results[i]);
    }
  }
}

void Detector::Predict(const cv::Mat &rgbaImage, std::vector<Object> *results,
                       double *preprocessTime, double *predictTime,
                       double *postprocessTime) {
  auto t = GetCurrentTime();
  Preprocess(rgbaImage);
  *preprocessTime = GetElapsedTime(t);
  t = GetCurrentTime();
  predictor_->Run();
  *predictTime = GetElapsedTime(t);
  t = GetCurrentTime();
  Postprocess(results);
  *postprocessTime = GetElapsedTime(t);
}

Pipeline::Pipeline(const std::string &modelDir, const std::string &labelPath,
                   const int cpuThreadNum, const std::string &cpuPowerMode,
                   int inputWidth, int inputHeight,
                   const std::vector<float> &inputMean,
                   const std::vector<float> &inputStd, float scoreThreshold) {
  detector_.reset(new Detector(modelDir, labelPath, cpuThreadNum, cpuPowerMode,
                               inputWidth, inputHeight, inputMean, inputStd,
                               scoreThreshold));
}

void Pipeline::VisualizeTrackerResults( const std::vector<STrack> stracks,cv::Mat *rgbaImage) {
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1.5f;
    float fontThickness = 2.0f;
  for (int i = 0; i < stracks.size(); i++)
  {
    STrack sTrack = stracks[i];
    vector<float> tlwh = sTrack.tlwh;
    Scalar s = tracker.get_color(sTrack.track_id);
      cv::putText(*rgbaImage, format("%d", sTrack.track_id), cv::Point2d(tlwh[0]+tlwh[2]-20, tlwh[1]+20),
                    fontFace, fontScale,s, fontThickness);
      cv::putText(*rgbaImage, format("%d,%d",(int)tlwh[0],(int)tlwh[1]),cv::Point2d(tlwh[0],tlwh[1]+20),
                fontFace, 1,s, 1);
    cv::putText(*rgbaImage, format("%d,%d",(int)(tlwh[0]+tlwh[2]),(int)(tlwh[1]+tlwh[3])),cv::Point2d(tlwh[0]+tlwh[2]-100, tlwh[1]+tlwh[3]-20),
                fontFace, 1,s, 1);
      rectangle(*rgbaImage, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
    std::vector<cv::Point> polygon;
    for (TrackFrame frame : sTrack.history) {
      cv::Point2f point;
      point.x = frame.tlwh[0]+frame.tlwh[2]/2;
      point.y = frame.tlwh[1]+frame.tlwh[3];
      cv::circle(*rgbaImage,point,1, cv::Scalar(0, 255, 0),-1);
      polygon.push_back(cv::Point(static_cast<int>(frame.tlwh[0]+frame.tlwh[2]/2), static_cast<int>(frame.tlwh[1]+frame.tlwh[3])));
    }
    cv::polylines(*rgbaImage, polygon, false, s, 1);
    cv::Point2f point2F;
    point2F.x = tlwh[0] + tlwh[2] / 2;
    point2F.y = tlwh[1] + tlwh[3];
    cv::circle(*rgbaImage,point2F,2, cv::Scalar(0, 255, 0),-1);
  }
}

void Pipeline::VisualizeResults(const std::vector<Object> &results,
                                cv::Mat *rgbaImage) {
  int oriw = rgbaImage->cols;
  int orih = rgbaImage->rows;
  for (int i = 0; i < results.size(); i++) {
    Object object = results[i];
      LOGD("VisualizeResults object.rect (%d,%d,%d,%d)", object.rect.x,object.rect.y,object.rect.width,object.rect.height);
    cv::Rect boundingBox = object.rect & cv::Rect(0, 0, oriw - 1, orih - 1);
    char text[255];
    sprintf(text, "%s %.2f",object.class_name.c_str(),object.prob);
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1.5f;
    float fontThickness = 1.0f;
    cv::Size textSize =
        cv::getTextSize(text, fontFace, fontScale, fontThickness, nullptr);
    // Draw roi object, text, and background
    cv::rectangle(*rgbaImage, boundingBox, object.fill_color, 2);
    cv::rectangle(*rgbaImage,
                  cv::Point2d(boundingBox.x,
                              boundingBox.y - round(textSize.height * 1.25f)),
                  cv::Point2d(boundingBox.x + boundingBox.width, boundingBox.y),
                  object.fill_color, -1);
    cv::putText(*rgbaImage, text, cv::Point2d(boundingBox.x, boundingBox.y),
                fontFace, fontScale, cv::Scalar(255, 255, 255), fontThickness);
  }


}

void Pipeline::VisualizeStatus(double preprocessTime, double predictTime,
                               double postprocessTime,double trackTime, cv::Mat *rgbaImage) {
  char text[255];
  cv::Scalar fontColor = cv::Scalar(255, 255, 255);
  int fontFace = cv::FONT_HERSHEY_PLAIN;
  double fontScale = 2.f;
  float fontThickness = 2;
  sprintf(text, "Preprocess: %.1f ms", preprocessTime); // NOLINT
  cv::Size textSize =
      cv::getTextSize(text, fontFace, fontScale, fontThickness, nullptr);
  textSize.height *= 1.25f;
  cv::Point2d offset(10, textSize.height + 15);
  cv::putText(*rgbaImage, text, offset, fontFace, fontScale, fontColor,
              fontThickness);
  sprintf(text, "Predict: %.1f ms", predictTime); // NOLINT
  offset.y += textSize.height;
  cv::putText(*rgbaImage, text, offset, fontFace, fontScale, fontColor,
              fontThickness);
  sprintf(text, "Postprocess: %.1f ms", postprocessTime); // NOLINT
  offset.y += textSize.height;
  cv::putText(*rgbaImage, text, offset, fontFace, fontScale, fontColor,
              fontThickness);
  sprintf(text, "Tracking: %.1f ms", trackTime); // NOLINT
  offset.y += textSize.height;
  cv::putText(*rgbaImage, text, offset, fontFace, fontScale, fontColor,
              fontThickness);
  if((!tracker.lineIn_.empty()&&!tracker.lineOut_.empty())
    ||!tracker.area_.empty()){
    sprintf(text, "in: %d out: %d",in_count,out_count);
    offset.y += textSize.height;
    cv::putText(*rgbaImage, text, offset, fontFace, fontScale, fontColor,
                fontThickness);
  }
  if(!tracker.welcomeLineIn_.empty()&&!tracker.welcomeLineOut_.empty()){
    sprintf(text, "welcome in: %d out: %d",welcome_in_count,welcome_out_count);
    offset.y += textSize.height;
    cv::putText(*rgbaImage, text, offset, fontFace, fontScale, fontColor,
                fontThickness);
  }
  if(!tracker.byeLineIn_.empty()&&!tracker.byeLineOut_.empty()){
    sprintf(text, "bye in: %d out:%d",bye_in_count,bye_out_count);
    offset.y += textSize.height;
    cv::putText(*rgbaImage, text, offset, fontFace, fontScale, fontColor,
                fontThickness);
  }
}

void Pipeline::drawPolygon(const std::vector<cv::Point2f>& area, cv::Mat *rgbaImage) {
  std::vector<cv::Point> polygon;
  for (const auto& point : area) {
    polygon.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
  }
  cv::polylines(*rgbaImage, polygon, true, cv::Scalar(152, 251, 152), 3); // 红色线条，线宽为3
}

void Pipeline::drawLine(const std::vector<cv::Point2f>& line,cv::Scalar color, cv::Mat *rgbaImage) {
  std::vector<cv::Point> polygon;
  for (const auto& point : line) {
    polygon.push_back(cv::Point(static_cast<int>(point.x), static_cast<int>(point.y)));
  }
  cv::polylines(*rgbaImage, polygon, false,color, 2);
}

void Pipeline::setTrackingClassId(int classId){
  trackingClassId = classId;
}

bool Pipeline::setDynamicArea(std::vector<cv::Point2f> area) {
  return tracker.setDynamicArea(area);
}

bool Pipeline::setDynamicLine(std::vector<cv::Point2f> lineOut,std::vector<cv::Point2f> lineIn) {
  return tracker.setDynamicLine(lineOut,lineIn);
}
bool Pipeline::setWelcomeDynamicLine(std::vector<cv::Point2f> lineOut,std::vector<cv::Point2f> lineIn) {
  return tracker.setWelcomeDynamicLine(lineOut,lineIn);
}
bool Pipeline::setByeDynamicLine(std::vector<cv::Point2f> lineOut,std::vector<cv::Point2f> lineIn) {
  return tracker.setByeDynamicLine(lineOut,lineIn);
}

bool Pipeline::Process(cv::Mat &rgbaImage, std::string savedImagePath,std::vector<STrack>* outputs) {
  double preprocessTime = 0, predictTime = 0, postprocessTime = 0,trackProcessTime = 0;

  // Feed the image, run inference and parse the results
  std::vector<Object> results;
  detector_->Predict(rgbaImage, &results, &preprocessTime, &predictTime,
                     &postprocessTime);
  auto t = GetCurrentTime();
  vector<STrack> output_stracks = tracker.update(results,rgbaImage.cols,rgbaImage.rows);
  for (int i = 0; i < output_stracks.size(); i++) {
      STrack sTrack = output_stracks[i];
      if (sTrack.areaAction == 1 || sTrack.lineAction == 1) {
        out_count++;
      }
      if (sTrack.areaAction == 2 || sTrack.lineAction == 2) {
        in_count++;
      }
      if (sTrack.welcomeLineAction == 1) {
        welcome_out_count++;
      }
      if (sTrack.welcomeLineAction == 2) {
        welcome_in_count++;
      }
      if (sTrack.byeLineAction == 1) {
        bye_out_count++;
      }
      if (sTrack.byeLineAction == 2) {
        bye_in_count++;
      }
  }
  // Dump modified image if savedImagePath is set
  if (!savedImagePath.empty()) {
    VisualizeResults(results, &rgbaImage);
    VisualizeTrackerResults(output_stracks,&rgbaImage);
    drawPolygon(tracker.area_,&rgbaImage);
    drawLine(tracker.lineOut_,cv::Scalar(0, 56, 168),&rgbaImage);
    drawLine(tracker.lineIn_,cv::Scalar(255,223, 0),&rgbaImage);
    drawLine(tracker.welcomeLineOut_,cv::Scalar(0x66,0xD3,0xC0),&rgbaImage);
    drawLine(tracker.welcomeLineIn_,cv::Scalar(0xFD,0xDE,0x83),&rgbaImage);
    drawLine(tracker.byeLineOut_,cv::Scalar(0xFC, 0x84, 0x16),&rgbaImage);
    drawLine(tracker.byeLineIn_,cv::Scalar(0x36,0x36, 0x36),&rgbaImage);
    trackProcessTime = GetElapsedTime(t);
    VisualizeStatus(preprocessTime, predictTime, postprocessTime,trackProcessTime, &rgbaImage);
    cv::Mat bgrImage;
    cv::cvtColor(rgbaImage, bgrImage, cv::COLOR_RGBA2BGR);
    imwrite(savedImagePath, bgrImage);
  }
  std::copy(output_stracks.begin(), output_stracks.end(), std::back_inserter(*outputs));
  return true;
}
