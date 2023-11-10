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

#include "Native.h"
#include "Pipeline.h"
#include <android/bitmap.h>

#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     com_baidu_paddle_lite_demo_object_detection_Native
 * Method:    nativeInit
 * Signature:
 * (Ljava/lang/String;Ljava/lang/String;ILjava/lang/String;II[F[FF)J
 */
JNIEXPORT jlong JNICALL
Java_com_baidu_paddle_lite_demo_object_1detection_Native_nativeInit(
    JNIEnv *env, jclass thiz, jstring jModelDir, jstring jLabelPath,
    jint cpuThreadNum, jstring jCPUPowerMode, jint inputWidth, jint inputHeight,
    jfloatArray jInputMean, jfloatArray jInputStd, jfloat scoreThreshold) {
  std::string modelDir = jstring_to_cpp_string(env, jModelDir);
  std::string labelPath = jstring_to_cpp_string(env, jLabelPath);
  std::string cpuPowerMode = jstring_to_cpp_string(env, jCPUPowerMode);
  std::vector<float> inputMean = jfloatarray_to_float_vector(env, jInputMean);
  std::vector<float> inputStd = jfloatarray_to_float_vector(env, jInputStd);
  return reinterpret_cast<jlong>(
      new Pipeline(modelDir, labelPath, cpuThreadNum, cpuPowerMode, inputWidth,
                   inputHeight, inputMean, inputStd, scoreThreshold));
}

JNIEXPORT jboolean JNICALL
Java_com_baidu_paddle_lite_demo_object_1detection_Native_nativeSetTrackingClassId(JNIEnv* env, jclass thiz, jlong ctx, jint classId)
{
  if (classId < 0)
    return JNI_FALSE;
  Pipeline *pipeline = reinterpret_cast<Pipeline *>(ctx);
  pipeline->setTrackingClassId(classId);
  return JNI_TRUE;
}
/*
 * Class:     com_baidu_paddle_lite_demo_object_detection_Native
 * Method:    nativeRelease
 * Signature: (J)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_baidu_paddle_lite_demo_object_1detection_Native_nativeRelease(
    JNIEnv *env, jclass thiz, jlong ctx) {
  if (ctx == 0) {
    return JNI_FALSE;
  }
  Pipeline *pipeline = reinterpret_cast<Pipeline *>(ctx);
  delete pipeline;
  return JNI_TRUE;
}

/*
 * Class:     com_baidu_paddle_lite_demo_object_detection_Native
 * Method:    nativeProcess
 * Signature: (JIIIILjava/lang/String;)Z
 */
JNIEXPORT jboolean JNICALL
Java_com_baidu_paddle_lite_demo_object_1detection_Native_nativeProcess(
    JNIEnv *env, jclass thiz, jlong ctx, jobject jARGB8888ImageBitmap,
    jstring jsavedImagePath, jobject outputs) {
  if (ctx == 0) {
    return JNI_FALSE;
  }
// 1. 获取ArrayList和TrackingObject类的引用
  jclass arrayListClass = env->FindClass("java/util/ArrayList");
  jclass trackingObjectClass = env->FindClass("com/baidu/paddle/lite/demo/common/TrackingObject");

  // 2. 获取ArrayList的add方法和TrackingObject的字段ID
  jmethodID arrayListAddMethod = env->GetMethodID(arrayListClass, "add", "(Ljava/lang/Object;)Z");
  jfieldID trackIdField = env->GetFieldID(trackingObjectClass, "trackId", "I");
  jfieldID scoreField = env->GetFieldID(trackingObjectClass, "score", "F");
  jfieldID rectField = env->GetFieldID(trackingObjectClass, "rect", "Landroid/graphics/RectF;");
  jfieldID directionXField = env->GetFieldID(trackingObjectClass, "directionX", "I");
  jfieldID directionZField = env->GetFieldID(trackingObjectClass, "directionZ", "I");
  jfieldID inputWField = env->GetFieldID(trackingObjectClass, "inputW", "I");
  jfieldID inputHField = env->GetFieldID(trackingObjectClass, "inputH", "I");


    // Convert the android bitmap(ARGB8888) to the OpenCV RGBA image. Actually,
  // the data layout of AGRB8888 is R, G, B, A, it's the same as CV RGBA image,
  // so it is unnecessary to do the conversion of color format, check
  // https://developer.android.com/reference/android/graphics/Bitmap.Config#ARGB_8888
  // to get the more details about Bitmap.Config.ARGB8888
  auto t = GetCurrentTime();
  void *bitmapPixels;
  AndroidBitmapInfo bitmapInfo;
  if (AndroidBitmap_getInfo(env, jARGB8888ImageBitmap, &bitmapInfo) < 0) {
    LOGE("Invoke AndroidBitmap_getInfo() failed!");
    return JNI_FALSE;
  }
  if (bitmapInfo.format != ANDROID_BITMAP_FORMAT_RGBA_8888) {
    LOGE("Only Bitmap.Config.ARGB8888 color format is supported!");
    return JNI_FALSE;
  }
  if (AndroidBitmap_lockPixels(env, jARGB8888ImageBitmap, &bitmapPixels) < 0) {
    LOGE("Invoke AndroidBitmap_lockPixels() failed!");
    return JNI_FALSE;
  }
  cv::Mat bmpImage(bitmapInfo.height, bitmapInfo.width, CV_8UC4, bitmapPixels);
  cv::Mat rgbaImage;
  bmpImage.copyTo(rgbaImage);
  if (AndroidBitmap_unlockPixels(env, jARGB8888ImageBitmap) < 0) {
    LOGE("Invoke AndroidBitmap_unlockPixels() failed!");
    return JNI_FALSE;
  }
  LOGD("Read from bitmap costs %f ms", GetElapsedTime(t));

  std::string savedImagePath = jstring_to_cpp_string(env, jsavedImagePath);
  Pipeline *pipeline = reinterpret_cast<Pipeline *>(ctx);
  vector<STrack> output_stracks;
  bool modified = pipeline->Process(rgbaImage, savedImagePath,&output_stracks);
  t = GetCurrentTime();
  if (modified) {
    for (int i = 0; i < output_stracks.size(); i++)
    {
      STrack sTrack = output_stracks[i];
      vector<float> tlbr = sTrack.tlbr;
      jobject trackingObject = env->AllocObject(trackingObjectClass);
      env->SetIntField(trackingObject, trackIdField, sTrack.track_id);
      env->SetFloatField(trackingObject, scoreField, sTrack.score);
      env->SetIntField(trackingObject, directionXField, (int)sTrack.directionX);
      env->SetIntField(trackingObject, directionZField, (int)sTrack.directionZ);
      env->SetIntField(trackingObject, inputWField, (int)sTrack.input_w);
      env->SetIntField(trackingObject, inputHField, (int)sTrack.input_h);
      // 创建并设置RectF对象（假设已经存在rectClass和对应的field IDs）
      jclass rectFClass = env->FindClass("android/graphics/RectF");
      jobject rectF = env->AllocObject(rectFClass);
      // 设置RectF对象的字段（这里假设字段名称和类型已知）
      jfieldID leftField = env->GetFieldID(rectFClass, "left", "F");
      jfieldID topField = env->GetFieldID(rectFClass, "top", "F");
      jfieldID rightField = env->GetFieldID(rectFClass, "right", "F");
      jfieldID bottomField = env->GetFieldID(rectFClass, "bottom", "F");

      env->SetFloatField(rectF, leftField, jfloat(tlbr[0]));
      env->SetFloatField(rectF, topField, jfloat(tlbr[1]));
      env->SetFloatField(rectF, rightField, jfloat(tlbr[2]));
      env->SetFloatField(rectF, bottomField, jfloat(tlbr[3]));
      // 将RectF对象设置到TrackingObject的rect字段中
      env->SetObjectField(trackingObject, rectField, rectF);

      // 添加到ArrayList中
      env->CallBooleanMethod(outputs, arrayListAddMethod, trackingObject);

      // 删除局部引用，避免内存泄漏
      env->DeleteLocalRef(trackingObject);
      env->DeleteLocalRef(rectF);
      }
    // Convert the OpenCV RGBA image to the android bitmap(ARGB8888)
    if (rgbaImage.type() != CV_8UC4) {
      LOGE("Only CV_8UC4 color format is supported!");
      return JNI_FALSE;
    }

    if (AndroidBitmap_lockPixels(env, jARGB8888ImageBitmap, &bitmapPixels) <
        0) {
      LOGE("Invoke AndroidBitmap_lockPixels() failed!");
      return JNI_FALSE;
    }
    cv::Mat bmpImage(bitmapInfo.height, bitmapInfo.width, CV_8UC4,
                     bitmapPixels);
    rgbaImage.copyTo(bmpImage);
    if (AndroidBitmap_unlockPixels(env, jARGB8888ImageBitmap) < 0) {
      LOGE("Invoke AndroidBitmap_unlockPixels() failed!");
      return JNI_FALSE;
    }
    LOGD("Write to bitmap costs %f ms", GetElapsedTime(t));
  }

  return modified;
}

#ifdef __cplusplus
}
#endif
