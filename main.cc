// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include "ppapi/c/pp_errors.h"
#include "ppapi/c/ppb_opengles2.h"
#include "ppapi/cpp/completion_callback.h"
#include "ppapi/cpp/instance.h"
#include "ppapi/cpp/media_stream_video_track.h"
#include "ppapi/cpp/module.h"
#include "ppapi/cpp/rect.h"
#include "ppapi/cpp/var.h"
#include "ppapi/cpp/video_frame.h"
#include "ppapi/cpp/var_dictionary.h"
#include "ppapi/utility/completion_callback_factory.h"
#include <ppapi/c/pp_instance.h>
#include <ppapi/c/ppb.h>
#include "opencv/cv.h"
#include "opencv/cvaux.h"
#include "opencv/cxcore.h"
#include "opencv/highgui.h"
#include "opencv/ml.h"
#include "opencv/cxmisc.h"
#include "opencv/cvwimage.h"
#include "include/hand_detector_model.hpp"
//#include "include/support_structure.hpp"
using namespace HT;
using namespace std;
using namespace cv;
extern int errno;
stringstream err;

// When compiling natively on Windows, PostMessage can be #define-d to
// something else.
#ifdef PostMessage
#undef PostMessage
#endif

namespace {

// This object is the global object representing this plugin library as long
// as it is loaded.
class MediaStreamVideoModule : public pp::Module {
 public:
  	MediaStreamVideoModule() : pp::Module() {}
  	virtual ~MediaStreamVideoModule() {}
  	virtual pp::Instance* CreateInstance(PP_Instance instance);
};


class MediaStreamVideoDemoInstance : public pp::Instance{
 public:
  MediaStreamVideoDemoInstance(PP_Instance instance, pp::Module* module);
  virtual ~MediaStreamVideoDemoInstance();
  
  virtual void HandleMessage(const pp::Var& message_data);
 private:
  void ConfigureTrack();
  void RecognizeFace(pp::VideoFrame frame);
  void OnConfigure(int32_t result);
  void OnGetFrame(int32_t result, pp::VideoFrame frame);
  void update_mhi( IplImage* img, IplImage* dst);
  pp::MediaStreamVideoTrack video_track_;
  pp::CompletionCallbackFactory<MediaStreamVideoDemoInstance> callback_factory_;
  std::vector<int32_t> attrib_list_;
  // MediaStreamVideoTrack attributes:
  PP_VideoFrame_Format attrib_format_;
  int32_t attrib_width_;
  int32_t attrib_height_;
  //fps
  //double t;
  pp::Size frame_size_;
  int N;
  IplImage **buf;
  int last;
  // various tracking parameters (in seconds)
  double lasttime;
  double timestamp;
  IplImage* frame_;
  IplImage* down_sample_image;
  IplImage *mhi; // MHI
  IplImage *orient; // orientation
  IplImage *mask; // valid orientation mask
  IplImage *segmask; // motion segmentation map
  int trace_init;
};


MediaStreamVideoDemoInstance::MediaStreamVideoDemoInstance(
    PP_Instance instance, pp::Module* module)
    : pp::Instance(instance),
      callback_factory_(this),
      attrib_format_(PP_VIDEOFRAME_FORMAT_BGRA),
      attrib_width_(0),
      attrib_height_(0),
	N(4),
	last(0),
	buf(0),
	down_sample_image(0),
	mhi(0),
	orient(0),
	mask(0),
	segmask(0),
	lasttime(0),
	trace_init(0)
      {
	 
}


MediaStreamVideoDemoInstance::~MediaStreamVideoDemoInstance() {
}

void MediaStreamVideoDemoInstance::HandleMessage(const pp::Var& var_message) {
  if (!var_message.is_dictionary())
    return;
  pp::VarDictionary var_dictionary_message(var_message);
  pp::Var var_track = var_dictionary_message.Get("track");
  if (!var_track.is_resource())
      return;
  pp::Resource resource_track = var_track.AsResource();
  video_track_ = pp::MediaStreamVideoTrack(resource_track);

    ConfigureTrack();
}



void MediaStreamVideoDemoInstance::ConfigureTrack() {
  const int32_t attrib_list[] = {
      PP_MEDIASTREAMVIDEOTRACK_ATTRIB_FORMAT, attrib_format_,
      PP_MEDIASTREAMVIDEOTRACK_ATTRIB_WIDTH, attrib_width_,
      PP_MEDIASTREAMVIDEOTRACK_ATTRIB_HEIGHT, attrib_height_,
      PP_MEDIASTREAMVIDEOTRACK_ATTRIB_NONE
    };
  video_track_.Configure(attrib_list, callback_factory_.NewCallback(
        &MediaStreamVideoDemoInstance::OnConfigure));
  
}


void MediaStreamVideoDemoInstance::OnConfigure(int32_t result) {
  video_track_.GetFrame(callback_factory_.NewCallbackWithOutput(
      &MediaStreamVideoDemoInstance::OnGetFrame));
}


void MediaStreamVideoDemoInstance::OnGetFrame(
    int32_t result, pp::VideoFrame frame) {
  if (result != PP_OK)
    return;
  RecognizeFace(frame);
  video_track_.RecycleFrame(frame);
  video_track_.GetFrame(callback_factory_.NewCallbackWithOutput(
        &MediaStreamVideoDemoInstance::OnGetFrame));
  
}



void MediaStreamVideoDemoInstance::RecognizeFace(pp::VideoFrame frame){
  stringstream ss;
  char* data = static_cast<char*>(frame.GetDataBuffer());
  pp::Size size;
  frame.GetSize(&size);
  if (size != frame_size_) {
    frame_size_ = size;
  }
  int32_t width =frame_size_.width();
  int32_t height = frame_size_.height();
  
 
  frame_= cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 4);
  down_sample_image = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 4);
  memcpy(frame_->imageData, data, frame_->imageSize);
  cvResize(frame_, down_sample_image, CV_INTER_LINEAR);
  HandDetector* hand_detector = new HandDetectorModel();
  IplImage* b_image = cvCreateImage(cvSize(320,240), 8, 1);
  IplImage* g_image = cvCreateImage(cvSize(320,240), 8, 1);
  IplImage* r_image = cvCreateImage(cvSize(320,240), 8, 1);
  IplImage* depth_image = cvCreateImage(cvSize(320,240), 8, 1);
  IplImage* rgb_image = cvCreateImage(cvSize(320,240), 8, 3);
  IplImage* hand_mask = cvCreateImage(cvSize(320,240), 8, 1);
  cvZero(mask);
  cvSplit(down_sample_image, b_image, g_image, r_image, depth_image);
  cvMerge(r_image, g_image, b_image, NULL, rgb_image);


  ObjectState* object_state;
  CvRect rect;
  if(hand_detector->Initialize(rgb_image, depth_image, true)) {
    object_state = hand_detector->Detect(hand_mask);
    rect = object_state->GetRect();
  }
/*
catch (cv::Exception& e) {
  const char* err_msg = e.what();
  err << "opencv err" << err_msg;
  this->PostMessage(pp::Var(err.str()));
}
*/
  ss << "[";
  ss << "{" << "x:" << rect.x << ",width:" << rect.width;
  ss << ",y:" << rect.y << ",height:" << rect.height << "},\n";
  ss << "]";
  this->PostMessage(pp::Var(ss.str()));
	
  cvReleaseImage(&frame_);
  cvReleaseImage(&down_sample_image);
  cvReleaseImage(&b_image);
  cvReleaseImage(&g_image);
  cvReleaseImage(&r_image);
  cvReleaseImage(&depth_image);
  cvReleaseImage(&mask);
  cvReleaseImage(&rgb_image);
}


pp::Instance* MediaStreamVideoModule::CreateInstance(PP_Instance instance) {
  return new MediaStreamVideoDemoInstance(instance, this);
}

}// anonymous namespace

namespace pp {
// Factory function for your specialization of the Module object.
Module* CreateModule() {
  return new MediaStreamVideoModule();
}
}  // namespace pp

