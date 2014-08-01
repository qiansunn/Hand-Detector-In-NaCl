#include <opencv2/core/core.hpp>
#include <iostream>
#include "support_structure.hpp"

using namespace cv;

namespace HT {

//Base class for Detector module. Provide common interface

//Initialization of all detectors must be done using a rgb image which the user has to provide. */

class HandDetector {
 public:
  //Initialization method - returns true if initialization successful. Each detector class may define overloaded functions according to different needs.
  virtual bool Initialize(IplImage* rgb_image_, IplImage* depth_image_, bool use_color_, bool use_depth_) = 0;

  //Detect function to be called in the video loop for detection of the hand
  virtual ObjectState* Detect(IplImage* mask_) = 0;
};

}
