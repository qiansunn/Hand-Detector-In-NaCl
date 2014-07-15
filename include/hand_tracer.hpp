#include <opencv2/core/core.hpp>
#include <include/support_structures.hpp>
#include <vector>

using namespace cv;

namespace HT {

class HandTracker {

 protected:
  //Default destructor
  ~HandTracker() { }
 
 public:
  //Initialisation for the tracker with BGRD image
  virtual void Initialize(IplImage* raw_image) = 0;
  //Update tracker
  virtual ObjectState Update(IplImage* raw_image) = 0;
  
 private:
  IplImage* depth_image;
  IplImage* rgb_image;
  IplImage *b_image, *g_image, *r_image;
  IplImage* mask;
  IplImage* motion;
  int totalx;
  int totaly;
  int priorx;
  int priory;
  
  CvSize frame_size;
};

}
