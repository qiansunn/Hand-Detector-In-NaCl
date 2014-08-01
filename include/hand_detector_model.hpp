#include <opencv2/core/core.hpp>
#include <queue>
#include "hand_detector.hpp"
//#include "support_structure.hpp"
using namespace cv;
using namespace std;
namespace HT {
class HandDetectorModel : public HandDetector {
 public:
  HandDetectorModel();
  virtual bool Initialize(IplImage* rgb_image_, IplImage* depth_image_, bool use_color_, bool use_depth_);

  //Detect function to be called in the video loop for subsequent detection of the hand
  virtual ObjectState* Detect(IplImage* mask_);

  //Virtual Destructor for HandDetector class
  ~HandDetectorModel(){};
  IplImage* mask;
 private:
  void SegmentHand(IplImage* region);
  pair<int, int> SearchNearestPixel(IplImage* region);
  void ProcessNeighbor(int &pixelcount, double &mean,const short first, const short second);
  IplImage* rgb_image;
  IplImage* depth_image;
  bool use_depth;
  bool use_color;
  CvSize frame_size;

  CvScalar hsv_min, hsv_max;

  
  queue<pair<int, int> > _pixels;
  
};

}
