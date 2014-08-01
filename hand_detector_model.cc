#include "include/hand_detector_model.hpp"
#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv2/imgproc/imgproc_c.h"
//#include "include/support_structure.hpp"
using namespace cv;
using namespace std;

#define RGB_FORMAT IPL_DEPTH_8U
#define DEPTH_FORMAT IPL_DEPTH_8U

#define MAX_OBJECT_SIZE 10000
#define DEPTH_THR 10
#define SENSOR_MIN 40
#define SENSOR_MAX 210
#define EMPTY 0
#define HAND 255




namespace HT {


HandDetectorModel::HandDetectorModel(): 
    rgb_image(0),
    depth_image(0),
    use_depth(false),
    use_color(false)
    {
}


bool HandDetectorModel::Initialize(IplImage* rgb_image_, IplImage* depth_image_, bool use_color_, bool use_depth_) {
  if (use_color_) {
    if (rgb_image_->depth != RGB_FORMAT) 
      return false;
    rgb_image = rgb_image_;
    use_color = true;
    frame_size = cvSize(rgb_image_->width, rgb_image_->height);
    hsv_max = cvScalar(20, 150, 255, 0);
    hsv_min = cvScalar(0, 30, 80, 0);
  }
  if (use_depth_) {
    depth_image = depth_image_;
    use_depth = true;
    frame_size = cvSize(depth_image_->width, depth_image_->height);
  }  
  return true;
}



ObjectState* HandDetectorModel::Detect(IplImage* mask_) {
  mask = mask_;
  if ((use_color == false) && (use_depth == false))
    return (ObjectState*)0;
/*  if (use_color == true) {
    IplImage* hsv_image = cvCreateImage(frame_size, 8, 3);
    IplImage* hsv_mask = cvCreateImage(frame_size, 8, 1);
    IplImage* gray_image = cvCreateImage(frame_size, 8, 1);

    cvCvtColor(rgb_image, hsv_image, CV_RGB2HSV);
    cvInRangeS(hsv_image, hsv_min, hsv_max, hsv_mask);
    cvCvtColor(rgb_image, gray_image, CV_BGR2GRAY);
    cvMin(gray_image,hsv_mask,gray_image);
  }
*/
  if (use_depth == true) {
    SegmentHand(depth_image);
  }
  cvDilate(mask,mask,NULL,10);
  cvErode(mask,mask,NULL,10);
  
  CvMemStorage* g_storage = cvCreateMemStorage(0);
  CvSeq* contours = 0;
  CvRect rect;
  double largest_area = 0;
  int contour_size = cvFindContours(mask, g_storage, &contours, sizeof(CvContour), CV_RETR_CCOMP);

  if (contours) {
    for(CvSeq *c = contours; c!=NULL; c=c->h_next) {
    double a = cvContourArea(c);
      if (a > largest_area){
      largest_area = a;
      rect = cvBoundingRect(c);
      }
    }
  }
//  cvReleaseImage(&hsv_image);
//  cvReleaseImage(&hsv_mask);
//  cvReleaseImage(&gray_image);
  return new ObjectState(rect);
}



void HandDetectorModel::SegmentHand(IplImage* region) {
  
  pair<int, int> current = SearchNearestPixel(depth_image);
  if (current.first < 0) {
    return;
  }
  IplImage* visited = cvCreateImage(frame_size, 8, 1);
  cvZero(visited);
  double mean = CV_IMAGE_ELEM(depth_image,unsigned char,current.second,current.first);
  int minx=depth_image->width,miny=depth_image->height,maxx=0,maxy=0;
  int pixelcount = 1;
  _pixels.push(current);

  while ((!_pixels.empty()) && (pixelcount < MAX_OBJECT_SIZE)) {
    current = _pixels.front();
    _pixels.pop();

    if (current.first < minx) minx = current.first;
      else if (current.first > maxx) maxx = current.first;
    if (current.second < miny) miny = current.second;
      else if (current.second > maxy) maxy = current.second;

    if (current.first + 1 < region->width 
        && CV_IMAGE_ELEM(visited, unsigned char, current.second, current.first+1) == 0 ) {
      CV_IMAGE_ELEM(visited,unsigned char,current.second, current.first+1) = 255;
      ProcessNeighbor(pixelcount, mean, current.first + 1,current.second);
    }

    if (current.first - 1 > 0
        && CV_IMAGE_ELEM(visited, unsigned char, current.second, current.first-1) == 0 ) {
      CV_IMAGE_ELEM(visited,unsigned char,current.second, current.first-1) = 255;
      ProcessNeighbor(pixelcount, mean, current.first - 1,current.second);
    }

    if (current.second + 1 < region->height 
        && CV_IMAGE_ELEM(visited, unsigned char, current.second+1, current.first) == 0) {
      CV_IMAGE_ELEM(visited,unsigned char,current.second+1, current.first) = 255;
      ProcessNeighbor(pixelcount,mean,current.first,current.second + 1);
    }

    if (current.second - 1 > 0
        && CV_IMAGE_ELEM(visited, unsigned char, current.second-1, current.first) == 0) {
      CV_IMAGE_ELEM(visited,unsigned char,current.second-1, current.first) = 255;
      ProcessNeighbor(pixelcount,mean,current.first,current.second - 1);
    }
  }
  cvReleaseImage(&visited);
}


pair<int, int> HandDetectorModel::SearchNearestPixel(IplImage* region) {
  pair<int, int> pt;
  pt.first = -1;
  pt.second = -1;
  const unsigned char *depthptr;
  const unsigned char *regionptr;
  int max = 1;
  for (int y=0; y<region->height; y++) {
    depthptr = (const unsigned char*)(depth_image->imageData+y*depth_image->widthStep);
    regionptr = (const unsigned char*)(region->imageData+y*region->widthStep);
    for (int x=0; x<region->width; x++) {
      if(regionptr[x] == 0) continue;
      if(depthptr[x] > SENSOR_MIN && depthptr[x] < SENSOR_MAX && depthptr[x] > max
         && (CV_IMAGE_ELEM(depth_image,unsigned char,y+1,x)!=0) 
         && (CV_IMAGE_ELEM(depth_image,unsigned char,y-1,x)!=0)    
         && (CV_IMAGE_ELEM(depth_image,unsigned char,y,x+1)!=0)
         && (CV_IMAGE_ELEM(depth_image,unsigned char,y,x-1)!=0)) {
        max = depthptr[x];
        pt.first = x;
        pt.second = y;
      }
    }
  }
  return pt;
}


void HandDetectorModel::ProcessNeighbor(int &pixelcount, double &mean, const short first, const short second)
{
   
  unsigned char d = CV_IMAGE_ELEM(depth_image,unsigned char,second,first);

    if (CV_IMAGE_ELEM(mask, unsigned char, second, first) == EMPTY &&
         fabs(d-mean/pixelcount) < DEPTH_THR && d > SENSOR_MIN && d <= SENSOR_MAX)
    {
        pixelcount++;
        mean += d;
        CV_IMAGE_ELEM(mask, unsigned char, second, first) = HAND;
        _pixels.push(pair<int, int>(first,second));
    }
}



}
