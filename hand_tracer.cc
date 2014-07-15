#include <include/hand_detector.hpp>
#include <include/hand_tracer.hpp>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <queue>

#include <iostream>

using namespace std;
using namespace cv;

const unsigned char EMPTY = 0;
const unsigned char HAND = 255;
const unsigned short SENSOR_MIN = 400;
const unsigned short SENSOR_MAX = 7000;

void selectTarget(Mat &_mask);
void segmentHand(cv::Mat &mask, Rect &region, const cv::Mat &depth);
pair<int, int> searchNearestPixel(const Mat &depth, Rect &region);
void processNeighbor(int &pixelcount, double &mean, cv::Mat &mask, const short first, const short second, const cv::Mat &depth);
void mouse_callback(int event,int x,int y,int flag,void* param);

class HandTracer: public HandDetector {
 public:
  HandTracer(IplImage*);
  int initialized;
 private:
  Mat mask;
  IplImage* frame
  IplImage* red_image;
  IplImage* blue_image;
  IplImage* green_image;
  IplImage* rgb_image
}
/* 
 class gpCapture {
  public:
	VideoCapture cap;
	VideoCapture cap1;
	Mat rgbImg;
	Mat depthImg, depthMap;
	int useVideo;

	gpCapture(void) {
		useVideo = -1;
	}

	gpCapture(int h) {
		cap.open(h);
		if(h == CV_CAP_OPENNI)
			useVideo = 0;
		else
			useVideo = 1;
	}

	gpCapture(string h) {
		string rgbN = h+"/rgb/rgb%03d.png";
		string depthN = h+"/depth/depth%03d.png";
		cap.open(rgbN);
		cap1.open(depthN);
		useVideo = 2;
		depthMap = Mat(480, 640, CV_16UC1);
	}

	bool update(void) {
		bool result;
		if(useVideo == 0) {
			result = cap.grab();
			cap.retrieve(rgbImg, CV_CAP_OPENNI_BGR_IMAGE);
			cap.retrieve(depthMap, CV_CAP_OPENNI_DEPTH_MAP);
		}
		else if(useVideo == 1) {
			result = cap.grab();
			cap.retrieve(rgbImg);
			depthMap = Mat();
		}
		else if(useVideo == 2) {
			result = (cap.grab() && cap1.grab());
			if(result) {
				cap.retrieve(rgbImg);
				cap1.retrieve(depthImg);
				for(int i=0; i<depthImg.rows; i++) {
					for(int j=0; j<depthImg.cols; j++) {
						depthMap.at<ushort>(i,j) = (int(depthImg.at<Vec3b>(i,j)[0])*256 + int(depthImg.at<Vec3b>(i,j)[1]));
					}
				}
			}
		}
		return result;
	}
} capture;

*/

HandTracer::HandTracer() 
    {
}

bool selected, drawing_box;
Rect box;


void HandTracer::Initialize(IplImage* raw_image) {
  if(raw->nChannels != 4)
    return;
  HandDetector* hd = new HandDetector();
  frame_size = cvSize(raw_image);
  b_image = cvCreateImage(frame_size, 8, 1);
  g_image = cvCreateImage(frame_size, 8, 1);
  r_image = cvCreateImage(frame_size, 8, 1);
  depth_image = cvCreateImage(frame_size, 8, 1);
  rgb_image = cvCreateImage(frame_size, 8, 3);
  mask = cvCreateImage(frame_size, 8, 1);
  motion = cvCreateImage(frame_size, 8, 1);

  cvSplit(raw_image, b_image, g_image, r_image, depth_image);
  cvMerge(r_image, g_image, b_image, NULL, rgb_image);
  
  hd->Initialize(rgb_image, depth_image, true);
  hd->Detect(mask);
  prior = mask;

}


ObjectState HandDetector::Update(IplImage* img, IplImage* dst) {
  double motionangle;
  timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds
  CvSize size = cvSize(img->width,img->height); // get current frame size
  int i, idx1 = last, idx2;
  IplImage* silh;
  if (!mhi || mhi->width != size.width || mhi->height != size.height ) {
    if( buf == 0 ) {
      buf = (IplImage**)malloc(N*sizeof(buf[0]));
      memset( buf, 0, N*sizeof(buf[0]));
    }
    for (i = 0; i < N; i++ ) {
      cvReleaseImage(&buf[i]);
      buf[i] = cvCreateImage( size, IPL_DEPTH_8U, 1 );
      cvZero( buf[i] );
    }
      cvReleaseImage( &mhi );
      cvReleaseImage( &orient );
      cvReleaseImage( &segmask );
      cvReleaseImage( &mask );
      mhi = cvCreateImage( size, IPL_DEPTH_32F, 1 );
      cvZero( mhi ); // clear MHI at the beginning
      orient = cvCreateImage( size, IPL_DEPTH_32F, 1 );
      segmask = cvCreateImage( size, IPL_DEPTH_32F, 1 );
      mask = cvCreateImage( size, IPL_DEPTH_8U, 1 );
  }
  if ((!priorx)&&(!priory)) {
    priorx=totalx;
    priory=totaly;
  cvCopy( img, buf[last]); // convert frame to grayscale
  idx2 = (last + 1) % N; // index of (last - (N-1))th frame
  last = idx2;
  silh = buf[idx2];
  cvAbsDiff( buf[idx1], buf[idx2], silh ); // get difference between frames
  for (int y=0;y<silh->height;y++ ){
    char *ptr = (char*)(silh->imageData+y*silh->widthStep);
    for (int x=0; x<silh->width; x++) {
      if (ptr[x] != 0) {
      totalx = totalx+x;
      totaly = totaly+y;}
      }
    }
  int num=cvCountNonZero(silh);
  if (num!=0) {
    totalx=totalx/num;
    totaly=totaly/num;
  }
  printf("  DEBUG:x1=%d y1=%d num=%d px2=%d py=%d\n",totalx,totaly,num,priorx,priory);
  if ((num>25000)&&((timestamp-lasttime)>0.5)) {
    if((totalx-priorx)!=0)
    motionangle=(((double)totaly-(double)priory)/((double)totalx-(double)priorx));
    printf("  DEBUG:motionangle=%f\n",motionangle);
    if ((motionangle>1)||(motionangle<-1)) {
      if((totaly-priory)<0){
        err<<"{up}";
        fprintf(stderr,"  DEBUG:up\n");
        }
      else {
        err<<"{down}";
        fprintf(stderr,"  DEBUG:down\n");
      }
    }
    else {
      if((totalx-priorx)<0) {
        err<<"{right}";
        fprintf(stderr,"\nright>->->->->>>>>>>>>>>>>>>>>>\n");
                }
                else
                {
                        err<<"{lift}";
                        fprintf(stderr,"\nlift<<<<<<<<<<<<<<<<<<<\n");
                }
        }
        this->PostMessage(pp::Var(err.str()));
        lasttime = timestamp;


    //cvZero( dst );
}





