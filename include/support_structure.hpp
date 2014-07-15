#include <opencv/cv.h>
namespace HT {
class ObjectState {
 public:
  ObjectState(CvRect rect_) {
    rect=rect_;
  }
  CvRect GetRect() {
    return rect;
  }
 private:
  CvRect rect;
};


}
