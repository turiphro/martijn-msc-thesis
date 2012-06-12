/* View all available webcams */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <vector>

#define CAM_HEIGHT 480   /* resolution of cameras */
#define CAM_WIDTH  640   /* (same for all cams for now) */


using namespace std;
using namespace cv;

void callbackFlipx(int state, void *i)
{
  *(int *)i = state;
}
void callbackFlipy(int state, void *i)
{
  *(int *)i = state;
}
void callbackBrightness(int value, void *cam)
{
  ((VideoCapture *)cam)->set(CV_CAP_PROP_BRIGHTNESS, (double)value/255);

}


int main()
{

  vector<VideoCapture *> cams;
  vector<int> flipped_x;
  vector<int> flipped_y;
  vector<int> brightness;


  int device = 0;
  int result = 1;

  /* search for cameras */
  while (result) {
    cout << "Trying to open camera " << device << endl;
    cams.push_back(new VideoCapture(device++));
    flipped_x.push_back(0);
    flipped_y.push_back(0);
    brightness.push_back(cams.back()->get(CV_CAP_PROP_BRIGHTNESS));

    result = cams.back()->isOpened();
  }
  cams.pop_back(); /* delete last one */
  flipped_x.pop_back();
  flipped_y.pop_back();
  brightness.pop_back();


  cout << "Cameras found: " << cams.size() << endl;

  /* open windows */
  for (int i=0; i<cams.size(); i++) {
    namedWindow(boost::lexical_cast<string>(i), CV_WINDOW_NORMAL);
    cams.at(i)->set(CV_CAP_PROP_FRAME_WIDTH, CAM_WIDTH);
    cams.at(i)->set(CV_CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
    /* add buttons for filters */
    stringstream buttNamex, buttNamey, tbarName;
    buttNamex << "Cam " << boost::lexical_cast<string>(i) << ": flip x";
    buttNamey << "Cam " << boost::lexical_cast<string>(i) << ": flip y";
    tbarName  << "Cam " << boost::lexical_cast<string>(i) << ": brightness";
    createButton(buttNamex.str(), callbackFlipx, &flipped_x.at(i), CV_CHECKBOX);
    createButton(buttNamey.str(), callbackFlipy, &flipped_y.at(i), CV_CHECKBOX);
    //createTrackbar(tbarName.str(), NULL, &brightness.at(i), 255, callbackBrightness, &cams.at(i));
    createTrackbar(tbarName.str(), "", &brightness.at(i), 255, callbackBrightness, cams.at(i));

  }
 
  /* show webcams */
  Mat frame;
  while (1) {
    for (int i=0; i<cams.size(); i++) {
      /* grab frame */
      *cams.at(i) >> frame;
      /* apply filters */
      if (flipped_x.at(i)) flip(frame, frame, 1);
      if (flipped_y.at(i)) flip(frame, frame, 0);
      /* show image */

      imshow(boost::lexical_cast<string>(i), frame);
    }
    if(waitKey(30) == 'q')
      break;
  }

  return 0;
} 
