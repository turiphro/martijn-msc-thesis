#include "keypoint_tracker.hpp"

KeypointTracker::KeypointTracker(vector<vector<KeyPoint> *>* kpp, vector<Mat *>* kdp, int kp_ref, int frame_ref, int type)
{
  this->kpp = kpp;
  this->kdp = kdp;
  this->kp_ref = kp_ref;
  this->frame_ref = this->frame_lastseen = this->frame_firstseen = frame_ref;
  this->frame_lasttested = this->frame_firsttested = frame_ref;
  //this->type = type;
  if (type!=0)
    cout << "[!!] Not implemented yet: other types!" << endl;

  track = new int[kpp->size()]; // assuming size will not change
  for (int i=0; i<kpp->size(); i++)
    track[i] = -1;
  track[frame_ref] = kp_ref;

  distinctiveness = new double[kpp->size()];
  for (int i=0; i<kpp->size(); i++)
    distinctiveness[i] = 0;
  distinctiveness[frame_ref] = 1;
}

KeypointTracker::~KeypointTracker()
{
  delete[] track;
  delete[] distinctiveness;
}


int KeypointTracker::bestMatch(int position1, int position2, double maxRatio)
{
  if (track[position2] > -1)
    return track[position2];  // recal from memory
  if (track[position1] == -1)
    return -2;                // error: start position unknown
  if (kdp->at(position1)->data == NULL || kdp->at(position2)->data == NULL)
    return -2;                // error: not all features calculated yet

  // BruteForce, BruteForce-L1, BruteForce-Hamming, FlannBased
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
  vector<vector<DMatch> > matches;
  Mat mask = Mat::zeros( kdp->at(position1)->size[0],
                         kdp->at(position2)->size[0],
                         0 );
  for (int j=0; j<mask.size[1]; j++)
    mask.at<uchar>(track[position1], j) = 1;
  matcher->knnMatch(*kdp->at(position1),
                    *kdp->at(position2),
                    matches,
                    2,
                    mask,
                    true);

  /*
  cout << "Matches (" << position1 << "," << position2 << ")" << endl;
  for (int i=0; i<matches.at(0).size(); i++)
    cout << matches.at(0).at(i).queryIdx << " - " << matches.at(0).at(i).trainIdx << ": " << matches.at(0).at(i).distance << endl;
  */

  // ratio test
  double ratio = matches.at(0).at(0).distance / matches.at(0).at(1).distance;
  distinctiveness[position2] = 1 - ratio;
  cout << "ratio for frame " << position2 << ": " << ratio << endl;

  if (ratio < maxRatio)
    return matches.at(0).at(0).trainIdx;
  else // feature not distinctive enough
    return -1;

}

void KeypointTracker::trackAll()
{
  trackUntil(kpp->size());
}

void KeypointTracker::trackUntil(int position)
{
  if (position < frame_firsttested) {
    // track backwards
    for (int i = frame_firsttested-1; i >= position; i--) {
      track[i] = bestMatch(frame_firstseen, i);
      if (track[i] == -2)
        return; // error: stop tracking
      frame_firsttested = i;
      if (track[i] > -1)
        frame_firstseen = i;
    }
  } else if (position > frame_lasttested) {
    // track forward
    for (int i = frame_lasttested+1; i <= position; i++) {
      track[i] = bestMatch(frame_lastseen, i);
      if (track[i] == -2)
        return; // error: stop tracking
      frame_lasttested = i;
      if (track[i] > -1)
        frame_lastseen = i;
    }
  }
  // else: already done
}

int KeypointTracker::getBestMatch(int position)
{
  return track[position];
}

void KeypointTracker::plotDistinctiveness(Mat& output, int frame_curr)
{
  plot(output, distinctiveness, kpp->size(), frame_curr);
}

void KeypointTracker::plot(Mat& output, double *data, int count, int frame_curr, Scalar colour, bool reDraw, Size size)
{
  output = Mat::zeros(size, CV_8UC3);
  rectangle(output, Point(0,0), Point(size), Scalar(255,255,255),-1);
  // find scale
  double x_max = (double) count;
  double y_max = 0.0;
  for (int i=0; i<count; i++)
    if (data[i]>y_max)
      y_max = data[i];
  double step_x = (double)(size.width-10)/x_max;
  double step_y = (double)(size.height-10)/y_max;

  // draw axes
  line(output, Point(5,5), Point(5, size.height-5), Scalar(50,50,50));
  line(output, Point(5,size.height-5), Point(size.width-5, size.height-5), Scalar(50,50,50));

  // draw graph
  for (int i=1; i<count; i++)
    line(output,
         Point(5 + step_x*(i-1), size.height - 5 - data[i-1]*step_y ),
         Point(5 + step_x*(i),   size.height - 5 - data[i]*step_y),
         colour,
         1,
         4);

  // draw init point
  circle(output, Point(5 + step_x*frame_ref, 5), 3, Scalar(0,255,0), -1);
  // draw current frame
  if (frame_curr>-1)
    circle(output, Point(5 + step_x*frame_curr, size.height - 5 - data[frame_curr]*step_y), 3, Scalar(255,0,0), -1);

}

