#include "keypoint_tracker.hpp"

KeypointTracker::KeypointTracker(vector<vector<KeyPoint> *>* kpp, vector<Mat *>* kdp, int kp_ref, int frame_ref, int type)
{
  this->kpp = kpp;
  this->kdp = kdp;
  this->kp_ref = kp_ref;
  this->frame_ref = frame_ref;
  this->frame_lastseen = frame_ref;
  this->frame_lasttested = frame_ref;
  //this->type = type;
  if (type!=0)
    cout << "[!!] Not implemented yet: other types!" << endl;
  track = new int[kpp->size()];
  for (int i=0; i<kpp->size(); i++)
    track[i] = -1;
  track[frame_ref] = kp_ref;
}

KeypointTracker::~KeypointTracker()
{
  delete[] track;
}


int KeypointTracker::bestMatch(int position1, int position2, double maxRatio)
{
  if (track[position1] == -1)
    return -1;                // start position unknown
  if (track[position2] != -1)
    return track[position2];  // recal from memory

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
  // track backwards
  if (position < frame_lasttested) {
    // TODO
    cout << "[!!] TO IMPLEMENT: backwards tracking" << endl;
    return;
  }
  
  // track forward
  for (int i = frame_lasttested+1; i <= position; i++) {
    track[i] = bestMatch(frame_lastseen, i);
    frame_lasttested = i;
    if (track[i] != -1)
      frame_lastseen = i;
  }

}

int KeypointTracker::getBestMatch(int position)
{
  return track[position];
}

