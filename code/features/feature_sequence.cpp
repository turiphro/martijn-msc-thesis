#include "feature_sequence.hpp"

FeatureSequence::FeatureSequence(SequenceCapture* sc,
                                 const string& pointDetector,
                                 const string& pointDescriptor,
                                 const string& edgeDetector)
{
  this->sc = sc;
  // better parameter control? make less general objects
  fd = FeatureDetector::create(pointDetector);
  de = DescriptorExtractor::create(pointDescriptor);
  this->edgeDetector = edgeDetector;

  int frame_count = sc->getFrameCount();
  if (frame_count==-1)
    frame_count = 1; // infinite stream: 'one frame' only
  while (frame_count--) {
    kp.push_back(new vector<KeyPoint>);
    kd.push_back(new Mat());
    edges.push_back(new Mat());
  }
}

FeatureSequence::~FeatureSequence()
{
  while (kp.size()) {
    delete kp.back();
    kp.pop_back();
    delete kd.back();
    kd.pop_back();
    delete edges.back();
    edges.pop_back();
  }
}

void FeatureSequence::clearCache()
{
  int frame_count = sc->getFrameCount();
  if (frame_count==-1)
    frame_count = 1; // infinite stream: 'one frame' only
  for (int i=0; i<frame_count; i++) {
    kp.at(i)->clear();
  }
}

int FeatureSequence::getNearestKeypoint(Point2f point, int position)
{
  if (kp.at(position)->size()==0)
    return -1;

  KeyPoint* nearest = &kp.at(position)->at(0);
  double d = norm(nearest->pt - point);
  int index = 0;
  for (int i=1; i<kp.at(position)->size(); i++)
    if (norm(kp.at(position)->at(i).pt - point) < d) {
      nearest = &kp.at(position)->at(i);
      d = norm(nearest->pt - point);
      index = i;
    }
  return index;
}

bool FeatureSequence::calculateFeaturesNextFrame()
{
  int position = sc->getPosition();
  Mat frame;
  if (!sc->read(frame))
    return false;

  // check if already calculated
  if ( sc->isBounded() && (*kp.at(position)).size() > 0 )
    return true;

  // interest points
  fd->detect(frame, *kp.at(position));
  SIFT foo; // this prevents segfault somehow (which makes no sense)
  de->compute(frame, *kp.at(position), *kd.at(position));

  // edges
  Mat frame_bw;
  cvtColor(frame, frame_bw, CV_RGB2GRAY);
  if (edgeDetector.compare("HARRIS")==0) {
    cornerHarris(frame_bw, *edges.at(position), 2, 3, 0.04);
    // TODO: fix type of frame_bw
  } else {
    if (edgeDetector.compare("CANNY")!=0)
      cout << "[!!] Using CANNY instead of " << edgeDetector << endl;
    GaussianBlur(frame_bw, frame_bw, Size(0,0), 2);
    Canny(frame_bw, *edges.at(position), 50, 100);
  }

  // TODO: make point/edge graph

  return true;
}

bool FeatureSequence::calculateFeaturesOf(int frame)
{
  sc->setPosition(frame);
  return calculateFeaturesNextFrame();
}

// calculate all features for all frames
bool FeatureSequence::calculateFeatures()
{
  // this makes no sense for camera streams
  if (!sc->isOpened() || !sc->isBounded())
    return false;

  sc->setPosition(0);
  while (calculateFeaturesNextFrame())
    ;
  return true;
}

bool FeatureSequence::getFeaturesOf(int frame,
                                    vector<KeyPoint>& keypoints,
                                    Mat& descriptors,
                                    Mat& edges)
{
  keypoints = *kp.at(frame);
  descriptors = *kd.at(frame);
  edges = *this->edges.at(frame);
}

bool FeatureSequence::visualiseFeaturesOf(int position, Mat& output, bool richDraw, Scalar colourPoints, Scalar colourEdges)
{
  sc->setPosition(position);
  Mat frame;
  if (!sc->read(frame))
    return false;

  visualiseFeatures(frame, output,
                    kp.at(position), *edges.at(position),
                    richDraw, colourPoints, colourEdges);

  /* give some keypoints another colour:
  vector<KeyPoint> special_keypoints;
  for (int i=0; i<20; i++)
    special_keypoints.push_back( kp.at(position)->at(i));
  Mat foo;
  visualiseFeatures(output, output, &special_keypoints, foo,
                    richDraw, Scalar(0,255,0), colourEdges);
  */
}

bool FeatureSequence::visualiseFeatures(Mat& frame, Mat& output, vector<KeyPoint>* keypoints, Mat& frame_edges, bool richDraw, Scalar colourPoints, Scalar colourEdges)
{

  if (frame_edges.data != NULL) {
    // overlay edges on image
    Mat frame_edges_rgb, frame_minus_edges;
    vector<Mat> channels;
    channels.push_back(frame_edges*colourEdges[0]/255.0);
    channels.push_back(frame_edges*colourEdges[1]/255.0);
    channels.push_back(frame_edges*colourEdges[2]/255.0);
    merge(channels, frame_edges_rgb);
  
    add(frame, 0.0, frame_minus_edges, 255 - frame_edges);
    double alpha = 1, beta = 1;
    addWeighted(frame_minus_edges, alpha, frame_edges_rgb, beta, 0, output);
  } else {
    output = frame;
  }

  // draw interest points
  int drawing = (richDraw) ? DrawMatchesFlags::DRAW_RICH_KEYPOINTS : DrawMatchesFlags::DEFAULT;
  drawKeypoints(output, *keypoints, output, colourPoints, drawing);
  return true;
}


bool FeatureSequence::visualiseMatchesOf(int position1, int position2, Mat& output, bool flowDraw, bool richDraw, bool redrawImg, Scalar colourMatches, Scalar colourSingles)
{
  if ( !sc->isBounded()
    || position1 < 0 || position2 < 0
    || position1 >= sc->getFrameCount()
    || position2 >= sc->getFrameCount()
    || (*kp.at(position1)).size() == 0
    || (*kp.at(position2)).size() == 0
     )
     return false;

  int normType = (edgeDetector.compare("ORB")==0) ? NORM_HAMMING : NORM_L1;
  BFMatcher matcher(normType, true);
  //FlannBasedMatcher matcher;
  vector<DMatch> matches;
  vector<KeyPoint> kp1, kp2;
  Mat descr1, descr2;
  Mat frame1, frame2, edges;

  getFeaturesOf(position1, kp1, descr1, edges);
  getFeaturesOf(position2, kp2, descr2, edges);
  matcher.match(descr1, descr2, matches);
  if (!sc->setPosition(position1) || !sc->read(frame1))
    return false;
  if (!sc->setPosition(position2) || !sc->read(frame2))
    return false;

  // draw matches of interest points
  if (!flowDraw) {
    // two image matching drawing (as provided by OpenCV)
    int drawing = (richDraw) ? DrawMatchesFlags::DRAW_RICH_KEYPOINTS : DrawMatchesFlags::DEFAULT;
    if (!redrawImg)
      drawing = drawing & DrawMatchesFlags::DRAW_OVER_OUTIMG;
    drawMatches(frame1, kp1, frame2, kp2, matches, output,
                colourMatches, colourSingles, vector<char>(),
                drawing);
  } else {
    // flow vector drawing (user-defined function)
    if (redrawImg)
      output = frame2;
    drawFlowVectors(output, &kp1, &kp2, &matches, 50, colourMatches, colourSingles, true, richDraw);
  }

  return true;
}


void FeatureSequence::drawFlowVectors(Mat& output, vector<KeyPoint>* kp1, vector<KeyPoint>* kp2, vector<DMatch>* matches, int minDistance, Scalar colourMatches, Scalar colourSingles, bool redrawKeypoints, bool richDraw)
{
  // alternatively, use PCL's PCLHistogramVisualizer

  Point p1, p2;
  bool* inlier = new bool[kp2->size()];
  for (int i=0; i<kp2->size(); i++)
    inlier[i] = false;

  // draw flow vectors (lines)
  for (int i=0; i<matches->size(); i++) {
    p1 = kp1->at(matches->at(i).queryIdx).pt;
    p2 = kp2->at(matches->at(i).trainIdx).pt;
    if (norm(p2-p1) < minDistance) {
      line(output, p1, p2, colourMatches);
      inlier[matches->at(i).trainIdx] = true;
    }
  }

  // draw keypoints
  if (redrawKeypoints) {
    vector<KeyPoint> kp_in, kp_out;
    for (int i=0; i<kp2->size(); i++)
      if (inlier[i])
        kp_in.push_back(kp2->at(i));
      else
        kp_out.push_back(kp2->at(i));
    int drawing = (richDraw) ? DrawMatchesFlags::DRAW_RICH_KEYPOINTS : DrawMatchesFlags::DEFAULT;
    drawKeypoints(output, kp_in, output, colourMatches, drawing);
    drawKeypoints(output, kp_out, output, colourSingles, drawing);
  }
}

