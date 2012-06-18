#include <iostream>

#include "../io/sequence_capture.hpp"
#include "../features/feature_sequence.hpp"
#include "../features/keypoint_tracker.hpp"

using namespace std;


void
usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <path> [<pointDetector> [<pointDescriptor> [<edgeDetector>]]]" << endl;
  cout << " where path is the path to an AVI file, directory containing" << endl;
  cout << " a sequence of images, or webcam device number;" << endl;
  cout << " pointDetector: {SIFT, SURF, ORB, FAST, STAR, MSER,";
  cout << " GFTT, HARRIS, Dense, SimpleBlob};" << endl;
  cout << " pointDescriptor: {SIFT, SURF, ORB, BRIEF};" << endl;
  cout << " edgeDetector: {CANNY, HARRIS}" << endl;
  exit(1);
}

/* useful functions */
bool
isInteger(char* arg)
{
  while (*arg)
    if (*arg<='9' && *arg>='0')
      arg++;
    else
      return false;
  return true;
}

/* callback */
typedef struct trackerData {
  Point point;
  bool startTracking;
} trackerData;

void onMouse(int event, int x, int y, int, void* td)
{
  if (event != CV_EVENT_LBUTTONDOWN)
    return;

  ((trackerData*)td)->point = Point(x,y);
  ((trackerData*)td)->startTracking = true;
}


/* main */
int
main (int argc, char** argv)
{

  if (argc==1)
    usage(argv[0]);

  string pointDetector = (argc>2) ? argv[2]: "SIFT";
  string pointDescriptor = (argc>3) ? argv[3]: "SIFT";
  string edgeDetector = (argc>4) ? argv[4]: "CANNY";

  SequenceCapture* sc;
  if (isInteger(argv[1]))
    sc = new SequenceCapture(atoi(argv[1]));
  else
    sc = new SequenceCapture(argv[1]);

  FeatureSequence* fs = new FeatureSequence(sc, pointDetector,
                               pointDescriptor, edgeDetector);

  namedWindow("features", CV_WINDOW_NORMAL);

  trackerData td;
  td.startTracking = false;
  setMouseCallback("features", onMouse, &td);
  KeypointTracker* tracker = NULL;

  bool paused = false;
  bool richDraw = true, matchDraw = true;
  Mat output;
  int count = sc->getFrameCount();
  int i = 0, last_i = -1;
  while (true) {
    if (count>-1 && i>=count-1 && !paused) {
      i = count - 1; // end of animation: stay at last frame
      paused = true;
    }
    if (!paused || i != last_i) {
      cout << "Calculating features for frame " << i << endl;
      fs->calculateFeaturesOf(i*(count>-1));
      fs->visualiseFeaturesOf(i*(count>-1), output, richDraw);
      if (matchDraw)
        fs->visualiseMatchesOf(i-1, i, output, true, richDraw, false);
      
      if (td.startTracking) {
        td.startTracking = false;
        cout << "Starting tracking at location " << td.point << " for frame " << i << endl;
        Point2f point(td.point.x, td.point.y);
        int nearest = fs->getNearestKeypoint(point, i);
        if (tracker)
          delete tracker;
        tracker = new KeypointTracker(&fs->kp, &fs->kd, nearest, i);
      }
      // draw tracked interest point
      if (tracker) {
        tracker->trackUntil(i);
        if (tracker->getBestMatch(i) != -1)
          circle(output, fs->kp.at(i)->at(tracker->getBestMatch(i)).pt, 10, Scalar(0,255,0), 4);
      }

      imshow("features", output);
    }
    last_i = i;
    switch(waitKey(30)) {
     case '?':
      cout << "Keyboard shortcuts:" << endl;
      cout << " q/ESC:    quit" << endl;
      cout << " --- navigation ---" << endl;
      cout << " space:    pause/resume" << endl;
      cout << " ->/n:     next frame" << endl;
      cout << " <-/p:     prev frame" << endl;
      cout << " PGUP:     +25 frames" << endl;
      cout << " PGDN:     -25 frames" << endl;
      cout << " f/b/HOME: first frame" << endl;
      cout << " l/e/END:  last frame" << endl;
      cout << " l/e/END:  last frame" << endl;
      cout << " --- visuals ---" << endl;
      cout << " 1:        rich keypoint draw on/off" << endl;
      cout << " 2:        match arrows draw on/off" << endl;
      cout << " --- tracking ---" << endl;
      cout << " <click>   start tracking interest point" << endl;
      cout << " t         stop tracking" << endl;
      break;
     case 'q':
     case 27: // esc
      exit(0);
     case 32: // space
     case 13: // return
      paused = !paused;
      break;
     case 'p':
     case 65361: // left arrow (same for all keyboards?)
      if (i>0 && count>-1)
        i--;
      break;
     case 'n':
     case 65363: // right arrow
      if (i+1<count || count==-1)
        i++;
      break;
     case 65365: // page up
      if (count>-1)
        i = max(0, i-25);
      break;
     case 65366: // page down
      if (count != -1)
        i = min(count-1, i+25);
      break;
     case 'f':
     case 'b':
     case 65360: // home
      i = (count==-1) ? -1 : 0;
      break;
     case 'l':
     case 'e':
     case 65367: // end
      i = (count==-1) ? -1 : count-1;
      break;
     case '1':
      richDraw = !richDraw;
      last_i = -2; // refresh
      break;
     case '2':
      matchDraw = !matchDraw;
      last_i = -2; // refresh
      break;
     case 't':
      if (tracker) {
        delete tracker;
        tracker = NULL;
        last_i = -2; // refresh
      }
      break;
    }

    if (!paused)
      i++;
  }

  return (0);
}
