#include <iostream>

#include "../io/sequence_capture.hpp"
#include "../objects/feature_sequence.hpp"

using namespace std;


void
usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <path> [<pointDetector> [<pointDescriptor> [<edgeDetector>]]]" << endl;
  cout << " where path is the path to an AVI file or";
  cout << " directory containing a sequence of images," << endl;
  cout << " pointDetector: {SIFT, SURF, ORB, FAST, STAR, MSER,";
  cout << " GFTT, HARRIS, Dense, SimpleBlob}," << endl;
  cout << " pointDescriptor: {SIFT, SURF, ORB, BRIEF}," << endl;
  cout << " edgeDetector: {CANNY, HARRIS}" << endl;
  exit(1);
}

int
main (int argc, char** argv)
{

  if (argc==1)
    usage(argv[0]);

  string pointDetector = (argc>2) ? argv[2]: "SIFT";
  string pointDescriptor = (argc>3) ? argv[3]: "SIFT";
  string edgeDetector = (argc>4) ? argv[4]: "CANNY";


  SequenceCapture* sc = new SequenceCapture(argv[1]);
  //SequenceCapture* sc = new SequenceCapture(0);
  FeatureSequence* fs = new FeatureSequence(sc, pointDetector,
                               pointDescriptor, edgeDetector);

  namedWindow("features", CV_WINDOW_NORMAL);
  Mat output;
  int count = sc->getFrameCount();
  for (int i=0; i<count || count==-1; i++) {
    cout << "Calculating features for frame " << i << endl;
    fs->calculateFeaturesNextFrame();
    fs->visualiseFeaturesOf(i*(count>-1), output);
    imshow("features", output);
    waitKey(100);
  }

  return (0);
}
