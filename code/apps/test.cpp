#include <iostream>

#include "../io/sequence_capture.hpp"
#include "../objects/feature_sequence.hpp"

using namespace std;


void
usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <path>" << endl;
  cout << " where path is the path to an AVI file or";
  cout << " directory containing a sequence of images" << endl;
  exit(1);
}

int
main (int argc, char** argv)
{

  if (argc==1)
    usage(argv[0]);

  SequenceCapture* sc = new SequenceCapture(argv[1]);
  FeatureSequence* fs = new FeatureSequence(sc, "SIFT");

  namedWindow("features");
  Mat output;
  for (int i=0; i<sc->getFrameCount(); i++) {
    cout << "Calculating features for frame " << i << endl;
    fs->calculateFeaturesNextFrame();
    fs->visualiseFeaturesOf(i, output);
    imshow("features", output);
    waitKey(300);
  }

  return (0);
}
