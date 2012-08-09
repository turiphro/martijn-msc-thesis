#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../objects/occupancy_grid.hpp"

using namespace std;


void usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <sfm> <imgs> <carve>" << endl;
  cout << " where sfm is the path to an nvm, ply, out or txt file" << endl;
  cout << " imgs is the path to the directory containing the images" << endl;
  cout << " and carve is an octree file (.ot or .bt)" << endl;
  exit(1);
}


/* main */
int main (int argc, char** argv)
{

  if (argc<4)
    usage(argv[0]);

  // make octree object
  OccupancyGrid occgrid(argv[1], argv[2], 1);
  occgrid.load(argv[3]);

  /*
  // calculate annotated images
  vector<Mat> output;
  cout << "Annotating images.." << endl;
  occgrid.visualisePath(&output, "circle", 0.5);
  */

  // visualise
  cout << "Visualise.." << endl;
  int k;
  int i=0, last_i=-1;
  int count = occgrid.sfm->poses.size();
  bool paused = false;
  VideoWriter* recorder = NULL;
  Mat output;
  namedWindow("visualise", CV_WINDOW_NORMAL);
  while (1) {
    if (occgrid.visualisePose(output, i, "circle", 0.2)) {

      // TODO: buffer

      imshow("visualise", output);
      //imshow("visualise", output.at(i));
      
      // if recording, save frame
      if (last_i != i && recorder)
        *recorder << output;
      last_i = i;

    } else {
      cout << "Image not used for carving: " << i << endl;
    }

    k = waitKey(100);
    switch (k) {
      case '?':
        cout << "Keyboard shortcuts:" << endl;
        cout << " q/ESC:    quit" << endl;
        cout << " --- navigation ---" << endl;
        cout << " r:        refresh" << endl;
        cout << " space:    pause/resume" << endl;
        cout << " ->/n:     next frame" << endl;
        cout << " <-/p:     prev frame" << endl;
        cout << " PGUP:     +25 frames" << endl;
        cout << " PGDN:     -25 frames" << endl;
        cout << " f/b/HOME: first frame" << endl;
        cout << " l/e/END:  last frame" << endl;
        cout << " g + tty#: goto frame #" << endl;
        cout << " --- various ---" << endl;
        cout << " s         start/stop saving visible frames" << endl;
        break;
      case 'q':
      case 27:  // ESC
        destroyWindow("visualise");
        exit(0);
      case 'r':
        last_i = -1; // refresh
        break;
      case 's':
        if (recorder) {
          // stop recording
          cout << "Stopped recording at frame " << i << endl;
          delete recorder;
          recorder = NULL;
        } else {
          // start recording
          cout << "Starting recording from frame " << i << endl;
          time_t tim; time(&tim); string ctim = (string)ctime(&tim);
          string record_filename = "output " + ctim.substr(0,ctim.size()-1) + ".avi";
          recorder = new VideoWriter(record_filename, CV_FOURCC('D','I','V','X'),
                                     10, output.size(), true);
          if (paused)
            last_i = -1; // save next frame if paused
        }
        break;
      case ' ':
      case 13:  // return
        paused = !paused;
        break;
      case 'n':
      case 65363: // right arrow
        i++;
        if (i==count)
          i--;
        break;
      case 'p':
      case 65361: // left arrow (same for all keyboards?)
        i--;
        if (i<0)
          i=0;
        break;
      case 65365: // page up
        i = max(0, i-25);
        break;
      case 65366: // page down
        i = min(count-1, i+25);
        break;
      case 'f':
      case 'b':
      case 65360: // home
        i = 0;
        break;
      case 'l':
      case 'e':
      case 65367: // end
        i = count-1;
        break;
      case 'g': // goto
        cout << "goto: ";
        cin >> i;
        if (i<0)
          i = 0;
        if (i>=count)
          i = count-1;
        break;
      }
      if (!paused) {
        i++;
        if (i==count) {
          i--;
          paused = true;
        }
      }
  }

  return 0;
}
