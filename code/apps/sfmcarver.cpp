#include <iostream>

#include "../objects/occupancy_grid.hpp"

using namespace std;


void usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <sfm> <imgs> <save> [<method> [<resolution> [<param1>]]" << endl;
  cout << " sfm:    path to an nvm, ply, out or txt file" << endl;
  cout << " imgs:   path to the directory containing the images" << endl;
  cout << " save:   filename for saving the octree (.ot, or .bt for binary)" << endl;
  cout << " method: carve method (0-2); default: 2" << endl;
  cout << " resol.: voxelgrid sizes (determines smallest octree node size); default: 250" << endl;
  cout << " param1: first parameter of given method; default: 0.1" << endl;
  exit(1);
}


/* main */
int main (int argc, char** argv)
{
  
  if (argc < 4)
    usage(argv[0]);

  // settings
  int method = 2;
  int resolution = 250;
  double param1 = 0.01;
  bool extent = false;
  bool graphcut = false;

  // command line arguments
  if (argc >= 5)
    method = atoi(argv[4]);
  if (argc >= 6)
    resolution = atoi(argv[5]);
  if (argc >= 7)
    param1 = atof(argv[6]);

  // carve
  OccupancyGrid occgrid(argv[1], argv[2], resolution);
  if (extent)
    occgrid.extentVisibilityLists(0.2);
  occgrid.carve(method, param1);
  if (graphcut)
    occgrid.graphcut(0.5);

  // save; use extension (.bt, .ot) to determine file format
  bool binary = (string(argv[3]).find_last_of(".bt") > -1);
  occgrid.save(argv[3], binary);
  cout << "Result saved as " << argv[3];
  if (binary)
    cout << " (binary)";
  cout << endl;

  return 0;
}
