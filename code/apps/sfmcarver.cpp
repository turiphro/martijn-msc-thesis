#include <iostream>

#include "../objects/occupancy_grid.hpp"

using namespace std;


void usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <sfm> <imgs> <save>" << endl;
  cout << " where sfm is the path to an nvm, ply, out or txt file" << endl;
  cout << " imgs is the path to the directory containing the images" << endl;
  cout << " and save is the optional filename for saving the octree (.ot, or .bt for binary)" << endl;
  exit(1);
}


/* main */
int main (int argc, char** argv)
{
  
  if (argc<4)
    usage(argv[0]);

  // settings
  int carve = 1;
  int resolution = 500;
  double param1 = 0.1;
  bool extent = false;
  bool graphcut = false;

  // carve
  OccupancyGrid occgrid(argv[1], argv[2], resolution);
  if (extent)
    occgrid.extentVisibilityLists(0.2);
  occgrid.carve(true, true, carve, param1);
  if (graphcut)
    occgrid.graphcut(0.5);

  // save
  // TODO: use extension (.bt, .ot) to determine boolean
  occgrid.save(argv[3], false);
  cout << "Result saved as " << argv[3] << endl;

  return 0;
}
