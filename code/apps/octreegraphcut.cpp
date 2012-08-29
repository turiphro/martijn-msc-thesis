#include <iostream>

#include "../objects/occupancy_grid.hpp"

using namespace std;


void usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <octree in> <octree out> [<gamma> [<unknown>]]" << endl;
  cout << " octree in/out: filename of octree file (.ot, or .bt for binary)" << endl;
  cout << " gamma:         weight of voxel prob difference for pairwise cost; default: 1" << endl;
  cout << " unknown:       occupancy probability of unknown (un-initialised) voxels; default: 0.2" << endl;
  exit(1);
}


/* main */
int main (int argc, char** argv)
{
  
  if (argc < 3)
    usage(argv[0]);

  // settings
  double gamma =   0.6; // penalty for discontinuity in space
                        // (weight for probability difference pairwise cost)
                        // > 1 does not seem to make a difference
  double unknown = 0.2; // prior; below 0.5 makes sense

  // command line arguments
  if (argc >= 4)
    gamma = atof(argv[3]);
  if (argc >= 5)
    unknown = atof(argv[4]);

  // open file
  OccupancyGrid occgrid;
  occgrid.load(argv[1]);
  occgrid.graphcut(gamma, unknown);

  // save; use extension (.bt, .ot) to determine file format
  bool binary = (string(argv[2]).find_last_of(".bt") > -1);
  occgrid.save(argv[2], binary);
  cout << "Result saved as " << argv[2];
  if (binary)
    cout << " (binary)";
  cout << endl;

  return 0;
}
