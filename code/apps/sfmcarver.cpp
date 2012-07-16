#include <iostream>

#include "../objects/occupancy_grid.hpp"

using namespace std;


void usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <sfm> [<save>]" << endl;
  cout << " where sfm is the path to an nvm, ply, out or txt file" << endl;
  cout << " and save is the optional filename for saving the octree" << endl;
  exit(1);
}


/* main */
int main (int argc, char** argv)
{
  
  if (argc==1)
    usage(argv[0]);

  OccupancyGrid occgrid(argv[1], 1000);
  occgrid.carve(true, false, 0);

  if (argc>2) {
    // TODO: use extension (.bt, .ot) to determine boolean
    occgrid.save(argv[2], false);
    cout << "Result saved!" << endl;
  }

  return 0;
}
