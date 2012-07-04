#include <iostream>

#include "../objects/occupancy_grid.hpp"

using namespace std;


void usage(char* name)
{
  cout << "Usage:" << endl;
  cout << " " << name << " <path>" << endl;
  cout << " where path is the path to an nvm, ply, out or txt file." << endl;
  exit(1);
}


/* main */
int main (int argc, char** argv)
{
  
  if (argc==1)
    usage(argv[0]);

  OccupancyGrid occgrid(argv[1]);
  occgrid.carve();

  if (argc>2) {
    // TODO: use extension (.bt, .ot) to determine boolean
    occgrid.save(argv[2], false);
    cout << "Result saved!" << endl;
  }

  return 0;
}
