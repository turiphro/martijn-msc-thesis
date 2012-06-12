/* reader for output of SfM programs; extracts camera poses and points */
/* note: would be nice to split into seperate readers */

#ifndef SFMREADER_H
#define SFMREADER_H

#include <iostream>
#include <string>
#include <vector>
#include <dirent.h>
#include "../objects/Camera.hpp"
#include "../objects/InterestPoint.hpp"

using namespace std;


class SfMReader
{

  private:
    string path;
    vector<Camera> cameras;
    vector<InterestPoint> interestPoints;
    // Consider using a pointcloud for this
    // see example at: http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php#adding-custom-ptype

  public:
    SfMReader(string path);
    ~SfMReader();
    

}
