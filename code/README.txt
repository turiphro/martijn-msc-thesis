=====================
    INSTALLATION
=====================

Installation instructions are provided for Linux (Ubuntu/Debian) and Windows. All paths are relative to the corresponding root of unpacked archive files.
The Structure from Motion tools are available online, or provided as Windows/Linux binaries in sfm/
Libraries are also available online, or provided in the mentioned versions as source code in lib/

1. Install OpenCV (I used 2.4.1)

    Website: http://opencv.willowgarage.com

    Linux Ubuntu, older version (2.3):
      $ sudo add-apt-repository ppa:gijzelaar/cuda
      $ sudo add-apt-repository ppa:gijzelaar/opencv2.3
      $ sudo apt-get update
      $ sudo apt-get install libcv-dev

    Linux Ubuntu, newest version from source:
     $ sudo apt-get install cmake pkg-config libavformat-dev libswscale-dev libavcodec-dev libavfilter-dev libpython2.7 python-dev python2.7-dev python-numpy libgstreamer0.10-0-dbg libgstreamer0.10-0  libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libdc1394-22-dev libdc1394-22 libdc1394-utils libavformat-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libgtk2.0-dev openni-dev sphinx-common
     $ mkdir release && cd release
     $ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_PYTHON_SUPPORT=ON -D WITH_QT=ON -D WITH_XINE=ON -D WITH_V4L=ON -D WITH_OPENGL=ON -D WITH_OPENNI=ON -D WITH_TBB=ON -D BUILD_DOCUMENTATION=ON -D BUILD_EXAMPLES=ON release ..
     $ make && sudo make install

    Windows: 
     Run binary executable from http://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.0/


2. Install Point Cloud Library (I used 1.6)

    Website: http://pointclouds.org

    Linux Ubuntu:
     $ sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
     $ sudo apt-get update;
     $ sudo apt-get install libpcl-all

    Others (including Windows):
     Run binary executable or compile source from http://pointclouds.org/downloads/


3. Install OctoMap (I used 1.4.2)

    Website: http://octomap.sourceforge.net

    Linux Ubuntu:
     $ sudo apt-get install cmake doxygen libqt4-dev libqt4-opengl-dev
     $ cd build && cmake .. && make && sudo make install

    Windows (Cygwin):
     $ cd build && cmake .. && make && make install


4. Install one or more of the following Structure from Motion estimators.
   VisualSFM seems to give the best results, so I recommend going for that one.
   Bundler and Voodoo output files are partly supported, but currently not
   everything will work.

   Website: http://www.cs.washington.edu/homes/ccwu/vsfm/

   a. VisualSFM (I used 0.5.17)

        Linux (tested on Ubuntu):
          Need to install SiftGPU, which needs some hacks in the source code (!)
          Need to install PBA, which needs some hacks in the source code as well
          (include sys/types.h, stdio.h, stdlib.h)
          For GPU acceleration, need to install CUDA as well.
          Otherwise, download Lowe's sift binary (extract binary only to vsfm/bin/):
          http://www.cs.ubc.ca/~lowe/keypoints/

        Windows:
          Executable seems to work fine after installing the CUDA toolbox. You may
          get errors concerning some CUDA dll file; search and download it from
          The Internet and put it in the same directory as the VisualSfM executable.

        Optionally, one could additionally install CMVS/PMVS for dense reconstruction
        (for comparison only; the dense output is not used for carving).

   b. Voodoo (I used 1.2.0 beta)

        Download binary from:
        http://www.digilab.uni-hannover.de/download.html

        Note that Voodoo does not export visibility information for feature points.
        Therefore, important parts of the program (i.e. carving) will not work using
        Voodoo's output. The viewers will work fine.

   c. Bundler (I used 0.4)

        Windows:
          Download and install binary from: http://phototour.cs.washington.edu/bundler/

        Linux (Ubuntu/Debian):
          $ sudo apt-get install gfortran libgfortran3 liblapack-dev libblas-dev \
                                 libblacs-mpi-dev libminpack1 libf2c2-dev libann-dev
          Compile from source (http://phototour.cs.washington.edu/bundler/)
          Need to make some patches as suggested by gcc.
          $ sudo cp lib/libANN_char.so /usr/lib/



5. Compile.

   Instructions:
     mkdir build && cd build
     cmake ..
     make

   Tested on Ubuntu 12.04. Should work on other operating systems as well.
   On Windows, you might want to install Cygwin for easy compiling and running.



=====================
       USAGE
=====================

1. Structure from Motion

   VisualSfM:
     1. Load images:               File -> Open+ Multiple Images
     2. Calc+Match SIFT features:  SfM  -> Pairwise Matching -> Compute Missing Match
     3. Reconstruct (Bundle Adj):  SfM  -> Reconstruct 3D
     4. Save sparse point cloud:   SfM  -> Save NV Match (.nvm)
     5. Dense (optional):          SfM  -> Run CMVS/PMVS

   Voodoo:
     1. Load images:               File -> Open -> Sequence
     2. Calc+Track features:       Track (bottom panel)
     3. Save:                      File -> Save -> Textfile (.txt)

   Bundler:
     1. Run provided script from within direct of images (Cygwin)
     2. Result is saved in bundle/bundle.out


2. Visualisation of SfM results

   ./sfmviewer <sfm> [<images>]
   where sfm is the path to an nvm, ply, out or txt file,
   and images is the path to the corresponding directory with images.

   Select points or cameras with Shift + Click.
   Enable/disable line drawing with 'd'.
   More shortcuts listed with 'h' (PCL) and '?' (custom).


3. Space Carving

   ./sfmcarver <sfm> <imgs> <save> [<method> [<resolution> [<param1>]]
     sfm:    path to an nvm, ply, out or txt file
     imgs:   path to the directory containing the images
     save:   filename for saving the octree (.ot, or .bt for binary)
     method: carve method (0-3); default: 2;
             0=discretised, 1=vis, 2=vis+occ veto, 3=vis+occ+extend vis lists
     resol.: voxelgrid sizes (determines smallest octree node size); default: 250
     param1: first parameter of given method; default: 0.1


4. Visualisation of Space Carve results

   octovis <carve>
     where carve is an octree file (.ot or .bt)

   or:

   ./carveviewer <sfm> <imgs> <carve>
     where sfm is the path to an nvm, ply, out or txt file
     imgs is the path to the directory containing the images
     and carve is an octree file (.ot or .bt)
   

5. Optional: regularisation

   ./octreegraphcut <octree in> <octree out> [<gamma> [<unknown>]]
     octree in/out: filename of octree file (.ot, or .bt for binary)
     gamma:         weight of voxel prob difference for pairwise cost; default: 1
     unknown:       occupancy probability of unknown (un-initialised) voxels; default: 0.2


6. Optional: feature visualisation

   ./featureviewer <path> [<pointDetector> [<pointDescriptor> [<edgeDetector>]]]
     where path is the path to an AVI file, directory containing
     a sequence of images, or webcam device number;
     pointDetector: {SIFT, SURF, ORB, FAST, STAR, MSER, GFTT, HARRIS, Dense, SimpleBlob};
     pointDescriptor: {SIFT, SURF, ORB, BRIEF};
     edgeDetector: {CANNY, HARRIS}


