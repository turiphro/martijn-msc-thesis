=====================
    INSTALLATION
=====================

1. Install OpenCV (I used 2.4.1)

    Ubuntu:
     sudo apt-get install cmake pkg-config libavformat-dev libswscale-dev libavcodec-dev libavfilter-dev libpython2.7 python-dev python2.7-dev python-numpy libgstreamer0.10-0-dbg libgstreamer0.10-0  libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libdc1394-22-dev libdc1394-22 libdc1394-utils libavformat-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libgtk2.0-dev openni-dev sphinx-common
     mkdir release && cd release
     cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_PYTHON_SUPPORT=ON -D WITH_QT=ON -D WITH_XINE=ON -D WITH_V4L=ON -D WITH_OPENGL=ON -D WITH_OPENNI=ON -D WITH_TBB=ON -D BUILD_DOCUMENTATION=ON -D BUILD_EXAMPLES=ON release ..
     make && sudo make install

    debian instructions: http://opencv.willowgarage.com/wiki/InstallGuide%20%3A%20Debian


2. Install Point Cloud Library (I used 1.5)

    Ubuntu:
     sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
     sudo apt-get update
     sudo apt-get install libpcl-all

    Others:
     http://pointclouds.org/downloads/


3. Install OctoMap (I used 1.4.2)

    http://octomap.sourceforge.net/

    Ubuntu:
     sudo apt-get install cmake doxygen libqt4-dev libqt4-opengl-dev
     cd build && cmake .. && make && sudo make install


4. Install one or more of the following Structure from Motion estimators.
   VisualSFM seems to give the best results, so I recommend going for that one.
   Bundler and Voodoo output files are partly supported, but currently not
   everything will work.

   a. VisualSFM (I used 0.5.17)

        Instructions: http://www.cs.washington.edu/homes/ccwu/vsfm/install.html#linux
        Windows:
          seems to work fine with installed CUDA toolbox. You may get errors
          concerning some CUDA dll file; search and download it from The Internet and
          put it in the same directory as the VisualSfM executable.
        Linux:
          Need to install SiftGPU, which needs some hacks in the source code (!)
          Need to install PBA, which needs some hacks in the source code
          (include sys/types.h, stdio.h, stdlib.h)
          For GPU acceleration, need to install CUDA as well.
          Otherwise, download Lowe's sift binary (extract binary only to vsfm/bin/):
          http://www.cs.ubc.ca/~lowe/keypoints/

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
          download binary from http://phototour.cs.washington.edu/bundler/
        Linux (Ubuntu/Debian):
          sudo apt-get install gfortran libgfortran3 liblapack-dev libblas-dev \
                               libblacs-mpi-dev libminpack1 libf2c2-dev libann-dev
          (not sure which ones are really necessary)
          Compile from source (http://phototour.cs.washington.edu/bundler/)
          Need to make some patches as suggested by gcc.
          sudo cp lib/libANN_char.so /usr/lib/



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

todo

