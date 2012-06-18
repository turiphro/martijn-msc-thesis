=====================
Software requirements
=====================

1. Install OpenCV (I used 2.4.1)

    Ubuntu:
     sudo apt-get install cmake pkg-config libavformat-dev libswscale-dev libavcodec-dev libavfilter-dev libpython2.7 python-dev python2.7-dev python-numpy libgstreamer0.10-0-dbg libgstreamer0.10-0  libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libdc1394-22-dev libdc1394-22 libdc1394-utils libavformat-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libtbb-dev libqt4-dev libgtk2.0-dev openni-dev sphinx-common
     mkdir release && cd release
     cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_PYTHON_SUPPORT=ON -D WITH_QT=ON -D WITH_XINE=ON -D WITH_V4L=ON -D WITH_OPENGL=ON -D WITH_OPENNI=ON -D WITH_TBB=ON -D BUILD_DOCUMENTATION=ON -D BUILD_EXAMPLES=ON release ..
     make && sudo make install

    debian instructions: http://opencv.willowgarage.com/wiki/InstallGuide%20%3A%20Debian


2. Install Point Cloud Library (I got 1.5)

    Ubuntu:
     sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
     sudo apt-get update
     sudo apt-get install libpcl-all

    Others:
     http://pointclouds.org/downloads/


3. Download Voodoo (I got 1.2.0 beta)

    http://www.digilab.uni-hannover.de/download.html


4. Install VisualSFM (I got 0.5.17)

    Instructions: http://www.cs.washington.edu/homes/ccwu/vsfm/install.html#linux
    Need to install SiftGPU, which needs some hacks in the source code (!)
    Need to install PBA, which needs some hacks in the source code (include sys/types.h, stdio.h, stdlib.h)
    For GPU acceleration, need to install CUDA as well.
    Download Lowe's sift binary (extract binary only to vsfm/bin/): http://www.cs.ubc.ca/~lowe/keypoints/


5. Install Bundler (I got 0.4)

    sudo apt-get install gfortran libgfortran3 liblapack-dev libblas-dev libblacs-mpi-dev libminpack1 libf2c2-dev libann-dev
    (not sure which ones are really necessary)
    Compile from source.
    Need to make some patches as suggested by gcc.
    sudo cp lib/libANN_char.so /usr/lib/



Compile instructions:

 mkdir build && cd build
 cmake ..
 make

