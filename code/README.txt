=====================
Software requirements
=====================

>> OpenCV 2.3.1

    Ubuntu:
     sudo apt-get install cmake pkg-config libavformat-dev libswscale-dev libavcodec-dev libavfilter-dev libpython2.7 python-dev python2.7-dev python-numpy libgstreamer0.10-0-dbg libgstreamer0.10-0  libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libdc1394-22-dev libdc1394-22 libdc1394-utils 
     mkdir release && cd release
     cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_PYTHON_SUPPORT=ON -D BUILD_EXAMPLES=ON release ..
     make && sudo make install

    debian instructions: http://opencv.willowgarage.com/wiki/InstallGuide%20%3A%20Debian
