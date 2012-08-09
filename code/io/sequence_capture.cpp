#include "sequence_capture.hpp"

// constructor for webcam device
SequenceCapture::SequenceCapture(int device)
{
  type = CAMERA;
  videoCapture = new VideoCapture(device);
  position = 0;
}

// constructor for avi files or directory with images
SequenceCapture::SequenceCapture(string path)
{
  if (path.length()>=4 &&
      path.substr(path.length()-4).compare(".avi") == 0) {
    // avi file
    type = VIDEO;
    videoCapture = new VideoCapture(path);
    position = 0;
  } else {
    // assuming directory with images
    type = IMAGES;
    this->path = path;
    position = 0;
    images.clear();
    struct dirent *dirstruct;
    DIR* dp = opendir(path.c_str());
    string d_name, ext;
    while ((dirstruct = readdir(dp)) != NULL) {
      d_name = string(dirstruct->d_name);
      // check if valid image (simply check the extension)
      ext = d_name.substr(d_name.find_last_of(".") + 1);
      boost::algorithm::to_lower(ext);
      // more than these can be supported; see
      // http://opencv.willowgarage.com/documentation/cpp/reading_and_writing_images_and_video.html#cv-imread
      if (ext == "png" || ext == "jpg" || ext == "jpeg" || ext == "bmp")
        images.push_back(d_name);
    }
    closedir(dp);
    sort(images.begin(), images.end());
    if (images.size()==0)
      cerr << "[!!] Warning: no valid images found in " << path << "!" << endl;
  }
}

SequenceCapture::~SequenceCapture()
{
  // release
  switch (type) {
   case CAMERA:
   case VIDEO:
    videoCapture->release();
    break;
   case IMAGES:
    break;
  }
}


bool SequenceCapture::isOpened()
{
  switch (type) {
   case CAMERA:
   case VIDEO:
    return videoCapture->isOpened();
   case IMAGES:
    return (images.size()>0);
  }
}

bool SequenceCapture::isBounded()
{
  return (type!=CAMERA);
}

bool SequenceCapture::read(Mat& image)
{
  switch (type) {
   case CAMERA:
   case VIDEO:
    return videoCapture->read(image);
   case IMAGES:
    if (position >= images.size())
      return false;
    image = imread(path + "/" + images.at(position++));
    return (image.data!=NULL);
  }
}

SequenceCapture& SequenceCapture::operator>>(Mat& image)
{
  read(image);
  return *this; // ?
}

bool SequenceCapture::setPosition(int position)
{
  // position is frame number / image, starting from 0
  switch (type) {
   case CAMERA:
    return true; // has no meaning for camera
   case VIDEO:
    double frames;
    frames = videoCapture->get(CV_CAP_PROP_FRAME_COUNT);
    return videoCapture->set(CV_CAP_PROP_POS_AVI_RATIO, (double)position/frames);
   case IMAGES:
    if (position >= images.size())
      return false;
    this->position = position;
    return true;
  }

}

bool SequenceCapture::setPosition(string filename)
{
  if (type != IMAGES) {
    cerr << "[!!] Error: this sequence is not based on an image directory" << endl;
    return false;
  }

  // remove directory names
  size_t found = filename.find_last_of("/\\");
  if (found != string::npos)
    filename = filename.substr(found+1);

  // search filename
  for (int i=0; i<images.size(); i++)
    if (images[i] == filename)
      return setPosition(i);

  cerr << "[!!] Filename " << filename << " not found in image directory (" << path << "): " << endl;
  return false;
}

int SequenceCapture::getPosition()
{
  return position;
}

int SequenceCapture::getFrameCount()
{
  switch (type) {
   case CAMERA:
    return -1;
   case VIDEO:
    return (int)videoCapture->get(CV_CAP_PROP_FRAME_COUNT);
   case IMAGES:
    return images.size();
  }
}
