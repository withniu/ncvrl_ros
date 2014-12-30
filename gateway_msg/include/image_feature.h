#ifndef IMAGE_FEATURE_H_
#define IMAGE_FEATURE_H_

#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//#include <glog/logging.h>

enum
{
//    READ_BUFFER_SIZE = 0x100000,^M
  SIFT_NAME= ('S'+ ('I'<<8)+('F'<<16)+('T'<<24)),
  MSER_NAME= ('M'+ ('S'<<8)+('E'<<16)+('R'<<24)),
  RECT_NAME= ('R'+ ('E'<<8)+('C'<<16)+('T'<<24)),
  //SIFT_VERSION_2=('V'+('2'<<8)+('.'<<16)+('0'<<24)),
  //SIFT_VERSION_3=('V'+('3'<<8)+('.'<<16)+('0'<<24)),
  SIFT_VERSION_4=('V'+('4'<<8)+('.'<<16)+('0'<<24)),
  SIFT_VERSION_5=('V'+('5'<<8)+('.'<<16)+('0'<<24)),
  SIFT_EOF = (0xff+('E'<<8)+('O'<<16)+('F'<<24)),
};



class ImageFeature
{
private:
  std::vector<cv::KeyPoint> keypoints_;		// Keypoints
  cv::Mat descriptors_;				// Descriptor
public:

  cv::Mat img_;					//< Image
  ImageFeature() 
  {
    cv::namedWindow("vis");
  }
  virtual ~ImageFeature() {}

  /** \brief Load image and feature binary file
   *
   */
  void load(std::string filename)
  {

    //    LOG(INFO) << "Loading image " << filename << "...";
    img_ = cv::imread(filename);
//    LOG(INFO) << "Loaded.";
    std::string filename_sift = filename.substr(0, filename.size() - 3) + "sift";
    std::ifstream ifs(filename_sift.c_str(), std::ios::binary | std::ios::in);
 
    int name, version, num_pts, dim_loc, dim_des, sift_eof, sorted = 0;
    ifs.read((char *)&name, sizeof(int));
    ifs.read((char *)&version, sizeof(int));
    // TODO: check name and version
/*    if (name != SIFT_NAME)
      std::cout << "Invalid name." << std::endl;
    if (version != SIFT_VERSION_4)
      std::cout << "Invalid value4." << std::endl;
    if (version != SIFT_VERSION_5)
      std::cout << "Invalid value5." << std::endl;
*/
    ifs.read((char *)&num_pts, sizeof(int));
    ifs.read((char *)&dim_loc, sizeof(int));
    ifs.read((char *)&dim_des, sizeof(int));
    
//    printf("\n%d, %d, %d, %d, %d\n", name, version, num_pts, dim_loc, dim_des);
    
    if (num_pts > 0 && dim_loc > 0 && dim_des == 128)
    {
      char *data_loc = new char[num_pts * dim_loc * sizeof(float)];
      unsigned char *data_des = new unsigned char[num_pts * dim_des * sizeof(unsigned char)];
      ifs.read((char *)data_loc, num_pts * dim_loc * sizeof(float));
      ifs.read((char *)data_des, num_pts * dim_des * sizeof(unsigned char));
      ifs.read((char *)&sift_eof, sizeof(int));
      keypoints_.resize(num_pts);
      cv::Mat descriptors(num_pts, dim_des, CV_8U, (unsigned char *)data_des);
      descriptors.copyTo(descriptors_);
      
      
      //      std::cout << descriptors_.row(1) << std::endl;
      
      for (size_t i = 0; i < num_pts; ++i)
      {
        float *x = (float *)data_loc + i * dim_loc + 0;
        float *y = (float *)data_loc + i * dim_loc + 1;
        float *s = (float *)data_loc + i * dim_loc + 3;
        float *o = (float *)data_loc + i * dim_loc + 4;

        keypoints_[i] = cv::KeyPoint(*x, *y, *s, (*o) / M_PI * 180);
      
//        std::cout << *x << ", " << *y << ", " << *o << ", " << *s << std::endl;
     
      }



      delete data_loc;
      delete data_des;
    }
    ifs.close();
  }
  
  void getDescriptorRow(int idx_kp, int idx_dst, cv::Mat &des) const
  {
    descriptors_.row(idx_kp).copyTo(des.row(idx_dst));
  
  }
  
  cv::KeyPoint getKeypoint(int idx_kp) const
  {
    return keypoints_[idx_kp];
  }
  
  /** \brief Load SIFT features and a point for visualiztion NVM point
   *  \param[in] idx_kp Index of keypoint, -1 for all
   *  \param[in] pt Point to draw
   */
  void draw(int idx_kp = -1, cv::Point2f pt = cv::Point2f(0.0, 0.0)) 
  { 
    std::vector<cv::KeyPoint> keypoints;
    if (idx_kp < 0)
      keypoints = keypoints_;
    else
      keypoints.push_back(keypoints_[idx_kp]);

    cv::Mat img;
    cv::drawKeypoints(img_, keypoints, img, cv::Scalar::all(-1), 4);
//    cv::circle(img, pt, 15.0, cv::Scalar(255, 0, 0), 10);
    double d = 15;
    cv::line(img, pt + cv::Point2f(d, 0), pt - cv::Point2f(d, 0), cv::Scalar(0, 0, 255));
    cv::line(img, pt + cv::Point2f(0, d), pt - cv::Point2f(0, d), cv::Scalar(0, 0, 255));


    cv::FileStorage fs("load_img.yml", cv::FileStorage::WRITE);
    fs << "load" << descriptors_;
    fs.release();

    cv::imshow("vis", img);
    cv::imshow("descriptor", descriptors_);
    
    cv::waitKey(0);
  }

};



#endif
