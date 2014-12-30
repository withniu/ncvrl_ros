

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/flann/flann.hpp>

#include "SiftGPU.h"
#include "gl.h"
#include "image_feature.h"

int main()
{
  char * argv[] = {"-fo", "-1", "-tc2", "7680", "-da"};
  SiftGPU sift;
  sift.ParseParam(5, argv);
  int support = sift.CreateContextGL();
  if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
  {   
    printf("SiftGPU failed.");
    return -1;
  }   

  cv::Mat img = cv::imread("test.jpg");
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, CV_BGR2GRAY);

  sift.RunSIFT(img_gray.cols, img_gray.rows, img_gray.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);

  int num = sift.GetFeatureNum();
  //get feature count
  //allocate memory for readback
  std::vector<float> descriptors(128 * num);
  std::vector<SiftGPU::SiftKeypoint> keys(num);
  //read backkeypoints and normalized descritpros
  //specify NULL if you don’t need keypionts or descriptors
  sift.GetFeatureVector(&keys[0], &descriptors[0]);

  std::vector<cv::KeyPoint> keypoints;
  for (size_t i = 0; i < keys.size(); ++i)
  {
    SiftGPU::SiftKeypoint key = keys[i];
    cv::KeyPoint kp(key.x, key.y, key.s, -key.o / 3.1416 * 180);

    keypoints.push_back(kp);
  }

  cv::drawKeypoints(img, keypoints, img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imwrite("sift.png", img);
  //cv::Mat query_desc(num, 128, CV_32F, &descriptors[0]);
  cv::Mat query_desc(num, 128, CV_32F);
  for (int r = 0; r < num; ++r)
    for (int c = 0; c < 128; ++c)
    {
      query_desc.at<float>(r, c) = descriptors[r * 128 + c];
    }
  cv::Mat query_img;
  query_desc.convertTo(query_img, CV_8U, 512.0);
  cv::imwrite("query_img.png", query_img);
  cv::FileStorage fs("query_img.yml", cv::FileStorage::WRITE);
  fs << "query" << query_img;
  fs.release();
  

  ImageFeature imf;
  imf.load("00000.jpg");
  imf.draw();

  return 0;

}
