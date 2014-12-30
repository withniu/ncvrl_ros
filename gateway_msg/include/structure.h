#ifndef STRUCTURE_H_
#define STRUCTURE_H_

#include <string>
#include <fstream>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <flann/flann.hpp>
//#include <glog/logging.h>
#include "image_feature.h"

#define DIM_DES 128
//#define DEBUG_MATCHING

struct Point
{
  double x, y, z;
  unsigned char r, g, b;
  int num_obs;
};

class Structure
{
private:
  std::vector<cv::Point3f> points_;           //< 3D metric points
  flann::Matrix<float> *database_;            //< FLANN Database
  flann::Index<flann::L2<float> > *kdtree_;   //< FLANN k-d tree
#ifdef DEBUG_MATCHING  
  std::vector<cv::Mat> patches_;              //< Feature patches for debug
#endif

public:
  // TODO:
  cv::Mat descriptors_;
  
  Structure() 
  {
#ifdef DEBUG_MATCHING  
    cv::namedWindow("patch");
#endif
  }

  virtual ~Structure() {}


  /** \brief Save points to PCD ascii format, use color (255,255,255)
   *  \param[in] filename PCD filename
   */
  void savePCD(const std::string &filename) const
  {
    std::ofstream ofs(filename.c_str(), std::ios::out);
    ofs << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    ofs << "VERSION .7" << std::endl;
    ofs << "FIELDS x y z rgb" << std::endl;
    ofs << "SIZE 4 4 4 4" << std::endl;
    ofs << "TYPE F F F F" << std::endl;
    ofs << "COUNT 1 1 1 1" << std::endl;
    ofs << "HEIGHT 1" << std::endl;
    ofs << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    ofs << "WIDTH " << points_.size() << std::endl;
    ofs << "POINTS " << points_.size() << std::endl;
    ofs << "DATA ascii" << std::endl;

    for (size_t i = 0; i < points_.size(); ++i)
    {
      const cv::Point3f &pt = points_[i];
      // Use white as default color
      ofs << pt.x << " " << pt.y << " " << pt.z << " " << float(255 << 16 | 255 << 8 | 255) << std::endl;
    }
    ofs.close();
  }

  /** \brief Load NVM
   *  \param[in] filename NVM filename
   */
  void load(const std::string &filename)
  {
    std::ifstream ifs(filename.c_str(), std::ios::in);
    std::string line;
    
    // 0. Header
    std::getline(ifs, line);    // Header
    std::getline(ifs, line);    // Blank line

    // 1. Camera
    int num_cam;
    ifs >> num_cam;
    std::cout << num_cam;
    std::getline(ifs, line);    // Blank line
    std::vector<std::string> filenames(num_cam);
    for (size_t i = 0; i < num_cam; ++i)
    {
      std::getline(ifs, line);
      std::stringstream ss(line);
      ss >> filenames[i];
      //std::cout << line << std::endl;
    }
    // Sift features of each image
    std::vector<ImageFeature> imfs(num_cam);
    for (size_t i = 0; i < num_cam; ++i)
    {
      std::cout << i << ", " << filenames[i] << std::endl;
      imfs[i].load(filenames[i]);
    }
    
    std::getline(ifs, line);    // Blank line
    
    // 2. Points
    int num_pts;
    ifs >> num_pts;
    std::cout << num_pts << std::endl;
    double x, y, z;
    int r, g, b;
//    double cx = 353.5;
//    double cy = 247.6;
    double center_x = 376.0;
    double center_y = 240.0;

    // Load similar transform to convert sfm frame to metric
    cv::FileStorage fs_param("param.yml", cv::FileStorage::READ);
    double s;
    cv::Mat tf;
    fs_param["tf_sfm_to_vicon"] >> tf;
    fs_param["scale_sfm_to_metric"] >> s;
    fs_param.release();

    //std::cout << s << std::endl;
    //std::cout << tf << std::endl;

    // Load position and descritpor for each 3D point
    descriptors_ = cv::Mat(num_pts, DIM_DES, CV_8U);
    points_.resize(num_pts);

#ifdef DEBUG_MATCHING
    patches_.resize(num_pts);
#endif
    
    for (size_t i = 0; i < num_pts; ++i)
    {
      ifs >> x;
      ifs >> y;
      ifs >> z;
      ifs >> r;
      ifs >> g; 
      ifs >> b;
      // Transform to metric
      cv::Mat pt4 = (cv::Mat_<float>(4, 1) << x * s, y * s, z * s, 1.0);
      cv::Mat pt4_w = tf * pt4;
      
      points_[i] = cv::Point3f(pt4_w.at<float>(0) / pt4_w.at<float>(3), 
                               pt4_w.at<float>(1) / pt4_w.at<float>(3), 
                               pt4_w.at<float>(2) / pt4_w.at<float>(3));
      int num_obs; // Number of observation
      ifs >> num_obs;
      
      //std::cout << i << "," << x << "," << y << "," << z << "," << r << "," << g << "," << b << "," << num_obs << std::endl;
      
      double scale_max = 0;
      for (size_t j = 0; j < num_obs; ++j)
      {
        int idx_img, idx_kp;
        double u, v;
        ifs >> idx_img; // Image index
        ifs >> idx_kp;  // Feature index
        ifs >> u;       // Location x wrt image center (not principle point)
        ifs >> v;       // Location y wrt image center (not principle point)
//        std::cout << idx_img << "," << idx_kp << "," << u << "," << v << "," << filenames[idx_img] << std::endl;
        cv::KeyPoint kp = imfs[idx_img].getKeypoint(idx_kp);
        
        // Use the descriptor with largest scale
        // TODO: More robust method
        double scale = kp.size;
        if (scale > scale_max)
        {
#ifdef DEBUG_MATCHING
          int win = 100;
          cv::Rect roi(0, 0, 0, 0);
          roi += cv::Point2i(kp.pt.x - 0.5 * win + 2 * win, kp.pt.y - 0.5 * win + 2 * win);
          roi += cv::Size(win, win);
          cv::Mat img_draw, img_border;
          imfs[idx_img].img_.copyTo(img_draw);
          cv::circle(img_draw, kp.pt, 10, cv::Scalar(0, 0, 255), 2);
          cv::copyMakeBorder(img_draw, img_border, 2 * win, 2 * win, 2 * win, 2 * win, cv::BORDER_CONSTANT, 0);
          cv::Mat patch(img_border, roi);
          patch.copyTo(patches_[i]);
#endif          
          imfs[idx_img].getDescriptorRow(idx_kp, i, descriptors_);
          scale_max = scale;
        }
//        imf.draw(idx_kp, cv::Point2f(center_x + u, center_y + v));  // Draw target SIFT feature
//        imf.draw(-1, cv::Point2f(cx + u, cy + v));    // Draw all SIFT features

      }
    }
    ifs.close();

#ifdef DEBUG_MATCHING
    cv::imwrite("des.png", descriptors_);
#endif  
  }

  void buildDatabase()
  {
    // Database matrix
    int dim_desc = descriptors_.cols;
    int num_pts = descriptors_.rows;
    database_ = new flann::Matrix<float> (new float[num_pts * dim_desc], num_pts, dim_desc);
    // TODO: More efficient copy
    for (int r = 0; r < num_pts; ++r)
    {
      for (int c = 0; c < dim_desc; ++c)
      {
        (*database_)[r][c] = float(descriptors_.at<unsigned char>(r, c));
      }
    }
    std::cout << "Database done." << std::endl;
  }

  void freeDatabase()
  {   
    delete [] (*database_).ptr();
    delete database_;
  }  

  void buildFlannIndex()
  {   
    kdtree_ = new flann::Index<flann::L2<float> >(*database_, flann::KDTreeIndexParams(8));
    kdtree_->buildIndex();
    std::cout << "Index done." << std::endl;
  }

  bool localize(cv::Mat &img, 
                const cv::Mat &query_descs,
		            const std::vector<cv::KeyPoint> keypoints,
                const cv::Mat &camera_matrix,
                cv::Mat &rvec, cv::Mat &tvec)
//                cv::Mat &rvec_gt, cv::Mat &tvec_gt)
//                std::vector<int> &indices_inlier,
//                std::vector<std::pair<cv::Point3f, cv::Point2f> > &matches)
  {
    // 1. Build query flann matrix
    flann::Matrix<float> query((float *)query_descs.data, query_descs.rows, query_descs.cols);
/*
    flann::Matrix<float> query(new float[query_descs.rows * query_descs.cols], query_descs.rows, query_descs.cols);
    for (int r = 0; r < query_descs.rows; ++r)
    {
        for (int c = 0; c < query_descs.cols; ++c)
        {
          query[r][c] = floor(query_descs.at<float>(r, c) * 512.0f);
        }
      }
*/
//      t.getCurrentTime("1. Load SIFT descriptors");

      // 2. KNN search for correspondences
      int nn = 2;
      flann::Matrix<int> indices(new int[query_descs.rows * nn], query_descs.rows, nn);
      flann::Matrix<float> dists(new float[query_descs.rows * nn], query_descs.rows, nn);

      kdtree_->knnSearch(query, indices, dists, nn, flann::SearchParams(256));
//      delete [] query.ptr();
//      t.getCurrentTime("2. KNN search for correspondences");


      // 3. Build correspondences
      std::vector<cv::Point2f> points_2d;
      std::vector<cv::Point3f> points_3d;

      // Check correspondences and form 2D/3D points
      int j = 0;
      for (int i = 0; i < query_descs.rows; ++i)
      {
        float dist1 = dists[i][0];
        float dist2 = dists[i][1];
     
        if (dist1 / dist2 < 0.7)
        {
          int index = indices[i][0];
#ifdef DEBUG_MATCHING
          char buffer[100];
          sprintf(buffer, "patch_%03d_%05d.png", j, index);
          cv::imwrite(buffer, patches_[index]);
#endif
          cv::Point3f pt3 = points_[index];
          cv::Point2f pt2 = keypoints[i].pt;
          points_2d.push_back(pt2);
          points_3d.push_back(pt3);
          j++;
        }
      }


//      std::cout << DEBUG_INFO << "# of descriptors = " << query_descs.size() << std::endl;
//      std::cout.flush();

      delete [] indices.ptr();
      delete [] dists.ptr();

      // Check # of matches
      if (points_2d.size() < 30)
      {
        std::cout << "Not enough correspondences." << std::endl;
        return false;
      }
//      t.getCurrentTime("3. Build correspondences");

      // 4. PnP
      // RANSAC PnP
      std::vector<int> indices_inlier;
      //cv::solvePnPRansac(points_3d, points_2d, camera_matrix, cv::Mat::zeros(5, 1, CV_32F), rvec, tvec, false, 30, 10.0, 300, indices_inlier, CV_EPNP);
        cv::solvePnPRansac(points_3d, points_2d, camera_matrix, cv::noArray(), rvec, tvec, true, 20, 2.0, 100, indices_inlier, CV_ITERATIVE);

#ifdef INFO_DRAW
      std::vector<cv::Point2f> points_2d_proj;
      projectPoints(points_3d, rvec, tvec, camera_matrix, cv::noArray(), points_2d_proj);
      //projectPoints(points_3d, rvec_gt, tvec_gt, camera_matrix, cv::noArray(), points_2d_proj);
      for (size_t i = 0; i < points_2d.size(); ++i)
      {
        cv::Point2f pt2 = points_2d[i];

        cv::circle(img, pt2, 2, cv::Scalar(0, 255, 0), 1);
        cv::Point2f pt2_proj = points_2d_proj[i];
        cv::circle(img, pt2, 2, cv::Scalar(255, 0, 0), 1);
        cv::line(img, pt2, pt2_proj, cv::Scalar(255, 255, 0), 1);
        
//        char buffer[100];
//        sprintf(buffer, "%d", i);
//       cv::putText(img, std::string(buffer), pt2, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 5));
      }
      for (size_t i = 0; i < indices_inlier.size(); ++i)
      {
        cv::Point2f pt2 = points_2d[indices_inlier[i]];
        cv::circle(img, pt2, 3, cv::Scalar(0, 0, 255), 1);
      }
#endif      

#ifdef DEBUG_MATCHING
      cv::waitKey(0);
#endif

      /*      
      // Optionally refine using inliers and LM
      if (indices_inlier.size() > 10)
      {
        std::vector<cv::Point2f> inliers_2d;
        std::vector<cv::Point3f> inliers_3d;
        for (size_t i = 0; i < indices_inlier.size(); ++i)
        {
          inliers_2d.push_back(points_2d[indices_inlier[i]]);
          inliers_3d.push_back(points_3d[indices_inlier[i]]);
        }
        cv::solvePnP(inliers_3d, inliers_2d, camera_matrix, cv::Mat::zeros(5, 1, CV_32F), rvec, tvec, true, CV_ITERATIVE);
//        cv::Rodrigues(rvec, R);
//        hconcat(R, tvec, tf);
//        std::cout << DEBUG_INFO << "[R|t] after refinement = " << std::endl << tf << std::endl;
//        std::cout.flush();
      }
      else
      {
        std::cout <<  "Skip refinement..."<< std::endl;
      }
//      t.getCurrentTime("4. PnP");
*/
/*
      // 5. Output
      matches.clear();
      for (size_t i = 0; i < points_2d.size(); ++i)
      {
        matches.push_back(std::pair<cv::Point3f, cv::Point2f>(points_3d[i], camera_->distort(points_2d[i])));
      }
      std::cout << "Done!" << std::endl;
*/
      return true;
    }




};

#endif
