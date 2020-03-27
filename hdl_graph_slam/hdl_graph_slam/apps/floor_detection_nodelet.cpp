#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/point_cloud.h>

#include <std_msgs/Time.h>
#include <sensor_msgs/PointCloud2.h>
#include <hdl_graph_slam/FloorCoeffs.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/PCLPointCloud2.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PointIndices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <chrono>

namespace pcl {

   /** \brief Convert a XYZRGB point type to a XYZHSV
     * \param[in] in the input XYZRGB point 
     * \param[out] out the output XYZHSV point
     */
   inline void 
   _PointXYZRGBtoXYZHSV (const PointXYZRGB& in,
                        PointXYZHSV&       out)
   {
     const unsigned char max = std::max (in.r, std::max (in.g, in.b));
     const unsigned char min = std::min (in.r, std::min (in.g, in.b));
 
     out.x = in.x; out.y = in.y; out.z = in.z;
     out.v = static_cast <float> (max) / 255.f;
 
     if (max == 0) // division by zero
     {
       out.s = 0.f;
       out.h = 0.f; // h = -1.f;
       return;
     }
 
     const float diff = static_cast <float> (max - min);
     out.s = diff / static_cast <float> (max);
 
     if (min == max) // diff == 0 -> division by zero
     {
       out.h = 0;
       return;
     }
 
     if      (max == in.r) out.h = 60.f * (      static_cast <float> (in.g - in.b) / diff);
     else if (max == in.g) out.h = 60.f * (2.f + static_cast <float> (in.b - in.r) / diff);
     else                  out.h = 60.f * (4.f + static_cast <float> (in.r - in.g) / diff); // max == b
 
     if (out.h < 0.f) out.h += 360.f;
   }

   /* \brief Convert a XYZHSV point type to a XYZRGB
     * \param[in] in the input XYZHSV point 
     * \param[out] out the output XYZRGB point
     */
   inline void 
   _PointXYZHSVtoXYZRGB (const PointXYZHSV&  in,
                        PointXYZRGB&        out)
   {
     out.x = in.x; out.y = in.y; out.z = in.z;
     if (in.s == 0)
     {
       out.r = out.g = out.b = static_cast<std::uint8_t> (255 * in.v);
       return;
     } 
     float a = in.h / 60;
     int   i = static_cast<int> (std::floor (a));
     float f = a - static_cast<float> (i);
     float p = in.v * (1 - in.s);
     float q = in.v * (1 - in.s * f);
     float t = in.v * (1 - in.s * (1 - f));
 
     switch (i)
     {
       case 0:
       {
         out.r = static_cast<std::uint8_t> (255 * in.v);
         out.g = static_cast<std::uint8_t> (255 * t);
         out.b = static_cast<std::uint8_t> (255 * p);
         break;
       }
       case 1:
       {
         out.r = static_cast<std::uint8_t> (255 * q); 
         out.g = static_cast<std::uint8_t> (255 * in.v); 
         out.b = static_cast<std::uint8_t> (255 * p); 
         break;
       }
       case 2:
       {
         out.r = static_cast<std::uint8_t> (255 * p);
         out.g = static_cast<std::uint8_t> (255 * in.v);
         out.b = static_cast<std::uint8_t> (255 * t);
         break;
       }
       case 3:
       {
         out.r = static_cast<std::uint8_t> (255 * p);
         out.g = static_cast<std::uint8_t> (255 * q);
         out.b = static_cast<std::uint8_t> (255 * in.v);
         break;
       }
       case 4:
       {
         out.r = static_cast<std::uint8_t> (255 * t);
         out.g = static_cast<std::uint8_t> (255 * p); 
         out.b = static_cast<std::uint8_t> (255 * in.v); 
         break;
       }
       default:
       {
         out.r = static_cast<std::uint8_t> (255 * in.v); 
         out.g = static_cast<std::uint8_t> (255 * p); 
         out.b = static_cast<std::uint8_t> (255 * q);
         break;
       }      
     }
   }

inline void 
_PointCloudXYZRGBtoXYZHSV (const PointCloud<PointXYZRGB>& in,
                         PointCloud<PointXYZHSV>&       out)
{
 out.width   = in.width;
 out.height  = in.height;
 for (const auto &point : in.points)
 {
   PointXYZHSV p;
   _PointXYZRGBtoXYZHSV (point, p);
   out.points.push_back (p);
 }
}

inline void 
   _PointCloudXYZHSVtoXYZRGB (const PointCloud<PointXYZHSV>& in,
                             PointCloud<PointXYZRGB>&       out)
   {
     out.width   = in.width;
     out.height  = in.height;
     for (const auto &point : in.points)
     {
       PointXYZRGB p;
       PointXYZHSV phsv = point;
       _PointXYZHSVtoXYZRGB (phsv, p);
       out.points.push_back (p);
     }
   }
}

namespace hdl_graph_slam {

class FloorDetectionNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZRGB PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FloorDetectionNodelet() {}
  virtual ~FloorDetectionNodelet() {}

  virtual void onInit() {
    NODELET_DEBUG("initializing floor_detection_nodelet...");
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = nh.subscribe("/filtered_points", 2, &FloorDetectionNodelet::cloud_callback, this);
    floor_pub = nh.advertise<hdl_graph_slam::FloorCoeffs>("/floor_detection/floor_coeffs", 2);

    read_until_pub = nh.advertise<std_msgs::Header>("/floor_detection/read_until", 2);
    floor_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor_filtered_points", 2);
    floor_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/floor_points", 2);
    outliers_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/outliers", 2);
    lines_pub = nh.advertise<sensor_msgs::PointCloud2>("/floor_detection/lines", 2);
  }


private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    tilt_deg = private_nh.param<double>("tilt_deg", 0.0);                          // approximate sensor tilt angle [deg]
    sensor_height = private_nh.param<double>("sensor_height", 2.0);                // approximate sensor height [m]
    height_clip_range= private_nh.param<double>("height_clip_range", 1.0);         // points with heights in [sensor_height - height_clip_range, sensor_height + height_clip_range] will be used for floor detection
    floor_pts_thresh = private_nh.param<int>("floor_pts_thresh", 512);             // minimum number of support points of RANSAC to accept a detected floor plane
    floor_normal_thresh = private_nh.param<double>("floor_normal_thresh", 10.0);   // verticality check thresold for the detected floor plane [deg]
    use_normal_filtering = private_nh.param<bool>("use_normal_filtering", true);   // if true, points with "non-"vertical normals will be filtered before RANSAC
    normal_filter_thresh = private_nh.param<double>("normal_filter_thresh", 20.0); // "non-"verticality check threshold [deg]

    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");
    thd_filtered = private_nh.param<double>("thd_filtered", 0.1);
    thd_inliers = private_nh.param<double>("thd_inliers", 0.1);
    h_lo = private_nh.param<int>("h_lo", 0);
    s_lo = private_nh.param<int>("s_lo", 0);
    v_lo = private_nh.param<int>("v_lo", 168);
    h_hi = private_nh.param<int>("h_hi", 172);
    s_hi = private_nh.param<int>("s_hi", 111);
    v_hi = private_nh.param<int>("v_hi", 255);
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if(cloud->empty()) {
      return;
    }

    // floor detection

    // auto tic = std::chrono::steady_clock::now();
    boost::optional<Eigen::Vector4f> floor = detect(cloud);
    // auto toc = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() << "[µs]" << std::endl;

    // publish the detected floor coefficients
    hdl_graph_slam::FloorCoeffs coeffs;
    coeffs.header = cloud_msg->header;
    if(floor) {
      coeffs.coeffs.resize(4);
      for(int i=0; i<4; i++) {
        coeffs.coeffs[i] = (*floor)[i];
      }
    }

    floor_pub.publish(coeffs);

    // for offline estimation
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = points_topic;
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);
  }

  /**
   * @brief detect the floor plane from a point cloud
   * @param cloud  input cloud
   * @return detected floor plane coefficients
   */
  boost::optional<Eigen::Vector4f> detect(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    // compensate the tilt rotation
    Eigen::Matrix4f tilt_matrix = Eigen::Matrix4f::Identity();
    tilt_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(tilt_deg * M_PI / 180.0f, Eigen::Vector3f::UnitY()).toRotationMatrix();

    // filtering before RANSAC (height and normal filtering)
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cloud, *filtered, tilt_matrix);
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height + height_clip_range), false);
    filtered = plane_clip(filtered, Eigen::Vector4f(0.0f, 0.0f, 1.0f, sensor_height - height_clip_range), true);

    // if(use_normal_filtering) {
    //   filtered = normal_filtering(filtered);
    // }

    pcl::transformPointCloud(*filtered, *filtered, static_cast<Eigen::Matrix4f>(tilt_matrix.inverse()));

    if(floor_filtered_pub.getNumSubscribers()) {
      filtered->header = cloud->header;
      floor_filtered_pub.publish(filtered);
    }

    // too few points for RANSAC
    if(filtered->size() < floor_pts_thresh) {
      printf("filtered points length too small: %ld\n", filtered->size());
      return boost::none;
    }

    // RANSAC
    pcl::SampleConsensusModelPlane<PointT>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointT>(filtered));
    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setDistanceThreshold(thd_filtered);
    ransac.computeModel();

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    ransac.getInliers(inliers->indices);

    // too few inliers
    if(inliers->indices.size() < floor_pts_thresh) {
      printf("too few inliers: %ld\n", inliers->indices.size());
      return boost::none;
    }

    // verticality check of the detected floor's normal
    Eigen::Vector4f reference = tilt_matrix.inverse() * Eigen::Vector4f::UnitZ();

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);

    // double dot = coeffs.head<3>().dot(reference.head<3>());
    // if(std::abs(dot) < std::cos(floor_normal_thresh * M_PI / 180.0)) {
    //   // the normal is not vertical
    //   printf("the normal is not vertical\n");
    //   return boost::none;
    // }

    if(floor_points_pub.getNumSubscribers()) {
      pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(filtered);
      extract.setIndices(inliers);
      extract.filter(*inlier_cloud);
      inlier_cloud->header = cloud->header;
      floor_points_pub.publish(inlier_cloud);
    }

    if(outliers_pub.getNumSubscribers()) {  
      auto filtered_all = cloud;
      pcl::PointIndices::Ptr inliers_all(new pcl::PointIndices);
      model_p->setInputCloud(filtered_all);
      model_p->selectWithinDistance(coeffs, thd_inliers, inliers_all->indices);
      pcl::PointCloud<PointT>::Ptr outlier_cloud(new pcl::PointCloud<PointT>);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(filtered_all);
      extract.setNegative(true);
      extract.setIndices(inliers_all);
      extract.filter(*outlier_cloud);
      outlier_cloud->header = cloud->header;
      outliers_pub.publish(outlier_cloud);
    }

    if(lines_pub.getNumSubscribers()) {

      pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
      pcl::ExtractIndices<PointT> extract;
      extract.setInputCloud(filtered);
      extract.setIndices(inliers);
      extract.filter(*inlier_cloud);

      pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZHSV>);
      pcl::_PointCloudXYZRGBtoXYZHSV(*inlier_cloud, *cloud_filtered);

      // for (const auto & pt : *cloud_filtered) {
      //   std::cout << "cloud_filtered.x: " << pt.x << std::endl;
      // }

      // std::cout << "1. cloud_filtered.size()" << cloud_filtered->size() << std::endl;
      // filter hue
      pcl::PassThrough<pcl::PointXYZHSV> pass;
      pass.setInputCloud (cloud_filtered);
      pass.setFilterFieldName ("h");
      pass.setFilterLimits (double(h_lo)/180., double(h_hi)/180.);
      pass.setFilterLimitsNegative (false);
      pass.filter (*cloud_filtered);

      // std::cout << "2. cloud_filtered.size()" << cloud_filtered->size() << std::endl;
      // std::cout << "h_lo" << double(h_lo)/180. << "h_hi" << double(h_hi)/180. << std::endl;
      // filter saturation
      pass.setInputCloud (cloud_filtered);
      pass.setFilterFieldName ("s");
      pass.setFilterLimits (double(s_lo)/255., double(s_hi)/255.);
      pass.setFilterLimitsNegative (false);
      pass.filter (*cloud_filtered);

      // std::cout << "3. cloud_filtered.size()" << cloud_filtered->size() << std::endl;
      // std::cout << "s_lo" << double(s_lo)/255. << "s_hi" << double(s_hi)/255. << std::endl;
      // filter value
      pass.setInputCloud (cloud_filtered);
      pass.setFilterFieldName ("v");
      pass.setFilterLimits (double(v_lo)/255., double(v_hi)/255.);
      pass.setFilterLimitsNegative (false);
      pass.filter (*cloud_filtered);

      // std::cout << "4. cloud_filtered.size()" << cloud_filtered->size() << std::endl;
      // std::cout << "v_lo" << double(v_lo)/255. << "v_hi" << double(v_hi)/255. << std::endl;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::_PointCloudXYZHSVtoXYZRGB(*cloud_filtered, *cloud_filtered_rgb);

      cloud_filtered_rgb->header = cloud->header;
      lines_pub.publish(cloud_filtered_rgb);
    }

    // make the normal upward
    if(coeffs.head<3>().dot(Eigen::Vector3f::UnitZ()) < 0.0f) {
      coeffs *= -1.0f;
    }

    return Eigen::Vector4f(coeffs);
  }

  /**
   * @brief plane_clip
   * @param src_cloud
   * @param plane
   * @param negative
   * @return
   */
  pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative) const {
    pcl::PlaneClipper3D<PointT> clipper(plane);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    clipper.clipPointCloud3D(*src_cloud, indices->indices);

    pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(src_cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*dst_cloud);

    return dst_cloud;
  }

  /**
   * @brief filter points with non-vertical normals
   * @param cloud  input cloud
   * @return filtered cloud
   */
  pcl::PointCloud<PointT>::Ptr normal_filtering(const pcl::PointCloud<PointT>::Ptr& cloud) const {
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(10);
    ne.setViewPoint(0.0f, 0.0f, sensor_height);
    ne.compute(*normals);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    filtered->reserve(cloud->size());

    for (int i = 0; i < cloud->size(); i++) {
      float dot = normals->at(i).getNormalVector3fMap().normalized().dot(Eigen::Vector3f::UnitZ());
      if (std::abs(dot) > std::cos(normal_filter_thresh * M_PI / 180.0)) {
        filtered->push_back(cloud->at(i));
      }
    }

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;
  }


private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // ROS topics
  ros::Subscriber points_sub;

  ros::Publisher floor_pub;
  ros::Publisher floor_points_pub;
  ros::Publisher outliers_pub;
  ros::Publisher lines_pub;
  ros::Publisher floor_filtered_pub;

  std::string points_topic;
  ros::Publisher read_until_pub;

  // floor detection parameters
  // see initialize_params() for the details
  double tilt_deg;
  double sensor_height;
  double height_clip_range;

  int floor_pts_thresh;
  double floor_normal_thresh;

  bool use_normal_filtering;
  double normal_filter_thresh;
  double thd_filtered;
  double thd_inliers;
  int h_lo;
  int s_lo;
  int v_lo;
  int h_hi;
  int s_hi;
  int v_hi;
};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::FloorDetectionNodelet, nodelet::Nodelet)
