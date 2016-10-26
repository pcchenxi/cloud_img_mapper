#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace cv;

class Cloud_Image_Mapper
{
  tf::TransformListener *tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  CvFont font_;
  //sensor_msgs::PointCloud2 transform_cloud(sensor_msgs::PointCloud2ConstPtr cloud_in, string frame_target);

public:
  Mat img_ori, image_label;
  Mat img_label_grey_, img_label_color_;
  Cloud_Image_Mapper(tf::TransformListener *tf_listener)
  {
    tf_listener_ = tf_listener;
  }


  pcl::PointCloud<pcl::PointXYZRGB> transform_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud_in, string frame_target)
  {
    ////////////////////////////////// transform ////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
    tf::StampedTransform to_target;
    
    // cout << frame_target << "  image frame: " << cloud_in.header.frame_id << endl;

    try 
    {
      //ros::Time acquisition_time = info_msg->header.stamp;
      //  ros::Duration timeout(1.0 / 1);
        //         tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
        //                               acquisition_time, timeout);
        // tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
        //                              acquisition_time, transform);
      // tf_listener_->waitForTransform(frame_target, cloud_in.header.frame_id, cloud_in.header.stamp, ros::Duration(1.0));
      // tf_listener_->lookupTransform(frame_target, cloud_in.header.frame_id, cloud_in.header.stamp, to_target);
      tf_listener_->lookupTransform(frame_target, cloud_in.header.frame_id, ros::Time(0), to_target);
    }
    catch (tf::TransformException& ex) 
    {
      ROS_WARN("[draw_frames] TF exception in Cloud_Image_Mapper:\n%s", ex.what());
      // return cloud_in;
    }
    
    // cout << frame_target << "   " << cloud_in.header.frame_id << endl;
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix (to_target, eigen_transform);
    // pcl_ros::transformPointCloud (eigen_transform, cloud_in, cloud_out);
    pcl::transformPointCloud(cloud_in, cloud_out, eigen_transform);

    cloud_out.header.frame_id = frame_target;
    return cloud_out;
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud_image_mapping(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg,
                pcl::PointCloud<pcl::PointXYZRGB> velodyne_cloud)
  {   
    float map_resolution = 0.03;
    int map_rows = 10.0/map_resolution;
    int map_cols = 10.0/map_resolution;
    Mat label_on_ground = Mat(map_rows, map_cols, CV_8UC3,  Scalar(0));

    cout << "image in" << endl;
    // cloud_vision_label = new float[velodyne_cloud.points.size()];

    cv::Mat image, image_display;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
      image_display = image.clone();

      ostringstream file_name;
      //file_name << "/home/xi/workspace/catkin_rcv/image/" << ros::Time::now() << "_img.jpg";
    
      //ROS_ASSERT( cv::imwrite( file_name.str(), image ) );  
      // ROS_ASSERT( cv::imwrite( "img.jpg",  input_bridge->image ) );  
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return velodyne_cloud;
    }
    
    cv::Mat img_mindist(image.rows, image.cols, CV_8UC1, cv::Scalar(255));
    cv::Mat img_label(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

   // read camera information
    cam_model_.fromCameraInfo(info_msg);

    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud = transform_cloud (velodyne_cloud, cam_model_.tfFrame());
    
    for(int i = 0; i < pcl_cloud.points.size(); i++)
    {
      pcl::PointXYZRGB point = pcl_cloud.points[i];
      if(point.z < 0 || abs(point.x) > 6)
        continue;
      
      cv::Point3d pt_cv(point.x, point.y, point.z);
      
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      
      static const int RADIUS = 7;
      
      if(uv.x >= 0 && uv.x < image.cols && uv.y >= 0 && uv.y < image.rows)
      {
        uchar point_r = pcl_cloud.points[i].r;
        uchar point_g = pcl_cloud.points[i].g;
        uchar point_b = pcl_cloud.points[i].b;

        // cv::circle(image, uv, RADIUS, CV_RGB(point_r,point_g,point_b), -1);    

        // for grey scale label
        int color_grey = 0;
        if(point_r == 50)
          color_grey = 50;
        else if(point_r == 254)
          color_grey = 150;
        else if(point_r == 255)
          color_grey = 255;  


        // for depth image
        float max_dist = 10;
        if(point.z > 10)
          continue;

        float scaled_depth = 255/10 * point.z;
        uchar depth = 255 - scaled_depth;

        cv::circle(img_label, uv, 3, CV_RGB(depth, depth, depth), -1);    
        // cv::circle(img_label, uv, 5, CV_RGB(color_grey,color_grey,color_grey), -1); 
      
        // project it to ground plane
        cv::Vec3b intensity = image.at<cv::Vec3b>(uv.y, uv.x);
        uchar label = (intensity.val[0] + intensity.val[1] + intensity.val[2])/3;
        int row = velodyne_cloud.points[i].y/map_resolution + map_rows/2; 
        int col = velodyne_cloud.points[i].x/map_resolution + map_cols/2; 
        if(row < 0 || row >= map_rows || col < 0 || col >= map_cols)
          continue;
        // cout << row << " " << col << endl;
        col = map_cols - col;
        label_on_ground.ptr<cv::Vec3b>(row)[col] = intensity;
      }
    }
    
    // Mat element = getStructuringElement(MORPH_RECT, Size(18,18));
    // morphologyEx(img_label, img_label, MORPH_DILATE, element);

    Mat output;

    float alpht = 0.5;
    addWeighted(image, alpht, image_display, 1-alpht, 0, output);

    img_label_grey_  = img_label;
    img_label_color_ = output;

   ostringstream file_name;
   file_name << "/home/xi/workspace/catkin_rcv/image/" << ros::Time::now() << "_label.png";
  //  ROS_ASSERT( cv::imwrite( file_name.str(), img_label ) );  

   cv::imshow("label_on_ground", label_on_ground);
  //  cv::imshow("depth", img_label_grey_);
   cv::waitKey(5);

    return velodyne_cloud;
  }
};
