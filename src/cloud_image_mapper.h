#include <ros/ros.h>
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

class Cloud_Image_Mapper
{
  tf::TransformListener *tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  CvFont font_;
  
  //sensor_msgs::PointCloud2 transform_cloud(sensor_msgs::PointCloud2ConstPtr cloud_in, string frame_target);

public:
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
                pcl::PointCloud<pcl::PointXYZRGB> velodyne_cloud,
                float *cloud_vision_label)
  {   

    // cout << "image in" << endl;
    cloud_vision_label = new float[velodyne_cloud.points.size()];

    cv::Mat image, image_display;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
      image_display = image.clone();
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return velodyne_cloud;
    }
    
    cv::Mat img_mindist(image.rows, image.cols, CV_8UC1, cv::Scalar(255));
    
   // read camera information
    cam_model_.fromCameraInfo(info_msg);

    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud = transform_cloud (velodyne_cloud, cam_model_.tfFrame());
    
    //tf::Point pt = transform.getOrigin();
    
    for(int i = 0; i < pcl_cloud.points.size(); i++)
    {
      pcl::PointXYZRGB point = pcl_cloud.points[i];
      if(point.z < 0)
        continue;
      
      cv::Point3d pt_cv(point.x, point.y, point.z);
      
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      
      static const int RADIUS = 25;
      //cout << uv << endl;
      
      if(uv.x >= 0 && uv.x < image.cols && uv.y >= 0 && uv.y < image.rows)
      {
        
        uchar min_z = img_mindist.at<uchar>(uv.y, uv.x);
        if(point.z < min_z)
        {
          cv::Point p1, p2;
          p1.x = uv.x;
          p1.y = uv.y - 5;
          p2.x = uv.x;
          p2.y = uv.y + 5;        
          //cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
          cv::line(img_mindist, p1, p2, point.z/0.02, 5);
          
        }
         // cv::circle(img_mindist, uv, RADIUS, point.z/0.02, -1);
        
        // cv::Vec3b intensity = image.at<cv::Vec3b>(uv.y, uv.x);
        // uchar r = intensity.val[2];
        // uchar g = intensity.val[1];
        // uchar b = intensity.val[0];
        
        // pcl_cloud.points[i].r = r;
        // pcl_cloud.points[i].g = g;
        // pcl_cloud.points[i].b = b;
        
        // cv::Point p1, p2;
        // p1.x = uv.x;
        // p1.y = uv.y - 11;
        // p2.x = uv.x;
        // p2.y = uv.y + 11;        
        // //cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
        // cv::line(image, p1, p2, CV_RGB(255,0,0), 1);
       
      }
    }
    
    for(int i = 0; i < pcl_cloud.points.size(); i++)
    {
      pcl::PointXYZRGB point = pcl_cloud.points[i];
      if(point.z < 0)
        continue;
      
      cv::Point3d pt_cv(point.x, point.y, point.z);
      
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      
      static const int RADIUS = 5;
      //cout << uv << endl;
      
      if(uv.x >= 0 && uv.x < image.cols && uv.y >= 0 && uv.y < image.rows)
      {
        uchar point_r = pcl_cloud.points[i].r;
        uchar point_g = pcl_cloud.points[i].g;
        uchar point_b = pcl_cloud.points[i].b;

        uchar min_z = img_mindist.at<uchar>(uv.y, uv.x);
        if(abs(point.z/0.02 - min_z) < 10 )
	      // if(point.z/0.02 < min_z )
        {
          cv::Vec3b intensity = image_display.at<cv::Vec3b>(uv.y, uv.x);

          uchar r = intensity.val[2];
          uchar g = intensity.val[1];
          uchar b = intensity.val[0];
          
          velodyne_cloud.points[i].r = r;
          velodyne_cloud.points[i].g = g;
          velodyne_cloud.points[i].b = b;
          
          cloud_vision_label[i] = (r+g+b)/3;
          //cv::line(image, p1, p2, CV_RGB(255,0,0), 1);
        }
        cv::circle(image, uv, RADIUS, CV_RGB(point_r,point_g,point_b), -1);    
      }
    }
    
    cv::imshow("min", image);
    cv::waitKey(5);

    return velodyne_cloud;
  }
};
