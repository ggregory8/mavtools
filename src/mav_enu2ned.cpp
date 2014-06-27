
// These are left over from copy

//#include <ros/ros.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

//http://answers.ros.org/question/50113/transform-quaternion/
//#include <tf/transform_datatypes.h>
//#include <ar_track_alvar/AlvarMarker.h>
//#include <ar_track_alvar/AlvarMarkers.h>

/////////////////////////////////


// Convert ENU ROS pose data to NED for MAVLINK

// Converting NED to ENU came from:
// http://answers.ros.org/question/12808/how-to-solve-timestamps-of-odometry-and-imu-are-x-seconds-apart/

// Listening to /tf data is different to other topics, i.e not done through callbacks

// Transform stamped message
//http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html

//answers.ros.org/question/39250/getting-the-position-and-the-pose-of-robot-using-tf_listener
// Get pose information from tf listener
//http://answers.ros.org/question/41892/position-and-orientation-of-the-robot/

// Make sure tf is defined in CMakeLists.txt and package.xml
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

//#include <tf/transform_datatypes.h>
//#include </opt/ros/hydro/include/tf/transform_datatypes.h>
//#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  //ros::service::waitForService("spawn");
  //ros::ServiceClient add_turtle =
  //     node.serviceClient<turtlesim::Spawn>("spawn");
  //turtlesim::Spawn srv;
  //add_turtle.call(srv);

  //ros::Publisher turtle_vel =
       //node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;

  ROS_INFO("Starting ENU to NED conversion...");
    
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/local_origin", "/mav",
      //listener.lookupTransform("/camera", "/local_origin",
                              ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    // Left over from turtle example
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4 * atan2(transform.getOrigin().y(),
                                 transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));


//--------------------------------
    /*
static tf::TransformBroadcaster br;
  11   tf::Transform transform;
  12   transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  13   tf::Quaternion q;
  14   q.setRPY(msg->theta, 0, 0);
  15   transform.setRotation(q);
  16   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_na
*/
//--------------------------------
        //tf::StampedTransform transform_ned;
        tf::Transform transform_ned;
        
        // convert North-East-Down (NED) to East-North-Up (ENU) coordinates
        //transform_ned.getOrigin().y() = transform.getOrigin().y();
        //transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );

    double roll = transform.getRotation().x();
    double pitch = transform.getRotation().y();
    double yaw = transform.getRotation().z();
    double yaw2 = tf::getYaw(transform.getRotation());
  
  //tagPose.rotation.getRPY(roll, pitch, yaw);
  
    double dx = transform.getOrigin().x();    //tagPose.getOrigin().x();
    double dy = transform.getOrigin().y();    //tagPose.getOrigin().y();
    double dz = transform.getOrigin().z();    //tagPose.getOrigin().z();

    printf("ENU x: [%f], y: [%f], z: [%f] - R: [%f], P: [%f], Y: [%f]\n", dx, dy, dz, roll, pitch, yaw);
    
    transform_ned.setOrigin( tf::Vector3(transform.getOrigin().y(), transform.getOrigin().x(), -transform.getOrigin().z()) );
    //ROS_INFO("ENU x: [%f], y: [%f], z: [%f]", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    //ROS_INFO("NED x: [%f], y: [%f], z: [%f]", transform_ned.getOrigin().x(), transform_ned.getOrigin().y(), transform_ned.getOrigin().z());

    transform_ned.setRotation(transform.getRotation());
    
    static tf::TransformBroadcaster br;
    //tf::Transform transform;
    //transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    //tf::Quaternion q;
    //q.setRPY(msg->theta, 0, 0);
    //transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform_ned, ros::Time::now(), "local_origin", "NED_frame"));
    // Can't do it the way below because local_origin alread has a parent. Ideally need to flip all the transformations around so local_origin is the head
    //br.sendTransform(tf::StampedTransform(transform_ned, ros::Time::now(), "NED_frame", "local_origin"));

//tf::Transform xform;
//xform.setOrigin(tf::Vector3(1.0, 0, 0));
//xform.setRotation(tf::createQuaternionFromRPY(0, M_PI, 0));

        //imu_data.orientation.w = quaternion.w;
        //imu_data.orientation.x = quaternion.y;
        //imu_data.orientation.y = quaternion.x;
        //imu_data.orientation.z = -quaternion.z;

        //imu_data.linear_acceleration.x = accel.y;
        //imu_data.linear_acceleration.y = accel.x;
        //imu_data.linear_acceleration.z = -accel.z;

        //imu_data.angular_velocity.x = angRate.y;
        //imu_data.angular_velocity.y = angRate.x;
        //imu_data.angular_velocity.z = -angRate.z;

        //if(errcnt>0) errcnt--;

        //imu_data.header.stamp = ros::Time::now();
        //imu_data.header.frame_id = "gyro_link";
        //imu_data_pub_.publish(imu_data);
//----------------------------------------------------
        //ROS_INFO("Received ar_pose_marker message for AR Marker-ID: [%i]", ar_pose_marker.id);
    



    //turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};


