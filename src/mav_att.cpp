#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//http://answers.ros.org/question/50113/transform-quaternion/
#include <tf/transform_datatypes.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>

// Read the ar_pose_marker topic and convert the quaternion to Euler angles

//void QuaternionToEuler(tf::Quaternion q)
inline void QuaternionToEuler(const double &w, const double &x, const double &y, const double &z, 
        double *roll, double *pitch, double *yaw)
{
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    //double roll, pitch, yaw;
    m.getRPY(*roll, *pitch, *yaw);
    //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
}
/*
void Nav::compassCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Convert quaternion to RPY.
    tf::Quaternion q;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);
} // end compassCallback()
*/

//http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm?utm_source=Randlink_twitter&utm_medium=twitter&utm_campaign=Randlink
//heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy2 - 2*qz2)
//attitude = asin(2*qx*qy + 2*qz*qw) 
//bank = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx2 - 2*qz2)
/*
void QuaternionToEuler(const dQuaternion quaternion, vector3df &euler)
{
    dReal w,x,y,z;

    w = quaternion[0];
    x = quaternion[1];
    y = quaternion[2];
    z = quaternion[3];

    double sqw = w*w;    
    double sqx = x*x;    
    double sqy = y*y;    
    double sqz = z*z; 

    euler.Z = (irr::f32) (atan2(2.0 * (x*y + z*w),(sqx - sqy - sqz + sqw)) * (180.0f/irr::core::PI));
    euler.X = (irr::f32) (atan2(2.0 * (y*z + x*w),(-sqx - sqy + sqz + sqw)) * (180.0f/irr::core::PI));          
    euler.Y = (irr::f32) (asin(-2.0 * (x*z - y*w)) * (180.0f/irr::core::PI));

}
*/
//AlvarMarker.msg
//Header header
//uint32 id
//uint32 confidence
//geometry_msgs/PoseStamped pose

//ar_track_alvar::AlvarMarkers arPoseMarkers_;
//arMarkerPub_ = n.advertise < ar_track_alvar::AlvarMarkers > ("ar_pose_marker", 0);

/*
From IndividualMarkersNoKinect.cpp in ar_track_alvar
tf::Transform tagPoseOutput = CamToOutput * markerPose;

tf::poseTFToMsg (tagPoseOutput, ar_pose_marker.pose.pose);
ar_pose_marker.header.frame_id = output_frame;
ar_pose_marker.header.stamp = image_msg->header.stamp;
ar_pose_marker.id = id;
arPoseMarkers_.markers.push_back (ar_pose_marker);  
}
arMarkerPub_.publish (arPoseMarkers_);
*/

/*
//Copy payload from mavlink_msg (from ROS) to the new "real" mavlink message
copy(mavlink_ros_msg.payload64.begin(), mavlink_ros_msg.payload64.end(), msg.payload64);

(rosmavlink_msg.payload64).push_back(message.payload64[i]);
*/

void ar_poseCallback(ar_track_alvar::AlvarMarkers arPoseMarkers)
{

  /**
   * Convert mavlink_ros::Mavlink to mavlink_message_t
   */

    ar_track_alvar::AlvarMarker ar_pose_marker;
    
    ar_pose_marker = arPoseMarkers.markers[0]; //back();
    //geometry_msgs/PoseStamped pose;
    tf::Pose tagPose;

    tf::poseMsgToTF (ar_pose_marker.pose.pose, tagPose);
    //tf::poseTFToMsg (tagPoseOutput, ar_pose_marker.pose.pose);
    //arPoseMarkers.pop_back();
    //copy(arPoseMarkers, arPoseMarkers+1,ar_pose_marker)
    //ROS_INFO("Begin: [%i] End: [%i]", arPoseMarkers.begin(), arPoseMarkers.end());
    //ar_pose_marker.id = 9;
    
    //ar_pose_marker.pose.pose;

    ROS_INFO("Received ar_pose_marker message for AR Marker-ID: [%i]", ar_pose_marker.id);
    //ROS_INFO("x: [%i], y: [%i], z: [%i], w: [%i]", ar_pose_marker.id);
    
    //tf::Quaternion q(x, y, z, w);
    //tf::Matrix3x3 m(q);
    //double roll, pitch, yaw;
    //m.getRPY(*roll, *pitch, *yaw);
    
    double theta_r = tf::getYaw(tagPose.getRotation());
    //ROS_INFO("x: [%i]", roll);
    
    //fprintf(stderr, "ERROR: Wrote %d bytes but should have written %d\n", written, messageLength);

}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "mav_attitude_display");
  ros::NodeHandle n;
  ros::Rate r(1);
  
  //ros::Publisher pub_markers = n.advertise<visualization_msgs::MarkerArray>("mav_marker_array", 4);
  
  ROS_INFO ("Subscribing to ar_pose_marker topic");
  ros::Subscriber ar_pose_sub_ = n.subscribe ("ar_pose_marker", 1, ar_poseCallback);
    
    //mavlink_sub = mavlink_nh.subscribe("to", 1000, mavlinkCallback);

 
  ROS_INFO ("Going into loop listening to ar_pose_marker");

  // Go into a loop doing callbacks
  // Will exit when Ctrl-C is pressed
  ros::spin();

  //while (ros::ok())
  //{
    
    //pub_markers.publish(marker_array_msg);
    
  //  r.sleep();
  //}

  return 0;
}