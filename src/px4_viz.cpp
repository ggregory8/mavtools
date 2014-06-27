#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Should really add the ROS mavlink message to this package and change the below
#include "mavlink_ros/Mavlink.h"    // This is the definition for the format of the ROS message to the mavlink_ros node

#include "/home/ros/catkin_ws/src/mavlink_ros/include/mavlink/v1.0/pixhawk/mavlink.h"  // I couldn't find out how to indude the above so moved the whole mavlink folder to this local package include

#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>

#include <iostream>
#include <string>
#include <sstream>

using std::string;
using namespace std;    // instead of writing std::string etc..

////////////////////////////
// GG
//
// From defines.h 
// Should really add the header file in the includes
//
// Auto Pilot modes
// ----------------
#define STABILIZE 0                     // hold level position
#define ACRO 1                          // rate control
#define ALT_HOLD 2                      // AUTO control
#define AUTO 3                          // AUTO control
#define GUIDED 4                        // AUTO control
#define LOITER 5                        // Hold a single location
#define RTL 6                           // AUTO control
#define CIRCLE 7                        // AUTO control
#define LAND 9                          // AUTO control
#define OF_LOITER 10                    // Hold a single location using optical flow sensor
#define DRIFT 11                        // DRIFT mode (Note: 12 is no longer used)
#define SPORT 13                        // earth frame rate control
#define FLIP        14                  // flip the vehicle on the roll axis
#define AUTOTUNE    15                  // autotune the vehicle's roll and pitch gains
#define NUM_MODES   16

////////////////////////////////////////

// Settings
// ID of the MAV vehicle??
int sysid = 1;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 1;

int my_sysid = 255;          // ID of the linux station
int my_compid = 250;

int send_count = 0;

bool hb_enable = false;

// Desclare ROS publisher and subscribers here
ros::Publisher mavlink_pub;
//ros::Subscriber mavlink_sub;
//ros::Publisher mavlink_pub;
std::string frame_id("fcu");  // mavlink_ros_serial had this declared globally
 
mavlink_attitude_t att_last; 
sensor_msgs::ImuPtr imu_last(new sensor_msgs::Imu);
//sensor_msgs::Imu imu_last;

visualization_msgs::MarkerArray quad_marker_array;


//static inline void mavlink_msg_command_long_decode(const mavlink_message_t* msg, mavlink_command_long_t* command_long)
void display_ros_packet(mavlink_ros::Mavlink rosmavlink_msg)
{
  //mavlink_ros::Mavlink rosmavlink_msg;
  
  //mavlink_message_t msg;
  uint64_t bigNum = 0;
  uint8_t smallNum = bigNum & 0xFF;
  
  ROS_INFO("------------------------------");
  ROS_INFO("ROS Message Length = %d ", rosmavlink_msg.len);
  
  for (int h = 0; h < (rosmavlink_msg.len / 8) + 1; h++)
  {
    ROS_INFO("ROS Payload [%d] = %x ", h, rosmavlink_msg.payload64[h]);
    
    bigNum = rosmavlink_msg.payload64[h];
    smallNum = bigNum & 0xFF;
      
    for (int i = 0; i < 8; i++)
    {
      ROS_INFO("Byte [%d] = %d",i , smallNum); 
      bigNum = bigNum >> 8;
      smallNum = bigNum & 0xFF;
    
    }
  }


}

void display_mavlink_packet(mavlink_message_t msg)
{
  //mavlink_ros::Mavlink rosmavlink_msg;
  
  //mavlink_message_t msg;
  uint64_t bigNum = 0;
  uint8_t smallNum = bigNum & 0xFF;
  
  ROS_INFO("------------------------------");
  ROS_INFO("Mavlink Message Length = %d ", msg.len);

  for (int h = 0; h < (msg.len / 8) + 1; h++)
  {
    ROS_INFO("Mavlink Payload [%d] = %x ", h, msg.payload64[h]);
    
    bigNum = msg.payload64[h];
    smallNum = bigNum & 0xFF;
      
    for (int i = 0; i < 8; i++)
    {
      ROS_INFO("Byte [%d] = %d",i , smallNum); 
      bigNum = bigNum >> 8;
      smallNum = bigNum & 0xFF;
    
    }
  }

}


// This gives a segmentation fault at runtime
void compare_mavlink_packet(mavlink_ros::Mavlink rosmavlink_msg, mavlink_message_t msg)
{
  //mavlink_ros::Mavlink rosmavlink_msg;
  
  //mavlink_message_t msg;
  uint64_t bigNum_ros = 0;
  uint64_t bigNum = 0;
  uint8_t smallNum = bigNum & 0xFF;
  uint8_t smallNum_ros = bigNum_ros & 0xFF;
  
  ROS_INFO("------------------------------------");
  ROS_INFO("Compare Packets: Mavlink | ROS");
  ROS_INFO("Message Length:  %d      | %d", msg.len, rosmavlink_msg.len);
  
  
  for (int h = 0; h < 2; h++)
  {
   ROS_INFO("In loop");
 
    ROS_INFO("Payload [%d]:    %x", h, msg.payload64[0]);
    ROS_INFO("Payload [%d]:    %d", rosmavlink_msg.payload64[0]);
  
    ROS_INFO("Payload [%d]:    %x      | %x", h, msg.payload64[0], rosmavlink_msg.payload64[0]);
  
    bigNum = msg.payload64[h];
    smallNum = bigNum & 0xFF;
    bigNum_ros = rosmavlink_msg.payload64[h];
    smallNum_ros = bigNum_ros & 0xFF;

    for (int i = 0; i < 8; i++)
    {
      ROS_INFO("Byte [%d]:       %d     | %d",i , smallNum, smallNum_ros); 
      bigNum = bigNum >> 8;
      smallNum = bigNum & 0xFF;
      bigNum_ros = bigNum_ros >> 8;
      smallNum_ros = bigNum_ros & 0xFF;
    
    }
  }
  ROS_INFO("------------------------------------"); 
      
}
//mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, 3, 81,flightmode, MAV_STATE_ACTIVE);

void rosmavlink_pack(mavlink_message_t msg, mavlink_ros::Mavlink rosmavlink_msg)
{
  std_msgs::Header header;

  header.stamp = ros::Time::now();
  header.seq = send_count;
  //header.frame_id = 22;   //frame_id;
  header.frame_id = 1;   //frame_id;
  
  send_count = send_count + 1;

  //ROS_INFO("Arm message 2 ID: %d - %d", msg.msgid, rosmavlink_msg.msgid);


  rosmavlink_msg.header = header;
  //msg.len = MAVLINK_MSG_ID_HEARTBEAT_LEN;
  // Need to set the rosmavlink msgid here because at the other end it sets the mavlink msgid to the rosmavlink msgid
  rosmavlink_msg.msgid = msg.msgid;             
  rosmavlink_msg.len = msg.len;           // This needed?    
  rosmavlink_msg.seq = send_count;         // msg.seq;
  rosmavlink_msg.sysid = sysid;       // This or target sysid??
  rosmavlink_msg.compid = compid;      // This or target compid??
  //rosmavlink_msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;          // COMMAND_LONG #76 // msg.msgid; 
  for (int i = 0; i <= msg.len / 8; i++)
  {
    (rosmavlink_msg.payload64).push_back(msg.payload64[i]);
  }
  //compare_mavlink_packet(rosmavlink_msg, msg);
  //ROS_INFO("Arm message 3 ID: %d - %d", msg.msgid, rosmavlink_msg.msgid);

  //display_mavlink_packet(msg);
  //display_ros_packet(rosmavlink_msg);

  mavlink_pub.publish(rosmavlink_msg);
  //ROS_INFO("ROS Mavlink Message Sent");
  //fprintf(stderr, "ROS Mavlink Message Sent\n");
 
}

void set_mode(void)
{
  mavlink_ros::Mavlink rosmavlink_msg;
  
  mavlink_message_t msg;

  //mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t sp;
  mavlink_set_mode_t mypacket;
  
  // The following types are defined in common.h
  mypacket.target_system = sysid;         ///< The system setting the mode - shouldn't it be target ID?
  //mypacket.base_mode = MAV_AUTOPILOT_ARDUPILOTMEGA;     // The new base mode
  mypacket.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; // Arducopter uses custom messages
  mypacket.custom_mode = ACRO;      ///< The new autopilot-specific mode. This field can be ignored by an autopilot.
 
  mavlink_msg_set_mode_encode(my_sysid, my_compid, &msg, &mypacket);
  
  
  ROS_INFO("Sending set_mode message to mavlink_ros");

  rosmavlink_pack(msg, rosmavlink_msg);
}

void heart_beat(void)
{
  // From diydrones/forum/topics/mav-cmd-nav-waypoint-with-mavlink
  mavlink_ros::Mavlink rosmavlink_msg;
  
  mavlink_message_t msg;

  mavlink_heartbeat_t mypacket;
  
  // The following types are defined in common.h
  // I accidentally broke this thinking it was the set_mode function

  mypacket.type = 6;//MAV_TYPE_QUADROTOR;                   // MAV_TYPE ENUM
  mypacket.autopilot = 7;//MAV_AUTOPILOT_ARDUPILOTMEGA;     // MAV_AUTOPILOT ENUM
  //mypacket.autopilot = MAV_AUTOPILOT_GENERIC;     // MAV_AUTOPILOT ENUM
  //mypacket.autopilot = MAV_AUTOPILOT_PX4;     // MAV_AUTOPILOT ENUM
  mypacket.base_mode = 0;//MAV_AUTOPILOT_GENERIC;      // MAV_MODE_FLAGS ENUM
  mypacket.custom_mode = 0;  // Autpilot specific flags
  mypacket.system_status = 0;//MAV_STATE_STANDBY;           // MAV_STATE ENUM
  mypacket.mavlink_version = 3;                      // MAVLink version not writable by user
  
  msg.len = MAVLINK_MSG_ID_HEARTBEAT_LEN;
  
  mavlink_msg_heartbeat_encode(my_sysid, my_compid, &msg, &mypacket);
  
  ROS_INFO("Sending heartbeat message to mavlink_ros");

  rosmavlink_pack(msg, rosmavlink_msg);
}

void quad_arm(void)
{
  mavlink_ros::Mavlink rosmavlink_msg;
  
  mavlink_message_t msg;

  mavlink_command_long_t mypacket;
  
  mypacket.target_system = sysid; ///< System which should execute the command
  mypacket.target_component = compid; ///< Component which should execute the command, 0 for all components
  mypacket.confirmation = 0; ///< 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
  mypacket.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
  //mypacket.command = MAV_CMD_COMPONENT_ARM_DISARM;    // ID=400
  mypacket.param1 = 1;   // 1 = Arm, 0 = Disarm
  mypacket.param2 = 0;   // Not used. Do we need to define these?
  mypacket.param3 = 0;   // Not used. Do we need to define these?
  mypacket.param4 = 0;   // Not used. Do we need to define these?
  mypacket.param5 = 0;   // Not used. Do we need to define these?
  mypacket.param6 = 0;   // Not used. Do we need to define these?
  mypacket.param7 = 0;   // Not used. Do we need to define these?

  // Message ID and length are defined in the _encode functions
  mavlink_msg_command_long_encode(my_sysid, my_compid, &msg, &mypacket);

  //ROS_INFO("Arm message 2 ID: %d - %d", msg.msgid, rosmavlink_msg.msgid);
  
  //display_mavlink_packet(msg);
  //display_ros_packet(rosmavlink_msg);
  
  ROS_INFO("Sending quad_arm message to mavlink_ros");

  rosmavlink_pack(msg, rosmavlink_msg);

  //mavlink_pub.publish(rosmavlink_msg);
  
}

void init_quad_marker()
{
    visualization_msgs::Marker marker;
    visualization_msgs::Marker prop1;
    visualization_msgs::Marker prop2;
    visualization_msgs::Marker prop3;
    visualization_msgs::Marker prop4;
    visualization_msgs::Marker mav_dir;

    //visualization_msgs::MarkerArray marker_array_msg;

    // Set size of markerArray
    quad_marker_array.markers.resize(5);

    // Set all the common fields of the prop markers

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/mav";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    //marker.ns = "basic_shapes";
    //marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CYLINDER;
    

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    // The x and y position are
    marker.pose.position.z = 0;
    
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // This sets the width with x & y (can be oval) and the cylinder thickness with z
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
  
    // Setup the same for the arros
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    mav_dir.header.frame_id = "/px4";
    mav_dir.header.stamp = ros::Time::now();
    mav_dir.type = visualization_msgs::Marker::ARROW;
    mav_dir.action = visualization_msgs::Marker::ADD;
    
    mav_dir.ns = "mav_arrow";
    mav_dir.id = 5;
    
    mav_dir.pose.position.x = 0.175;
    mav_dir.pose.position.y = 0;
    mav_dir.pose.position.z = 0;
    
    mav_dir.pose.orientation.x = 0.0;
    mav_dir.pose.orientation.y = 0.0;
    //mav_dir.pose.orientation.z = -1.0;  // Normally the arrow points along the X axis. This rotates it to the Y axis
    mav_dir.pose.orientation.z = 0.0;
    mav_dir.pose.orientation.w = 1.0;
    
    mav_dir.scale.x = 0.25;      // Arrow length
    mav_dir.scale.y = 0.02;     // Arrow width
    mav_dir.scale.z = 0.02;     // Arrow height

    // Set the color -- be sure to set alpha to something non-zero!
    mav_dir.color.r = 0.7f;
    mav_dir.color.g = 1.0f;
    mav_dir.color.b = 0.4f;
    mav_dir.color.a = 1.0;

    mav_dir.lifetime = ros::Duration();

    // Copy the common marker to each prop
    prop1 = marker;
    prop2 = marker;
    prop3 = marker;
    prop4 = marker;

    // The namespace and ID must be unique for each shape
    prop1.ns = "prop1";
    prop1.id = 1;
    prop1.pose.position.x = 0.15;
    prop1.pose.position.y = 0.15;
    
    prop2.ns = "prop2";
    prop2.id = 2;
    prop2.pose.position.x = -0.15;
    prop2.pose.position.y = 0.15;
    
    prop3.ns = "prop3";
    prop3.id = 3;
    prop3.pose.position.x = 0.15;
    prop3.pose.position.y = -0.15;
    
    prop4.ns = "prop4";
    prop4.id = 4;
    prop4.pose.position.x = -0.15;
    prop4.pose.position.y = -0.15;
    
    
    //ROS_INFO (" before adding props to array");
    quad_marker_array.markers[0] = prop1;
    quad_marker_array.markers[1] = prop2;
    quad_marker_array.markers[2] = prop3;
    quad_marker_array.markers[3] = prop4;

    quad_marker_array.markers[4] = mav_dir;
    //marker_array_msg.markers.append(prop1);
    //marker_array_msg.markers.push_back (prop1);

    //ROS_INFO (" before publishung to topic");
    // Publish the markers
    //marker_pub.publish(prop1);
    //marker_pub.publish(prop2);
    //marker_pub.publish(prop3);
    //marker_pub.publish(prop4);
  
    //pub_markers.publish(marker_array_msg);


}

// from asctec_hl_interface
inline void angle2quaternion(const double &roll, const double &pitch, const double &yaw, double *w, double *x,
                             double *y, double *z)
{
  double sR2, cR2, sP2, cP2, sY2, cY2;
  sincos(roll * 0.5, &sR2, &cR2);
  sincos(pitch * 0.5, &sP2, &cP2);
  sincos(yaw * 0.5, &sY2, &cY2);

  // TODO: change rotation order
  // this follows AscTec's pre- 2012 firmware rotation order: Rz*Rx*Ry
//  *w = cP2 * cR2 * cY2 - sP2 * sR2 * sY2;
//  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
//  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
//  *z = cP2 * cR2 * sY2 + cY2 * sP2 * sR2;

  // Rz*Ry*Rx for 2012 firmware on the:
  *w = cP2 * cR2 * cY2 + sP2 * sR2 * sY2;
  *x = cP2 * cY2 * sR2 - cR2 * sP2 * sY2;
  *y = cR2 * cY2 * sP2 + cP2 * sR2 * sY2;
  *z = cP2 * cR2 * sY2 - cY2 * sP2 * sR2;
}

void mavlinkCallback(const mavlink_ros::Mavlink &mavlink_ros_msg)
{

  /**
   * Look for attitude messages and display markers to RViz
   */
  mavlink_message_t msg;
  msg.msgid = mavlink_ros_msg.msgid;

  //static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;

  //Copy payload from mavlink_ros_msg (from ROS topic) to the new "real" mavlink message
  copy(mavlink_ros_msg.payload64.begin(), mavlink_ros_msg.payload64.end(), msg.payload64);

  //ROS_INFO("Received ROS Mavlink message, Message-ID: [%i]", mavlink_ros_msg.msgid);
  //ROS_INFO("Received Mavlink message, Message-ID: [%i]", msg.msgid);
  //ROS_INFO("Seq: [%i]", mavlink_ros_msg.seq);
  
  //switch (msg.msgid)
  switch (mavlink_ros_msg.msgid)
  {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
      //ROS_INFO("Heartbeat Message Received.");
      //printf("H");
      fprintf(stderr, "H");
    
    }
    break;

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
      //ROS_INFO("Global Position Message Received.");
      
      mavlink_global_position_int_t global_pos;
      //mavlink_msg_attitude_decode(&message, &att);
      mavlink_msg_global_position_int_decode(&msg, &global_pos);

      //ROS_INFO("Lat: [%f], Lon: [%f], Alt: [%f], Relative Alt: [%f]", global_pos.lat, global_pos.lon, global_pos.alt, global_pos.relative_alt);

    }
    break;

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:   //32
    {
      mavlink_local_position_ned_t local_pos;
      //mavlink_msg_attitude_decode(&message, &att);
      mavlink_msg_local_position_ned_decode(&msg, &local_pos);

      //ROS_INFO("Local NED x: [%f], y: [%f], z: [%f], vx: [%f], vy: [%f], vz: [%f]", local_pos.x, local_pos.y, local_pos.z, local_pos.vx, local_pos.vy, local_pos.vz);
      fprintf(stderr, "PX4 NED x %f y %f z %f vx %f vy %f vz %f\n", local_pos.x, local_pos.y, local_pos.z, local_pos.vx, local_pos.vy, local_pos.vz);
      //fprintf(stderr, ".");
    
      visualization_msgs::MarkerArray marker_array_msg;
      //draw_quad();
      // 
      
      //ROS_INFO("Att Last R: [%f], P: [%f], Y: [%f]", att_last.roll, att_last.pitch, att_last.yaw);

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(local_pos.x, local_pos.y, local_pos.z) );
      tf::Quaternion q;
      q.setRPY(att_last.roll, att_last.pitch, att_last.yaw);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_origin", "px4"));

      init_quad_marker();

    }
    break;

    case MAVLINK_MSG_ID_ATTITUDE:   //30
    {
      //if (imu_pub.getNumSubscribers() > 0)
      //{
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&msg, &att);

        //ROS_INFO("Att Roll: [%f]", att.roll);
        //printf("Att Roll2")
        sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu);

        angle2quaternion(att.roll, -att.pitch, -att.yaw, &(imu_msg->orientation.w), &(imu_last->orientation.x),
                         &(imu_last->orientation.y), &(imu_last->orientation.z));
        
        //ROS_INFO("Att R: [%f], P: [%f], Y: [%f], vR: [%f], vP: [%f], vY: [%f]", att.roll, att.pitch, att.yaw, att.rollspeed, att.pitchspeed, att.yawspeed);
        //ROS_INFO("Att R: [%f], P: [%f], Y: [%f]", att.roll, att.pitch, att.yaw);

        att_last.roll = att.roll;
        att_last.pitch = att.pitch;
        att_last.yaw = att.yaw;


      }
      break;
    
  }


  //Copy payload from mavlink_msg (from ROS) to the new "real" mavlink message
  //copy(mavlink_ros_msg.payload64.begin(), mavlink_ros_msg.payload64.end(), msg.payload64);

  //mavlink_finalize_message_chan(&msg, mavlink_ros_msg.sysid, mavlink_ros_msg.compid, MAVLINK_COMM_0,
        //                        mavlink_ros_msg.len, mavlink_crcs[msg.msgid]);

  /**
   * Send mavlink_message to UART
   */
  //if (verbose)
    
}


void arPoseCallback(ar_track_alvar::AlvarMarkers arPoseMarkers)
{
  /**
   * Convert ar_track_alvar::AlvarMarkers to mavlink_message_t
   */
  
    //g_print("In arPoseCallback\n");
    //printf("In arPoseCallback2");

//printf("send00..");
//fprintf(stderr, "send00");
      
  mavlink_ros::Mavlink rosmavlink_msg;    
  mavlink_message_t msg;
  //msg.msgid = mavlink_ros_msg.msgid;

  static uint8_t mavlink_crcs[] = MAVLINK_MESSAGE_CRCS;


  ar_track_alvar::AlvarMarker ar_pose_marker;
  
  /////////////////////////////////////////////////////////////////////////////////
  // There are two ways to get the ar_track_alvar pose to send to mavlink_ros
  // 1. Use the arPoseMarkers structure from this topic callback
  //    - This would be the best way to do it but I don't know the member names and how
  //      to extract position.x, y, z and roll, pitch yaw etc.
  //    - Then would have to get the x,y,z relative to local_origin
  // 2. Use the lookuptransform() function. Which I am currently doing
  //
  /////////////////////////////////////////////////////////////////////////////////

  
  /*
  Not using this method (1)

  // The line below was causing errors due to maybe there being no .markers[0]
  // - Should check size first
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

  double roll = tagPose.getRotation().x();
  double pitch = tagPose.getRotation().y();
  double yaw = tagPose.getRotation().z();
  double yaw2 = tf::getYaw(tagPose.getRotation());
  
  //tagPose.rotation.getRPY(roll, pitch, yaw);
  
  double dx = tagPose.getOrigin().x();
  double dy = tagPose.getOrigin().y();
  double dz = tagPose.getOrigin().z();
      

  ROS_INFO("Received ar_pose_marker message for AR Marker-ID: [%i]", ar_pose_marker.id);
  ROS_INFO("Position - x: [%f], y:[%f], z:[%f]", dx, dy, dz);
  ROS_INFO("Orientation - roll: [%f], pitch: [%f], yaw: [%f]", roll, pitch, yaw);
  ROS_INFO("yaw2: [%f]", yaw2);
        
//ROS_INFO("x: [%i], y: [%i], z: [%i], w: [%i]", ar_pose_marker.id);
  
  //tf::Quaternion q(x, y, z, w);
  //tf::Matrix3x3 m(q);
  //double roll, pitch, yaw;
  //m.getRPY(*roll, *pitch, *yaw);
  
  double theta_r = tf::getYaw(tagPose.getRotation());
  */



  /////////////////////////////////////////////
  // Method 2:
  //
  // This direct accessing the transform 
  // If using this method it should really be put in the main loop and not use a callback
  //
  // Improvement:
  // 1. The improve the waitfortransform part
  // 2. What if there is no transform available?

 //ROS_INFO("Getting MAV frame relative to local_origin...");
 
  tf::TransformListener listener;
  tf::StampedTransform transform;

  // Wait up to 3s for transform
  listener.waitForTransform("/local_origin", "/mav", ros::Time(0), ros::Duration(3.0));
  try{
    listener.lookupTransform("/local_origin", "/mav",
    //listener.lookupTransform("/mav", "/local_origin",
                            ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    printf("lookupTransform failed\n");
    ROS_ERROR("%s",ex.what());
  }

  
  //tf::StampedTransform transform_ned;
  tf::Transform transform_ned;
  
  //////////////////////////////////////////////////////////////////////////
  // Convert North-East-Down (NED) to East-North-Up (ENU) coordinates
  
  // Get MAV orientation in radians
  double rad_roll = transform.getRotation().x();
  double rad_pitch = transform.getRotation().y();
  double rad_yaw = transform.getRotation().z();
  double rad_yaw2 = tf::getYaw(transform.getRotation());

  double dx = transform.getOrigin().x();    //tagPose.getOrigin().x();
  double dy = transform.getOrigin().y();    //tagPose.getOrigin().y();
  double dz = transform.getOrigin().z();    //tagPose.getOrigin().z();

  printf("ar_pose x %f y %f z %f - R %f P %f Y %f", dx, dy, dz, rad_roll, rad_pitch, rad_yaw);
  

  // Construct VISION_POSITION_ESTIMATE message
  mavlink_vision_position_estimate_t vision_pos;
         
  //vision_pos.usec = ros::Time::now();   // I don't think this is usec??
  vision_pos.x = dx;
  vision_pos.y = dy;
  vision_pos.z = dz;
  
  vision_pos.roll = rad_roll;
  vision_pos.pitch = rad_pitch;
  vision_pos.yaw = rad_yaw;

  mavlink_msg_vision_position_estimate_encode(my_sysid, my_compid, &msg, &vision_pos);  
  
  //ROS_INFO("Sent ROS Mavlink to mavlink_ros, Message-ID: [%i]", msg.msgid);
  //fprintf(stderr, "Sent ROS Mavlink to mavlink_ros, Message-ID: [%i]\n", msg.msgid);

  // Pack the mavlink message into ROS message format to send to mavlink_ros node    
  rosmavlink_pack(msg, rosmavlink_msg); 
  printf(" - sub: %d\n", mavlink_pub.getNumSubscribers());
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "mavlink_send");
  
  // Reference code from mavlink_ros_serial.cpp
  //ros::init(argc, argv, "mavlink_ros_serial");
  // SETUP ROS
  //ros::NodeHandle mavlink_nh("/mavlink"); // always root namespace since I assume it's somewhat "broadcast"
  //mavlink_sub = mavlink_nh.subscribe("to", 1000, mavlinkCallback);
  //mavlink_pub = mavlink_nh.advertise<mavlink_ros::Mavlink>("from", 1000);


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
 
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  mavlink_pub = n.advertise<mavlink_ros::Mavlink>("/mavlink/to", 1000);
  

  // Subcribe to the /mavlink/from topic
  printf("Subscribing to /mavlink/from topic");
  //        mavlink_sub = arpose_nh.subscribe("ar_pose_marker", 1000, arPoseCallback);
  ros::Subscriber mavlink_sub = n.subscribe("/mavlink/from", 1000, mavlinkCallback);
  
  // Subcribe to the ar_pose_marker topic
  printf("Subscribing to /ar_pose_marker topic");
  ros::Subscriber ar_pose_sub = n.subscribe("ar_pose_marker", 1000, arPoseCallback);
  

  //mavlink_pub = mavlink_nh.advertise<mavlink_ros::Mavlink>("from", 1000);

  ros::Rate loop_rate(10);       // in Hz

  ROS_INFO("Mavlink_send started V1.1. Going into loop at 1Hz");


  #if MAVLINK_NEED_BYTE_SWAP
    ROS_INFO("Mavlink Byte Swap Enabled");
  #else
    ROS_INFO("Mavlink Byte Swap Disabled");
  #endif
  
  #if MAVLINK_CRC_EXTRA
    ROS_INFO("Mavlink Extra CRC Enabled");
  #else
    ROS_INFO("Mavlink Extra CRC Disabled");
  #endif
  ROS_INFO("Mavlink_STX: %d", int(MAVLINK_STX));
  ///////////////////////////////////////
  // Server stuff
  //ros::init(argc, argv, "add_two_ints_server");
  //ros::NodeHandle n;

  //ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  //ROS_INFO("Ready to add two ints.");
  //////////////////////////////////////////////

  // Initilise marker_array_msg for visualisation of quad in RViz
  init_quad_marker();

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    //std_msgs::String msg;
    //mavlink_ros::Mavlink rosmavlink_msg;

    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
