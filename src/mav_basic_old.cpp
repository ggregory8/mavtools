#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("mav_marker", 1);
  // This disn't work, apparently MarkerArray isn't part of visualization_msgs
  //ros::Publisher pub_marker = n.advertise<visualization_msgs::MarkerArray>("marker_array", 100);

  // Set our initial shape type to be a cylinder
  uint32_t shape = visualization_msgs::Marker::CYLINDER;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    visualization_msgs::Marker prop1;
    visualization_msgs::Marker prop2;
    visualization_msgs::Marker prop3;
    visualization_msgs::Marker prop4;
    
    //visualization_msgs::MarkerArray marker_array_msg;

    // Set all the common fields of the prop markers

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/camera";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    //marker.ns = "basic_shapes";
    //marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

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
  
    //marker_array_msg.markers[0] = prop1;
    //marker_array_msg.markers[1] = prop2;
    //marker_array_msg.markers[2] = prop3;
    //marker_array_msg.markers[3] = prop4;

    // Publish the markers
    marker_pub.publish(prop1);
    marker_pub.publish(prop2);
    marker_pub.publish(prop3);
    marker_pub.publish(prop4);
  
    //pub_marker.publish(marker_array_msg);


    
    //shape = visualization_msgs::Marker::CYLINDER;
    
    r.sleep();
  }
}