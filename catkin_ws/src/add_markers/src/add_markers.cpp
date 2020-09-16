  30 #include <ros/ros.h>
  31 #include <visualization_msgs/Marker.h>
  32 
  33 int main( int argc, char** argv )
  34 {
  35   ros::init(argc, argv, "basic_shapes");
  36   ros::NodeHandle n;
  37   ros::Rate r(1);
  38   ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  39 
  40   // Set our initial shape type to be a cube
  41   uint32_t shape = visualization_msgs::Marker::CUBE;
  42 
  43   while (ros::ok())
  44   {
  45     visualization_msgs::Marker marker;
  46     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  47     marker.header.frame_id = "/my_frame";
  48     marker.header.stamp = ros::Time::now();
  49 
  50     // Set the namespace and id for this marker.  This serves to create a unique ID
  51     // Any marker sent with the same namespace and id will overwrite the old one
  52     marker.ns = "basic_shapes";
  53     marker.id = 0;
  54 
  55     // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  56     marker.type = shape;
  57 
  58     // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  59     marker.action = visualization_msgs::Marker::ADD;
  60 
  61     // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  62     marker.pose.position.x = 0;
  63     marker.pose.position.y = 0;
  64     marker.pose.position.z = 0;
  65     marker.pose.orientation.x = 0.0;
  66     marker.pose.orientation.y = 0.0;
  67     marker.pose.orientation.z = 0.0;
  68     marker.pose.orientation.w = 1.0;
  69 
  70     // Set the scale of the marker -- 1x1x1 here means 1m on a side
  71     marker.scale.x = 1.0;
  72     marker.scale.y = 1.0;
  73     marker.scale.z = 1.0;
  74 
  75     // Set the color -- be sure to set alpha to something non-zero!
  76     marker.color.r = 0.0f;
  77     marker.color.g = 1.0f;
  78     marker.color.b = 0.0f;
  79     marker.color.a = 1.0;
  80 
  81     marker.lifetime = ros::Duration();
  82 
  83     // Publish the marker
  84     while (marker_pub.getNumSubscribers() < 1)
  85     {
  86       if (!ros::ok())
  87       {
  88         return 0;
  89       }
  90       ROS_WARN_ONCE("Please create a subscriber to the marker");
  91       sleep(1);
  92     }
  93     marker_pub.publish(marker);
  94 
  95     // Cycle between different shapes
  96     switch (shape)
  97     {
  98     case visualization_msgs::Marker::CUBE:
  99       shape = visualization_msgs::Marker::SPHERE;
 100       break;
 101     case visualization_msgs::Marker::SPHERE:
 102       shape = visualization_msgs::Marker::ARROW;
 103       break;
 104     case visualization_msgs::Marker::ARROW:
 105       shape = visualization_msgs::Marker::CYLINDER;
 106       break;
 107     case visualization_msgs::Marker::CYLINDER:
 108       shape = visualization_msgs::Marker::CUBE;
 109       break;
 110     }
 111 
 112     r.sleep();
 113   }
 114 }