#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <string.h>

class Marker
{
public:
  Marker(ros::NodeHandle *n)
  {
    ROS_INFO("Marker: Virtual object initialized!");
    _marker_pub = n->advertise<visualization_msgs::Marker>("visualization_marker", 1);
    _state_sub = n->subscribe("goal_command", 2, &Marker::setCoord, this);
    _coord_sub = n->subscribe("pick", 2, &Marker::dropPick, this);

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    _marker.header.frame_id = "map";
    _marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    _marker.ns = "basic_shapes";
    _marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    _marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    _marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    _marker.pose.position.x = 1;
    _marker.pose.position.y = 0;
    _marker.pose.position.z = 0;
    _marker.pose.orientation.x = 0.0;
    _marker.pose.orientation.y = 0.0;
    _marker.pose.orientation.z = 0.0;
    _marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    _marker.scale.x = 0.2;
    _marker.scale.y = 0.2;
    _marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    _marker.color.r = 0.0f;
    _marker.color.g = 1.0f;
    _marker.color.b = 0.0f;
    _marker.color.a = 1.0;

    _marker.lifetime = ros::Duration();
    // flag for object state; 0=not-picked
    _object_picked = false;
  }

  void dropPick(const std_msgs::Int32::ConstPtr &msg)
  {

    while (_marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
        return;
      ROS_WARN_ONCE("Marker: Please create a subscriber to the marker!");
      sleep(1);
    }

    // check the states of the object; 1=pick, 0=drop
    if (msg->data == 1)
    {
      ROS_INFO("Marker: Pick object command recieved!");
      _marker.visualization_msgs::Marker::DELETE;
      _marker.color.a = 0.0;
      _marker_pub.publish(_marker);
    }
    else
    {
      // return if object not picked before
      ROS_INFO("Marker: Drop object command recieved!");
      _marker.visualization_msgs::Marker::ADD;
      _marker.color.a = 1.0;
      _marker_pub.publish(_marker);
    }
  }

  void dropPick(const std_msgs::Int32& msg)
  {

    while (_marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
        return;
      ROS_WARN_ONCE("Marker: Please create a subscriber to the marker!");
      sleep(1);
    }

    // check the states of the object; 1=pick, 0=drop
    if (msg.data == 1)
    {
      ROS_INFO("Marker: Pick object command recieved!");
      _marker.visualization_msgs::Marker::DELETE;
      _marker.color.a = 0.0;
      _marker_pub.publish(_marker);
    }
    else
    {
      // return if object not picked before
      ROS_INFO("Marker: Drop object command recieved!");
      _marker.visualization_msgs::Marker::ADD;
      _marker.color.a = 1.0;
      _marker_pub.publish(_marker);
    }
  }

  void setCoord(const geometry_msgs::Point::ConstPtr &msg)
  {
    ROS_INFO("Marker: New coordinates received!");
    _marker.pose.position.x = msg->x;
    _marker.pose.position.y = msg->y;
  }

  void setCoord(const geometry_msgs::Point& msg)
  {
    ROS_INFO("Marker: New coordinates received!");
    _marker.pose.position.x = msg.x;
    _marker.pose.position.y = msg.y;
  }
  

  void test()
  {
    geometry_msgs::Point p;
    // init
    p.x = 4;
    p.y = -2;
    p.z = 0;
    std_msgs::Int32 gst;
    gst.data = 0;
    setCoord(p);
    dropPick(gst);
    // pick
    gst.data = 1;
    ros::Duration(5).sleep();
    dropPick(gst);
    // drop
    p.x = -5;
    p.y = -1;
    p.z = 0;
    gst.data = 0;
    setCoord(p);
    ros::Duration(5).sleep();
    dropPick(gst);
    ros::Duration(5).sleep();
  }

private:
  ros::NodeHandle _n;
  visualization_msgs::Marker _marker;
  ros::Publisher _marker_pub;
  ros::Subscriber _state_sub;
  ros::Subscriber _coord_sub;
  bool _object_picked;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  Marker m = Marker(&n);
  if (argc >= 2)
  {
<<<<<<< HEAD
    if (std::string(argv[1]) == "test")
    {
      ROS_INFO("Marker: Running tests!");
      m.test();
      ROS_INFO("Marker: Testing is done!");
=======
    if (argv[1] == "test")
    {
      ROS_INFO("Marker: Running tests!");
      m.test();
      ROS_INFO("Marker:Testing done!");
>>>>>>> 36a0a0a910ea283c455ee37c570672645e9c6ad9
      return 0;
    }
  }
  ros::spin();
  return 0;
}