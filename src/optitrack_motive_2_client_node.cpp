// Standard includes
#include <iostream>
#include <fstream>
#include <deque>
#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Includes for ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

// Include motion capture framework
#include "optitrack_motive_2_client/motionCaptureClientFramework.h"

namespace po = boost::program_options;
using namespace Eigen;

class OptitrackMotive2Client  {
public:

  OptitrackMotive2Client(std::string myIP, std::string serverIP) {

    // Init mocap framework
    agile::motionCaptureClientFramework mocap_ = agile::motionCaptureClientFramework(myIP, serverIP);

    // Used to convert mocap frame (NUE) to ROS ENU.
    R_NUE2ENU << 0, 0, 1,
                 1, 0, 0,
                 0, 1, 0;

    // Create a publisher for the base link pose
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/optitrack/pose", 1);

    // Create a subscriber for lag messages
    double lag = 0;
    lag_sub = n.subscribe("/optitrack/lag", 1, &OptitrackMotive2Client::lag_callback, this);

    while (ros::ok()) {
      // Wait for mocap packet
      mocap_.spin();
      
      // @TODO: Make getPackets return a list.
      std::vector<agile::Packet> mocap_packets = mocap_.getPackets();

      for (agile::Packet mocap_packet : mocap_packets) {

        // Skip this rigid body if tracking is invalid
        if (!mocap_packet.tracking_valid) continue;

        // Collect time offsets
        int64_t offset = mocap_packet.transmit_timestamp - mocap_packet.receive_timestamp;
        offsets.push_back(offset);
        // Make the queue stays the same size
        while (offsets.size() > window_size) offsets.pop_front();

        // Perform a rolling average over the offsets
        offset_between_windows_and_linux = 0;
        for (auto it = offsets.begin(); it != offsets.end(); it++) {
          offset_between_windows_and_linux += (*it)/offsets.size();
        }

        // Apply the offset to the exposure time
        uint64_t packet_ntime = mocap_packet.mid_exposure_timestamp - offset_between_windows_and_linux;
        double packet_time = packet_ntime/(double)1e9; // in seconds
        packet_time += lag;

        // Create a transformation message
        geometry_msgs::TransformStamped ts;

        // Add the timestamp created using the offsets
        // Add in the lag parameter to account for system lag
        ts.header.stamp = ros::Time(packet_time);

        // Add the frames
        ts.header.frame_id = "/map";
        ts.child_frame_id = "/laser";

        // Add the translation
        Vector3d positionENUVector = positionConvertNUE2ENU(mocap_packet.pos);
        ts.transform.translation.x = positionENUVector(0);
        ts.transform.translation.y = positionENUVector(1);
        ts.transform.translation.z = positionENUVector(2);
        // Add the rotation
        Quaterniond quaternionENUVector = quaternionConvertNUE2ENU(mocap_packet.orientation);
        ts.transform.rotation.x = quaternionENUVector.x();
        ts.transform.rotation.y = quaternionENUVector.y();
        ts.transform.rotation.z = quaternionENUVector.z();
        ts.transform.rotation.w = quaternionENUVector.w();

        // Create a pose for visualization in rviz
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ts.header.stamp;
        ps.header.frame_id = "/laser";

        // Publish the transform and the pose
        br.sendTransform(ts);
        pose_pub.publish(ps);
      }
      ros::spinOnce();
    }
  }

    
private:
  // ROS Communication
  ros::NodeHandle n;
  tf2_ros::TransformBroadcaster br;
  ros::Subscriber lag_sub;
  ros::Publisher pose_pub;

  // Variables to keep track of the time offset between the computers
  int64_t offset_between_windows_and_linux = std::numeric_limits<int64_t>::max();
  static const int window_size = 100;
  std::deque<int64_t> offsets;
  double lag;

  // Used to convert mocap frame (NUE) to ROS ENU.
  Eigen::Matrix3d R_NUE2ENU;

  void lag_callback(const std_msgs::Float64::ConstPtr & msg) {
    lag = msg -> data;
    std::cout << "lag: " << lag << std::endl;
  }

  Vector3d positionConvertNUE2ENU(double* positionNUE){
    Vector3d positionNUEVector, positionENUVector;
    positionNUEVector << positionNUE[0], positionNUE[1], positionNUE[2];
    
    positionENUVector = R_NUE2ENU * positionNUEVector;
    return positionENUVector;
  }

  Quaterniond quaternionConvertNUE2ENU(double* quaternionNUE){
    Quaterniond quaternionInNUE;
    quaternionInNUE.x() = quaternionNUE[0];
    quaternionInNUE.y() = quaternionNUE[1];
    quaternionInNUE.z() = quaternionNUE[2];
    quaternionInNUE.w() = quaternionNUE[3];

    Quaterniond quaternionInENU = Quaterniond(R_NUE2ENU * quaternionInNUE.normalized().toRotationMatrix()
                              * R_NUE2ENU.transpose());
    return quaternionInENU;
  }
};

int main(int argc, char *argv[]) {
  // Get CMDline arguments for server and local IP addresses.
  std::string szMyIPAddress; 
  std::string szServerIPAddress; 
  try {
    po::options_description desc ("Options");
    desc.add_options()
        ("help,h", "print usage message")
        ("local",po::value<std::string>(&szMyIPAddress),"local IP Address")
        ("server",po::value<std::string>(&szServerIPAddress), "server address");

    po::variables_map vm;
    try {
      po::store(po::parse_command_line(argc, argv, desc), vm);
      po::notify (vm);
    }
    catch (po::error& e) {
      std::cerr << e.what() << std::endl;
      return 0;
    }
    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }
  }
  catch (...) {}

  // Init ROS
  ros::init(argc, argv, "optitrack_motive_2_client_node");
  OptitrackMotive2Client(szMyIPAddress, szServerIPAddress);
  return 0;
}
