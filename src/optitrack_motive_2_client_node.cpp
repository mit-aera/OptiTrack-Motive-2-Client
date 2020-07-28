// Include motion capture framework
#include "optitrack_motive_2_client/motionCaptureClientFramework.h"
// Include ACL message types (https://bitbucket.org/brettlopez/acl_msgs.git)
#include "acl_msgs/ViconState.h"

// Includes for ROS
#include "ros/ros.h"

// Includes for node
#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace po = boost::program_options;
using namespace Eigen;

// Used to convert mocap frame (NUE) to LCM NED.
// static Eigen::Matrix3d R_NUE2NED = [] {
//     Eigen::Matrix3d tmp;
//     tmp <<  1, 0, 0,
//             0, 0, 1,
//             0, -1, 0;
//     return tmp;
// }();


// Used to convert mocap frame (NUE) to ROS ENU.
static Eigen::Matrix3d R_NUE2ENU = [] {
    Eigen::Matrix3d tmp;
    tmp <<  0, 0, 1,
            1, 0, 0,
            0, 1, 0;
    return tmp;
}();

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

int main(int argc, char *argv[])
{
  // Keep track of ntime offset.
  int64_t offset_between_windows_and_linux = std::numeric_limits<int64_t>::max();

  // Init ROS
  ros::init(argc, argv, "optitrack_motive_2_client_node");
  ros::NodeHandle n;

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

  // Init mocap framework
  agile::motionCaptureClientFramework mocap_ = agile::motionCaptureClientFramework(szMyIPAddress, szServerIPAddress);

  ROS_INFO("\nInitialized motion capture framework\n");
  // Some vars to calculate twist/acceleration and dts
  // Also keeps track of the various publishers
  std::map<int, ros::Publisher> rosPublishers;
  std::map<int, acl_msgs::ViconState> pastStateMessages;
  ros::Publisher velPublisher;
  tf2_ros::TransformBroadcaster broadcaster;
  velPublisher = n.advertise<geometry_msgs::Vector3Stamped>("/uav/velocity", 100);

  while (true){
    // Wait for mocap packet
    mocap_.spin();

    ROS_INFO("Got motion capture packet\n");
    std::vector<agile::Packet> mocap_packets = mocap_.getPackets();

    for (agile::Packet mocap_packet : mocap_packets){

      // @TODO: Make getPackets return a list.

      // Skip this rigid body if tracking is invalid
      if (!mocap_packet.tracking_valid)
        continue;

      // estimate the windows to linux constant offset by taking the minimum seen offset.
      // @TODO: Make offset a rolling average instead of a latching offset.
      int64_t offset = mocap_packet.transmit_timestamp - mocap_packet.receive_timestamp;
      if (offset < offset_between_windows_and_linux ){
        offset_between_windows_and_linux = offset;
      }
      uint64_t packet_ntime = mocap_packet.mid_exposure_timestamp - offset_between_windows_and_linux;

      // Get past state and publisher (if they exist)
      bool hasPreviousMessage = (rosPublishers.find(mocap_packet.rigid_body_id) != rosPublishers.end());
      ros::Publisher publisher;
      acl_msgs::ViconState lastState;
      acl_msgs::ViconState currentState;

      // Initialize publisher for rigid body if not exist.
      if (!hasPreviousMessage){
        std::string topic = "/" + mocap_packet.model_name + "/vicon";
        ROS_INFO("Making a publisher"); 

        publisher = n.advertise<acl_msgs::ViconState>(topic, 1);
        rosPublishers[mocap_packet.rigid_body_id] = publisher;
      } else {
        // Get saved publisher and last state
        publisher = rosPublishers[mocap_packet.rigid_body_id];
        lastState = pastStateMessages[mocap_packet.rigid_body_id];
      }

      // Add timestamp
      currentState.header.stamp = ros::Time(packet_ntime/1e9, packet_ntime%(int64_t)1e9);

      // Convert rigid body position from NUE to ROS ENU
      Vector3d positionENUVector = positionConvertNUE2ENU(mocap_packet.pos);
      currentState.pose.position.x = positionENUVector(0);
      currentState.pose.position.y = positionENUVector(1);
      currentState.pose.position.z = positionENUVector(2);
      // Convert rigid body rotation from NUE to ROS ENU
      Quaterniond quaternionENUVector = quaternionConvertNUE2ENU(mocap_packet.orientation);
      currentState.pose.orientation.x = quaternionENUVector.x();
      currentState.pose.orientation.y = quaternionENUVector.y();
      currentState.pose.orientation.z = quaternionENUVector.z();
      currentState.pose.orientation.w = quaternionENUVector.w();
      currentState.has_pose = true;

//    transform.setOrigin(tf2::Vector3(positionENUVector(0), positionENUVector(1), positionENUVector(2)));
//    transform.setRotation(tf2::Quaternion(quaternionENUVector.x(), 
//                                         quaternionENUVector.y(),
//                                         quaternionENUVector.z(),
//                                         quaternionENUVector.w()));

      // Loop through markers and convert positions from NUE to ENU
      // @TODO since the state message does not understand marker locations.

      if (hasPreviousMessage){
        // Calculate twist. Requires last state message.
        int64_t dt_nsec = packet_ntime - (lastState.header.stamp.sec*1e9 + lastState.header.stamp.nsec);
        currentState.twist.linear.x = (currentState.pose.position.x - lastState.pose.position.x)*1e9/dt_nsec;
        currentState.twist.linear.y = (currentState.pose.position.y - lastState.pose.position.y)*1e9/dt_nsec;
        currentState.twist.linear.z = (currentState.pose.position.z - lastState.pose.position.z)*1e9/dt_nsec;

        // Calculate rotational twist
        Quaterniond lastQuaternion;
        lastQuaternion.x() = lastState.pose.orientation.x;
        lastQuaternion.y() = lastState.pose.orientation.y;
        lastQuaternion.z() = lastState.pose.orientation.z;
        lastQuaternion.w() = lastState.pose.orientation.w;

        // @TODO: calculate the angular velocity from the two quaternions in body frame.
        Quaterniond twistQuaternion = quaternionENUVector * lastQuaternion.inverse();
        Vector3d twistEulerVector = twistQuaternion.toRotationMatrix().eulerAngles(2, 1, 0);

        currentState.twist.angular.x = 0;
        currentState.twist.angular.y = 0;
        currentState.twist.angular.z = 0;

        //currentState.twist.angular.x = twistEulerVector(0);
        //currentState.twist.angular.y = twistEulerVector(1);
        //currentState.twist.angular.z = twistEulerVector(2);
        //currentState.has_twist = true;

        // Calculate accelerations. Requires last state message.
        currentState.accel.x = (currentState.twist.linear.x - lastState.twist.linear.x)*1e9/dt_nsec;
        currentState.accel.y = (currentState.twist.linear.y - lastState.twist.linear.y)*1e9/dt_nsec;
        currentState.accel.z = (currentState.twist.linear.z - lastState.twist.linear.z)*1e9/dt_nsec;
        currentState.has_accel = true;

        // @TODO: Calculate angular acceleration
      }

      // Save state for future acceleration and twist computations
      pastStateMessages[mocap_packet.rigid_body_id] = currentState;

      // Publish ROS state.
      geometry_msgs::TransformStamped transformStamped;
      geometry_msgs::Vector3Stamped _vector3stamped;
      std::cout << "Time: " << currentState.header.stamp << std::endl;
      _vector3stamped.header.stamp = currentState.header.stamp;
      _vector3stamped.vector.x = currentState.twist.linear.x;
      _vector3stamped.vector.y = currentState.twist.linear.y;
      _vector3stamped.vector.z = currentState.twist.linear.z;

      transformStamped.header.stamp = currentState.header.stamp;
      transformStamped.header.frame_id = "mocap";
      transformStamped.child_frame_id = "uav/imu";
      transformStamped.transform.translation.x = currentState.pose.position.x;
      transformStamped.transform.translation.y = currentState.pose.position.y;
      transformStamped.transform.translation.z = currentState.pose.position.z;
      transformStamped.transform.rotation.x = currentState.pose.orientation.x;
      transformStamped.transform.rotation.y = currentState.pose.orientation.y;
      transformStamped.transform.rotation.z = currentState.pose.orientation.z;
      transformStamped.transform.rotation.w = currentState.pose.orientation.w;
      
      velPublisher.publish(_vector3stamped);
      broadcaster.sendTransform(transformStamped);
      ros::spinOnce();
      //publisher.publish(currentState);

    }
  }
}
