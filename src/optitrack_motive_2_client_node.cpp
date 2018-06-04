// Include motion capture framework
#include "optitrack_motive_2_client/motionCaptureClientFramework.h"
// Include ACL message types (https://bitbucket.org/brettlopez/acl_msgs.git)
#include "acl_msgs/ViconState.h"

// Includes for node
#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>

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
    
  // Some vars to calculate velocity and dts
  Vector3f last_position = Vector3f::Zero();
  uint64_t last_time=0;
  uint64_t dt;

  int64_t offset_between_windows_and_linux = std::numeric_limits<int64_t>::max();

  while (true){
    // Wait for mocap packet
    mocap_.spin();
    
    auto mocap_packet = mocap_.getPacket();

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

    // Convert rigid body position from NUE to ROS ENU
    // Convert rigid body rotation from NUE to ROS ENU
    
    // Loop through markers and convert positions from NUE to ENU

    // Calculate dt from last state message.

    // Calculate twist. Requires last state message.
    
    // Calculate accelerations. Requires last state message.

    // Pack ROS message

    // Save state for future acceleration and twist computations
    if (last_time == 0) {
      last_time = packet_ntime;
      // Save ros message in map indexed by rigid body id.
    }

    ///////////////////////////////////

      // Fill up LCM message
      // lcm_state_packet.orient[0] = quaternionRigidBodyInNED.w();
      // lcm_state_packet.orient[1] = quaternionRigidBodyInNED.x();
      // lcm_state_packet.orient[2] = quaternionRigidBodyInNED.y();
      // lcm_state_packet.orient[3] = quaternionRigidBodyInNED.z();

      // lcm_state_packet.position[0]    = positionInNED(0);
      // lcm_state_packet.position[1]    = positionInNED(1);
      // lcm_state_packet.position[2]    = positionInNED(2);


        // Vector3f velocityGlobal;
        // velocityGlobal << (positionInNED[0] - last_position[0]) /dt * 1e6,
        //                   (positionInNED[1] - last_position[1]) /dt * 1e6,
        //                   (positionInNED[2] - last_position[2]) /dt * 1e6;


  
        // Vector3f VelocityBody;

        // VelocityBody = quaternionRigidBodyInNED.toRotationMatrix().inverse() * velocityGlobal;

        // // lcm_state_packet.veloPositionBody[0] = VelocityBody(0);
        // // lcm_state_packet.veloPositionBody[1] = VelocityBody(1);
        // // lcm_state_packet.veloPositionBody[2] = VelocityBody(2);

        // last_position = Vector3f(positionInNED[0], positionInNED[1], positionInNED[2]);
        // last_time = lcm_state_packet.utime;

      // lcm.publish("poseMoCap", &lcm_state_packet);
  }
}
