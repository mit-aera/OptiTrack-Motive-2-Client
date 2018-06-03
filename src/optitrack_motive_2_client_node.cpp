#include "optitrack_motive_2_client/motionCaptureClientFramework.h"
#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>


namespace po = boost::program_options;
using namespace Eigen;

static Eigen::Matrix3f R_NUE2NED = [] {
    Eigen::Matrix3f tmp;
    tmp <<  1, 0, 0, 
            0, 0, 1, 
            0, -1, 0;
    return tmp;
}();

/// Calculate average orientation using quaternions
template<typename DataType, typename ForwardIterator>
Eigen::Quaternion<DataType> averageQuaternions(ForwardIterator const & begin, ForwardIterator const & end) {

  if (begin == end) {
    throw std::logic_error("Cannot average orientations over an empty range.");
  }

  Eigen::Matrix<DataType, 4, 4> A = Eigen::Matrix<DataType, 4, 4>::Zero();
  uint sum(0);
  for (ForwardIterator it = begin; it != end; ++it) {
    Eigen::Matrix<DataType, 1, 4> q(1,4);
    q(0) = it->w();
    q(1) = it->x();
    q(2) = it->y();
    q(3) = it->z();
    // q(0) = 0.837;
    // q(1) = -0.4455;
    // q(2) = 0.2122;
    // q(3) = 0.2348;
    
    A += q.transpose()*q;
    sum++;
  }
  A /= sum;

  Eigen::EigenSolver<Eigen::Matrix<DataType, 4, 4>> es(A);

  Eigen::Matrix<std::complex<DataType>, 4, 1> mat(es.eigenvalues());
  int index;
  mat.real().maxCoeff(&index);
  Eigen::Matrix<DataType, 4, 1> largest_ev(es.eigenvectors().real().block(0, index, 4, 1));



  return Eigen::Quaternion<DataType>(largest_ev(0), largest_ev(1), largest_ev(2), largest_ev(3));
}


Quaternionf calculateRotation(Vector3f mfl, Vector3f mbl, Vector3f mfr, Vector3f mbr, Quaternionf rigidBodyInNED) {

  Vector3f vx = R_NUE2NED * (mfr - mbr);
  Vector3f vy = R_NUE2NED * (mfr - mfl);
  Vector3f vz = vx.cross(vy);
  vy = vz.cross(vx);

  vx = vx/ vx.norm();
  vy = vy/ vy.norm();
  vz = vz/ vz.norm();


  // NED origin to real drone body transform
  Matrix3f R;
  R.row(0) = vx;
  R.row(1) = vy;
  R.row(2) = vz;

  
  double tol = 0.1;
  auto v_bl2fl = R_NUE2NED * (mfl - mbl);
  auto calc_error = v_bl2fl.dot(vy);
  if (calc_error > tol) {
      std::cout << "WARNING: bl marker is not in plane defined by other three markers." << std::endl;
  }

  std::cout << "Calculated Error: " << calc_error << std::endl;
  Quaternionf NED_to_real_body(R);

  // std::cout << "NED_to_real_body" << NED_to_real_body.z() << std::endl;
  // std::cout << "rigidBodyInNED" << rigidBodyInNED.z() << std::endl;

  // Apply inverse of difference between rigid body pose and actual drone body pose.
  Matrix3f rotation_offset = (R * rigidBodyInNED.normalized().toRotationMatrix()).inverse();
  
  Quaternionf rigid_body_to_real_body(rotation_offset);

  return rigid_body_to_real_body;
}

int main(int argc, char *argv[])
{
    agile::motionCaptureClientFramework mocap_;
    // alcm::LCM lcm;

    const int optval = 0x100000;
    socklen_t optval_size = 4;
    int server_frequency;


    in_addr MulticastAddress{}, MyAddress{}, ServerAddress{};
    int retval = -1;

    int mfl, mfr, mbr, mbl;
    std::string szMyIPAddress; 
    std::string szServerIPAddress; 

    try {
      po::options_description desc ("Options");
      desc.add_options()
          ("help,h", "print usage message")
          ("local",po::value<std::string>(&szMyIPAddress),"local IP Address")
          ("server",po::value<std::string>(&szServerIPAddress), "server address")
          ("mfl", po::value<int>(&mfl), "Marker Front Left")
          ("mfr", po::value<int>(&mfr), "Marker Front Right")
          ("mbr", po::value<int>(&mbr), "Marker Back Right")
          ("mbl", po::value<int>(&mbl), "Marker Back Left");

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

    char *my_address = new char[szMyIPAddress.length()+1];
    char *server_address = new char[szServerIPAddress.length()+1];

    std::strcpy (server_address, szServerIPAddress.c_str());
    std::strcpy (my_address, szMyIPAddress.c_str());

    // Open socket for listening
    auto DataSocket = socket(AF_INET, SOCK_DGRAM, 0);
    MulticastAddress.s_addr = inet_addr(MULTICAST_ADDRESS);
    printf("Client: %s\n", szMyIPAddress.c_str());
    printf("Server: %s\n", szServerIPAddress.c_str());
    printf("Multicast Group: %s\n", MULTICAST_ADDRESS);

    // ================ Create "Command" socket
    unsigned short port = 8000;
    auto CommandSocket = mocap_.CreateCommandSocket(inet_addr(szMyIPAddress.c_str()), port);
    if (CommandSocket == -1) {
    // error
      printf("Command socket creation error\n");
    } else {
      // [optional] set to non-blocking
      //u_long iMode=1;
      //ioctlsocket(CommandSocket,FIONBIO,&iMode);
      // set buffer
      setsockopt(CommandSocket, SOL_SOCKET, SO_RCVBUF, (char *) &optval, 4);
      getsockopt(CommandSocket,
               SOL_SOCKET,
               SO_RCVBUF,
               (char *) &optval,
               &optval_size);
      if (optval != 0x100000) {
        // err - actual size...
        printf("[CommandSocket] ReceiveBuffer size = %d\n", optval);
      }
    }

    // allow multiple clients on same machine to use address/port
    int value = 1;
    retval = setsockopt(DataSocket,
                        SOL_SOCKET,
                        SO_REUSEADDR,
                        (char *) &value,
                        sizeof(value));
    if (retval == -1) {
      close(DataSocket);
      printf("Error while setting DataSocket options\n");
      return -1;
    }

    struct sockaddr_in MySocketAddr{};
    memset(&MySocketAddr, 0, sizeof(MySocketAddr));
    MySocketAddr.sin_family = AF_INET;
    MySocketAddr.sin_port = htons(PORT_DATA);
    MySocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(DataSocket,
             (struct sockaddr *) &MySocketAddr,
             sizeof(struct sockaddr)) == -1) {
      printf("[PacketClient] bind failed\n");
      return 0;
    }
    // join multicast group
    struct ip_mreq Mreq{};
    Mreq.imr_multiaddr = MulticastAddress;
    Mreq.imr_interface = MyAddress;
    retval = setsockopt(DataSocket,
                        IPPROTO_IP,
                        IP_ADD_MEMBERSHIP,
                        (char *) &Mreq,
                        sizeof(Mreq));
    if (retval == -1) {
      printf("[PacketClient] join failed\n");
      return -1;
    }
    // create a 1MB buffer
    setsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *) &optval, 4);
    getsockopt(DataSocket, SOL_SOCKET, SO_RCVBUF, (char *) &optval, &optval_size);
    if (optval != 0x100000) {
      printf("[PacketClient] ReceiveBuffer size = %d\n", optval);
    }

    mocap_.setDataSocket(DataSocket);

    // ================ Server address for commands
    sockaddr_in HostAddr;

    memset(&HostAddr, 0, sizeof(HostAddr));
    HostAddr.sin_family = AF_INET;
    HostAddr.sin_port = htons(PORT_COMMAND);
    HostAddr.sin_addr.s_addr = inet_addr(szServerIPAddress.c_str());

    // send initial connect request
    agile::sPacket PacketOut{};
    PacketOut.iMessage = NAT_CONNECT;
    PacketOut.nDataBytes = 0;
    int nTries = 3;
    while (nTries--) {
      ssize_t iRet = sendto(CommandSocket,
                          (char *) &PacketOut,
                          4 + PacketOut.nDataBytes,
                          0,
                          (sockaddr *) &HostAddr,
                          sizeof(HostAddr));
      printf("Trying to connect\n");
      if (iRet != -1)
        printf("Connected!\n Waiting for server info in response.\n");

        // Wait for server response. 
        // This will contain the server tick frequency.
        char ip_as_str[INET_ADDRSTRLEN];
        ssize_t nDataBytesReceived;
        sockaddr_in TheirAddress{};
        agile::sPacket PacketIn{};
        socklen_t addr_len = sizeof(struct sockaddr);
        nDataBytesReceived = recvfrom(CommandSocket,
                                      (char *) &PacketIn,
                                      sizeof(agile::sPacket),
                                      0,
                                      (struct sockaddr *) &TheirAddress,
                                      &addr_len);

        // if ((nDataBytesReceived == 0) || (nDataBytesReceived == -1))
        //   continue;

        // debug - print message
        inet_ntop(AF_INET, &(TheirAddress.sin_addr), ip_as_str, INET_ADDRSTRLEN);
        printf("[Client] Received command from %s: Command=%d, nDataBytes=%d\n",
              ip_as_str, (int) PacketIn.iMessage, (int) PacketIn.nDataBytes);

        unsigned char *ptr = (unsigned char *) &PacketIn;
        agile::sSender_Server *server_info = (agile::sSender_Server *) (ptr + 4);

        std::cout << "server tick frequency: " << server_info->HighResClockFrequency << std::endl;
        server_frequency = server_info->HighResClockFrequency;
        // Done processing server response.
        break;
      printf("Initial connect request failed\n");
    }


    bool gotQuatOffset_ = false;
    const unsigned int  nAvgOffset     = 100;
    std::vector<Quaternionf> quats_;

    Vector3f mfl_p = Vector3f::Zero();
    Vector3f mfr_p = Vector3f::Zero();
    Vector3f mbr_p = Vector3f::Zero();
    Vector3f mbl_p = Vector3f::Zero();

    Matrix3f R_mocap2body;
    Quaternionf q_mocap2body;
    Vector3f last_position = Vector3f::Zero();
    uint64_t last_time=0;
    uint64_t dt;

    long offset_between_windows_and_linux = std::numeric_limits<long>::max();



    while (true){
        // Wait for mocap packet
        mocap_.spin();

        auto output = mocap_.getPacket();

        if (!output.tracking_valid)
          continue;

        int found_markers = 0;

        // Convert from mocap to body
        Quaternionf quaternionDroneInMocap, quaternionDroneInNED;
        quaternionDroneInMocap.w() = output.orientation[3];
        quaternionDroneInMocap.x() = output.orientation[0];
        quaternionDroneInMocap.y() = output.orientation[1];
        quaternionDroneInMocap.z() = output.orientation[2];

        Quaternionf quaternionRigidBodyInNED = Quaternionf(R_NUE2NED * quaternionDroneInMocap.normalized().toRotationMatrix()
                                      * R_NUE2NED.transpose());


        if (gotQuatOffset_) {
          // Pack packet into LCM.
          // agile::state_t lcm_state_packet;

          
          
          quaternionDroneInNED =   q_mocap2body * quaternionRigidBodyInNED ;
          // quaternionDroneInNED =  quaternionDroneInNED * q_mocap2body;



          Vector3f positionInMocap, positionInNED;
          positionInMocap << output.pos[0], output.pos[1], output.pos[2];
          positionInNED = R_NUE2NED * positionInMocap;

          // lcm_state_packet.utime = (uint64_t) (output.mid_exposure_timestamp / (double)server_frequency * 1e6);

         // calulate the windows to linux constant offset by taking the minimum seen offset.
          long offset = (uint64_t) (output.transmit_timestamp / (double)server_frequency * 1e6)- getTimestamp();
          if (offset < offset_between_windows_and_linux ){
            offset_between_windows_and_linux = offset;
          }

          // Correct the packet utime
          // lcm_state_packet.utime = lcm_state_packet.utime - offset_between_windows_and_linux;

          // Fill up LCM message
          // lcm_state_packet.orient[0] = quaternionDroneInNED.w();
          // lcm_state_packet.orient[1] = quaternionDroneInNED.x();
          // lcm_state_packet.orient[2] = quaternionDroneInNED.y();
          // lcm_state_packet.orient[3] = quaternionDroneInNED.z();

          // lcm_state_packet.position[0]    = positionInNED(0);
          // lcm_state_packet.position[1]    = positionInNED(1);
          // lcm_state_packet.position[2]    = positionInNED(2);

          if (last_time == 0) {
            // last_time = lcm_state_packet.utime;
            last_position = Vector3f(positionInNED[0], positionInNED[1], positionInNED[2]);
          }
          else {
            dt = (lcm_state_packet.utime - last_time);

            Vector3f velocityGlobal;
            velocityGlobal << (positionInNED[0] - last_position[0]) /dt * 1e6,
                              (positionInNED[1] - last_position[1]) /dt * 1e6,
                              (positionInNED[2] - last_position[2]) /dt * 1e6;


            //std::cout << "dt: " << dt << " " << lcm_state_packet.utime << " " << last_time << std::endl;
            //std::cout << "velocityGlobal: " << velocityGlobal << std::endl;

            Vector3f VelocityBody;

            VelocityBody = quaternionDroneInNED.toRotationMatrix().inverse() * velocityGlobal;

            // lcm_state_packet.veloPositionBody[0] = VelocityBody(0);
            // lcm_state_packet.veloPositionBody[1] = VelocityBody(1);
            // lcm_state_packet.veloPositionBody[2] = VelocityBody(2);

            last_position = Vector3f(positionInNED[0], positionInNED[1], positionInNED[2]);
            // last_time = lcm_state_packet.utime;
          }

          // lcm.publish("poseMoCap", &lcm_state_packet);
        }
        else {

          // Loop through and find the relevant markers for calibration
          for (auto marker: output.markers_) {
            if (marker.id == mfl) {
              mfl_p << marker.x, marker.y, marker.z;
              found_markers++;
            }
            if (marker.id == mbl) {
              mbl_p << marker.x, marker.y, marker.z;
              found_markers++;
            }
            if (marker.id == mbr) {
              mbr_p << marker.x, marker.y, marker.z;
              found_markers++;
            }
            if (marker.id == mfr) {
              mfr_p << marker.x, marker.y, marker.z;
              found_markers++;
            }
          }

          if (found_markers == 4) {
            std::cout << "MFL" << mfl_p << std::endl;
            Quaternionf q = calculateRotation(mfl_p, mbl_p, mfr_p, mbr_p, quaternionRigidBodyInNED);
            quats_.push_back(q);
            std::cout << "Quaternion: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
          }
          else {
            std::cout << "Could not find all 4 markers " << std::endl;
          }
        }

        if (quats_.size() == nAvgOffset) {
           q_mocap2body = averageQuaternions<float>(quats_.begin(), quats_.end());
           R_mocap2body = q_mocap2body.normalized().toRotationMatrix();

           gotQuatOffset_ = true;
           std::ofstream quatOffsetFile_;
           quatOffsetFile_.open("quatOffset");
           quatOffsetFile_ << q_mocap2body.w() << std::endl
                            << q_mocap2body.x() << std::endl
                            << q_mocap2body.y() << std::endl
                            << q_mocap2body.z() << std::endl;

           //std::cout << R_mocap2body << std::endl;

        }

    }

}
