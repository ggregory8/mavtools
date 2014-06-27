#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mavlink_send/Mavlink.h"    // This is the definition for the format of the ROS message to the mavlink_ros node

#include "mavlink/v1.0/pixhawk/mavlink.h"  // I couldn't find out how to indude the above so moved the whole mavlink folder to this local package include

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
  
  
/*
void request_all_msgs(void)
{
  mavlink_request_data_stream_t rqstDataStream = new mavlink_request_data_stream_t
  {
    req_message_rate = rate,
    req_stream_id = id,
    start_stop = 1,
    target_system = 1,
    target_component = 1,
  };
*/

/*
  public void requestDatastream(byte id, byte rate)
        {
            mavlink_request_data_stream_t rqstDataStream = new mavlink_request_data_stream_t
            {
                req_message_rate = rate,
                req_stream_id = id,
                start_stop = 1,
                target_system = 1,
                target_component = 0,
            };

            byte[] packet = GeneratePacket(rqstDataStream, MAVLINK_MSG_ID_REQUEST_DATA_STREAM);

            SendPacket(packet);
        }

       public byte[] GeneratePacket(byte[] packetData, byte msgID)
        {
            byte[] Packet = new byte[packetData.Length + 6 + 2];

            Packet[0] = (byte)254;
            Packet[1] = (byte)packetData.Length;
            Packet[2] = (byte)packetid;
            Packet[3] = (byte)255;
            Packet[4] = (byte)1;
            Packet[5] = msgID;

            for (int n = 0; n < packetData.Length; n++)
                Packet[6 + n] = packetData[n];

            ushort checksum = MavlinkCRC.crc_calculate(Packet, Packet[1] + 6);

            checksum = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_CRCS[msgID], checksum);
            
            byte ck_a = (byte)(checksum & 0xFF); ///< High byte
            byte ck_b = (byte)(checksum >> 8); ///< Low byte

            Packet[Packet.Length - 2] = ck_a;
            Packet[Packet.Length - 1] = ck_b;

            return Packet;
        }
//Overloaded function
        public byte[] GeneratePacket(object packetData, byte msgID)
        {
            int len = System.Runtime.InteropServices.Marshal.SizeOf(packetData);
            byte[] arr = new byte[len];
            IntPtr ptr = System.Runtime.InteropServices.Marshal.AllocHGlobal(len);
            System.Runtime.InteropServices.Marshal.StructureToPtr(packetData, ptr, true);
            System.Runtime.InteropServices.Marshal.Copy(ptr, arr, 0, len);
            System.Runtime.InteropServices.Marshal.FreeHGlobal(ptr);

            return GeneratePacket(arr, msgID);

        }

        public bool SendPacket(byte[] packetData)
        {
            try
            {
                if (MAVlinkPort.IsOpen)
                {
                    MAVlinkPort.Write(packetData, 0, packetData.Length);
                    return true;
                }
            }
            catch
            {
                return false;
            }
            return false;
        }
The serial port is created as:
        public bool Connect(string COMPort, int BaudRate)
        {
            try
            {
                MAVlinkPort = new System.IO.Ports.SerialPort(COMPort, BaudRate);
                MAVlinkPort.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(MAVlinkPort_DataReceived);
                MAVlinkPort.DtrEnable = true;   //Required to communicate with ArduPilot
                MAVlinkPort.RtsEnable = true;
                MAVlinkPort.Open();
                MAVlinkPort.DiscardInBuffer();
                IsConnected = true;
                return true;
            }
            catch
            {
                return false;
            }
        }


-----------
From gcs-copter Java:
- from APM heartbeat get info:
MAVLink.CURRENT_SYSID = m.sysID;
MAVLink.ARDUCOPTER_COMPONENT_ID = m.componentID;
- send a heartbeat presenting yourself:
msg_heartbeat msg = new msg_heartbeat();
msg.type = MAVLink.MAV_TYPE.MAV_TYPE_GCS;
msg.autopilot = MAVLink.MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC;
sendBytesToComm(MAVLink.createMessage(msg));
- and request all packets or only the ones you are interested in:
                msg_request_data_stream req = new msg_request_data_stream();
                req.req_message_rate = 20;
                req.req_stream_id = MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_ALL;
                req.start_stop = 1;
                req.target_system = MAVLink.CURRENT_SYSID;
                req.target_component = 0;
                sendBytesToComm( MAVLink.createMessage(req));

*/
  





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

  display_mavlink_packet(msg);
  display_ros_packet(rosmavlink_msg);

  mavlink_pub.publish(rosmavlink_msg);
  ROS_INFO("ROS Mavlink Message Sent");

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

/*
// From defines.h
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

// From GCS_Mavlink.pde

case MAVLINK_MSG_ID_SET_MODE:       // MAV ID: 11
    {
        // decode
        mavlink_set_mode_t packet;
        mavlink_msg_set_mode_decode(msg, &packet);

        // only accept custom modes because there is no easy mapping from Mavlink flight modes to AC flight modes
        if (packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
            if (set_mode(packet.custom_mode)) {
                result = MAV_RESULT_ACCEPTED;
            }
        }

        // send ACK or NAK
        mavlink_msg_command_ack_send_buf(msg, chan, MAVLINK_MSG_ID_SET_MODE, result);
        break;
    }
    */
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

// Callback for add_two_ints services
/*
bool add(gg_mavlink::AddTwoInts::Request  &req,
         gg_mavlink::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
*/

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
  //ros::Subscriber mavlink_sub = n.subscribe("from", 1000, mavlinkCallback);
  

  //mavlink_pub = mavlink_nh.advertise<mavlink_ros::Mavlink>("from", 1000);

  ros::Rate loop_rate(1);       // in Hz

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
    std_msgs::String msg;
    mavlink_ros::Mavlink rosmavlink_msg;

    string cmd_inputs = "";
    char cmd_inputc = {0};

    //while(true)
    //{
      cout << "Please enter a command char: ";
      getline(cin, cmd_inputs);

      if (cmd_inputs.length() == 1){
        /*
        if (cmd_inputs[0] == 'a'){
          cmd_inputc = cmd_inputs[0];
          quad_arm();
          break;
        }
        if (cmd_inputs[0] == 'h'){
          heart_beat();
          break;
        }
        */
        switch (cmd_inputs[0]) {
          case 'a':
            quad_arm();
            break;
          case 'h':
            heart_beat();
            break;
          case 'm':
            set_mode();
            break;
          default:
            cout << "Invalid char please try again" << endl;
            break;
        }
      
      }else {
        cout << "Enter one char please try again - Add usage help here" << endl;
      }
      
      
    //}
    cout << "You entered: " << cmd_inputc << endl << endl;

    // Not using this because this loop is block. Need to add a thread
    if (hb_enable) {
      heart_beat();
    }


    // mavlink_ros message structure
    // note this is different to the real mavlink message to the UAV

//uint8 len\n\
//uint8 seq\n\
//uint8 sysid\n\
//uint8 compid\n\
//uint8 msgid\n\
//bool fromlcm\n\
//uint64[] payload64\n\
  
    
    //ROS_INFO("%s", msg.data.c_str());

    
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //chatter_pub.publish(msg);
    //mavlink_pub.publish(rosmavlink_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
