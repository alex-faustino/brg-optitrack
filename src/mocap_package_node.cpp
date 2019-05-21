#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

// NatNet Includes
#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>

#include <boost/program_options.hpp>
#include <time.h>

// ROS Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include <sstream>

class Globals
{
public:
   
   // Parameters read from the command line
   static uint32_t localAddress;
   static uint32_t serverAddress;
   
   // State of the main() thread.
   static bool run;
};
uint32_t Globals::localAddress = 0;
uint32_t Globals::serverAddress = 0;
bool Globals::run = false;

// End the program gracefully.
void terminate(int)
{
   // Tell the main() thread to close.
   Globals::run = false;
}

// Set the global addresses from the command line.
void readOpts( int argc, char* argv[] )
{
   namespace po = boost::program_options;
   
   po::options_description desc("simple-example: demonstrates using NatNetLinux\nOptions");
   desc.add_options()
      ("help", "Display help message")
      ("local-addr,l", po::value<std::string>(), "Local IPv4 address")
      ("server-addr,s", po::value<std::string>(), "Server IPv4 address")
   ;
   
   po::variables_map vm;
   po::store(po::parse_command_line(argc,argv,desc), vm);
   
   if(
      argc < 5 || vm.count("help") ||
      !vm.count("local-addr") ||
      !vm.count("server-addr")
   )
   {
      std::cout << desc << std::endl;
      exit(1);
   }
   
   Globals::localAddress = inet_addr( vm["local-addr"].as<std::string>().c_str() );
   Globals::serverAddress = inet_addr( vm["server-addr"].as<std::string>().c_str() );
}

// This thread loop just prints frames as they arrive.
void printFrames(FrameListener& frameListener)
{
    // NodeHandle is the main access point to communications with the ROS system.
    // The first NodeHandle constructed will fully initialize this node, and the last
    // NodeHandle destructed will close down the node.
    ros::NodeHandle n;

    // The advertise() function is how you tell ROS that you want to
    // publish on a given topic name. This invokes a call to the ROS
    // master node, which keeps a registry of who is publishing and who
    // is subscribing. After this advertise() call is made, the master
    // node will notify anyone who is trying to subscribe to this topic name,
    // and they will in turn negotiate a peer-to-peer connection with this
    // node.  advertise() returns a Publisher object which allows you to
    // publish messages on that topic through a call to publish().  Once
    // all copies of the returned Publisher object are destroyed, the topic
    // will be automatically unadvertised.
    //
    // The second parameter to advertise() is the size of the message queue
    // used for publishing messages.  If messages are published more quickly
    // than we can send them, the number here specifies how many messages to
    // buffer up before throwing some away.
    ros::Publisher mocap_pub = n.advertise<geometry_msgs::Pose>("mocap", 1000);

    ros::Rate loop_rate(10);

    bool valid;
    MocapFrame frame;
    int hour,min,sec,fframe,subframe;
    timeval tim;
    double timeStamp;
    Globals::run = true;
    int count = 0;
    FILE *myfileMoCap;
    myfileMoCap = fopen("matrice-verification.txt","w");
    while(Globals::run)
    {
        while( true )
        {

            // This is a message object. You stuff it with data, and then publish it.
            geometry_msgs::Pose msg;

            // Try to get a new frame from the listener.
            MocapFrame frame(frameListener.pop(&valid).first);
            // Quit if the listener has no more frames.
            if( !valid )
                break;
            //std::cout << frame << std::endl;
	        std::vector<RigidBody> const& rBodies = frame.rigidBodies();
	
	        ROS_INFO("Frame Num: %d\n",frame.frameNum());
	        ROS_INFO("Rigid Body Position: %f\t%f\t%f\n", rBodies[0].location().x,rBodies[0].location().y,rBodies[0].location().z);
	        ROS_INFO("Rigid Body Orientation: %f\t%f\t%f\t%f\n", rBodies[0].orientation().qx,rBodies[0].orientation().qy,rBodies[0].orientation().qz,rBodies[0].orientation().qw);
	        gettimeofday(&tim,NULL);
	        timeStamp = tim.tv_sec+(tim.tv_usec/1000000.0);	
	        ROS_INFO("Time Packet Received: %f\n",timeStamp);
	        fprintf(myfileMoCap,"%f\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t\n",timeStamp,frame.frameNum(),rBodies[0].location().x,rBodies[0].location().y,rBodies[0].location().z,rBodies[0].orientation().qx,rBodies[0].orientation().qy,rBodies[0].orientation().qz,rBodies[0].orientation().qw);

            // Assign MoCap Message Data
            msg.position.x = rBodies[0].location().x;
            msg.position.y = rBodies[0].location().y;
            msg.position.z = rBodies[0].location().z;
            msg.orientation.x = rBodies[0].orientation().qx;
            msg.orientation.y = rBodies[0].orientation().qy;
            msg.orientation.z = rBodies[0].orientation().qz;
            msg.orientation.w = rBodies[0].orientation().qw;

            // Publish MoCap Pose messages
            mocap_pub.publish(msg);
            
            ros::spinOnce();

            loop_rate.sleep();
        }
		
        // Sleep for a little while to simulate work :)
        usleep(1000);
   }
   fclose(myfileMoCap);
}

int main(int argc, char **argv)
{
    // The ros::init() function needs to see argc and argv so that it can perform
    // any ROS arguments and name remapping that were provided at the command line.
    // For programmatic remappings you can use a different version of init() which takes
    // remappings directly, but for most command-line programs, passing argc and argv is
    // the easiest way to do it.  The third argument to init() is the name of the node.
    //
    // You must call one of the versions of ros::init() before using any other
    // part of the ROS system.
    ros::init(argc, argv, "talker");

    // Version number of the NatNet protocol, as reported by the server.
    unsigned char natNetMajor;
    unsigned char natNetMinor;
   
    // Sockets
    int sdCommand;
    int sdData;
   
    // Catch ctrl-c and terminate gracefully.
    signal(SIGINT, terminate);

    // Set addresses
    readOpts( argc, argv );
    // Use this socket address to send commands to the server.
    struct sockaddr_in serverCommands = NatNet::createAddress(Globals::serverAddress, NatNet::commandPort);

    // Create sockets
    sdCommand = NatNet::createCommandSocket( Globals::localAddress );
    sdData = NatNet::createDataSocket( Globals::localAddress );

    // Start the CommandListener in a new thread.
    CommandListener commandListener(sdCommand);
    commandListener.start();

    // Send a ping packet to the server so that it sends us the NatNet version
    // in its response to commandListener.
    NatNetPacket ping = NatNetPacket::pingPacket();
    ping.send(sdCommand, serverCommands);

    // Wait here for ping response to give us the NatNet version.
    commandListener.getNatNetVersion(natNetMajor, natNetMinor);

    // Start up a FrameListener in a new thread.
    FrameListener frameListener(sdData, natNetMajor, natNetMinor);
    frameListener.start();

    // This infinite loop simulates a "worker" thread that reads the frame
    // buffer each time through, and exits when ctrl-c is pressed.
    printFrames(frameListener);

    // Wait for threads to finish.
    frameListener.stop();
    commandListener.stop();
    frameListener.join();
    commandListener.join();
   
    // Epilogue
    close(sdData);
    close(sdCommand);

  return 0;

}
