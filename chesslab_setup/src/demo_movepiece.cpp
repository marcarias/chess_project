#include <iostream>
#include <ros/ros.h>
//#include <ros/package.h>
//#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <geometry_msgs/Pose.h>
//#include <sensor_msgs/JointState.h>

//#include <chesslab_setup/setrobconf.h>
#include <chesslab_setup/setobjpose.h>
//#include <chesslab_setup/attachobs2robot.h>
//#include <chesslab_setup/dettachobs.h>
//#include <chesslab_setup/ffplan.h>
//#include <chesslab_setup/ik.h>
#include <std_srvs/Empty.h>



int main( int argc, char** argv )
{
    ros::init(argc, argv, "demo_movepiece");
    ros::NodeHandle node;

    ROS_INFO("**** starting chesslab_setup client node to test the motion of a chess piece ****");

    //Set the initial scene
    if(argc!=1 && strcmp(argv[1], "initialize") == 0){
        std::cout<<"Initializing SCENE"<<std::endl;
        ros::service::waitForService("/chesslab_setup/resetscene");
        ros::ServiceClient resetscene_client = node.serviceClient<std_srvs::Empty>("/chesslab_setup/resetscene");
        std_srvs::Empty resetscene_srv;
        resetscene_client.call(resetscene_srv);
    }
    else{
        std::cout<<"Skipping Initialization"<<std::endl;
    }

    ////////////////////////////////////////////////////////
    //Wait for user to press a key
    std::cout<<"\nPRESS A KEY TO START..."<<std::endl;
    std::cin.get();

    //Move piece
    ros::service::waitForService("/chesslab_setup/setobjpose");
    ros::ServiceClient setobjpose_client = node.serviceClient<chesslab_setup::setobjpose>("/chesslab_setup/setobjpose");
    chesslab_setup::setobjpose setobjpose_srv;

    setobjpose_srv.request.objid = 201;
    setobjpose_srv.request.p.position.x = 0.075;
    setobjpose_srv.request.p.position.y = 0.175;
    setobjpose_srv.request.p.position.z = 0.02;
    setobjpose_srv.request.p.orientation.x = 0.0;
    setobjpose_srv.request.p.orientation.y = 0.0;
    setobjpose_srv.request.p.orientation.z = 0.707107;
    setobjpose_srv.request.p.orientation.w = -0.707107;

    std::cout << "Moving piece " << setobjpose_srv.request.objid <<" to: [" <<
        setobjpose_srv.request.p.position.x << ", " <<
        setobjpose_srv.request.p.position.y << ", " <<
        setobjpose_srv.request.p.position.z << ", " <<
        setobjpose_srv.request.p.orientation.x << ", " <<
        setobjpose_srv.request.p.orientation.y << ", " <<
        setobjpose_srv.request.p.orientation.z << ", " <<
        setobjpose_srv.request.p.orientation.w << "]" << std::endl;

    setobjpose_client.call(setobjpose_srv);
}
