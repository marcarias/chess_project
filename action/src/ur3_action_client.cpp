#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <action/gotoJoints.h>
#include <action/ik.h>
#include <ur3ik/UR3IK.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
// This is a general
// This is a general server that wraps the call to the FollowJointTrajectory actions
// All the services offered are general, except the one that sets a rectilinear trajectory in the Cartsian spac,
// that is particularized for the UR3 robot.

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

control_msgs::FollowJointTrajectoryGoal goal;
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *robotClient;

tf2_ros::Buffer tfBuffer;
uint numjoints = 6;
std::vector<std::string> jointnames;
std::vector<double> currentjoints;

bool updatedstates = false;
// double calibration_x = -0.015;
// double calibration_y = -0.005;
double calibration_x = 0;
double calibration_y = 0;
int substeps = 1;

//Callback function: Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: error_code is %d", result->error_code);
  //error_string is not showing the message...
  //see http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html for the meaning of the error codes.
  //ROS_INFO_STREAM("Answer: error_string is "<< result->error_string);
}

//Callback function: Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

//Callback function: Called every time feedback is received for the goal
void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
{
  std::stringstream feedback_message_names;
  feedback_message_names << "Got Feedback of joints (";
  for(int i=0;i<numjoints-1;i++)
      feedback_message_names << feedback->joint_names[i] << ",";
  feedback_message_names << feedback->joint_names[numjoints-1] << ")";
  ROS_INFO_STREAM(feedback_message_names.str());

  std::stringstream feedback_message_pos;
  feedback_message_pos << "              current positions are (";
  for(int i=0;i<numjoints-1;i++)
      feedback_message_pos << feedback->actual.positions[i] << ",";
  feedback_message_pos << feedback->actual.positions[numjoints-1] << ")";
  ROS_INFO_STREAM(feedback_message_pos.str());

  std::stringstream feedback_message_vel;
  feedback_message_vel << "              current velocities are (";
  for(int i=0;i<numjoints-1;i++)
      feedback_message_vel << feedback->actual.velocities[i] << ",";
  feedback_message_vel << feedback->actual.velocities[numjoints-1] << ")";
  ROS_INFO_STREAM(feedback_message_vel.str());
}


//Function to send the goal to the FollowJointTrajectory action server.
//Waits for the result for trajduration seconds.
//If not able to reach the goal within timeout, it is cancelled
bool moveRobotTrajectory(double trajduration)
{
    ROS_INFO("Moving robot in %f", trajduration);

    //Print the trajectory to be followed
    ROS_INFO("currentjoints: (%f,%f,%f,%f,%f,%f)",
                  currentjoints[0],
                  currentjoints[1],
                  currentjoints[2],
                  currentjoints[3],
                  currentjoints[4],
                  currentjoints[5]);
    for(int i=0; i < goal.trajectory.points.size(); i++){
         ROS_INFO("%d: (%f,%f,%f,%f,%f,%f)",i,
                  goal.trajectory.points[i].positions[0],
                  goal.trajectory.points[i].positions[1],
                  goal.trajectory.points[i].positions[2],
                  goal.trajectory.points[i].positions[3],
                  goal.trajectory.points[i].positions[4],
                  goal.trajectory.points[i].positions[5]);
    }


    //Set timestamp and send goal
    goal.trajectory.header.stamp = ros::Time::now() ;//+ ros::Duration(1.0); //To cHECK the +1
    robotClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    //Wait for the action to return. Timeout set to the req.trajduration plus the goal time tolerance)
    bool finished_before_timeout = robotClient->waitForResult(ros::Duration(trajduration) + goal.goal_time_tolerance);

    //Get final state
    actionlib::SimpleClientGoalState state = robotClient->getState();
    if (finished_before_timeout) {
        //Reports ABORTED if finished but goal not reached. Cause shown in error_code in doneCb callback
        //Reports SUCCEEDED if finished and goal reached
        ROS_INFO(" ***************** Robot action finished: %s  *****************",state.toString().c_str());
    } else {
        //Reports ACTIVE if not reached within timeout. Goal must be cancelled if we want to stop the motion, then the state will report PREEMPTED.
        ROS_ERROR("Robot action did not finish before the timeout: %s",
                state.toString().c_str());
        //Preempting task
        ROS_ERROR("I am going to preempt the task...");
        robotClient->cancelGoal();
    }
    //force to reset the traj before calling again move
    return finished_before_timeout;
}


bool findIK(std::vector<std::vector<double>> &iksolution, geometry_msgs::Pose ur3pose)
{
    ros::NodeHandle node;
    ros::service::waitForService("/UR3IK");
    ros::ServiceClient ur3ik_client = node.serviceClient<ur3ik::UR3IK>("/UR3IK");
    ur3ik::UR3IK ur3ik_srv;

    ur3ik_srv.request.theta_ref.resize(6);
    ur3ik_srv.request.theta_min.resize(6);
    ur3ik_srv.request.theta_max.resize(6);
    for(int i=0; i<6; i++) {
        ur3ik_srv.request.theta_ref[i] = 0.0;
        ur3ik_srv.request.theta_min[i] = -M_PI;
        ur3ik_srv.request.theta_max[i] = M_PI;
    }
    ur3ik_srv.request.pose = ur3pose;


    ROS_INFO_STREAM("Robot Pose: [" <<
        ur3ik_srv.request.pose.position.x << ", " <<
        ur3ik_srv.request.pose.position.y << ", " <<
        ur3ik_srv.request.pose.position.z << ", " <<
        ur3ik_srv.request.pose.orientation.x << ", " <<
        ur3ik_srv.request.pose.orientation.y << ", " <<
        ur3ik_srv.request.pose.orientation.z << ", " <<
        ur3ik_srv.request.pose.orientation.w << "]");

    ur3ik_client.call(ur3ik_srv);

    std::stringstream sstr;
    if(ur3ik_srv.response.status)
    {
        iksolution.resize(ur3ik_srv.response.ik_solution.size());
        sstr<<"The computed ik is:"<<std::endl;
        for(int i=0; i<ur3ik_srv.response.ik_solution.size(); i++)
        {
            for(int j=0; j<6; j++)
                iksolution[i].push_back(ur3ik_srv.response.ik_solution[i].ik[j]);

            sstr << "[";
            for(int j=0; j<5; j++)
            {
                sstr << ur3ik_srv.response.ik_solution[i].ik[j] <<", ";
            }
            sstr << ur3ik_srv.response.ik_solution[i].ik[5] << "]" << std::endl;
        }
        ROS_INFO_STREAM(sstr.str());
        return true;
    }
    else{
        ROS_INFO("Not able to compute the ik");
        return false;
    }
}

//Function to set a rectilinear trajectory
// deltaconf is used to define the goal: goal=currentjoints+deltaconf
// epsilon is the step size in joint space
// timefromstart defines the total trajectory time
bool setRectilinearTrajectory(geometry_msgs::Pose pose, double timefromstart)
{
        ROS_INFO("Setting new rectilinear trajectory ");
        geometry_msgs::TransformStamped act_pose;
        geometry_msgs::Pose des_pose;

        double delta_a;
        double delta_b;
        double delta_c;
        double delta_d;
        double delta_e;
        double delta_f;
        double delta_g;
        double min_dist;
        double closest_i;
        double dist;
        //set joint names
        goal.trajectory.joint_names.resize(numjoints);
        goal.trajectory.joint_names[0] = "team_A_shoulder_pan_joint";
        goal.trajectory.joint_names[1] = "team_A_shoulder_lift_joint";
        goal.trajectory.joint_names[2] = "team_A_elbow_joint";
        goal.trajectory.joint_names[3] = "team_A_wrist_1_joint";
        goal.trajectory.joint_names[4] = "team_A_wrist_2_joint";
        goal.trajectory.joint_names[5] = "team_A_wrist_3_joint";
        goal.trajectory.points.resize( substeps );

        for(int i=0; i<3; i++){
            try{
              act_pose =  tfBuffer.lookupTransform("team_A_base_link", "team_A_tool0", ros::Time(0));//, ros::Duration(0.1));
            }
            catch (tf2::TransformException ex ){
              ROS_ERROR("%d - %s",i,ex.what());
              //exception due to not yet available tf. Publish joint_states and retry.
              continue;
            }
          }
          delta_a = (pose.position.x - act_pose.transform.translation.x )/substeps;
          delta_b = (pose.position.y - act_pose.transform.translation.y )/substeps;
          delta_c = (pose.position.z - act_pose.transform.translation.z )/substeps;
          delta_d = (pose.orientation.x - act_pose.transform.rotation.x )/substeps;
          delta_e = (pose.orientation.y - act_pose.transform.rotation.y )/substeps;
          delta_f = (pose.orientation.z - act_pose.transform.rotation.z )/substeps;
          delta_g = (pose.orientation.w - act_pose.transform.rotation.w )/substeps;
          for (int k=0; k<substeps; k++){
            des_pose.position.x = act_pose.transform.translation.x + delta_a*(k+1);
            des_pose.position.y = act_pose.transform.translation.y + delta_b*(k+1);
            des_pose.position.z = act_pose.transform.translation.z + delta_c*(k+1);
            des_pose.orientation.x = act_pose.transform.rotation.x + delta_d*(k+1);
            des_pose.orientation.y = act_pose.transform.rotation.y + delta_e*(k+1);
            des_pose.orientation.z = act_pose.transform.rotation.z + delta_f*(k+1);
            des_pose.orientation.w = act_pose.transform.rotation.w + delta_g*(k+1);
            std::vector<std::vector<double>> iksolution;
            if(findIK(iksolution, des_pose))
            {
              min_dist = 1000000;
              closest_i = 0;
                for(int i=0; i < iksolution.size(); i++){
                  dist = 0;
                  for (int j=0; j<6; j++){
                    double diff;
                    diff = iksolution[i][j]-currentjoints[j];
                    while (diff < -M_PI) diff += 2*M_PI;
                    while (diff > M_PI) diff -= 2*M_PI;
                    dist = dist + pow(diff,2);
                  }
                  dist = sqrt(dist);
                  if (dist < min_dist)
                  {
                    min_dist = dist;
                    closest_i = i;
                  }
                }
                goal.trajectory.points[k].positions.resize( numjoints );
                goal.trajectory.points[k].velocities.resize( numjoints );
                goal.trajectory.points[k].accelerations.resize( numjoints );
                for(int j=0; j < numjoints; j++){
                    goal.trajectory.points[k].positions[j] = iksolution[closest_i][j];
                    currentjoints[j] =  iksolution[closest_i][j];
                }
                goal.trajectory.points[k].time_from_start = ros::Duration(timefromstart/substeps*(k+1));
            }
            else
            {
               //return false;
            }

          }
          for(int j=0; j < numjoints; j++){
            goal.trajectory.points[0].velocities[j] = 0.0;
            goal.trajectory.points[0].accelerations[j] = 0.0;
            goal.trajectory.points[substeps-1].velocities[j] = 0.0;
            goal.trajectory.points[substeps-1].accelerations[j] = 0.0;
          }
return true;

}


// A callback function to update the arm joint values - gripper joint is skipped
void updateCurrentJointStates(const sensor_msgs::JointState& msg) {
  for(int i=0; i<msg.name.size();i++)
  {
    //BE CAREFUL: order of joints is set according to the controller
    if(msg.name[i] == "team_A_shoulder_pan_joint") currentjoints[0]= msg.position[i];
    else if(msg.name[i] == "team_A_shoulder_lift_joint") currentjoints[1]= msg.position[i];
    else if(msg.name[i] == "team_A_elbow_joint") currentjoints[2]= msg.position[i];
    else if(msg.name[i] == "team_A_wrist_1_joint") currentjoints[3]= msg.position[i];
    else if(msg.name[i] == "team_A_wrist_2_joint") currentjoints[4]= msg.position[i];
    else if(msg.name[i] == "team_A_wrist_3_joint") currentjoints[5]= msg.position[i];

    /*
    std::stringstream ss;
    for(int j=0; j<6;j++)
      ss<<currentjoints[j]<<" ";
    ROS_INFO_STREAM(ss.str());
    */
  }
  updatedstates = true;
}



//! Function that wraps the call to the ur ik service


bool moveUR3(
       action::gotoJoints::Request &req,
       action::gotoJoints::Response &resp){
         ros::spinOnce();
         std::vector<std::vector<double>> iksolution;
         geometry_msgs::Pose pose;
         pose.position.x = req.px+calibration_x;
         pose.position.y = req.py+calibration_y;
         pose.position.z = req.pz;
         pose.orientation.x = req.qx;
         pose.orientation.y = req.qy;
         pose.orientation.z = req.qz;
         pose.orientation.w = req.qw;
         bool success = setRectilinearTrajectory(pose, req.trajduration);
         if (success){
           moveRobotTrajectory(req.trajduration);
           return true;
         }
         return false;
}

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "ur3_action_client");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tfListener(tfBuffer);
  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("/team_A_arm/joint_states", 1000, &updateCurrentJointStates);

  // Create a service
  ros::ServiceServer server = nh.advertiseService("move_ur3",&moveUR3);

  // Initialization
  currentjoints.resize(numjoints);
  robotClient = new Client("/team_A_arm/joint_trajectory_controller/follow_joint_trajectory");
  if(!robotClient->waitForServer(ros::Duration(5.0)))
  {
      ROS_ERROR(" *** action server not available *** ");
      exit(0);
  };

  // Wait for user to press a key
  std::cout<<"\nMOVE THE ROBOT TO A SAVE HOME CONFIGURATION AND PRESS A KEY TO START..."<<std::endl;
  std::cin.get();


  ros::spin();

  return 0;
}
