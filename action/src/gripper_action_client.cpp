#include <ros/ros.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GClient;

control_msgs::GripperCommandGoal goal;
actionlib::SimpleActionClient<control_msgs::GripperCommandAction> *gripperClient;

int numjoints = 1;
std::string jointname;
double currentjoint;
int signe = -1;
double max_effort = 10; //?

//Callback function: Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const control_msgs::GripperCommandResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  std::stringstream result_message;
  result_message << "Gripper Motion Result: Position = " << result->position << ", Effort = "<< result->effort<<std::endl;
  if(result->stalled) result_message << " - STALLED ";
  if(result->reached_goal) result_message << " - REACHED GOAL ";
  ROS_INFO_STREAM(result_message.str());
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
void feedbackCb(const control_msgs::GripperCommandFeedbackConstPtr& feedback)
{
  std::stringstream feedback_message;
  feedback_message << "Got Feedback of gripper: Position = " << feedback->position << ", Effort = "<< feedback->effort<<std::endl;
  if(feedback->stalled) feedback_message << " - STALLED ";
  if(feedback->reached_goal) feedback_message << " - REACHED GOAL ";
  ROS_INFO_STREAM(feedback_message.str());
}


//Function to send the goal to the GripperCommand action server.
//Waits for the result for trajduration seconds.
//If not able to reach the goal within timeout, it is cancelled
bool moveGripper()
{
    ROS_INFO("Moving gripper ");

    //Print the goal to be achieved
    ROS_INFO("currentjoint: %f", currentjoint);
    ROS_INFO("gripper command position: %f", goal.command.position);
    ROS_INFO("gripper command max_effort: %f", goal.command.max_effort);

    //Send goal
    gripperClient->sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    //Wait for the action to return. Timeout set to the trajduration
    bool finished_before_timeout = gripperClient->waitForResult(); //no finite timeout

    //Get final state
    actionlib::SimpleClientGoalState state = gripperClient->getState();
    if (finished_before_timeout) {
        //Reports ABORTED if finished but goal not reached. Cause shown in error_code in doneCb callback
        //Reports SUCCEEDED if finished and goal reached
        ROS_INFO(" ***************** Gripper action finished: %s  *****************",state.toString().c_str());
    } else {
        //Reports ACTIVE if not reached within timeout. Goal must be cancelled if we want to stop the motion, then the state will report PREEMPTED.
        ROS_ERROR("Gripper action did not finish before the timeout: %s",
                state.toString().c_str());
        //Preempting task
        ROS_ERROR("I am going to preempt the task...");
        gripperClient->cancelGoal();
    }
    return finished_before_timeout;
}

//Function to set a rectilinear trajectory
// deltaconf is used to define the goal: goal=currentjoints+deltaconf
// epsilon is the step size in joint space
// timefromstart defines the total trajectory time
void setAperture(double position, double max_effort)
{
        ROS_INFO("Setting gripper apperture %f",position);
        goal.command.position = position;
        goal.command.max_effort = max_effort;
}


// A callback function to update the arm joint values - gripper joint is skipped
void updateCurrentJointStates(const sensor_msgs::JointState& msg) {
  for(int i=0; i<msg.name.size();i++)
  {
    if(msg.name[i] == "team_A_gripper_right_driver_joint") currentjoint = msg.position[i];
  }
  //std::cout<<currentjoint<<std::endl;
}

bool gripperClose(
       std_srvs::Empty::Request &req,
       std_srvs::Empty::Response &resp){

         ros::spinOnce();
           ROS_INFO("closing gripper");
           setAperture(0.53, max_effort);
         moveGripper();
       return true;
}

bool gripperOpen(
       std_srvs::Empty::Request &req,
       std_srvs::Empty::Response &resp){

         ROS_INFO("opening gripper");
         setAperture(0.5, max_effort);
         moveGripper();
       return true;
}

int main(int argc, char **argv)
{
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "gripper_action_client");
  ros::NodeHandle nh;

  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("team_A_arm/joint_states", 1000, &updateCurrentJointStates);

  // Create a rosservice client
  ros::ServiceClient reactivateClient = nh.serviceClient<std_srvs::Trigger>("reactivate");

  // Create a rosservice action
  ros::ServiceServer server = nh.advertiseService("team_A_arm/gripper_open",&gripperOpen);
  ros::ServiceServer server2 = nh.advertiseService("team_A_arm/gripper_close",&gripperClose);
  // Initialization
  gripperClient = new GClient("team_A_arm/gripper/gripper_controller/gripper_cmd");

  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response resp;

  ros::service::waitForService("reactivate", ros::Duration(10));
  reactivateClient.call(req,resp);
  std::cout << "The gripper has been reactivated" << std::endl;

  ros::spin();
  return resp.success;
}
