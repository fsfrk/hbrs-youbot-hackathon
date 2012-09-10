
class RobotController
{
private :
	ros::NodeHandle node;
	tf::TransformListener listener;
	geometry_msgs::Pose pose;



public:
	RobotController(){};
	geometry_msgs::Pose getRobotPose(double &);
	bool isGoalAchievable();
	void makePlan();
	void executePlan();
	void forceRecovery();
	void isGoalReached();
};

