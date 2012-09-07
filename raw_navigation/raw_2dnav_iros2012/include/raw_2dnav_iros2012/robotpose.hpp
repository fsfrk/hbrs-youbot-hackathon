
class RobotPose
{
private :
	ros::NodeHandle node;
	tf::TransformListener listener;
	geometry_msgs::Pose pose;



public:
	RobotPose(){};
	geometry_msgs::Pose getRobotPose(double &);
};

