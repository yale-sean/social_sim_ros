#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
#include <string>

geometry_msgs::PoseArray poses;


void poseCallback(const geometry_msgs::PoseArray &msg)
{
	poses = msg;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_to_people");
	ros::NodeHandle n;

	ros::Subscriber pose_sub = n.subscribe("/social_sim/agent_positions", 10, poseCallback);
	ros::Publisher people_pub = n.advertise<people_msgs::People>("/people", 10);

	ros::Rate r(10);

	std::vector<geometry_msgs::Point> last_poses;
	ros::Time last_time;
	bool first_msg = true;
	std::vector<people_msgs::Person> people_vector;

	while(ros::ok())
	{
		int number_of_people = poses.poses.size();
		ros::Time current_time = ros::Time::now();
		for(int i = 0; i < number_of_people; i++)
		{
			people_msgs::Person person;
			person.name = "agent_" + std::to_string(i);
			person.position = poses.poses[i].position;
			if(first_msg)
			{
				geometry_msgs::Point zero_velocity;
				zero_velocity.x = 0;
				zero_velocity.y = 0;
				zero_velocity.z = 0;
				person.velocity = zero_velocity;
				last_poses.push_back(poses.poses[i].position);
			}
			else
			{
				person.velocity.x = (poses.poses[i].position.x - last_poses[i].x) / (current_time.toSec() - last_time.toSec());
				person.velocity.y = (poses.poses[i].position.y - last_poses[i].y) / (current_time.toSec() - last_time.toSec());
				person.velocity.z = (poses.poses[i].position.z - last_poses[i].z) / (current_time.toSec() - last_time.toSec());
				last_poses[i] = poses.poses[i].position;
			}
			// note: reliability, tags, taglines don't seem to do anything
			
			people_vector.push_back(person);
		}
		if(number_of_people > 0)
		{
			first_msg = false;
		}
		people_msgs::People people_msg;
		people_msg.header.stamp = current_time; 
		people_msg.header.frame_id = "map";
		people_msg.people = people_vector; 
		people_vector.clear();

		people_pub.publish(people_msg);
		ros::spinOnce();
		last_time = current_time;

		r.sleep();
	}

	return 0;
}

