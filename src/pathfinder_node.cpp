#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

//TODO : connaissance point actuelle AMCL ici
//TODO : 

#define NBPOINTS 4
#define PATHLENGTH 6

class pathfinder {
private:
	ros::NodeHandle n;

	ros::Subscriber sub_scan;
	ros::Subscriber sub_robot_moving;

	// communication with rotation_action
	ros::Publisher pub_rotation_to_do;
	ros::Subscriber sub_rotation_done;

	// communication with translation_action
	ros::Publisher pub_translation_to_do;
	ros::Subscriber sub_translation_done;

	bool cond_rotation;
	//boolean to check if there is a /rotation_to_do
	bool cond_translation;
	//boolean to check if there is a /translation_to_do
	bool gotonextpoint;
	bool new_goal_to_reach;

	float rotation_to_do;
	float rotation_done;
	float translation_to_do;
	float translation_done;

	// to store, process and display laserdata
	//geometry_msgs::Point

	//pour stocker chemin
	// Tableau des points
	geometry_msgs::Point positionPoints[NBPOINTS];
	// Tableau des points à parcourir
	int pathToDo[PATHLENGTH];
	int pointcible;


	geometry_msgs::Point goal_to_reach;
	geometry_msgs::Point goal_reached;

	bool new_rotation_done;
	bool new_translation_done;


public:
	pathfinder() {
		pathToDo[0] = 0;
		pathToDo[1] = 1;
		pathToDo[2] = 0;
		pathToDo[3] = 2;
		pathToDo[4] = 3;
		pathToDo[5] = 0;
		geometry_msgs::Point p1;
		p1.x = 13.2;
		p1.y = -8.6;
		p1.z = 0.4;
		geometry_msgs::Point p2;
		p2.x = 26;
		p2.y = -4.2;
		p2.z = -2.7;
		geometry_msgs::Point p3;
		p3.x = 18.1;
		p3.y = -24;
		p3.z = 1.8;
		geometry_msgs::Point p4;
		p4.x = 9.4;
		p4.y = 7;
		p4.z = -1.3;

		// Tableau des points à parcourir
		positionPoints[0] = p1;
		positionPoints[1] = p2;
		positionPoints[2] = p3;
		positionPoints[3] = p4;
		// 1: 13.2  -8.6  0.4
		// 2: 26  -4.2  -2.7
		// 3: 18.1  -24  1.8
		// 4: 9.4  7  -1.3

		//pointcible 0 = le point de départ
		pointcible = 1;
		gotonextpoint = true;
		new_goal_to_reach=false;
		/*
		for (int i = 0; i < PATHLENGTH; ++i)
			pathToDo[i]=i % NBPOINTS;
		*/


		// communication with rotation_action
		pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);
		sub_rotation_done = n.subscribe("rotation_done", 1, &pathfinder::rotation_doneCallback, this);
		cond_rotation = false;

		// communication with translation_action
		pub_translation_to_do = n.advertise<std_msgs::Float32>("translation_to_do", 0);
		sub_translation_done = n.subscribe("translation_done", 1, &pathfinder::translation_doneCallback, this);
		cond_translation = false;

		new_rotation_done = false;
		new_translation_done = false;

		//BOUCLE POUR PASSER D'UN POINT A L'AUTRE
		ros::Rate r(10);
			// this node will run at 10hz
			//while (pointcible < PATHLENGTH-1) {
		while(ros::ok()){
			ros::spinOnce();
			//each callback is called once to collect new data
			update();
			//processing of data
			r.sleep();
			//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
		}
	}

	//UPDATE: main processing
	/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
	void update() {
		//goal_to_reach = positionPoints[pathToDo[pointcible+1]];
		/*ROS_INFO("(pathfinder_node) /goal_to_reach : (%f, %f)", goal_to_reach.x, goal_to_reach.y);*/


		if (gotonextpoint && pointcible<PATHLENGTH)
		{
			//TODO : ajouter connaissance de la position actuelle via AMCL
			//TODO : calculer positionactuelle puis calculer x,y,z
			goal_to_reach = positionPoints[pathToDo[pointcible]];
			// goal_to_reach.x -= positionPoints[pathToDo[pointcible-1]].x;
			// goal_to_reach.y -= positionPoints[pathToDo[pointcible-1]].y;
			// goal_to_reach.z -= positionPoints[pathToDo[pointcible-1]].z;
			//goal_to_reach = positionPoints[pathToDo[pointcible]] - positionactuelle;
			pointcible++;
			gotonextpoint=false;
			new_goal_to_reach=true;

		}
		else if(pointcible>=PATHLENGTH){
			ROS_INFO("END END END END END END END END END END END END END END END");
			exit(0);
		}

		// we receive a new /goal_to_reach and robair is not doing a translation or a rotation
		if ( ( new_goal_to_reach ) && ( !cond_translation ) && ( !cond_rotation ) ) {

			ROS_INFO("(pathfinder_node) /goal_to_reach received: (%f, %f)", goal_to_reach.x, goal_to_reach.y);

			// we have a rotation and a translation to perform
			// we compute the /translation_to_do
			translation_to_do = sqrt( ( goal_to_reach.x - positionPoints[pathToDo[pointcible-2]].x )*( goal_to_reach.x - positionPoints[pathToDo[pointcible-2]].x )
			 + ( goal_to_reach.y - positionPoints[pathToDo[pointcible-2]].y )*( goal_to_reach.y - positionPoints[pathToDo[pointcible-2]].y ) );
			ROS_INFO("(pathfinder_node) trans calculated : %f",translation_to_do);
			if ( translation_to_do ) {
				cond_translation = true;

				//we compute the /rotation_to_do
				cond_rotation = true;
				rotation_to_do = acos( (goal_to_reach.x-positionPoints[pathToDo[pointcible-2]].x) / translation_to_do );

				if ( goal_to_reach.y < 0 )
					rotation_to_do *=-1;

				//we first perform the /rotation_to_do
				ROS_INFO("(pathfinder_node) /rotation_to_do: %f", rotation_to_do*180/M_PI);
				std_msgs::Float32 msg_rotation_to_do;
				// envoie de l'information
				msg_rotation_to_do.data = rotation_to_do;
				pub_rotation_to_do.publish(msg_rotation_to_do);

			}

		}

		new_goal_to_reach = false;

		//we receive an ack from rotation_action_node. So, we perform the /translation_to_do
		if ( new_rotation_done ) {
			ROS_INFO("(pathfinder_node) /rotation_done : %f", rotation_done*180/M_PI);
			cond_rotation = false;
			new_rotation_done = false;

			//the rotation_to_do is done so we perform the translation_to_do
			ROS_INFO("(pathfinder_node) /translation_to_do: %f", translation_to_do);
			std_msgs::Float32 msg_translation_to_do;
			msg_translation_to_do.data = translation_to_do;
			pub_translation_to_do.publish(msg_translation_to_do);
		}

		//we receive an ack from translation_action_node. So, we send an ack to the moving_persons_detector_node
		if ( new_translation_done ) {
			ROS_INFO("(pathfinder_node) /translation_done : %f\n", translation_done);
			cond_translation = false;
			new_translation_done = false;
			gotonextpoint=true;

			ROS_INFO(" ");
			ROS_INFO("(pathfinder_node) waiting for a /goal_to_reach");
		}
	}


	//CALLBACKS
	/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

	void rotation_doneCallback(const std_msgs::Float32::ConstPtr& a) {
		// process the angle received from the rotation node
		new_rotation_done = true;
		rotation_done = a->data;
	}

	void translation_doneCallback(const std_msgs::Float32::ConstPtr& r) {
		// process the range received from the translation node
		new_translation_done = true;
		translation_done = r->data;
	}

	float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
		return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
	}

};


int main(int argc, char **argv){

	ROS_INFO("(pathfinder_node) waiting for /odometry and /map_server");
	ros::init(argc, argv, "pathfinder");
	pathfinder bsObject;
	ros::spin();
	return 0;

}
