#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>


#define NBPOINTS 10
#define PATHLENGTH 10;

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

	    bool cond_rotation;//boolean to check if there is a /rotation_to_do
	    bool cond_translation;//boolean to check if there is a /translation_to_do

	    float rotation_to_do;
	    float rotation_done;
	    float translation_to_do;
	    float translation_done;

	    // to store, process and display laserdata
	    //geometry_msgs::Point

	    //pour stocker chemin
	    geometry_msgs::Point points[NBPOINTS];
	    int positionPoints[PATHLENGTH]; // Tableau des points
	    int pathToDo[PATHLENGTH]; // Tableau des points à parcourir
	    int currentpoint;

	    geometry_msgs::Point goal_to_reach;
    	geometry_msgs::Point goal_reached;


	public:

	pathfinder() {
	    currentpoint = 0;
	    for (int i = 0; i < PATHLENGTH; ++i)
	    {
	        pathToDo[i]=i % NBPOINTS;
	    }

	    // communication with rotation_action
	    pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);
	    sub_rotation_done = n.subscribe("rotation_done", 1, &pathfinder::rotation_doneCallback, this);
	    cond_rotation = false;

	    // communication with translation_action
	    pub_translation_to_do = n.advertise<std_msgs::Float32>("translation_to_do", 0);
	    sub_translation_done = n.subscribe("translation_done", 1, &pathfinder::translation_doneCallback, this);
	    cond_translation = false;

	    //BOUCLE POUR PASSER D'UN POINT A L'AUTRE
	    ros::Rate r(10);// this node will run at 10hz
	    while (currentpoint < PATHLENGTH-1) {
	        ros::spinOnce();//each callback is called once to collect new data
	        update();//processing of data
	        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
	    }
	}

	//UPDATE: main processing
	/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
	void update() {
	    while( currentpoint < PATHLENGTH ){
		    goal_to_reach = positionPoints[pathToDo[currentpoint+1]];
		    ROS_INFO("(pathfinder_node) /goal_to_reach : (%f, %f)", goal_to_reach.x, goal_to_reach.y);

		    // Calcul translation
		    translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

		    // Calcul rotation
		    rotation_to_do = acos( goal_to_reach.x / translation_to_do );


		    // Envoie l'info aux noeuds de rotation et de translation

		    ROS_INFO("(pathfinder_node) /rotation_to_do: %f", rotation_to_do*180/M_PI);
            std_msgs::Float32 msg_rotation_to_do;
            // envoie de l'information
            msg_rotation_to_do.data = rotation_to_do;
            pub_rotation_to_do.publish(msg_rotation_to_do);

		    ROS_INFO("(pathfinder_node) /translation_to_do: %f", translation_to_do);
            std_msgs::Float32 msg_translation_to_do;
            msg_translation_to_do.data = translation_to_do;
            pub_translation_to_do.publish(msg_translation_to_do);

		    currentpoint++;
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

	// Distance between two points
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
