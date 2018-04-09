#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>


#define NBPOINTS 10
#define PATHLENGTH 10

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
	    bool gotonextpoint;

	    float rotation_to_do;
	    float rotation_done;
	    float translation_to_do;
	    float translation_done;

	    // to store, process and display laserdata
	    //geometry_msgs::Point

	    //pour stocker chemin
	    geometry_msgs::Point positionPoints[NBPOINTS]; // Tableau des points
	    int pathToDo[PATHLENGTH]; // Tableau des points à parcourir
	    int currentpoint;


	    geometry_msgs::Point goal_to_reach;
	    geometry_msgs::Point goal_reached;

	    bool new_rotation_done;
	    bool new_translation_done;


	public:
		pathfinder() {
			pathToDo[0] = 1;
			pathToDo[1] = 2;
			pathToDo[2] = 1;
			pathToDo[3] = 3;
			pathToDo[4] = 4;
            pathToDo[5] = 1; // Tableau des points à parcourir
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

            positionPoints[0] = p1;
            positionPoints[1] = p2;
            positionPoints[2] = p3;
            positionPoints[3] = p4;
            // 1: 13.2  -8.6  0.4
            // 2: 26  -4.2  -2.7
            // 3: 18.1  -24  1.8
            // 4: 9.4  7  -1.3

            currentpoint = 0;
            gotonextpoint=true;
            /*
            for (int i = 0; i < PATHLENGTH; ++i)
            {
                pathToDo[i]=i % NBPOINTS;
            }
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
            ros::Rate r(10);// this node will run at 10hz
            //while (currentpoint < PATHLENGTH-1) {
            while(ros::ok()){
                ros::spinOnce();//each callback is called once to collect new data
                update();//processing of data
                r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
            }
        }

        //UPDATE: main processing
        /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
        void update() {
        	// goal_to_reach = positionPoints[pathToDo[currentpoint+1]];
        	// ROS_INFO("(pathfinder_node) /goal_to_reach : (%f, %f)", goal_to_reach.x, goal_to_reach.y);

         //    // Calcul translation
        	// translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

         //    // Calcul rotation
        	// rotation_to_do = acos( goal_to_reach.x / translation_to_do );


         //    // Envoie l'info aux noeuds de rotation et de translation

        	// ROS_INFO("(pathfinder_node) /rotation_to_do: %f", rotation_to_do*180/M_PI);
        	// std_msgs::Float32 msg_rotation_to_do;
         //    // envoie de l'information
        	// msg_rotation_to_do.data = rotation_to_do;
        	// pub_rotation_to_do.publish(msg_rotation_to_do);

        	// ROS_INFO("(pathfinder_node) /translation_to_do: %f", translation_to_do);
        	// std_msgs::Float32 msg_translation_to_do;
        	// msg_translation_to_do.data = translation_to_do;
        	// pub_translation_to_do.publish(msg_translation_to_do);

        	// currentpoint++;
        	if (gotonextpoint)
        	{
        		goal_to_reach = positionPoints[pathToDo[currentpoint+1]];
        	}

            //*************************************
           	// we receive a new /goal_to_reach and robair is not doing a translation or a rotation
        	if ( ( new_goal_to_reach ) && ( !cond_translation ) && ( !cond_rotation ) ) {

        		ROS_INFO("(decision_node) /goal_to_reach received: (%f, %f)", goal_to_reach.x, goal_to_reach.y);

		        // we have a rotation and a translation to perform
		        // we compute the /translation_to_do
        		translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

        		if ( translation_to_do ) {
        			cond_translation = true;

		            //we compute the /rotation_to_do
        			cond_rotation = true;
        			rotation_to_do = acos( goal_to_reach.x / translation_to_do );

        			if ( goal_to_reach.y < 0 )
        				rotation_to_do *=-1;

		            //we first perform the /rotation_to_do
        			ROS_INFO("(decision_node) /rotation_to_do: %f", rotation_to_do*180/M_PI);
        			std_msgs::Float32 msg_rotation_to_do;
		            // envoie de l'information
        			msg_rotation_to_do.data = rotation_to_do;
        			pub_rotation_to_do.publish(msg_rotation_to_do);

        		}
        		else {
        			geometry_msgs::Point msg_goal_reached;
        			msg_goal_reached.x = 0;
        			msg_goal_reached.y = 0;

        			ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
        			pub_goal_reached.publish(msg_goal_reached);
        		}

        	}

        	new_goal_to_reach = false;

		    //we receive an ack from rotation_action_node. So, we perform the /translation_to_do
        	if ( new_rotation_done ) {
        		ROS_INFO("(decision_node) /rotation_done : %f", rotation_done*180/M_PI);
        		cond_rotation = false;
        		new_rotation_done = false;

		        //the rotation_to_do is done so we perform the translation_to_do
        		ROS_INFO("(decision_node) /translation_to_do: %f", translation_to_do);
        		std_msgs::Float32 msg_translation_to_do;
        		msg_translation_to_do.data = translation_to_do;
        		pub_translation_to_do.publish(msg_translation_to_do);
        	}

		    //we receive an ack from translation_action_node. So, we send an ack to the moving_persons_detector_node
        	if ( new_translation_done ) {
        		ROS_INFO("(decision_node) /translation_done : %f\n", translation_done);
        		cond_translation = false;
        		new_translation_done = false;

		        //the translation_to_do is done so we send the goal_reached to the detector/tracker node
        		geometry_msgs::Point msg_goal_reached;
        		ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);

        		msg_goal_reached.x = goal_reached.x;
        		msg_goal_reached.y = goal_reached.y;
        		msg_goal_reached.z = goal_reached.z;
        		pub_goal_reached.publish(msg_goal_reached);

        		ROS_INFO(" ");
        		ROS_INFO("(decision_node) waiting for a /goal_to_reach");
        	}
		} // update


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
