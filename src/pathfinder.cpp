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

    ros::Publisher pub_moving_persons_detector;
    ros::Publisher pub_moving_persons_detector_marker;

    // to store, process and display laserdata
    //geometry_msgs::Point 

    //pour stocker chemin
    geometry_msgs::Point point[NBPOINTS];
    int pathToDo[PATHLENGTH];
    int currentpoint;

public:

pathfinder() {
    currentpoint = 0;
    for (int i = 0; i < PATHLENGTH; ++i)
    {
        pathToDo[i]=i % NBPOINTS;
    }
   /*// communication with cmd_vel
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &translation_action::odomCallback, this);
    cond_translation = false;

    // communication with decision
    pub_translation_done = n.advertise<std_msgs::Float32>("translation_done", 1);
    sub_translation_to_do = n.subscribe("translation_to_do", 1, &translation_action::translation_to_doCallback, this);//this is the translation that has to be performed

    // communication with obstacle_detection
    sub_obstacle_detection = n.subscribe("closest_obstacle", 1, &translation_action::closest_obstacleCallback, this);

    error_integral = 0;
    error_previous = 0;

    new_translation_to_do = false;
    new_odom = false;
    init_obstacle = false;
*/

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
    geometry_msgs::Point targetpoint;
	//while(currentpoint < PATHLENGTH-1){
		targetpoint = pathToDo[currentpoint+1];
		calcul_rotation(pathToDo[currentpoint],targetpoint); //TODO
		faire_rotation(); //TODO
		calcul_translation(pathToDo[currentpoint],targetpoint); //TODO
		faire_translation(); //TODO
		currentpoint++;
	//}

}

}
