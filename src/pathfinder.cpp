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

            // communication with rotation_action
            pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);
            sub_rotation_done = n.subscribe("rotation_done", 1, &decision::rotation_doneCallback, this);
            cond_rotation = false;
        
            // communication with translation_action
            pub_translation_to_do = n.advertise<std_msgs::Float32>("translation_to_do", 0);
            sub_translation_done = n.subscribe("translation_done", 1, &decision::translation_doneCallback, this);
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

        int main(int argc, char **argv){

            ROS_INFO("(pathfinder_node) waiting for /odometry and /map_server");
            ros::init(argc, argv, "pathfinder");
            pathfinder bsObject;
            ros::spin();
            return 0;
        }

        // Distance between two points
        float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {
            return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));
        }
}
