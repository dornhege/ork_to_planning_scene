#include "ork_to_planning_scene/ork_to_planning_scene.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ork_to_planning_scene");
    ros::NodeHandle nh;

    ork_to_planning_scene::OrkToPlanningScene otps;

    ros::spin();
}

