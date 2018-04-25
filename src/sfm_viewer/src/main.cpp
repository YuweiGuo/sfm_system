/* standard include */
#include <iostream>

/* selbst def */
#include "viewer.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, SFM_VIEWER_NODE_NAME);
    if(!ros::ok())
    {
        std::cout << "ros is error !" << std::endl;
        return -1;
    }
    VIEWER viewer;
    viewer.run();
}
