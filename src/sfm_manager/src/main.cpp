/* standart libs*/
#include <iostream>

// self defined Classes
#include "sfm_manager.h"

int main (int argc, char** argv)
{
    /* ros init */
    ros::init(argc, argv, SFM_MANAGER_NODE_NAME);
    if(!ros::ok())
    {
        std::cout << "ros is error !" << std::endl;
        return -1;
    }
    /* get sfm manager*/
    SFM_MANAGER sfm_manager;
    /* run */
    sfm_manager.Run();
    /* stop */
    ros::shutdown();
    return 0;
}
