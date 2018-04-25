/* standart libs*/
#include <iostream>

// self defined Classes
#include "sfm_ba.h"

int main (int argc, char** argv)
{
    /* ros init */
    ros::init(argc, argv, SFM_BA_NODE_NAME);
    if(!ros::ok())
    {
        std::cout << "ros is error !" << std::endl;
        return -1;
    }
    /* kriegen den sfm manager*/
    SFM_BA sfm_ba;
    /* run */
    sfm_ba.Run();
    /* stop */
    ros::shutdown();
    return 0;
}
