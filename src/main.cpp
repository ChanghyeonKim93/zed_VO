#include <iostream>
#include <ros/ros.h>
#include "vo_at.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "vo_node");

    VOAT* vo_with_zed = new VOAT();

    // Main loop start
	size_t image_counter = 1;
	ros::Rate loop_rate(100);
    vo_with_zed->initialize_cameras();
	while( ros::ok() ) 
    {
  		ros::spinOnce();
		if ( vo_with_zed->get_toggle() == true) // From gcs, this node receives the "VO ON" service signal.
        {
            vo_with_zed->publish_pose();
		}
        else { // From gcs, no signal is transmitted. This node do not any work.     
            if(image_counter % 50 == 1) printf("[TIMEOUT] VO node : not activated yet!\n");
        }
		image_counter++;
		loop_rate.sleep();
	}


    delete vo_with_zed;

	return 0;
}
