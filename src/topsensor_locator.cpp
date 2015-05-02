#include "topsensor_locator.h"

#include <fstream>

void receivedMsg(const std_msgs::Float32MultiArray & msg)
{
    if( (3.3 - msg.data[0]) < min_val_right )
        min_val_right = 3.3 - msg.data[0];

    INFO_STREAM( "hoi"   );
}

void receivedMsg2(const std_msgs::Float32MultiArray & msg)
{
    if( (3.3 - msg.data[1]) < min_val_left )
        min_val_left = 3.3 - msg.data[1];

    INFO_STREAM( "hallo"   );
}

// /amigo/whiskergripper/top_measurements/data[0]  -> rechts
// /amigo/whiskergripper/whisker_measurements/data[1]  -> links

int main(int argc, char** argv)
{

    /* initialize node */
    ros::init(argc, argv, THIS_NODE);
    ros::NodeHandle n;
    ros::Rate r(THIS_NODE_RATE);

    ros::Publisher min_vals_left_pub = n.advertise<std_msgs::Float64>("/amigo/whiskergripper/min_val_topsens_left", 0);
    ros::Publisher min_vals_right_pub = n.advertise<std_msgs::Float64>("/amigo/whiskergripper/min_val_topsens_right", 0);
    ros::Subscriber pressure_sens_sub = n.subscribe("/amigo/whiskergripper/top_measurements", 1000, receivedMsg);
    ros::Subscriber whisker_sens_sub = n.subscribe("/amigo/whiskergripper/whisker_measurements", 1000, receivedMsg2);

    std_msgs::Float64 msg;

    /* main update loop */
    while(ros::ok())
    {

        msg.data = min_val_left;
        min_vals_left_pub.publish(msg);

        msg.data = min_val_right;
        min_vals_right_pub.publish(msg);

        /* wait for next sample */
        r.sleep();
        ros::spinOnce();

    }

    std::ofstream f_out;
    f_out.open("out.log");
    f_out << min_val_left << '\n' << min_val_right;
    f_out.close();


    return 0;
}
