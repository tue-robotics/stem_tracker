#include "whisker_simulator.h"

void ymlToMsg(tue::Configuration& config, std_msgs::Float32MultiArray& msg)
{
    msg.data.clear();
    double tmp;
    int n_whiskers = 0;

    if (config.readArray("whisker_values"))
    {
        while(config.nextArrayItem())
        {
            config.value("w",tmp);
            msg.data.push_back(tmp);
            ++n_whiskers;
        }
    }

    std_msgs::MultiArrayDimension dim;
    dim.size = n_whiskers;
    dim.label = "whiskers_from_sim";
    msg.layout.dim.push_back(dim);

    return;
}

int main(int argc, char** argv)
{

    std_msgs::Float32MultiArray whisker_values_msg;

    /* initialize configuration */
    tue::Configuration config;

    /* load configuration */
    if (argc >= 2)
    {
        std::string yaml_filename = argv[1];
        config.loadFromYAMLFile(yaml_filename);
    }
    else
    {
        std::string this_package_dir = ros::package::getPath(THIS_PACKAGE);
        config.loadFromYAMLFile(this_package_dir + "/config/whisker_sim_values.yml");
    }

    if (config.hasError())
    {
        ERROR_STREAM("Could not load configuration: " << config.error());
        return 1;
    }

    ymlToMsg(config, whisker_values_msg);

    /* initialize node */
    ros::init(argc, argv, THIS_PACKAGE);
    ros::NodeHandle n;
    ros::Rate r(3);

    ros::Publisher whisker_pub = n.advertise<std_msgs::Float32MultiArray>("/amigo/whiskergripper/measurements", 0);

    /* main update loop */
    while(ros::ok())
    {

        if (config.sync())
        {
            /* config file changed */
            ymlToMsg(config, whisker_values_msg);

        }

        if (!config.hasError())
        {
            whisker_pub.publish(whisker_values_msg);
        }

        else
        {
            ERROR_STREAM("Could not load configuration: " << config.error());
            return 1;
        }

        /* wait for next sample */
        r.sleep();
        ros::spinOnce();

    }

    return 0;
}
