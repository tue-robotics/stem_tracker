#include "whisker_simulator.h"

void ymlToWhiskerMsg(tue::Configuration& config, std_msgs::Float32MultiArray& msg)
{
    msg.data.clear();
    double tmp;

    if (config.readArray("whisker_values"))
    {
        while(config.nextArrayItem())
        {
            config.value("w",tmp);
            msg.data.push_back(tmp);
        }
        config.endArray();
    }

    return;
}

void ymlToPressureMsg(tue::Configuration& config, std_msgs::Float32MultiArray& msg)
{
    msg.data.clear();
    double tmp;

    if (config.readArray("pressure_values"))
    {
        while(config.nextArrayItem())
        {
            config.value("p",tmp);
            msg.data.push_back(tmp);
        }
        config.endArray();
    }
    return;
}

int main(int argc, char** argv)
{

    std_msgs::Float32MultiArray whisker_values_msg, pressure_values_msg;
    int this_node_rate;

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

    config.value("rate",this_node_rate);
    ymlToWhiskerMsg(config, whisker_values_msg);
    ymlToPressureMsg(config, pressure_values_msg);

    /* initialize node */
    ros::init(argc, argv, THIS_NODE);
    ros::NodeHandle n;
    ros::Rate r(this_node_rate);

    ros::Publisher pressure_pub = n.advertise<std_msgs::Float32MultiArray>("/amigo/whiskergripper/top_measurements", 0);
    ros::Publisher whisker_pub = n.advertise<std_msgs::Float32MultiArray>("/amigo/whiskergripper/whisker_measurements", 0);

    /* main update loop */
    while(ros::ok())
    {

        if (config.sync())
        {
            /* config file changed */
            ymlToWhiskerMsg(config, whisker_values_msg);
            ymlToPressureMsg(config, pressure_values_msg);

        }

        if (!config.hasError())
        {
            whisker_pub.publish(whisker_values_msg);
            pressure_pub.publish(pressure_values_msg);
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
