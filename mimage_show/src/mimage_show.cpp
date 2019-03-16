#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class MImageShow
{
public:
    MImageShow(const ros::NodeHandle &private_nh = ros::NodeHandle("~"),
               const std::string &topic_sub = "color",
               const int queueSize = 5) :
            private_nh(private_nh), topic_sub(topic_sub), queueSize(queueSize), running(true) {}

    ~MImageShow()
    {
        running = false;
        ROS_INFO("my image show stop");
    }

    void start()
    {
        ROS_INFO("my image show start");

        image_transport::ImageTransport it(private_nh);
        subscriber = it.subscribe(topic_sub.c_str(), queueSize, &MImageShow::callback, this);
    }

    void callback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv::imshow("image", cv_bridge::toCvShare(msg, "bgr8")->image);
            cv::waitKey(30);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

private:
    ros::NodeHandle private_nh;
    int queueSize;
    bool running;
    std::string topic_sub;

    image_transport::Subscriber subscriber;
};

class MImageShowNodelet : public nodelet::Nodelet
{
public:
    MImageShowNodelet() : Nodelet(), p(nullptr) {}

    ~MImageShowNodelet() override
    {
        NODELET_DEBUG("Exiting nodelet...");
        delete p;
    }

private:
    void onInit() override
    {
        NODELET_DEBUG("Initializing nodelet...");
        p = new MImageShow(getPrivateNodeHandle());
        p->start();
    }

    MImageShow *p;
};

PLUGINLIB_DECLARE_CLASS(MImageShow, MImageShowNodelet, MImageShowNodelet, nodelet::Nodelet);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mimage_show");
    ros::NodeHandle private_nh("~");

    MImageShow my_node(private_nh, "color");

    my_node.start();

    ros::spin();

    ros::shutdown();
    return 0;
}