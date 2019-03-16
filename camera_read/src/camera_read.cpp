#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraRead
{
public:
    CameraRead(const ros::NodeHandle &private_nh = ros::NodeHandle("~"),
               const std::string &topic_pub = "color",
               const int queueSize = 5) :
            private_nh(private_nh), topic_pub(topic_pub), queueSize(queueSize), running(true) {}

    ~CameraRead()
    {
        running = false;
        ROS_INFO("camera read stop");
    }

    void start()
    {
        ROS_INFO("camera read start");

        // open default camera
        cap.open(0);

        image_transport::ImageTransport it(private_nh);
        publisher = it.advertise(topic_pub.c_str(), queueSize);

        run();
    }

    void run()
    {
        cv::Mat image_cv;
        cv_bridge::CvImage image_ros;
        while (ros::ok() && running)
        {
            cap >> image_cv;

            image_ros.header.stamp = ros::Time::now();
            image_ros.encoding = sensor_msgs::image_encodings::BGR8;
            image_ros.image = image_cv;
            publisher.publish(image_ros.toImageMsg());
        }
    }

private:
    bool running;
    ros::NodeHandle private_nh;
    int queueSize;
    std::string topic_pub;

    image_transport::Publisher publisher;

    cv::VideoCapture cap;
};

class CameraReadNodelet : public nodelet::Nodelet
{
public:
    CameraReadNodelet() : Nodelet(), p(nullptr) {}

    ~CameraReadNodelet() override
    {
        NODELET_DEBUG("Exiting nodelet...");
        delete p;
    }

private:
    void onInit() override
    {
        NODELET_DEBUG("Initializing nodelet...");
        p = new CameraRead(getPrivateNodeHandle());
        p->start();
    }

    CameraRead *p;
};

PLUGINLIB_DECLARE_CLASS(CameraRead, CameraReadNodelet, CameraReadNodelet, nodelet::Nodelet);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_read");
    ros::NodeHandle private_nh("~");

    CameraRead my_node(private_nh, "color");

    my_node.start();

    ros::spin();

    ros::shutdown();
    return 0;
}