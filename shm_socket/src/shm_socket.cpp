/*
 * Author: Jens Nyman (nymanjens.nj@gmail.com)
 */

#include <unistd.h>
#include <boost/algorithm/string.hpp>
#include <string>
#include <map>
#include <stdlib.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include "shm_socket/shm_socket.h"
using namespace std;
using namespace shm;
using namespace boost;

void* Video::memloc = NULL;

void* Video::operator new(unsigned int size) {
    void* old_memloc = memloc;
    memloc = (void*) ((int) memloc + size);
    return old_memloc;
}

Video::Video(int width, int height, int size, const char* name) {
    valid = 0;
    time = 0.0;
    last_seen_time = 0.0;
    this->width = width;
    this->height = height;
    this->size = size;
    strcpy(this->name, name);
}

void convert_bgr_to_rgb(Video *video, const unsigned char* src) {
    unsigned char* dest = video->data;
    for (int i = 0; i < video->size; i += 3) {
        dest[i] = src[i + 2];
        dest[i + 1] = src[i + 1];
        dest[i + 2] = src[i];
    }
}

void navDataCallback(const std_msgs::String::ConstPtr& msg) {
    /*** decode navdata string to navdata map ***/
    string m = msg->data;
    std::vector<std::string> items;
    boost::split(items, m, boost::is_any_of(","));
    map<string, double> navdata;
    for (unsigned int i = 0; i < items.size(); i++) {
        std::vector<std::string> keyval;
        boost::split(keyval, items[i], boost::is_any_of(":"));
        navdata[keyval[0]] = atof(keyval[1].c_str());
    }
    /*** put data in the shared memory ***/
    shared_data* shdata = shared_data::get_or_create();
    // 3D position
    shdata->nav.x = navdata["x"];
    shdata->nav.y = navdata["y"];
    shdata->nav.altitude = navdata["altitude"];
    // velocities
    shdata->nav.vx = navdata["vx"];
    shdata->nav.vy = navdata["vy"];
    // angles
    shdata->nav.phi = navdata["phi"];
    shdata->nav.psi = navdata["psi"];
    shdata->nav.theta = navdata["theta"];
    // extra data
    shdata->nav.num_frames = (long) navdata["num_frames"];
    shdata->nav.battery = navdata["battery"];
    shdata->nav.timestamp = navdata["timestamp"];
    shdata->nav.spacebar_pressed = (int) navdata["spacebar_pressed"];
    // debug info
    //ROS_INFO("got navdata #%d", (int) shdata->nav.num_frames);
}

void videoCallback(const sensor_msgs::ImageConstPtr& msg, int vid_type) {
    // get the segment.
    shared_data* shdata = shared_data::get_or_create();
    // check size and get video object
    Video *video;
    if (0 <= vid_type && vid_type <= 2) {
        video = &shdata->vid[vid_type];
    } else {
        ROS_ERROR("invalid vid_type (%d)", vid_type);
        exit(1);
    }
    // get encoding
    string encoding = (string) msg->encoding;
    // set video data
    if (encoding == "rgb8") {
        memcpy(video->data, &msg->data[0], video->size);
        video->setEncoding("rgb8");
    } else if (encoding == "bgr8") {
        convert_bgr_to_rgb(video, &msg->data[0]);
        video->setEncoding("rgb8");
    } else {
        ROS_ERROR("encoding is not supported (encoding=%s)", encoding.c_str());
    }
    // set video time
    video->time = msg->header.stamp.toSec();
    // set valid_data
    video->valid = 1;
    // debug info
    //ROS_INFO("got frame of %s: <%s, t=%f>", video->name, encoding.c_str(), video->time);
}

void videoCallback0(const sensor_msgs::ImageConstPtr& msg) {
    videoCallback(msg, 0);
}

void videoCallback1(const sensor_msgs::ImageConstPtr& msg) {
    videoCallback(msg, 1);
}

void videoCallback2(const sensor_msgs::ImageConstPtr& msg) {
    videoCallback(msg, 2);
}

ros::NodeHandle *global_nh;

void publishSlamDataFrom(slam_data slamdata, ros::Publisher publisher) {
    // parse data to map
    map<string, double> data;
    // 3D position
    data["x"] = slamdata.x;
    data["y"] = slamdata.y;
    data["h"] = slamdata.h;
    // Euler angles
    data["phi"] = slamdata.phi;
    data["psi"] = slamdata.psi;
    data["theta"] = slamdata.theta;
    // extra data
    data["confidence"] = isnormal(slamdata.confidence) ? slamdata.confidence : 0;
    data["num_found_points"] = slamdata.num_found_points;
    data["timestamp"] = slamdata.timestamp;
    // rotation matrix
    char* extradata = slamdata.rotmx;
    extradata[199]= 0;
    
    // parse map to string
    ostringstream s;
    s << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1);
    for (std::map< string, double>::const_iterator iter = data.begin(); iter != data.end(); ++iter)
        s << iter->first << ":" << iter->second << ",";
    s << "rotmx:" << extradata << ",";
    // publish string
    std_msgs::String str;
    str.data = s.str();
    publisher.publish(str);
}

void publishSlamData() {
    shared_data* shdata = shared_data::get_or_create();
    ros::Publisher publisher = global_nh->advertise<std_msgs::String > ("/ardrone/slamdata", 1);
    ros::Publisher publisher2 = global_nh->advertise<std_msgs::String > ("/ardrone/slamdata2", 1);

    while (!ros::isShuttingDown()) {
        // check if SLAM2 is active
        bool slam2active = shdata->slam.timestamp - 5 < shdata->slam2.timestamp && shdata->slam2.timestamp < shdata->slam.timestamp + 5;
        if (slam2active) {
            /*** wait for SLAM2 if SLAM2 is active ***/
            double oldTimestamp2 = shdata->slam2.timestamp;
            while (!ros::isShuttingDown() && oldTimestamp2 == shdata->slam2.timestamp)
                usleep(1000);
        } else {
            /*** wait for SLAM to return data, and publish it ***/
            double oldTimestamp = shdata->slam.timestamp;
            while (!ros::isShuttingDown() && oldTimestamp == shdata->slam.timestamp)
                usleep(1000);
        }
        // publish slamdata and slamdata2
        publishSlamDataFrom(shdata->slam, publisher);
        if (slam2active)
            publishSlamDataFrom(shdata->slam2, publisher2);
    }
}

void publishMapPoints() {
    shared_data* shdata = shared_data::get_or_create();
    ros::Publisher publisher = global_nh->advertise<std_msgs::String > ("/map/points", 1);

    // enfoce refresh rate
    ros::Rate rate(5); // in hz
    while (!ros::isShuttingDown()) {
        /*** wait for map to return data, and publish it ***/
        // wait untill valid
        double oldTimestamp = shdata->points.timestamp;
        rate.sleep();
        while (!ros::isShuttingDown() && oldTimestamp == shdata->points.timestamp)
            usleep(1000);
        // get data
        point_data* points = shdata->points.p;
        // parse map to string
        ostringstream s;
        s << std::scientific << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1);
        for (int i = 0; i < shdata->points.size; i++)
            s << points[i].x << "," << points[i].y << "," << points[i].h << "|";
        // publish string
        std_msgs::String str;
        str.data = s.str();
        publisher.publish(str);
        //ROS_INFO("published to /map/points: %s", s.str().c_str());
    }
}

void publishSignals() {
    shared_data* shdata = shared_data::get_or_create();
    ros::Publisher publisher = global_nh->advertise<std_msgs::String > ("/slam/signals", 1);
    std_msgs::String str;

    // enfoce refresh rate
    ros::Rate rate(15); // in hz
    while (!ros::isShuttingDown()) {
        /*** check wallviz ready signal ***/
        if (shdata->sig_wallviz_ready.signaled) {
            char buf[10];
            sprintf(buf, "w\n%d", shdata->sig_wallviz_ready.get());
            str.data = buf;
            publisher.publish(str);
            //ROS_INFO("published to /slam/signals: %s", buf);
        }

        /*** check door signal ***/
        if (shdata->sig_door.signaled) {
            char buf[10];
            sprintf(buf, "d\n%d", shdata->sig_door.get());
            str.data = buf;
            publisher.publish(str);
            //ROS_INFO("published to /slam/signals: %s", buf);
        }
        rate.sleep();
    }
}

int main(int argc, char **argv) {
    // create the segment.
    shared_data* shdata = shared_data::get_or_create();

    // initialise data
    Video::memloc = &shdata->vid[0];
    new Video(VID0_WIDTH, VID0_HEIGHT, VID0_SIZE, "video0");
    new Video(VID1_WIDTH, VID1_HEIGHT, VID1_SIZE, "video1");
    new Video(VID1_WIDTH, VID1_HEIGHT, VID1_SIZE, "video2");

    // create ROS node
    ros::init(argc, argv, "ptam_shm_socket");
    ros::NodeHandle nh;
    global_nh = &nh;
    // subscribe (navdata)
    string navdata_channel = "/ardrone/navdata/elf";
    ros::Subscriber sub = nh.subscribe(navdata_channel.c_str(), 1, navDataCallback);
    ROS_INFO("subscribing to %s...", navdata_channel.c_str());
    // subscribe (video)
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subs[3];
    subs[0] = it.subscribe("/ardrone_video0/image", 1, videoCallback0);
    subs[1] = it.subscribe("/ardrone_video1/image", 1, videoCallback1);
    subs[2] = it.subscribe("/ardrone_video2/image", 1, videoCallback2);
    ROS_INFO("subscribing to /ardrone_video0-2/image");
    // start publisher
    thread slamDataPublisherThread = thread(publishSlamData);
    thread mapPointPublisherThread = thread(publishMapPoints);
    thread signalPublisherThread = thread(publishSignals);

    // spin() periodically checks for new messages
    ros::spin();
    cout << endl;
    return 0;
}
