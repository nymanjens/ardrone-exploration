/* 
 * Author: Jens Nyman (nymanjens.nj@gmail.com)
 */

#include "VideoSource.h"
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <cvd/image_convert.h>
#include <gvars3/instances.h>
#include "shm_socket/shm_socket.h"

// settings
#define   VIDEO_NUM    0
#define   VIDEO2_NUM   1


namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace shm;

VideoSource::VideoSource() {
    // get the segment.
    shared_data* shdata = shared_data::get_or_create();
    Video* video = &shdata->vid[VIDEO_NUM];
    Video* video2 = &shdata->vid[VIDEO2_NUM];

    // wait untill data is valid
    cout << "  [VideoSource_ROS] Waiting untill shared data is valid (SHM_KEY=" << SHM_KEY << ")...";
    video->waitUntillValid();
    cout << " done" << endl;
    cout << "  [VideoSource_ROS] subscribed to " << video->name << endl;

    // set size
    mirSize = CVD::ImageRef(video->width, video->height);
    mirSize2 = CVD::ImageRef(video2->width, video2->height);
};

ImageRef VideoSource::Size() {
    return mirSize;
};

ImageRef VideoSource::Size2() {
    return mirSize2;
};

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB) {
    // get data
    shared_data* shdata = shared_data::get_or_create();
    Video* video = &shdata->vid[VIDEO_NUM];

    // wait untill new frame
    video->waitUntillNewFrame();

    // set imRGB
    BasicImage<Rgb<byte> > image((Rgb<byte>*) video->data, mirSize);
    imRGB.copy_from(image);

    // set imBW
    video->convertToGrayScale(imBW.data());
}

void VideoSource::GetAndFillFrameBW2(Image<byte> &imBW) {
    // get data
    shared_data* shdata = shared_data::get_or_create();
    Video* video = &shdata->vid[VIDEO2_NUM];

    // wait untill new frame (not necessary)
    //video->waitUntillNewFrame();

    // set imBW
    video->convertToGrayScale(imBW.data());
}

}
