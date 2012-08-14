/*
 * Author: Jens Nyman (nymanjens.nj@gmail.com)
 */

#ifndef H_SHM_SOCKET
#define H_SHM_SOCKET
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <time.h>
#include <iostream>
#include <string.h>
#include <cstdlib>

/*** settings ***/
// shared memory key
#define SHM_KEY         1927
#define SHM_OBJ_MARGIN  256

// video settings
//#define VID0_WIDTH      320 // 1/2 VGA (camera's on AR.Drone)
//#define VID0_HEIGHT     240
//#define VID0_WIDTH      640 // VGA (axis fisheye camera)
//#define VID0_HEIGHT     480
//#define VID0_WIDTH      1280 // underwater-camera Tim
//#define VID0_HEIGHT     720
#define VID0_WIDTH      848 // underwater-camera Tim
#define VID0_HEIGHT     480
#define VID1_WIDTH      176
#define VID1_HEIGHT     144

// generic video settings
#define VID0_SIZE       (VID0_WIDTH * VID0_HEIGHT * 3)
#define VID1_SIZE       (VID1_WIDTH * VID1_HEIGHT * 3)
#define MAX_VID_SIZE    VID0_SIZE

// other settings
#define SHM_SOCKET_MAX_NUM_POINTS      15000

using namespace std;

namespace shm {

    class Video {
    public:
        /*** raw data ***/
        unsigned char data[MAX_VID_SIZE];
        /*** video parameters ***/
        int valid;
        double time;
        double last_seen_time;
        int height;
        int width;
        int size; // in bytes
        /*** meta info ***/
        char encoding[10];
        char name[10];

        /*** constructor methods ***/
        Video(int width, int height, int size, const char* name);
        static void* memloc;
        void* operator new(unsigned int size);

        /*** utility methods ***/
        void setEncoding(const char* enc) {
            strcpy(encoding, enc);
        }

        void waitUntillValid() {
            valid = 0;
            while (valid != 1)
                usleep(100);
        }

        void waitUntillNewFrame() {
            while (time == last_seen_time)
                usleep(100);
            last_seen_time = time;
        }

        void convertToGrayScale(unsigned char* dataGray) {
            if (strcmp(encoding, "rgb")) {
                for (int pos = 0, i = 0; i < size; pos++, i += 3)
                    dataGray[pos] = 0.2989 * data[i] + 0.5870 * data[i + 1] + 0.1140 * data[i + 2];
            } else {
                cerr << "convertToGrayScale(): unable to convert format " << encoding << endl;
                exit(1);
            }
        }
    };

    struct nav_data {
        /*** data ***/
        // 3D position
        double x, y, altitude;
        // velocities
        double vx, vy;
        // Euler angles
        double phi, psi, theta;
        // extra data
        long num_frames;
        double battery;
        double timestamp;
        int spacebar_pressed;

        /*** methods ***/
        bool get_spacebar_pressed() {
            if (spacebar_pressed) {
                spacebar_pressed = 0;
                return true;
            }
            return false;
        }
    };

    struct slam_data {
        // 3D position
        double x, y, h;
        // Euler angles
        double phi, psi, theta;
        // extra data
        double confidence, num_found_points, timestamp;
        // rotation matrix string
        char rotmx[200];
    };

    struct point_data {
        double x, y, h;
    };

    struct points_data {
        // meta-data
        int size; // = num poins
        double timestamp;
        // points
        point_data p[SHM_SOCKET_MAX_NUM_POINTS];
    };

    struct signal {
        bool signaled;
        int map_id;

        signal() {
            signaled = false;
        }

        void set(int mpid) {
            signaled = true;
            map_id = mpid;
        }

        int get() {
            if(signaled){
                signaled = false;
                return map_id;
            }
            return -1;
        }
    };

    struct shared_data {
        Video vid[3];
        nav_data nav;
        slam_data slam;
        slam_data slam2;
        points_data points;
        signal sig_wallviz_ready; // signalizes when wallviz can start building the map
        signal sig_door; // signalizes when passing through door

        static shared_data * get_or_create() {
            static shared_data* shdata = NULL;
            if (shdata != NULL)
                return shdata;
            int shm_id;
            if ((shm_id = shmget(SHM_KEY, sizeof (shared_data), IPC_CREAT | 0666)) < 0
                    || (shdata = (shared_data *) shmat(shm_id, 0, 0)) == (shared_data *) - 1) {
                cerr << "Error: shmget() or shmat() failed" << endl;
                exit(1);
            }
            return shdata;
        }
    };
}

#endif
