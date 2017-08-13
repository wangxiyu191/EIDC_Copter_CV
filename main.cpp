#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>

#include "opencv2/imgproc.hpp"
#include "opencv2/optflow.hpp"
#include "opencv2/highgui.hpp"


#include "serial.h"

using namespace std;
using namespace cv;
using namespace cv::optflow;
uchar *buffer;

#define IMAGEWIDTH 320
#define IMAGEHEIGHT 240

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define FILE_VIDEO1 "/dev/video0"
#define SERIAL_PATH "/dev/ttySAC3"
#define SERIAL_PATH_1 "/dev/ttySAC4"

static int fd;
struct v4l2_streamparm setfps;
struct v4l2_capability cap;
struct v4l2_fmtdesc fmtdesc;
struct v4l2_format fmt, fmtack;
struct v4l2_requestbuffers req;
struct v4l2_buffer buf;
enum v4l2_buf_type type;

int init_v4l2(void);

int v4l2_grab(void);

int do_cv(void);

void serve_serial(void);

void serve_serial1(void);

long long *sum_x = nullptr;
long long *sum_y = nullptr;

uint16_t *offest_x = nullptr;
uint16_t *offest_y = nullptr;

int *mission_id = nullptr;

int main() {

    pid_t f_pid;

    sum_x = (long long *) mmap(NULL, sizeof(long long), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    sum_y = (long long *) mmap(NULL, sizeof(long long), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    offest_x = (uint16_t *) mmap(NULL, sizeof(uint16_t), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    offest_y = (uint16_t *) mmap(NULL, sizeof(uint16_t), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    mission_id = (int *) mmap(NULL, sizeof(int), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    *mission_id = 3;
    *sum_x = 0;
    *sum_y = 0;
    f_pid = fork();
    if (f_pid < 0) {
        puts("fork error");
        return 1;
    } else if (f_pid == 0) {
        do_cv();
    } else {
        pid_t f1_pid;
        f1_pid = fork();
        if (f1_pid < 0) {
            puts("fork error 1");
            return 1;
        } else if (f1_pid == 0) {
            serve_serial();
        } else {
            serve_serial1();
        }

    }
    return 0;
}

struct DataForMCU {
    const uint8_t Head = 0xff;
    const char X_chr = 'x';
    uint16_t x;
    const uint8_t Mid = 0xfe;
    const char Y_chr = 'y';
    uint16_t y;
    const uint8_t Tail = 0xf0;
    uint8_t TL_Flag; //1:takeoff 0:land;
};

void serve_serial1(void) {
    long long last_us;
    int fd = open(SERIAL_PATH_1, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        puts("cant open serial");
        exit(1);
    }
    set_speed(fd, 38400);
    if (set_Parity(fd, 8, 1, 'N') == FALSE) {
        printf("Set Parity Error\n");
        exit(0);
    }
    puts("serial1 inited");

    pid_t read_pid;
    read_pid = fork();
    if (read_pid < 0) {
        puts("fork error");
        return;
    } else if (read_pid == 0) {
        //read
        while (1) {
            char command;
            read(fd, &command, 1);
            if (command >= 'A' && command <= 'Z') {
                //A: find black point
                //B: find green car
                *mission_id = command - 'A' + 1;
            } else {
                *mission_id = 'A';
            }
        }
    } else {
        //write
        while (1) {
            if (*mission_id == 3) {
                usleep(1000 * 50);
                continue;
            }

            DataForMCU tmp;
            tmp.x = *offest_x;
            tmp.y = *offest_y;
            tmp.TL_Flag = 1;
            write(fd, &tmp, sizeof(tmp));
            usleep(1000 * 50);
        }
    }

}


void serve_serial(void) {
    long long last_us = 0;
    int fd = open(SERIAL_PATH, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        puts("cant open serial");
        exit(1);
    }
    set_speed(fd, 115200);
    if (set_Parity(fd, 8, 1, 'N') == FALSE) {
        printf("Set Parity Error\n");
        exit(0);
    }
    puts("serial inited");
    char buf[1000];
    int buf_len = 0;
    char tx_buf[100];

    timeval now_time;
    while (1) {
        char c;
        read(fd, &c, 1);
        if (c == '\n') {
            buf[buf_len] = 0;
            if (strcmp(buf, "HELLO") == 0) {
                write(fd, "HELLO\n", 6);
                gettimeofday(&now_time, NULL);
                last_us = (long long) now_time.tv_sec * (long long) 10e6 + now_time.tv_usec;
                *sum_x = 0;
                *sum_y = 0;
                puts("handshake");
            } else if (strcmp(buf, "DATA") == 0) {

                gettimeofday(&now_time, NULL);
                long long now_us = (long long) now_time.tv_sec * (long long) 10e6 + now_time.tv_usec;
                sprintf(tx_buf, "%lld\n%lld\n%lld\n", *sum_x, *sum_y, now_us - last_us);
                write(fd, tx_buf, strlen(tx_buf));
                write(STDOUT_FILENO, tx_buf, strlen(tx_buf));
                *sum_x = 0;
                *sum_y = 0;
                gettimeofday(&now_time, NULL);
                last_us = (long long) now_time.tv_sec * (long long) 10e6 + now_time.tv_usec;
                puts("send_value");
            } else if (strcmp(buf, "EXIT") == 0) {
                break;
            }
            buf_len = 0;
        } else if (isalpha(c)) {
            buf[buf_len++] = c;
        } else {
            printf("get wrong char:%c\n", c);
        }
    }

}


void do_optflow(const Mat &origin, Mat &prev, const Ptr<DenseOpticalFlow> algo) {
    vector<Point2f> points[2];
    Mat grey;
    Mat flow;
    Mat result;


    cvtColor(origin, grey, CV_RGB2GRAY);
    if (prev.empty()) {
        grey.copyTo(prev);
    }

    algo->calc(prev, grey, flow);

    Scalar sumMat = sum(flow);

    Mat flowMat[2];
    split(flow, flowMat);

    int count0 = countNonZero(flowMat[0]);
    int count1 = countNonZero(flowMat[1]);

    //int count = flow.cols*flow.rows;
    //printf("X=%lld\tY=%lld\n",*sum_x,*sum_y);
    double ave[2];
    ave[0] = 1.0 * sumMat[0] / count0 * (flow.cols * flow.rows);
    ave[1] = 1.0 * sumMat[1] / count1 * (flow.cols * flow.rows);
    *sum_x += ave[0];
    *sum_y += ave[1];
    grey.copyTo(prev);
    //cv::swap(grey, prev);

//        //extraxt x and y channels
//        cv::Mat xy[2]; //X,Y
//        cv::split(flow, xy);
//
////calculate angle and magnitude
//        cv::Mat magnitude, angle;
//        cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);
//
////translate magnitude to range [0;1]
//        double mag_max;
//        cv::minMaxLoc(magnitude, 0, &mag_max);
//        magnitude.convertTo(magnitude, -1, 1.0 / mag_max);
//
////build hsv image
//        cv::Mat _hsv[3], hsv;
//        _hsv[0] = angle;
//        _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
//        _hsv[2] = magnitude;
//        cv::merge(_hsv, 3, hsv);
//
////convert to BGR and show
//        cv::Mat bgr;//CV_32FC3 matrix
//        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
//        cv::imshow("optical flow", bgr);

    //cv::waitKey(0);

}

void do_objfind(const Mat &origin) {
    Mat chosen;
    //choose
    Mat mask;
    Mat grey;
    Mat hsv;
    switch (*mission_id) {
        case 1:
            //black
            cvtColor(origin, grey, CV_BGR2GRAY);
            equalizeHist(grey, grey);
            inRange(grey, Scalar(0), Scalar(20), mask);
            break;
        case 2:
            //green
            cvtColor(origin, hsv, CV_BGR2HSV);
            inRange(hsv, Scalar(45, 100, 50), Scalar(75, 255, 255), mask);
            break;
        default:
            return;
    }
    //imshow("origin",origin);
    //origin.copyTo(chosen,mask);
    //imshow("chosen",chosen);
    //imshow("lab",hsv);
    //inRange(hsv,Scalar(156,43,46),Scalar(180,255,255),mask2);
    erode(mask, mask, Mat());
    dilate(mask, mask, Mat());
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    if (contours.size() > 0) {
        double largest_area = 0;
        int largest_area_id = 0;
        for (int i = 0; i < contours.size(); i++) {
            double tmp = contourArea(contours[i]);
            if (tmp > largest_area) {
                largest_area = tmp;
                largest_area_id = i;
            }
        }
        Point2f center;
        float radius;
        minEnclosingCircle(contours[largest_area_id], center, radius);
        if ((*mission_id == 1 && radius > 10) ||
            (*mission_id == 2 && radius > 10 && radius < 110)) {
            Moments M = moments(contours[largest_area_id]);
            center = Point2f(M.m10 / M.m00, M.m01 / M.m00);
            *offest_x = (uint16_t) center.x;
            *offest_y = (uint16_t) center.y;
        } else {
            *offest_x = (uint16_t) 0xffff;
            *offest_y = (uint16_t) 0xffff;
        }
        //printf("x=%u y=%u r=%f\n",*offest_x,*offest_y,radius);
    } else {
        *offest_x = (uint16_t) 0xFFFF;
        *offest_y = (uint16_t) 0xFFFF;
    }


}


int do_cv() {
    printf("first~~\n");
    if (init_v4l2() == FALSE) {
        printf("Init fail~~\n");
        exit(1);
    }
    printf("second~~\n");
    if (v4l2_grab() == FALSE) {
        printf("grab fail~~\n");
        exit(2);
    }
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    printf("third~~\n");

    system("sudo /home/fa/Projects/opflow/matrix-gpio_out");

    cv::setNumThreads(cv::getNumberOfCPUs() - 1);

    Mat raw_input;
    Mat origin;
    double t;
    Mat prev;
    int preset = DISOpticalFlow::PRESET_FAST;
    Ptr<DenseOpticalFlow> algo = createOptFlow_DIS(preset);

    while (1) {
        t = (double) getTickCount();
        ioctl(fd, VIDIOC_DQBUF, &buf);
        t = (double) getTickCount() - t;
        //printf("used time1 is %gms\n", (t / (getTickFrequency())) * 1000);

        t = (double) getTickCount();
        buf.index = 0;
        raw_input = Mat(IMAGEHEIGHT, IMAGEWIDTH, CV_8UC3, (void *) buffer);//CV_8UC3


        imdecode(raw_input, 1).copyTo(origin);
        ioctl(fd, VIDIOC_QBUF, &buf);


        if (origin.empty())
            printf("No img\n");
        else {
            thread objfind_thread(do_objfind, origin);
            thread optflow_thread(do_optflow, origin, std::ref(prev), algo);
            optflow_thread.join();
            objfind_thread.join();
        }

        //grey.copyTo(prev);

        t = (double) getTickCount() - t;
        //printf("used time2 is %gms\n", (t / (getTickFrequency())) * 1000);

    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    return 0;
}

int init_v4l2(void) {
    if ((fd = open(FILE_VIDEO1, O_RDWR)) == -1) {
        printf("Opening video device error\n");
        return FALSE;
    }
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        printf("unable Querying Capabilities\n");
        return FALSE;
    } else {
        printf("Driver Caps:\n"
                       "  Driver: \"%s\"\n"
                       "  Card: \"%s\"\n"
                       "  Bus: \"%s\"\n"
                       "  Version: %d\n"
                       "  Capabilities: %x\n",
               cap.driver,
               cap.card,
               cap.bus_info,
               cap.version,
               cap.capabilities);

    }

    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE) {
        printf("Camera device %s: support capture\n", FILE_VIDEO1);
    }
    if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING) {
        printf("Camera device %s: support streaming.\n", FILE_VIDEO1);
    }

    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format: \n");
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
        printf("\t%d. %s\n", fmtdesc.index + 1, fmtdesc.description);
        fmtdesc.index++;
    }
    //set fmt
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMAGEWIDTH;
    fmt.fmt.pix.height = IMAGEHEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; //*************************V4L2_PIX_FMT_YUYV****************
    fmt.fmt.pix.field = V4L2_FIELD_NONE;


    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        printf("Setting Pixel Format error\n");
        return FALSE;
    }
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == -1) {
        printf("Unable to get format\n");
        return FALSE;
    } else {
        printf("fmt.type:\t%d\n", fmt.type);
        printf("pix.pixelformat:\t%c%c%c%c\n", fmt.fmt.pix.pixelformat & 0xFF, (fmt.fmt.pix.pixelformat >> 8) & 0xFF, \
               (fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
        printf("pix.height:\t%d\n", fmt.fmt.pix.height);
        printf("pix.field:\t%d\n", fmt.fmt.pix.field);
    }

    setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps.parm.capture.timeperframe.numerator = 513;
    setfps.parm.capture.timeperframe.denominator = 61612;
    //ioctl(fd,VIDIOC_S_PARM,&setfps);
    ioctl(fd, VIDIOC_G_PARM, &setfps);
    printf("%d/%d\n", setfps.parm.capture.timeperframe.numerator, setfps.parm.capture.timeperframe.denominator);


    printf("init %s is OK\n", FILE_VIDEO1);
    //exit(0);
    return TRUE;
}

int v4l2_grab(void) {
    //struct v4l2_requestbuffers req = {0};
    //4  request for 4 buffers
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        printf("Requesting Buffer error\n");
        return FALSE;
    }
    //5 mmap for buffers
    buffer = (uchar *) malloc(req.count * sizeof(*buffer));
    if (!buffer) {
        printf("Out of memory\n");
        return FALSE;
    }
    unsigned int n_buffers;
    for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
        //struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            printf("Querying Buffer error\n");
            return FALSE;
        }

        buffer = (uchar *) mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (buffer == MAP_FAILED) {
            printf("buffer map error\n");
            return FALSE;
        }
        printf("Length: %d\nAddress: %p\n", buf.length, (void *) buffer);
        printf("Image Length: %d\n", buf.bytesused);
    }
    //6 queue
    for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
        buf.index = n_buffers;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_QBUF, &buf)) {
            printf("query buffer error\n");
            return FALSE;
        }
    }
    //7 starting
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        printf("stream on error\n");
        return FALSE;
    }
    return TRUE;
}
