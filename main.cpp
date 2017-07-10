#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <termios.h>
#include <wiringPi.h>
#include <softPwm.h>

using namespace std;
using namespace cv;
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

int main() {
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
    namedWindow("origin", CV_WINDOW_AUTOSIZE);
    namedWindow("prew", CV_WINDOW_AUTOSIZE);
    namedWindow("gray", CV_WINDOW_AUTOSIZE);


    TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
    Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 500;
    bool needToInit = false;
    bool nightMode = false;

    vector<Point2f> points[2];
    Mat raw_input;
    Mat origin;
    Mat grey;
    Mat prev_grey;
    Mat flow;
    double t;

    while (1) {
        t = (double) getTickCount();
        ioctl(fd, VIDIOC_DQBUF, &buf);
        t = (double) getTickCount() - t;
        printf("used time1 is %gms\n", (t / (getTickFrequency())) * 1000);

        t = (double) getTickCount();
        buf.index = 0;
        raw_input = Mat(IMAGEHEIGHT, IMAGEWIDTH, CV_8UC3, (void *) buffer);//CV_8UC3


        imdecode(raw_input, 1).copyTo(origin);
        ioctl(fd, VIDIOC_QBUF, &buf);

        if (origin.empty()) printf("No img\n");
        imshow("origin", origin);


        cvtColor(origin, grey, CV_RGB2GRAY);


        if (nightMode)
            origin = Scalar::all(0);

        if (needToInit) {
            // automatic initialization
            goodFeaturesToTrack(grey, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
            cornerSubPix(grey, points[1], subPixWinSize, Size(-1, -1), termcrit);
        } else if (!points[0].empty()) {
            puts("aaa");
            vector<uchar> status;
            vector<float> err;
            if (prev_grey.empty())
                grey.copyTo(prev_grey);
            calcOpticalFlowPyrLK(prev_grey, grey, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            size_t i, k;
            for (i = k = 0; i < points[1].size(); i++) {

                if (!status[i])
                    continue;

                points[1][k++] = points[1][i];
                circle(origin, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
            }
            points[1].resize(k);
        }





//            for (int y = 0; y < origin.rows; y += 5) {
//                for (int x = 0; x < origin.cols; x += 5) {
//                    const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
//                    line(origin, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)),
//                         Scalar(255, 0, 0));
//                    circle(origin, Point(x, y), 1, Scalar(0, 0, 0), -1);
//                }
//            }
        imshow("prew", origin);
//            grey.copyTo(prev_grey);

//
        needToInit = false;

        char c = (char) waitKey(10);
        if (c == 27)
            break;
        switch (c) {
            case 'r':
                needToInit = true;
                break;
            case 'c':
                points[0].clear();
                points[1].clear();
                break;
            case 'n':
                nightMode = !nightMode;
                break;
        }

        std::swap(points[1], points[0]);
        cv::swap(prev_grey, grey);
        t = (double) getTickCount() - t;
        printf("used time2 is %gms\n", (t / (getTickFrequency())) * 1000);

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
        printf("Length: %d\nAddress: %p\n", buf.length, buffer);
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
