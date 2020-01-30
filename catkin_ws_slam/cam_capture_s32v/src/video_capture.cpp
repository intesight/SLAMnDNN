/*
 * video_capture.c
 *
 *  Created on: Dec 24, 2015
 *      Author: Jim
 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>              /* low-level i/o control (open)*/
#include <errno.h>
#include <string.h>             /* strerror show errno meaning */
#include <sys/stat.h>           /* getting information about files attributes */
#include <linux/videodev2.h>    /* v4l2 structure */
#include <sys/mman.h>           /* memory mapping */
#include <unistd.h>             /* read write close */
#include <sys/time.h>           /* for select time */
#include <limits.h>             /* for UCHAR_MAX */
#include "video_capture.h"
#include "parkinggo/capture_v4l2.h"
#include <stdio.h>
#include <pthread.h>
#include <map>

using namespace std;

static map<string, string> DATA_FILES = 
{
    {"panorama_config", "panorama_config.txt"},
    {"front", "front.yuv"},
    {"back", "back.yuv"},
    {"left", "left.yuv"},
    {"right", "right.yuv"},
    {"Lut_Front_Single_View", "Lut_Front_Single_View.bin"},
    {"Lut_Front", "Lut_Front.bin"},
    {"Lut_Back", "Lut_Back.bin"},
    {"Lut_Left", "Lut_Left.bin"},
    {"Lut_Right", "Lut_Right.bin"},
    {"Wt_Front", "Wt_Front.bin"},
    {"Wt_Back", "Wt_Back.bin"},
    {"Wt_Left", "Wt_Left.bin"},
    {"Wt_Right", "Wt_Right.bin"}
};

static string base_dir;

unsigned char mem_video_frontyuyv[1280*720*2];
unsigned char mem_video_backyuyv[1280*720*2];
unsigned char mem_video_leftyuyv[1280*720*2];
unsigned char mem_video_rightyuyv[1280*720*2];

unsigned char mem_video_frontuyvy[1280*720*2];
unsigned char mem_video_backuyvy[1280*720*2];
unsigned char mem_video_leftuyvy[1280*720*2];
unsigned char mem_video_rightuyvy[1280*720*2];



unsigned int Lut_Fsv_View[1280*720];
unsigned int Lut_Front[LUT_POS_FB];
unsigned int Lut_Back[LUT_POS_FB];
unsigned int Lut_Left[LUT_POS_LR];
unsigned int Lut_Right[LUT_POS_LR];
unsigned long int Wt_Lut_Front[LUT_WT_FB];
unsigned long int Wt_Lut_Back[LUT_WT_FB];
unsigned long int Wt_Lut_Left[LUT_WT_LR];
unsigned long int Wt_Lut_Right[LUT_WT_LR];
unsigned char  SVM_BUFFERuyvy[DST_WIDTH * DST_HIGHT * 2];
unsigned char  SVM_BUFFERuyvy270[DST_WIDTH * DST_HIGHT * 2];
unsigned char  SVM_BUFFERyuyv[DST_WIDTH * DST_HIGHT * 2];


unsigned char  front_puyvy[IMG_WIDTH * IMG_HEIGHT * 2];
unsigned char  back_puyvy[IMG_WIDTH * IMG_HEIGHT * 2];
unsigned char  left_puyvy[IMG_WIDTH * IMG_HEIGHT * 2];
unsigned char  right_puyvy[IMG_WIDTH * IMG_HEIGHT * 2];

unsigned char  front_pyuyv[IMG_WIDTH * IMG_HEIGHT * 2];
unsigned char  back_pyuyv[IMG_WIDTH * IMG_HEIGHT * 2];
unsigned char  left_pyuyv[IMG_WIDTH * IMG_HEIGHT * 2];
unsigned char  right_pyuyv[IMG_WIDTH * IMG_HEIGHT * 2];
int   size_img  = IMG_WIDTH * IMG_HEIGHT * 2;


CvPoint1  car_up_left, car_down_right;

static char dev_name[20] = "/dev/video2";
static int fd = -1; /* vidoe0 file descriptor*/

/* Queried image buffers! */
struct buffer* buffers = NULL;
static unsigned int n_buffers = 0;
struct v4l2_buffer buf_in_while_loop;
static fd_set fds;
struct timeval tv;
unsigned char mem_tmp_T0[1280*720*2];
unsigned char mem_tmp_T1[1280*720*2];
unsigned char mem_tmp_T2[1280*720*2];
unsigned char mem_tmp_T3[1280*720*2];

UInt32_t* p_lut_front_test, *p_lut_back_test, *p_lut_left_test, *p_lut_right_test;
UInt32_t *p_wt_front_test, *p_wt_back_test, *p_wt_left_test, *p_wt_right_test;

TabBev **bev_Table[4];


void uyvy_to_yuyv(uchar* pYuyv, uchar* pUyvy, Int32_t width, Int32_t height)
{

	for (Int32_t i = 0; i < height; i++)
	{
		for (Int32_t j = 0; j < width; j++)
		{
			pYuyv[i * width * 2 + j * 2] = pUyvy[i * width * 2 + j * 2 + 1];  //Y
			pYuyv[i * width * 2 + j * 2 + 1] = pUyvy[i * width * 2 + j * 2];  //UV
		}
	}
}


void yuyv_to_uyvy(uchar* pUyvy, uchar* pYuyv, Int32_t width, Int32_t height)
{

	for (Int32_t i = 0; i < height; i++)
	{
		for (Int32_t j = 0; j < width; j++)
		{
			pUyvy[i * width * 2 + j * 2 + 1] = pYuyv[i * width * 2 + j * 2];  //Y
			pUyvy[i * width * 2 + j * 2] = pYuyv[i * width * 2 + j * 2 + 1];  //UV
		}
	}
}




/* wrapped errno display function by v4l2 API */
static void errno_exit(const char * s) {
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

/* wrapped ioctrl function by v4l2 API */
static int xioctl(int fd, int request, void * arg) {
	int r;
	do{
		r = ioctl(fd, request, arg);
	}
	while(-1 == r && EINTR == errno);
	return r;
}

static void open_device() {
	struct stat st;
	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev_name);
		exit(EXIT_FAILURE);
	}
	fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);
	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
}

/*
 * 1. Memory mapped buffers are located in device memory and must be allocated with this ioctl
 * 	  before they can be mapped into the application's address space
 * 2. set four images in buffer
 */
static void init_mmap() {
	struct v4l2_requestbuffers req;
	CLEAR(req);
	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	/* Initiate Memory Mapping */
	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support memory mapping\n", dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}
	/* video output requires at least two buffers, one displayed and one filled by the application */
	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
		exit(EXIT_FAILURE);
	}
	buffers = (buffer*)calloc(req.count, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}
	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;
		/* Query the status of a buffer */
		if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)){
			errno_exit("VIDIOC_QUERYBUF");
		}
		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = mmap(NULL /* start anywhere */, buf.length,
		PROT_READ | PROT_WRITE /* required */,
		MAP_SHARED /* recommended */, fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start){
			errno_exit("mmap");
		}
	}
}

/* set video streaming format here(width, height, pixel format, cropping, scaling) */
static void init_device() {
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	unsigned int min;
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n", dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n", dev_name);
		exit(EXIT_FAILURE);
	}
	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
		exit(EXIT_FAILURE);
	}
	CLEAR(fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = IM_WIDTH;
	fmt.fmt.pix.height = IM_HEIGHT;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)){
		errno_exit("VIDIOC_S_FMT");
	}
	/* YUYV sampling 4 2 2, so bytes per pixel is 2*/
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min){
		fmt.fmt.pix.bytesperline = min;
	}
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min){
		fmt.fmt.pix.sizeimage = min;
	}
	init_mmap();
}

static void start_capturing() {
	unsigned int i;
	enum v4l2_buf_type type;
	for (i = 0; i < n_buffers; ++i) {
		struct v4l2_buffer buf;
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		/* enqueue an empty (capturing) or filled (output) buffer in the driver's incoming queue */
		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)){
			errno_exit("VIDIOC_QBUF");
		}
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	/* Start streaming I/O */
	if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)){
		errno_exit("VIDIOC_STREAMON");
	}
}

static void close_device() {
	if (-1 == close(fd))
		errno_exit("close");
	fd = -1;
}

static void stop_capturing() {
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)){
		errno_exit("VIDIOC_STREAMOFF");
	}
}

static void uninit_device() {
	unsigned int i;
	for (i = 0; i < n_buffers; ++i){
		if (-1 == munmap(buffers[i].start, buffers[i].length)){
			errno_exit("munmap");
		}
	}
	free(buffers);
}

void init_video_capture(char* dev){
	strcpy(dev_name, dev);

	open_device();
	init_device();
	start_capturing();
}

char video_capture(){
	unsigned int i,j;
	FD_ZERO(&fds);
	FD_SET(fd, &fds);
	FD_SET(fileno(stdin), &fds);
	/* Timeout. */
	tv.tv_sec = 2;
	tv.tv_usec = 0;
	select(fd + 1, &fds, NULL, NULL, &tv);

	if(FD_ISSET(fd, &fds)){
		CLEAR(buf_in_while_loop);
		buf_in_while_loop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf_in_while_loop.memory = V4L2_MEMORY_MMAP;
		/* dequeue from buffer */
		if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf_in_while_loop)){
			switch(errno){
			case EAGAIN:
				return 0;
			default:
				errno_exit("VIDIOC_DQBUF");
			}
		}

		unsigned char* dst_streams = (unsigned char*)buffers[buf_in_while_loop.index].start;
		unsigned char tag = dst_streams[720*1920*2+1280*2+100];
		///printf("data=%x\n", tag);

//		if(tag == 0x6e || tag == 0xb7 || tag == 0xa2 || tag == 0xa8) {
        if(tag == 0x6e || tag == 0x4c) {
			for(i =0;i<720;i++)
				memcpy((unsigned char *)mem_tmp_T0+i*1280*2,(unsigned char *)dst_streams+1920*2*i,1280*2);
			for(i=720;i<1080;i++)
				memcpy((unsigned char *)mem_tmp_T1+1280*2*(i-720), (unsigned char *)dst_streams+1920*2*i, 1280*2);
			for(i=0;i<360;i++)
				memcpy((unsigned char *)mem_tmp_T1+1280*2*(360+i), (unsigned char *)dst_streams+i*1920*2+1280*2, 640*2);
			for(i=360;i<720;i++)
				memcpy((unsigned char *)mem_tmp_T1+640*2+i*1280*2, (unsigned char *)dst_streams+i*1920*2+1280*2, 640*2);
         ///   for(i=720;i<1080;i++)
           ///     for(j=0;j< 640;j++)
           ////     printf("zxj=== data=%x,data1=%x\n", dst_streams[720*1920*2+1280*2+j], dst_streams[720*1920*2+1280*2+j+1]);
//		} else if(tag == 0x99 || tag == 0x51 || tag == 0x55 || tag == 0x50) {
        } else if(tag == 0x99 || tag == 0xad) {
			for(i =0;i<720;i++)
				memcpy((unsigned char *)mem_tmp_T2+i*1280*2,(unsigned char *)dst_streams+1920*2*i,1280*2);
			for(i=720;i<1080;i++)
				memcpy((unsigned char *)mem_tmp_T3+1280*2*(i-720), (unsigned char *)dst_streams+1920*2*i, 1280*2);
			for(i=0;i<360;i++)
				memcpy((unsigned char *)mem_tmp_T3+1280*2*(360+i), (unsigned char *)dst_streams+i*1920*2+1280*2, 640*2);
			for(i=360;i<720;i++)
				memcpy((unsigned char *)mem_tmp_T3+640*2+i*1280*2, (unsigned char *)dst_streams+i*1920*2+1280*2, 640*2);
		} else {
		    printf("unkown image");
		}

		/* queue-in buffer */
		if(-1 == xioctl(fd, VIDIOC_QBUF, &buf_in_while_loop)){
			errno_exit("VIDIOC_QBUF");
		}
	}
	return 0;
}

#if 0
char video_capture(){
	unsigned int i;
	FD_ZERO(&fds);
	FD_SET(fd, &fds);
	FD_SET(fileno(stdin), &fds);
	/* Timeout. */
	tv.tv_sec = 2;
	tv.tv_usec = 0;
	select(fd + 1, &fds, NULL, NULL, &tv);

	if(FD_ISSET(fd, &fds)){
		CLEAR(buf_in_while_loop);
		buf_in_while_loop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf_in_while_loop.memory = V4L2_MEMORY_MMAP;
		/* dequeue from buffer */
		if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf_in_while_loop)){
			switch(errno){
			case EAGAIN:
				return 0;
			default:
				errno_exit("VIDIOC_DQBUF");
			}
		}

		unsigned char* dst_streams = (unsigned char*)buffers[buf_in_while_loop.index].start;
		unsigned char yuv2rgb;
	        yuv2rgb = (dst_streams[0]-0.344*(dst_streams[1]-128)-(0.714*(dst_streams[3]-128)))>UCHAR_MAX?UCHAR_MAX:(dst_streams[0]-0.344*(dst_streams[1]-128)-(0.714*(dst_streams[3]-128)));
		if(yuv2rgb == 0x0) {
			for(i =0;i<720;i++)
				memcpy((unsigned char *)mem_tmp_T0+i*1280*2,(unsigned char *)dst_streams+1920*2*i,1280*2);
			for(i=720;i<1080;i++)
				memcpy((unsigned char *)mem_tmp_T1+1280*2*(i-720), (unsigned char *)dst_streams+1920*2*i, 1280*2);
			for(i=0;i<360;i++)
				memcpy((unsigned char *)mem_tmp_T1+1280*2*(360+i), (unsigned char *)dst_streams+i*1920*2+1280*2, 640*2);
			for(i=360;i<720;i++)
				memcpy((unsigned char *)mem_tmp_T1+640*2+i*1280*2, (unsigned char *)dst_streams+i*1920*2+1280*2, 640*2);

		} else {
			for(i =0;i<720;i++)
				memcpy((unsigned char *)mem_tmp_T2+i*1280*2,(unsigned char *)dst_streams+1920*2*i,1280*2);
			for(i=720;i<1080;i++)
				memcpy((unsigned char *)mem_tmp_T3+1280*2*(i-720), (unsigned char *)dst_streams+1920*2*i, 1280*2);
			for(i=0;i<360;i++)
				memcpy((unsigned char *)mem_tmp_T3+1280*2*(360+i), (unsigned char *)dst_streams+i*1920*2+1280*2, 640*2);
			for(i=360;i<720;i++)
				memcpy((unsigned char *)mem_tmp_T3+640*2+i*1280*2, (unsigned char *)dst_streams+i*1920*2+1280*2, 640*2);

		}

		/* queue-in buffer */
		if(-1 == xioctl(fd, VIDIOC_QBUF, &buf_in_while_loop)){
			errno_exit("VIDIOC_QBUF");
		}
	}
	return 0;
}
#endif 
void video_capture_streams(unsigned char* mem_video_front, unsigned char* mem_video_back, unsigned char* mem_video_left, unsigned char* mem_video_right)
{
	if( !video_capture() ) //get camera 1,2 stream
	{
//		printf("get camera 1,2 stream\n");
	}
	if( !video_capture() ) //get camera 3,4 stream
	{
//		printf("get camera 3,4 stream\n");
	}

	memcpy((unsigned char*)mem_video_front, (unsigned char *)mem_tmp_T0, 1280*720*2);
	memcpy((unsigned char*)mem_video_back, (unsigned char *)mem_tmp_T1, 1280*720*2);
	memcpy((unsigned char*)mem_video_left, (unsigned char *)mem_tmp_T2, 1280*720*2);
	for(int i =0;i<720;i++)
		memcpy((unsigned char*)mem_video_right+i*1280*2,(unsigned char *)mem_tmp_T3+1280*2*i,1280*2);
}

void free_video_capture(){
	stop_capturing();
	uninit_device();
	close_device();
}
Int32_t  load_config_file(const char* base_path, CvPoint1& car_up_left ,CvPoint1& car_down_right,
						  Int32_t& front_fov_height,  Int32_t& back_fov_height,
						  Int32_t& left_fov_width,Int32_t& right_fov_width)
{
	Int32_t ret = 0;
	char str[256];

	char path[1024];
    char* ptr;

	base_dir = base_path;
	
	FILE* p_config_file = fopen((base_dir + DATA_FILES["panorama_config"]).c_str(), "rt");
	fgets(str, 256, p_config_file);
	fscanf(p_config_file, "%d\n", &car_up_left.y);
	fscanf(p_config_file, "%d\n", &car_up_left.x);
	fscanf(p_config_file, "%d\n", &car_down_right.y);
	fscanf(p_config_file, "%d\n", &car_down_right.x);

	fgets(str, 256, p_config_file);
	fscanf(p_config_file, "%d\n", &front_fov_height);
	fscanf(p_config_file, "%d\n", &back_fov_height);
	fscanf(p_config_file, "%d\n", &left_fov_width);
	fscanf(p_config_file, "%d\n", &right_fov_width);
	fclose(p_config_file);
	return ret;
}

void ReadOrigialmage(void)
{
	int i= 0;
	char *str;
	FILE* fp_front = NULL;
	FILE* fp_back = NULL;
	FILE* fp_left = NULL;
	FILE* fp_right = NULL;
	printf("\nread 4 image begin\n");

	if((fp_front = fopen((base_dir + DATA_FILES["front"]).c_str(), "rb")) == NULL)
	{
		printf("\nFail to fopen front.yuv\n");
	}
	else
	{
		fread(front_puyvy, sizeof(unsigned char), size_img, fp_front);
		fflush(fp_front);
		fclose(fp_front);
	}

	if ((fp_back = fopen((base_dir + DATA_FILES["back"]).c_str(), "rb")) == NULL)
	{
		printf("Fail to fopen back.yuv\n");
	}
	else
	{
		fread(back_puyvy, sizeof(unsigned char), size_img, fp_back);
		fflush(fp_back);
		fclose(fp_back);
	}

	if ((fp_left = fopen((base_dir + DATA_FILES["left"]).c_str(), "rb")) == NULL)
	{
		printf("Fail to fopen left.yuv\n");
	}
	else
	{
		fread(left_puyvy, sizeof(unsigned char), size_img, fp_left);
		fflush(fp_left);
		fclose(fp_left);
	}

	if ((fp_right = fopen((base_dir + DATA_FILES["right"]).c_str(), "rb")) == NULL)
	{
		printf("Fail to fopen right.yuv\n");
	}
	else
	{
		fread(right_puyvy, sizeof(unsigned char), size_img, fp_right);
		fflush(fp_right);
		fclose(fp_right);
	}
	printf("read 4 image end\n");

	//   for (i = 0; i < IMG_HEIGHT; i++) //get each row  address of every camera data
	///  {
	///      CAPTURE_MEM_FRONT_cpy[i] = (unsigned int*)(&front_p[i * IMG_WIDTH * 2]);
	///      CAPTURE_MEM_back_cpy[i] = (unsigned int*)(&back_p[i * IMG_WIDTH * 2]);
	///      CAPTURE_MEM_left_cpy[i] = (unsigned int*)(&left_p[i * IMG_WIDTH * 2]);
	///      CAPTURE_MEM_right_cpy[i] = (unsigned int*)(&right_p[i * IMG_WIDTH * 2]);
	//  }
	printf("get each row  address success\n");

}


void ReadLut(void)
{
	int i;
	FILE* filelut;
	FILE * fp_fsvlut_file;



	///read single view resize of lut
	if((fp_fsvlut_file= fopen((base_dir + DATA_FILES["Lut_Front_Single_View"]).c_str(), "rb"))== NULL){
		printf("Lut_Front_Single_View.bin was not opened\n");
	}

	else	{// front single view lut
		fread(Lut_Fsv_View,sizeof(unsigned int),LUT_FSV_VIEW,fp_fsvlut_file);    //SINGLEVIEW_SIZES
		fflush(fp_fsvlut_file);
		fclose(fp_fsvlut_file);
		printf("Lut_Front_Single_View.bin success !\n");
	}


	if((fp_fsvlut_file= fopen((base_dir + DATA_FILES["Lut_Front"]).c_str(), "rb"))== NULL){
		printf("Lut_Front.bin was not opened\n");
	}

	else	{
		fread(Lut_Front,sizeof(unsigned int),LUT_POS_FB,fp_fsvlut_file);    //SINGLEVIEW_SIZES 660608
		fflush(fp_fsvlut_file);
		fclose(fp_fsvlut_file);
		printf("Lut_Front.bin  success !\n");
	}

	if((fp_fsvlut_file= fopen((base_dir + DATA_FILES["Lut_Back"]).c_str(), "rb"))== NULL){
		printf("Lut_Back.bin was not opened\n");
	}

	else	{
		fread(Lut_Back,sizeof(unsigned int),LUT_POS_FB,fp_fsvlut_file);    //SINGLEVIEW_SIZES 658944
		fflush(fp_fsvlut_file);
		fclose(fp_fsvlut_file);
		printf("Lut_Back.bin  success !\n");
	}

	if((fp_fsvlut_file= fopen((base_dir + DATA_FILES["Lut_Left"]).c_str(), "rb"))== NULL){
		printf("Lut_Left.bin was not opened\n");
	}

	else	{
		fread(Lut_Left,sizeof(unsigned int),LUT_POS_LR,fp_fsvlut_file);    //SINGLEVIEW_SIZES 722944
		fflush(fp_fsvlut_file);
		fclose(fp_fsvlut_file);
		printf("Lut_Left.bin success !\n");
	}

	if((fp_fsvlut_file= fopen((base_dir + DATA_FILES["Lut_Right"]).c_str(), "rb"))== NULL){
		printf("Lut_Right.binwas not opened\n");
	}

	else	{
		fread(Lut_Right,sizeof(unsigned int),LUT_POS_LR,fp_fsvlut_file);    //SINGLEVIEW_SIZES 720896
		fflush(fp_fsvlut_file);
		fclose(fp_fsvlut_file);
		printf("Lut_Right.bin   success !\n");
	}




	if((fp_fsvlut_file= fopen((base_dir + DATA_FILES["Wt_Front"]).c_str(), "rb"))== NULL){
		printf("Wt_Front.binwas not opened\n");
	}

	else
	{
		fread(Wt_Lut_Front,sizeof(UInt64_t),LUT_WT_FB,fp_fsvlut_file);    //SINGLEVIEW_SIZES
		fflush(fp_fsvlut_file);
		fclose(fp_fsvlut_file);
		printf("Wt_Front.bin   success !\n");
	}

	if((fp_fsvlut_file= fopen((base_dir + DATA_FILES["Wt_Back"]).c_str(), "rb"))== NULL){
		printf("Wt_Back.binwas not opened\n");
	}

	else	{
		fread(Wt_Lut_Back,sizeof(UInt64_t),LUT_WT_FB,fp_fsvlut_file);    //SINGLEVIEW_SIZES
		fflush(fp_fsvlut_file);
		fclose(fp_fsvlut_file);
		printf("Wt_Back.bin   success !\n");
	}

	if((fp_fsvlut_file= fopen((base_dir + DATA_FILES["Wt_Left"]).c_str(), "rb"))== NULL){
		printf("Wt_Left.binwas not opened\n");
	}

	else	{
		fread(Wt_Lut_Left,sizeof(UInt64_t),LUT_WT_LR,fp_fsvlut_file);    //SINGLEVIEW_SIZES
		fflush(fp_fsvlut_file);
		fclose(fp_fsvlut_file);
		printf("Wt_Left.bin   success !\n");
	}

	if((fp_fsvlut_file= fopen((base_dir + DATA_FILES["Wt_Right"]).c_str(), "rb"))== NULL){
		printf("Wt_Right.binwas not opened\n");
	}

	else	{
		fread(Wt_Lut_Right,sizeof(UInt64_t),LUT_WT_LR,fp_fsvlut_file);    //SINGLEVIEW_SIZES
		fflush(fp_fsvlut_file);
		fclose(fp_fsvlut_file);
		printf("Wt_Right.bin   success !\n");
	}

	p_lut_front_test=Lut_Front;
	p_wt_front_test = (unsigned int *)Wt_Lut_Front;

	p_lut_back_test=Lut_Back;
	p_wt_back_test = (unsigned int *)Wt_Lut_Back;

	p_lut_left_test=Lut_Left;
	p_wt_left_test =(unsigned int *) Wt_Lut_Left;

	p_lut_right_test=Lut_Right;
	p_wt_right_test = (unsigned int *)Wt_Lut_Right;

	///TabBev **bev_Table[4];

	Bev_Tab_Init(p_lut_front_test, p_wt_front_test, 0, DST_WIDTH, car_up_left.y, bev_Table);
	Bev_Tab_Init(p_lut_back_test, p_wt_back_test, 1, DST_WIDTH, car_up_left.y, bev_Table);
	Bev_Tab_Init(p_lut_left_test, p_wt_left_test, 2, car_up_left.x , DST_HIGHT, bev_Table);
	Bev_Tab_Init(p_lut_right_test, p_wt_right_test, 3, car_up_left.x , DST_HIGHT, bev_Table);

}
Int32_t Bev_Tab_Init(UInt32_t* p_pos_fusion_lut, UInt32_t* p_wt_lut, Int32_t camid, Int32_t fov_width, Int32_t fov_height, TabBev **bev_Table[4])
{
	Int32_t wt_upleft, wt_upright, wt_downleft, wt_downright;
	UInt32_t item, wt_item,i,j;

	bev_Table[camid] = (TabBev**)malloc(sizeof(TabBev*)* fov_height);
	for ( i = 0; i < fov_height; i++)
	{
		TabBev* p = (TabBev*)malloc(sizeof(TabBev)* fov_width);
		bev_Table[camid][i] = p;
	}
	if (bev_Table == NULL)
	{
		printf("malloc error");
	}
	for ( i = 0; i < fov_height; i++)
	{
		for ( j = 0; j < fov_width; j++)
		{
			item = p_pos_fusion_lut[i * fov_width + j];
			bev_Table[camid][i][j].point_pos.x = (item >> 10) & 0x7FF;
			bev_Table[camid][i][j].point_pos.y = (item >> 21);
			bev_Table[camid][i][j].wt_fusion = item & 0x3FF;

			wt_item = p_wt_lut[2 * i * fov_width + 2 * j];
			bev_Table[camid][i][j].wt_upleft = wt_item >> 16;
			bev_Table[camid][i][j].wt_upright = wt_item & 0xFFFF;
			wt_item = p_wt_lut[2 * i * fov_width + 2 * j + 1];
			bev_Table[camid][i][j].wt_downleft = wt_item >> 16;
			bev_Table[camid][i][j].wt_downright = wt_item & 0xFFFF;
		}
	}
	return 0;
}

Int32_t bev_process(
		uchar* result_image,
		uchar* front_image_uyvy,
		uchar* back_image_uyvy,
		uchar* left_image_uyvy,
		uchar* right_image_uyvy,
		CvPoint1 car_up_left,
		CvPoint1 car_down_right,
		Int32_t frontback_fov_height,
		Int32_t leftright_fov_width,
		Int32_t result_width,
		Int32_t source_width,
		TabBev **bev_Table[4])
{
	Int32_t ret = 0;
	uchar* p_src, *p_src1, *p_src2;
	memset(result_image, 0, DST_WIDTH * DST_HIGHT * 2);
	Int32_t yuv[3];
	Int32_t result_widthstep, src_widthstep;
	CvRect1 region_roi;
	Int32_t camid, fusion_pic1, fusion_pic2;

	result_widthstep = result_width << 1;
	src_widthstep = source_width << 1;

	UInt32_t loop_count = 0;
//for yuyv to uyvy format

	yuyv_to_uyvy(mem_video_frontuyvy,front_image_uyvy,IMG_WIDTH, IMG_HEIGHT);
	yuyv_to_uyvy(mem_video_backuyvy,back_image_uyvy,IMG_WIDTH, IMG_HEIGHT);
	yuyv_to_uyvy(mem_video_leftuyvy,left_image_uyvy,IMG_WIDTH, IMG_HEIGHT);
	yuyv_to_uyvy(mem_video_rightuyvy,right_image_uyvy,IMG_WIDTH, IMG_HEIGHT);
	front_image_uyvy = mem_video_frontuyvy;
	back_image_uyvy = mem_video_backuyvy;
	left_image_uyvy =mem_video_leftuyvy;
	right_image_uyvy = mem_video_rightuyvy;


//	Float64_t total_time_1 = 0, total_time_2 = 0;
	while (loop_count < 1)
	{
///#ifdef TIME_CONSUME
///		time_t  start_time = clock();
///#endif
		//front left
		p_src1 = front_image_uyvy;
		p_src2 = left_image_uyvy;
		fusion_pic1 = 0;
		fusion_pic2 = 2;
		region_roi = cvRect1(0, 0, car_up_left.x, car_up_left.y);
		analysis_fusion_region_lut_uyvy(result_image, p_src1, p_src2, fusion_pic1, fusion_pic2, &region_roi, result_widthstep, src_widthstep, bev_Table);

#ifdef TIME_CONSUME
		time_t  time1_1 = clock();
		double time0_1 = (double)(clock() - start_time);
		//printf("image_result_test cost :%f\n", time0_1);
		total_time_1 += time0_1;
#endif
		loop_count++;
	}
//	printf("front left1 \n");

///    time_t time1_1 = clock();

	//front
	region_roi = cvRect1(car_up_left.x, 0, car_down_right.x - car_up_left.x + 1, frontback_fov_height);
	camid = 0;
	p_src = front_image_uyvy;
	analysis_single_region_lut_uyvy(result_image, p_src, &region_roi, camid, result_widthstep, src_widthstep, bev_Table);

//	printf("front left2\n");
#ifdef TIME_CONSUME
	time_t  time1_2 = clock();
	double time0_1 = (double)(clock() - time1_1);
	printf("front cost :%f\n", time0_1);
#endif
///    time1_1 = clock();

	//front right
	p_src1 = front_image_uyvy;
	p_src2 = right_image_uyvy;
	fusion_pic1 = 0;
	fusion_pic2 = 3;
	region_roi = cvRect1(car_down_right.x + 1, 0, car_up_left.x, car_up_left.y);
	analysis_fusion_region_lut_uyvy(result_image, p_src1, p_src2, fusion_pic1, fusion_pic2, &region_roi, result_widthstep, src_widthstep, bev_Table);
//	printf("front right3 \n");
#ifdef TIME_CONSUME
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);
	printf("front right cost :%f\n", time0_1);
#endif
///    time1_1 = clock();

	//left
	region_roi = cvRect1(0, car_up_left.y, car_up_left.x, car_down_right.y - car_up_left.y + 1);
	analysis_single_region_lut_uyvy(result_image, left_image_uyvy, &region_roi, 2, result_widthstep, src_widthstep, bev_Table);
//	printf(" left4 \n");
#ifdef TIME_CONSUME
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);
	printf("left cost :%f\n", time0_1);
#endif
///    time1_1 = clock();

	//right
	region_roi = cvRect1(car_down_right.x + 1, car_up_left.y, result_width - car_down_right.x - 1, car_down_right.y - car_up_left.y + 1);
	analysis_single_region_lut_uyvy(result_image, right_image_uyvy, &region_roi, 3, result_widthstep, src_widthstep, bev_Table);
//	printf("right5 \n");

#ifdef TIME_CONSUME
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);
	printf("right cost :%f\n", time0_1);
#endif


///    time1_1 = clock();
	//back left
	p_src1 = left_image_uyvy;
	p_src2 = back_image_uyvy;
	fusion_pic1 = 2;
	fusion_pic2 = 1;
//	printf("right55 \n");
	region_roi = cvRect1(0, car_down_right.y + 1, car_up_left.x, frontback_fov_height);
//	printf("right56 \n");
	analysis_fusion_region_lut_uyvy(result_image, p_src1, p_src2, fusion_pic1, fusion_pic2, &region_roi, result_widthstep, src_widthstep, bev_Table);
//	printf("back 6\n");

#ifdef TIME_CONSUME
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);
	printf("back left cost :%f\n", time0_1);
#endif
	//back
	region_roi = cvRect1(car_up_left.x, car_down_right.y + 1, car_down_right.x - car_up_left.x + 1, frontback_fov_height);
	analysis_single_region_lut_uyvy(result_image, back_image_uyvy, &region_roi, 1, result_widthstep, src_widthstep, bev_Table);

///    time1_1 = clock();
	//back right

	p_src1 = right_image_uyvy;
	p_src2 = back_image_uyvy;
	fusion_pic1 = 3;
	fusion_pic2 = 1;
	region_roi = cvRect1(car_down_right.x + 1, car_down_right.y + 1, car_up_left.x, frontback_fov_height);
	analysis_fusion_region_lut_uyvy(result_image, p_src1, p_src2, fusion_pic1, fusion_pic2, &region_roi, result_widthstep, src_widthstep, bev_Table);


#ifdef TIME_CONSUME
	time1_2 = clock();
	time0_1 = (double)(clock() - time1_1);
	printf("back right cost :%f\n", time0_1);
#endif

	return ret;
}

CvRect1  cvRect1( int x, int y, int width, int height )
{
	CvRect1 r;

	r.x = x;
	r.y = y;
	r.width = width;
	r.height = height;

	return r;
}

void analysis_fusion_region_lut_uyvy(uchar* result_image,
									 uchar* p_src1,
									 uchar* p_src2,
									 Int32_t fusion_pic1,
									 Int32_t fusion_pic2,
									 CvRect1* region_roi,
									 Int32_t result_widthstep,
									 Int32_t src_widthstep,
									 TabBev **bev_Table[4])
{
	Int32_t yuv[2], x, y, row, col,i,j;
	if (fusion_pic1 == 0 && fusion_pic2 == 2) //front_left
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				x = bev_Table[fusion_pic1][i][j].point_pos.x;
				y = bev_Table[fusion_pic1][i][j].point_pos.y;
				col = j - region_roi->x;
				row = i - region_roi->y;

				if (bev_Table[fusion_pic1][i][j].wt_fusion == 1023)
				{
					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upleft
							   + p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upright
							   + p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downleft
							   + p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downright) >> 16)
							;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
							;
				}
				else if (bev_Table[fusion_pic2][row][col].wt_fusion == 1023)
				{
					x = bev_Table[fusion_pic2][row][col].point_pos.x;
					y = bev_Table[fusion_pic2][row][col].point_pos.y;
					yuv[0] = ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upleft
							   + p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upright
							   + p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downleft
							   + p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downright) >> 16)
							;

					yuv[1] = ((p_src2[y * src_widthstep + x * 2]))
							;
				}
				else  // fusion region
				{
					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upleft
							   + p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upright
							   + p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downleft
							   + p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downright) >> 16)
							 * bev_Table[fusion_pic1][i][j].wt_fusion >> 10;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
							 * bev_Table[fusion_pic1][i][j].wt_fusion >> 10;


					x = bev_Table[fusion_pic2][row][col].point_pos.x;
					y = bev_Table[fusion_pic2][row][col].point_pos.y;
					yuv[0] += ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upleft
								+ p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upright
								+ p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downleft
								+ p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downright) >> 16)
							  * bev_Table[fusion_pic2][row][col].wt_fusion >> 10;
					yuv[1] += ((p_src2[y * src_widthstep + x * 2]))
							  * bev_Table[fusion_pic2][row][col].wt_fusion >> 10;
				}

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
	else if ((fusion_pic1 == 0 && fusion_pic2 == 3) || (fusion_pic1 == 2 && fusion_pic2 == 1)) //front_right  and  left_back
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				if (bev_Table[fusion_pic1][i][j].wt_fusion == 1023)
				{
					x = bev_Table[fusion_pic1][i][j].point_pos.x;
					y = bev_Table[fusion_pic1][i][j].point_pos.y;
					//col = j - region_roi->x;
					//row = i - region_roi->y;

					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upleft
							   + p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upright
							   + p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downleft
							   + p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downright) >> 16)
							;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
							;
				}
				else if (bev_Table[fusion_pic1][i][j].wt_fusion == 0)
				{
					col = j - region_roi->x;
					row = i - region_roi->y;
					x = bev_Table[fusion_pic2][row][col].point_pos.x;
					y = bev_Table[fusion_pic2][row][col].point_pos.y;

					yuv[0] = ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upleft
							   + p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upright
							   + p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downleft
							   + p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downright) >> 16)
							;

					yuv[1] = ((p_src2[y * src_widthstep + x * 2]))
							;
				}
				else
				{
					x = bev_Table[fusion_pic1][i][j].point_pos.x;
					y = bev_Table[fusion_pic1][i][j].point_pos.y;
					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upleft
							   + p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_upright
							   + p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downleft
							   + p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][j].wt_downright) >> 16)
							 * bev_Table[fusion_pic1][i][j].wt_fusion >> 10;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
							 * bev_Table[fusion_pic1][i][j].wt_fusion >> 10;

					col = j - region_roi->x;
					row = i - region_roi->y;
					x = bev_Table[fusion_pic2][row][col].point_pos.x;
					y = bev_Table[fusion_pic2][row][col].point_pos.y;
					yuv[0] += ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upleft
								+ p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_upright
								+ p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downleft
								+ p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][col].wt_downright) >> 16)
							  * bev_Table[fusion_pic2][row][col].wt_fusion >> 10;
					yuv[1] += ((p_src2[y * src_widthstep + x * 2]))
							  * bev_Table[fusion_pic2][row][col].wt_fusion >> 10;
				}

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
	else   //right_back
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				if (bev_Table[fusion_pic1][i][col = j - region_roi->x].wt_fusion == 1023)
				{
					col = j - region_roi->x;
					//row = i - region_roi->y;
					x = bev_Table[fusion_pic1][i][col].point_pos.x;
					y = bev_Table[fusion_pic1][i][col].point_pos.y;

					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_upleft
							   + p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_upright
							   + p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_downleft
							   + p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_downright) >> 16)
							;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
							;
				}
				else if (bev_Table[fusion_pic1][i][col = j - region_roi->x].wt_fusion == 0)
				{
					//col = j - region_roi->x;
					row = i - region_roi->y;
					x = bev_Table[fusion_pic2][row][j].point_pos.x;
					y = bev_Table[fusion_pic2][row][j].point_pos.y;

					yuv[0] = ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_upleft
							   + p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_upright
							   + p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_downleft
							   + p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_downright) >> 16)
							;

					yuv[1] = ((p_src2[y * src_widthstep + x * 2]))
							;
				}
				else
				{
					col = j - region_roi->x;
					x = bev_Table[fusion_pic1][i][col].point_pos.x;
					y = bev_Table[fusion_pic1][i][col].point_pos.y;
					yuv[0] = ((p_src1[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_upleft
							   + p_src1[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_upright
							   + p_src1[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_downleft
							   + p_src1[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic1][i][col].wt_downright) >> 16)
							 * bev_Table[fusion_pic1][i][col].wt_fusion >> 10;

					yuv[1] = ((p_src1[y * src_widthstep + x * 2]))
							 * bev_Table[fusion_pic1][i][col].wt_fusion >> 10;

					//col = j - region_roi->x;
					row = i - region_roi->y;
					x = bev_Table[fusion_pic2][row][j].point_pos.x;
					y = bev_Table[fusion_pic2][row][j].point_pos.y;
					yuv[0] += ((p_src2[y * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_upleft
								+ p_src2[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_upright
								+ p_src2[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_downleft
								+ p_src2[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[fusion_pic2][row][j].wt_downright) >> 16)
							  * bev_Table[fusion_pic2][row][j].wt_fusion >> 10;
					yuv[1] += ((p_src2[y * src_widthstep + x * 2]))
							  * bev_Table[fusion_pic2][row][j].wt_fusion >> 10;
				}

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
}




void analysis_single_region_lut_uyvy(
		uchar* result_image, uchar* p_src,
		CvRect1* region_roi,
		Int32_t camid,
		Int32_t result_widthstep,
		Int32_t src_widthstep,
		TabBev **bev_Table[4])
{
	Int32_t yuv[2], x, y, row, col,i,j;
	if (camid == 0 || camid == 2) //0:front , 2:left
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				x = bev_Table[camid][i][j].point_pos.x;
				y = bev_Table[camid][i][j].point_pos.y;

				yuv[0] = ((p_src[y * src_widthstep + x * 2 + 1] * bev_Table[camid][i][j].wt_upleft
						   + p_src[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][i][j].wt_upright
						   + p_src[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[camid][i][j].wt_downleft
						   + p_src[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][i][j].wt_downright) >> 16)
						;

				yuv[1] = ((p_src[y * src_widthstep + x * 2]))
						;

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
	else if (camid == 3)  //right
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				//row = i - region_roi->y;
				col = j - region_roi->x;
				x = bev_Table[camid][i][col].point_pos.x;
				y = bev_Table[camid][i][col].point_pos.y;

				yuv[0] = ((p_src[y * src_widthstep + x * 2 + 1] * bev_Table[camid][i][col].wt_upleft
						   + p_src[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][i][col].wt_upright
						   + p_src[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[camid][i][col].wt_downleft
						   + p_src[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][i][col].wt_downright) >> 16)
						;

				yuv[1] = ((p_src[y * src_widthstep + x * 2]))
						;

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
	else    //back
	{
		for ( i = region_roi->y; i < region_roi->height + region_roi->y; i++)
		{
			for ( j = region_roi->x; j < region_roi->width + region_roi->x; j++)
			{
				row = i - region_roi->y;
				//col = j - region_roi->x;
				x = bev_Table[camid][row][j].point_pos.x;
				y = bev_Table[camid][row][j].point_pos.y;

				yuv[0] = ((p_src[y * src_widthstep + x * 2 + 1] * bev_Table[camid][row][j].wt_upleft
						   + p_src[y * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][row][j].wt_upright
						   + p_src[(y + 1) * src_widthstep + x * 2 + 1] * bev_Table[camid][row][j].wt_downleft
						   + p_src[(y + 1) * src_widthstep + (x + 1) * 2 + 1] * bev_Table[camid][row][j].wt_downright) >> 16)
						;

				yuv[1] = ((p_src[y * src_widthstep + x * 2]))
						;

				result_image[(i)* result_widthstep + (j)* 2 + 1] += yuv[0];
				result_image[(i)* result_widthstep + (j)* 2] += yuv[1];
			}
		}
	}
}

int  saveframe(char *str, void *p, int length, int is_oneframe)
{
	FILE *fd;

	if (is_oneframe) {
		fd = fopen(str, "wb");
	}
	else {
		fd = fopen(str, "a");
	}

	if (!fd) {
		printf("Open file error\n");
		return -1;
	}
	if (fwrite(p, 1, length, fd)) {
		printf("Write file successfully\n");
		fclose(fd);
		return 0;
	}
	else {
		printf("Write file fail\n");
		fclose(fd);
		return -1;
	}
}




void rotate270(unsigned char *src,unsigned char *dst,int width,int height) {
	int copyBytes = 4;
	int bytesPerLine = width << 1;
	int step = height << 2;
	int offset = bytesPerLine - copyBytes;

	unsigned char *dest = dst;
	unsigned char *source = src;
	unsigned char *psrc = NULL;
	unsigned char *pdst[2] = {NULL, NULL};
	int i, j, k;
	unsigned char temp;

	for (i = 0; i < bytesPerLine; i += copyBytes) {
		pdst[1] = dest;
		pdst[0] = dest + (height << 1);
		psrc = source + offset;

		for (j = 0; j < height; ++j) {
			k = j % 2;
			*((unsigned int *) pdst[k]) = *((unsigned int *) psrc);

			if (1 == k) {
				temp = *(pdst[1] + 1);
				*(pdst[1] + 1) = *(pdst[0] - 1);
				*(pdst[0] - 1) = temp;
			}

			pdst[k] += copyBytes;
			psrc += bytesPerLine;
		}

		dest += step;
		source -= copyBytes;


	}
}
