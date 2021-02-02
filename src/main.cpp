#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <stdlib.h>
#include <chrono>
#include <ctime>
#include "libuvc/libuvc.h"
#include <queue>
#include <mutex>
#include <thread>

#define RICOH_VID 0x05ca
#define RICOH_PID 0x2715
#define WIDTH 3840
#define HEIGHT 1920
#define FPS 29

const char* RICOH_DYROS_LEFT_SERIAL = "12100633";
const char* RICOH_DYROS_RIGHT_SERIAL = "12100433";
static ros::Publisher * ricoh_pub;
std::queue<std_msgs::UInt8MultiArray> left_msgs;
std::queue<std_msgs::UInt8MultiArray> right_msgs;
std_msgs::UInt8MultiArray pub_msg;
std::mutex l_m;
std::mutex r_m;

int publish_left_counter = 0;
int publish_right_counter = 0;
int left_counter = 0;
int right_counter = 0;

using namespace std;
using namespace chrono;

system_clock::time_point startTime = system_clock::now();


void publisher(){
	cout << " Start Stereo Publishing.. " << endl;
	while(true){
		l_m.lock();
		r_m.lock();
		//cout << "left msg size : " << left_msgs.size() << "right msg size : " << right_msgs.size()<< endl;
		if(left_msgs.size() >= 1 && right_msgs.size() >= 1){
			std_msgs::UInt8MultiArray left = left_msgs.front();
			std_msgs::UInt8MultiArray right = right_msgs.front();
	
			double capture_diff = (double(left.layout.dim[2].size) + double(left.layout.dim[3].size * 10e-10)) - \
																	(double(right.layout.dim[2].size) + double(right.layout.dim[3].size * 10e-10));
			//cout << "capture diff: " << capture_diff <<", " << 2.0 / FPS <<  ", " << left_msgs.size() << ", " << right_msgs.size() << endl;
			if(left_msgs.size() == right_msgs.size() && abs(capture_diff) < 2.0/FPS){

				pub_msg.layout.dim[3].size = (left_msgs.front()).layout.dim[0].size; // copy left data bytes
				pub_msg.layout.dim[4].size = (right_msgs.front()).layout.dim[0].size; // copy right data bytes
				
				pub_msg.data.resize(pub_msg.layout.dim[3].size + pub_msg.layout.dim[4].size);
				memcpy(&(pub_msg.data[0]), &((left_msgs.front()).data[0]), pub_msg.layout.dim[3].size);
				memcpy(&(pub_msg.data[0]) + pub_msg.layout.dim[3].size, &((right_msgs.front()).data[0]), pub_msg.layout.dim[4].size);	
				
				left_msgs.pop();				
				right_msgs.pop();


				ricoh_pub->publish(pub_msg);
				//counter++;

				cout << "left msg size : " << left_msgs.size() << " right msg size : " << right_msgs.size()<< " capture diff: " <<capture_diff <<endl;
			}
			else if(left_msgs.size() > right_msgs.size()) left_msgs.pop();
			else if(left_msgs.size() < right_msgs.size()) right_msgs.pop();
			
			
		}
		
		r_m.unlock();
		l_m.unlock();
	}

}

void msg_setup(std_msgs::UInt8MultiArray * stream_msg){
	stream_msg->layout.dim.push_back(std_msgs::MultiArrayDimension()); // height
	stream_msg->layout.dim.push_back(std_msgs::MultiArrayDimension()); // width
	stream_msg->layout.dim.push_back(std_msgs::MultiArrayDimension()); // fps
	stream_msg->layout.dim.push_back(std_msgs::MultiArrayDimension()); // left data_bytes
	stream_msg->layout.dim.push_back(std_msgs::MultiArrayDimension()); // right data_bytes



	stream_msg->layout.dim[0].label = "height";
	stream_msg->layout.dim[0].size = HEIGHT;
	stream_msg->layout.dim[1].label = "width";
	stream_msg->layout.dim[1].size = WIDTH;
	stream_msg->layout.dim[2].label = "fps";
	stream_msg->layout.dim[2].size = FPS;
	stream_msg->layout.dim[3].label = "data_bytes";
}


void left_callback(uvc_frame_t * frame, void * ptr){
	uvc_frame_t * bgr;
	uvc_error_t ret;
	std_msgs::UInt8MultiArray left_stream_msg;
	left_stream_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	left_stream_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	left_stream_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	left_stream_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());

	left_stream_msg.layout.dim[0].label = "data_bytes";
	left_stream_msg.layout.dim[0].size = frame->data_bytes;
	left_stream_msg.layout.dim[1].size = left_counter++;
	left_stream_msg.layout.dim[2].size = int((frame->capture_time_finished).tv_sec);
	left_stream_msg.layout.dim[3].size = int((frame->capture_time_finished).tv_nsec);

	uint8_t * frame_ptr = (uint8_t*)(frame->data);;
	left_stream_msg.data.resize(frame->data_bytes);
	memcpy(&(left_stream_msg.data[0]), frame_ptr, frame->data_bytes);
	
	l_m.lock();

	left_msgs.push(left_stream_msg);
	l_m.unlock();	
	//cout << "left: " << int((frame->capture_time_finished).tv_sec) << ", " << (frame->capture_time_finished).tv_nsec * 10e-10 << endl;
	//cout << int((frame->capture_time_finished).tv_sec) << ", " << (frame->capture_time_finished).tv_nsec * 10e-10 << endl;
}


void right_callback(uvc_frame_t * frame, void * ptr){
	uvc_frame_t * bgr;
	uvc_error_t ret;
	std_msgs::UInt8MultiArray right_stream_msg;
	right_stream_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	right_stream_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	right_stream_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	right_stream_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	right_stream_msg.layout.dim[0].label = "data_bytes";
	right_stream_msg.layout.dim[0].size = frame->data_bytes;
	right_stream_msg.layout.dim[1].size = right_counter++;
	right_stream_msg.layout.dim[2].size = int((frame->capture_time_finished).tv_sec);
	right_stream_msg.layout.dim[3].size = int((frame->capture_time_finished).tv_nsec);

	uint8_t * frame_ptr = (uint8_t*)(frame->data);

	right_stream_msg.data.resize(frame->data_bytes);
	memcpy(&(right_stream_msg.data[0]), frame_ptr, frame->data_bytes);
	
	r_m.lock();
	right_msgs.push(right_stream_msg);
	//cout << "right msg size: " << right_msgs.size() << endl; 
	r_m.unlock();
	//cout << "right: " << int((frame->capture_time_finished).tv_sec) << ", " << (frame->capture_time_finished).tv_nsec * 10e-10 << endl;
	//cout << unsigned(frame_ptr[frame->data_bytes]) << endl;
	
}


int main(int argc, char **argv){
	ros::init(argc, argv, "richo");
	ros::NodeHandle richo_nh;
	ros::Publisher _link = richo_nh.advertise<std_msgs::UInt8MultiArray>("/ricoh_h264_stream", 1);
	ricoh_pub = &_link;

	uvc_context_t * left_context;
	uvc_device_t * left_device;
	uvc_device_handle_t * left_device_handle;
	uvc_stream_ctrl_t left_stream_control;
	uvc_error_t left_res;
	
	uvc_context_t * right_context;
	uvc_device_t * right_device;
	uvc_device_handle_t * right_device_handle;
	uvc_stream_ctrl_t right_stream_control;
	uvc_error_t right_res;

	msg_setup(&pub_msg);
	left_res = uvc_init(&left_context, NULL);
	right_res = uvc_init(&right_context, NULL);

	if(left_res != UVC_SUCCESS) {
		uvc_perror(left_res, "uvc_init");
		return left_res;	
	}
	cout << "uvc left context initialized..." << endl;

	if(right_res != UVC_SUCCESS) {
		uvc_perror(right_res, "uvc_init");
		return right_res;	
	}
	
	cout << "uvc right context initialized..." << endl;
	

	cout << "start finding Ricoh theta z1 devices..." << endl;

	left_res = uvc_find_device(left_context, &left_device, RICOH_VID, RICOH_PID, RICOH_DYROS_LEFT_SERIAL);
	right_res = uvc_find_device(right_context, &right_device, RICOH_VID, RICOH_PID, RICOH_DYROS_RIGHT_SERIAL); 

	if(left_res != UVC_SUCCESS){
		uvc_perror(left_res, "uvc_find_device");	
	}else cout << "Left Ricoh found!" << endl;
	
	
	if(right_res != UVC_SUCCESS){
		uvc_perror(right_res, "uvc_find_device");	
	}else cout << "Right Ricoh found!" << endl;
	

	left_res = uvc_open(left_device, &left_device_handle);
	right_res = uvc_open(right_device, &right_device_handle);

	if(left_res != UVC_SUCCESS){
		uvc_perror(left_res, "uvc_open");
		cout << " make sure left Ricoh Theta is turned on , set mode to Live Streaming mode" << endl;
		return left_res;
	}else cout << "left Ricoh is opened" << endl;
	
	if(right_res != UVC_SUCCESS){
		uvc_perror(right_res, "uvc_open");
		cout << " make sure right Ricoh Theta is turned on , set mode to Live Streaming mode" << endl;
		return right_res;
	}else cout << "right Ricoh is opened" << endl;
	

	cout << " [ left Ricoh Device Information ] " << endl;
	uvc_print_diag(left_device_handle, stderr);
	
	cout << " [ right Ricoh Device Information ] " << endl;
	uvc_print_diag(right_device_handle, stderr);
	
	// Ricoh Only provides H.264 streaming mode! MJpeg (x), RGB(x), BGR(x)
	left_res = uvc_get_stream_ctrl_format_size(left_device_handle, &left_stream_control, UVC_FRAME_FORMAT_H264, WIDTH, HEIGHT, FPS);
	right_res = uvc_get_stream_ctrl_format_size(right_device_handle, &right_stream_control, UVC_FRAME_FORMAT_H264, WIDTH, HEIGHT, FPS);
	

	uvc_print_stream_ctrl(&left_stream_control, stderr);
	uvc_print_stream_ctrl(&right_stream_control, stderr);
	
	if(left_res != UVC_SUCCESS || right_res != UVC_SUCCESS){
		uvc_perror(left_res, "get_mode");
		cout << "Device doesn't provide according stream setting, modify your setting" << endl;
	}else{
		// Start video stream. call callback function	
		void * tmp_;
		void * tmp__;
		left_res = uvc_start_streaming(left_device_handle, &left_stream_control, left_callback, tmp_, 0);
 		right_res = uvc_start_streaming(right_device_handle, &right_stream_control, right_callback, tmp__, 0);
		
		if(left_res != UVC_SUCCESS || right_res != UVC_SUCCESS){
			if(left_res != UVC_SUCCESS) cout << "Left failed.." << endl;
			if(right_res != UVC_SUCCESS) cout << "Right failed.." << endl;
			uvc_perror(right_res, "start_streaming");	
			cout << "streaming failed.." << endl;	
		}else{
			cout << "Start streaming through ROS node.." << endl;	
			thread pub(publisher);
			sleep(5000000000000000000);	
		}
	}


	

	return 0;
}
