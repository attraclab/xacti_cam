/*
	This program is made by Rasheed Kittinanthapanya, 
	Robot Developer and CTO from ATTRACLAB.
	Some part of libuvc is from example of libuvc repository 
	and sample code from Xacti's engineer.

*/

#include <memory>
#include <chrono>
#include <stdio.h>
#include <cassert>
#include <iostream>
#include <type_traits>
#include <math.h>
#include <libusb-1.0/libusb.h>
#include <libuvc/libuvc.h>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <cstdint>

#define VID_WIDTH  1920
#define VID_HEIGHT 1080
#define VIDEO_OUT "/dev/video30"

using namespace std::chrono_literals;

/// Global variables ///
struct uvc_context;
typedef struct uvc_context uvc_context_t;
struct uvc_device;
typedef struct uvc_device uvc_device_t;
struct uvc_device_handle;
typedef struct uvc_device_handle uvc_device_handle_t;
struct uvc_frame;
typedef struct uvc_frame uvc_frame_t;
static cv::Mat f_rgb;
int64_t last_frame_stamp;


/// Callback function to get frame from libuvc ///
void cb(uvc_frame_t *frame, void *ptr){

	uvc_frame_t *rgb_frame;
  	uvc_error_t ret;
  	enum uvc_frame_format *frame_format = (enum uvc_frame_format *)ptr;

	rgb_frame = uvc_allocate_frame(frame->width * frame->height * 3);
	if (!rgb_frame)
	{
	    printf("unable to allocate bgr frame!");
	    return;
	}

	// JPEG→RGB
	ret = uvc_mjpeg2rgb(frame, rgb_frame);
	if (ret)
	{
	    uvc_perror(ret, "uvc_mjpeg2rgb");
	    uvc_free_frame(rgb_frame);
	    printf("uvc_mjpeg2rgb ERROR!");
	    return;
	}

	cv::Mat rgb_mat(rgb_frame->height, rgb_frame->width, CV_8UC3, rgb_frame->data);
	f_rgb = rgb_mat.clone();

	uvc_free_frame(rgb_frame);
	// printf("callback \n");
	// last_frame_stamp = std::chrono::microseconds(1).count(); //std::chrono::high_resolution_clock::now();
	last_frame_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

}


class CameraControl : public rclcpp::Node {

	public:

		/// Parameter callback ///
		rcl_interfaces::msg::SetParametersResult parametersCallback(
	        const std::vector<rclcpp::Parameter> &parameters)
	    {
	        rcl_interfaces::msg::SetParametersResult result;
	        result.successful = true;
	        result.reason = "success";
	        for (const auto &parameter : parameters)
	        {
	            if ((parameter.get_name() == "local_play") && (parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)){
	                this->local_play = parameter.as_bool();
	                RCLCPP_INFO(this->get_logger(), "Parameter 'local_play' changed: %d", this->local_play);
	                if (this->local_play == false){
	                	cv::destroyAllWindows();
	                }
	            }
	        }
	        return result;
	    }

	    /// Class constructor ///
		CameraControl() : Node("xacti_camera_control") {

			RCLCPP_INFO(this->get_logger(), "start camera_control");

			bool ret = this->uvcInit(NULL);

			/// ROS params ///
			this->declare_parameter("local_play", false);

			this->local_play = this->get_parameter("local_play").as_bool();

			param_callback_handle_ = this->add_on_set_parameters_callback(
				std::bind(&CameraControl::parametersCallback, this, std::placeholders::_1));

			/// Pub/Sub ///
			gimbal_enable_sub = this->create_subscription<std_msgs::msg::Bool>
			("/xacti/gimbal/restart", 10, std::bind(&CameraControl::gimbal_enable_callback, this, std::placeholders::_1));
			gimbal_pan_sub = this->create_subscription<std_msgs::msg::Int16>
			("/xacti/gimbal/pan", 10, std::bind(&CameraControl::gimbal_pan_callback, this, std::placeholders::_1));
			gimbal_tilt_sub = this->create_subscription<std_msgs::msg::Int16>
			("/xacti/gimbal/tilt", 10, std::bind(&CameraControl::gimbal_tilt_callback, this, std::placeholders::_1));

			take_photo_sub = this->create_subscription<std_msgs::msg::Bool>
			("/xacti/camera/photo_capture", 10, std::bind(&CameraControl::take_photo_callback, this, std::placeholders::_1));
			camera_orientation_sub = this->create_subscription<std_msgs::msg::Int8>
			("/xacti/camera/orientation", 10, std::bind(&CameraControl::camera_orientation_callback, this, std::placeholders::_1));
			camera_focus_mode_sub = this->create_subscription<std_msgs::msg::Int8>
			("/xacti/camera/focus_mode", 10, std::bind(&CameraControl::camera_focus_mode_callback, this, std::placeholders::_1));
			camera_focus_sub = this->create_subscription<std_msgs::msg::Int32>
			("/xacti/camera/focus_mm", 10, std::bind(&CameraControl::camera_focus_callback, this, std::placeholders::_1));
			camera_recording_sub = this->create_subscription<std_msgs::msg::Bool>
			("/xacti/camera/video_record", 10, std::bind(&CameraControl::camera_recording_callback, this, std::placeholders::_1));
			camera_video_res_sub = this->create_subscription<std_msgs::msg::Int8>
			("/xacti/camera/video_resolution", 10, std::bind(&CameraControl::camera_video_res_callback, this, std::placeholders::_1));
			optical_zoom_sub = this->create_subscription<std_msgs::msg::Int32>
			("/xacti/camera/optical_zoom", 10, std::bind(&CameraControl::camera_optical_zoom_callback, this, std::placeholders::_1));
			camera_iso_sub = this->create_subscription<std_msgs::msg::Int8>
			("/xacti/camera/iso", 10, std::bind(&CameraControl::camera_iso_callback, this, std::placeholders::_1));
			camera_aperture_sub = this->create_subscription<std_msgs::msg::Int8>
			("/xacti/camera/aperture", 10, std::bind(&CameraControl::camera_aperture_callback, this, std::placeholders::_1));
			
			rmw_qos_profile_t custom_qos = rmw_qos_profile_default; //rmw_qos_profile_default; //rmw_qos_profile_sensor_data;
	        image_pub = image_transport::create_publisher(this, "/xacti/camera/view", custom_qos); 
			
	        /// Start streaming and setup output video ///
			if (ret){
				/// start stream from libuvc
				this->startStream();

				/// setup video output
				this->initVideoOut();
				
			}

			RCLCPP_INFO(this->get_logger(), "======== Following Topics are subscribed =========");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/orientation      [std_msgs/msg/Int8]");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/photo_capture    [std_msgs/msg/Bool]");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/focus_mode       [std_msgs/msg/Int8]");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/focus_mm         [std_msgs/msg/Int32]");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/video_record     [std_msgs/msg/Bool]");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/video_resolution [std_msgs/msg/Int8]");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/optical_zoom     [std_msgs/msg/Int32]");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/iso              [std_msgs/msg/Int8]");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/aperture         [std_msgs/msg/Int8]");
			RCLCPP_INFO(this->get_logger(), "/xacti/gimbal/restart          [std_msgs/msg/Bool]");
			RCLCPP_INFO(this->get_logger(), "/xacti/gimbal/pan              [std_msgs/msg/Int16]");
			RCLCPP_INFO(this->get_logger(), "/xacti/gimbal/tilt             [std_msgs/msg/Int16]");
			RCLCPP_INFO(this->get_logger(), "======= Following Topics are published ============");
			RCLCPP_INFO(this->get_logger(), "/xacti/camera/view             [sensor_msgs/msg/Image]");
			RCLCPP_INFO(this->get_logger(), "================ ROS Parameters ===================");
			RCLCPP_INFO(this->get_logger(), "local_play: %d", this->local_play);
			
			/// Loop ///
			timer_ = this->create_wall_timer(30ms, std::bind(&CameraControl::timer_callback, this));
		}

	private:

		////////////////////////////////////
		/// UVC Camera set/get functions ///
		////////////////////////////////////
		bool uvcInit(const char *serial)
		{	
			RCLCPP_INFO(this->get_logger(), "Init and Open Camera");

			// Init
			uvc_error_t res_init = uvc_init(&m_ctx, NULL);
		    if (res_init < 0)
		    {
		        uvc_perror(res_init, "uvc_init");
		        assert(res_init == 0);
		        RCLCPP_ERROR(this->get_logger(), "uvcInit: init ERROR");
		        exit(1);
		    }

		    // Open
		    uvc_error_t res_open;
		    if ((res_open = uvc_find_device(m_ctx, &m_dev,0x296b, 0, serial)) < 0)
		    {
		        uvc_perror(res_open, "uvc_find_device"); // CX-GBXXX未接続
		        RCLCPP_ERROR(this->get_logger(), "uvcInit: find device ERROR");
		        exit(1);
		        return false;
		    }

		    if ((res_open = uvc_open(m_dev, &m_devh)) < 0)
		    {
		    	RCLCPP_ERROR(this->get_logger(), "uvcInit: open ERROR");
		        uvc_perror(res_open, "uvc_open");
		        exit(1);
		        return false;
		    }

		    /// Disable UAVCAN
			struct DataUAVCANEnableSetting
	        {
	            uint8_t bEnable; //! 0 : OFF, 1 : ON(default)
	        };
	        DataUAVCANEnableSetting data{};
	        data.bEnable = 0;
	        bool ret = this->SetCameraCtrl(0x07, 0x1e, &data, sizeof(data));

		    return true;
		}

		bool SetCameraCtrl(uint8_t unit_id, uint8_t cotrol_id, void *data, int length)
		{
		    if (!m_devh)
		    {
		        uvc_perror(UVC_ERROR_INVALID_DEVICE, "SetCameraCtril");
		        return false;
		    }

		    if (uvc_set_ctrl(m_devh, unit_id, cotrol_id, data, length) != length)
		    {
		        uvc_perror(UVC_ERROR_OTHER, "SetCameraCtril");
		        return false;
		    }

		    return true;
		}

		bool GetCameraCtrl(uint8_t unit_id, uint8_t cotrol_id, void *data, int length)
		{
		    if (!m_devh)
		    {
		        uvc_perror(UVC_ERROR_INVALID_DEVICE, "GetCameraCtril");
		        return false;
		    }

		    if (uvc_get_ctrl(m_devh, unit_id, cotrol_id, data, length, UVC_GET_CUR) != length)
		    {
		        uvc_perror(UVC_ERROR_OTHER, "GetCameraCtril");
		        return false;
		    }

		    return true;
		}

		bool startStream(){
			

			if (!m_devh)
		    {
		        return false;
		    }

		    uvc_error_t res;
		    const uvc_format_desc_t *format_desc = uvc_get_format_descs(m_devh);
		    const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
		    enum uvc_frame_format frame_format = UVC_FRAME_FORMAT_MJPEG;
		    m_video_width = 1920;
		    m_video_height = 1080;
		    m_video_fps = 30;
		    if (frame_desc)
		    {
		        m_video_width = frame_desc->wWidth;
		        m_video_height = frame_desc->wHeight;
		        m_video_fps = 10000000 / frame_desc->dwDefaultFrameInterval;
		    }


		    uvc_stream_ctrl_t ctrl;
		    res = uvc_get_stream_ctrl_format_size(
		        m_devh, &ctrl,
		        frame_format,
		        m_video_width, m_video_height, m_video_fps);

		    res = uvc_start_streaming(m_devh, &ctrl, cb, (void *) 12345, 0);
	        if (res < 0)
	        {
	            uvc_perror(res, "start_streaming");
	            return false;
	        }

	        this->start_stream_flag = true;

		}

		void initVideoOut()
		{
			this->output_video = open(VIDEO_OUT, O_RDWR);
			if(this->output_video < 0) {
			    RCLCPP_ERROR(this->get_logger(), "video out: could not open output VIDEO_OUT!\n");
			    this->allow_video_out = false;
			} else {
				this->allow_video_out = true;
			}

			// acquire video format from device
			struct v4l2_format vid_format;
			memset(&vid_format, 0, sizeof(vid_format));
			vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

			if (ioctl(this->output_video, VIDIOC_G_FMT, &vid_format) < 0) {
				RCLCPP_ERROR(this->get_logger(), "video out: unable to get video format!\n");
				this->allow_video_out = false;
			} else {
				this->allow_video_out = true;
			}
			

			// configure desired video format on device
			this->framesize = VID_WIDTH * VID_HEIGHT * 3;
			vid_format.fmt.pix.width = VID_WIDTH;
			vid_format.fmt.pix.height = VID_HEIGHT;
			vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
			vid_format.fmt.pix.sizeimage = this->framesize;
			vid_format.fmt.pix.field = V4L2_FIELD_NONE;

			if (ioctl(this->output_video, VIDIOC_S_FMT, &vid_format) < 0) {
				RCLCPP_ERROR(this->get_logger(), "video out: unable to set video format!\n");
				this->allow_video_out = false;
			} else {
				this->allow_video_out = true;
			}

		}

		//////////////////////////////
		/// ROS Callback functions ///
		//////////////////////////////
		void gimbal_enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
		{
			/*
				Listen either true or false, then gimbal will restart
			*/

			RCLCPP_INFO(this->get_logger(), "Gimbal enable");

			struct DataGimbalRestart
	        {
	            uint8_t bGimbalCalibration; //! 0 : execute
	        };
	        DataGimbalRestart data{};
	        data.bGimbalCalibration = 0;
	        bool ret = this->SetCameraCtrl(0x07, 0x0a, &data, sizeof(data));

	        if (ret == false){
	        	RCLCPP_ERROR(this->get_logger(), "Gimbal Enable: communication error");
	        }
			

		}

		void gimbal_pan_callback(const std_msgs::msg::Int16::SharedPtr msg)
		{
			/*
				Input value in degree

				yaw : -85 deg to 85 deg
			*/
			uint16_t pan_lsb = msg->data * 100;

			if (pan_lsb < 0){

				pan_lsb = 65536 + pan_lsb;  //pan_lsb is in negative already, so we just use +

			}

			RCLCPP_INFO(this->get_logger(), "Gimbal Pan: pan_lsb %d", pan_lsb);


			struct DataGimbalControl
	        {
	            uint8_t pitch_cmd_type;
	            uint8_t yaw_cmd_type;
	            uint16_t pitch_cmd_value;
	            uint16_t yaw_cmd_value;
	        };
	        DataGimbalControl data{};
	        data.pitch_cmd_type = 0;
	        data.yaw_cmd_type = 2;
	        data.pitch_cmd_value = 0;
	        data.yaw_cmd_value = pan_lsb;
	        bool ret = this->SetCameraCtrl(0x08, 0x3, &data, sizeof(data));

	        if (ret == false){
	        	RCLCPP_ERROR(this->get_logger(), "Gimbal Pan: communication error");
	        }

		}

		void gimbal_tilt_callback(const std_msgs::msg::Int16::SharedPtr msg)
		{
			/*
				Input value in degree

				tilt (lower mount): -115 deg to 45 deg  
				tilt (upper mount): -45 deg to 115 deg
			*/

			uint16_t tilt_lsb = msg->data * 100;

			if (tilt_lsb < 0){

				tilt_lsb = 65536 + tilt_lsb;  //pan_lsb is in negative already, so we just use +

			}

			RCLCPP_INFO(this->get_logger(), "Gimbal Tilt: tilt_lsb %d", tilt_lsb);


			struct DataGimbalControl
	        {
	            uint8_t pitch_cmd_type;
	            uint8_t yaw_cmd_type;
	            uint16_t pitch_cmd_value;
	            uint16_t yaw_cmd_value;
	        };
	        DataGimbalControl data{};
	        data.pitch_cmd_type = 2;
	        data.yaw_cmd_type = 0;
	        data.pitch_cmd_value = tilt_lsb;
	        data.yaw_cmd_value = 0;
	        bool ret = this->SetCameraCtrl(0x08, 0x3, &data, sizeof(data));

	        if (ret == false){
	        	RCLCPP_ERROR(this->get_logger(), "Gimbal Tilt: communication error");
	        }

		}

		void take_photo_callback(const std_msgs::msg::Bool::SharedPtr msg)
		{
			/*
				Listen either true or false, then camera will take a photo and save to SD card.
			*/

			RCLCPP_INFO(this->get_logger(), "Take photo");

			struct DataTakePhoto
	        {
	            uint8_t bTakePhoto; //! 0 : execute
	        };
	        DataTakePhoto data{};
	        data.bTakePhoto = 0;
	        bool ret = this->SetCameraCtrl(0x06, 0x9, &data, sizeof(data));

	        if (ret == false){
	        	RCLCPP_ERROR(this->get_logger(), "Take Photo: communication error");
	        }
			

		}

		void camera_orientation_callback(const std_msgs::msg::Int8::SharedPtr msg)
		{
			/*
				Attach orientation
				0 : Lower side (camera at down position)
				1 : Upper side (camera at up position)
				2 : Upper Yaw fixed (motor at yaw is not activated, it has to fix by something)
				3 : Auto judge (default)

			*/
			if (msg->data == 0){
				RCLCPP_INFO(this->get_logger(), "Camera orientation: %d Lower side", msg->data);
			} else if (msg->data == 1){
				RCLCPP_INFO(this->get_logger(), "Camera orientation: %d Upper side", msg->data);
			} else if (msg->data == 2){
				RCLCPP_INFO(this->get_logger(), "Camera orientation: %d Upper with no yaw control", msg->data);
			} else if (msg->data == 3){
				RCLCPP_INFO(this->get_logger(), "Camera orientation: %d Auto judge", msg->data);
			} else {
				RCLCPP_WARN(this->get_logger(), "Camera orientation: %d is not in any option, please put between 0-3", msg->data);
			}
			

			struct DataCameraOrientation
	        {
	            uint8_t bPosition; //! 0 : lower side, 1 : upper side, 2 : upper yaw fixed, 3 : auto judge (default) 
	        };
	        DataCameraOrientation data{};
	        data.bPosition = msg->data;
	        bool ret = this->SetCameraCtrl(0x07, 0x1b, &data, sizeof(data));

			if (ret == false){
	        	RCLCPP_ERROR(this->get_logger(), "Camera Orientation: communication error");
	        }

		}

		void camera_focus_mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
		{
			/*
				Focus mode
				0 : MF (default)
				1 : S-AF
				2 : C-AF 
			*/

			RCLCPP_INFO(this->get_logger(), "Camera Focus Mode: %d", msg->data);

			struct DataCameraFocusMode
	        {
	            uint32_t bFocusMode; //! 0 : execute
	        };
	        DataCameraFocusMode data{};
	        data.bFocusMode = msg->data;
	        bool ret = this->SetCameraCtrl(0x06, 0x1c, &data, sizeof(data));

	        if (ret == false){
	        	RCLCPP_ERROR(this->get_logger(), "Camera Focus Mode: communication error");
	        }
			

		}

		void camera_focus_callback(const std_msgs::msg::Int32::SharedPtr msg)
		{
			/*
				Focus Position
				300 - 100000(default) mm

			*/

			RCLCPP_INFO(this->get_logger(), "Camera Focus %d mm", msg->data);

			struct DataCameraFocus
	        {
	            uint32_t bFocusPosition; //! 0 : execute
	        };
	        DataCameraFocus data{};
	        data.bFocusPosition = msg->data;
	        bool ret = this->SetCameraCtrl(0x06, 0x1d, &data, sizeof(data));

	        if (ret == false){
	        	RCLCPP_ERROR(this->get_logger(), "Camera Focus: communication error");
	        }
			

		}

		void camera_recording_callback(const std_msgs::msg::Bool::SharedPtr msg)
		{
			/*
				Start or Stop recording video.
				True : start recording
				Fakse : stop recording

			*/
			RCLCPP_INFO(this->get_logger(), "Camera Recording: %d", msg->data);

			if (msg->data != this->camera_recording){

				if (msg->data == true){
					RCLCPP_INFO(this->get_logger(), "Camera Recording: Start");
				} else {
					RCLCPP_INFO(this->get_logger(), "Camera Recording: Stop");
				}

				this->camera_recording = msg->data;

				struct DataRecording
		        {
		            uint8_t bRecording; //! 0 : execute
		        };
		        DataRecording data{};
		        data.bRecording = msg->data;
		        bool ret = this->SetCameraCtrl(0x06, 0xb, &data, sizeof(data));

		        if (ret == false){
		        	RCLCPP_ERROR(this->get_logger(), "Camera Recording: communication error");
		        }
			} else {
				RCLCPP_WARN(this->get_logger(), "Camera Recording: Already on %d stage", msg->data);
			}
			
		}

		void camera_video_res_callback(const std_msgs::msg::Int8::SharedPtr msg)
		{
			/*
				Change the video resolution
				0 : 4K (default)
				1 : 2.7K
				2 : Full HD
				3 : HD

			*/
			if (msg->data == 0){
				RCLCPP_INFO(this->get_logger(), "Camera Video Resolution %d 4K", msg->data);
			} else if (msg->data == 1){
				RCLCPP_INFO(this->get_logger(), "Camera Video Resolution %d 2.7K", msg->data);
			} else if (msg->data == 2){
				RCLCPP_INFO(this->get_logger(), "Camera Video Resolution %d FHD", msg->data);
			} else if (msg->data == 3){
				RCLCPP_INFO(this->get_logger(), "Camera Video Resolution %d HD", msg->data);
			} else {
				RCLCPP_INFO(this->get_logger(), "Camera Video Resolution %d unknown...", msg->data);
			}
			

			struct DataVideoResolution
	        {
	            uint8_t bVideoResolution; //! 0 : execute
	        };
	        DataVideoResolution data{};
	        data.bVideoResolution = msg->data;
	        bool ret = this->SetCameraCtrl(0x06, 0xf, &data, sizeof(data));

	        if (ret == false){
	        	RCLCPP_ERROR(this->get_logger(), "Camera Video Resolution: communication error");
	        }
			

		}

		void camera_optical_zoom_callback(const std_msgs::msg::Int32::SharedPtr msg)
		{
			/*
				Do optical zoom, data from 100 to 250 with increment by 10

			*/

			uint32_t zoom_val = msg->data;

			RCLCPP_INFO(this->get_logger(), "Camera Zoom: %d", zoom_val);
			

			struct DataZoom
	        {
	            uint32_t bZoom; //! 0 : execute
	        };
	        DataZoom data{};
	        data.bZoom = zoom_val;
	        bool ret = this->SetCameraCtrl(0x07, 0x1a, &data, sizeof(data));

	        if (ret == false){
	        	RCLCPP_ERROR(this->get_logger(), "Camera Zoom: communication error");
	        }
			

		}

		void camera_iso_callback(const std_msgs::msg::Int8::SharedPtr msg)
		{
			/* 
			ISO Sensitivity
				0 : auto
				1 : 125 (default)
				2 : 160
				3 : 200
				4 : 250
				5 : 320
				6 : 400
				7 : 500
				8 : 640
				9 : 800
			   10 : 1000
			   11 : 1250
			   12 : 1600
			   13 : 2000
			   14 : 2500
			   15 : 3200
			   16 : 4000
			   17 : 5000
			   18 : 6400 
			*/

			RCLCPP_INFO(this->get_logger(), "Camera ISO: %d", msg->data);

			if ((0 <= msg->data) && (msg->data <= 18)){

				struct DataIso
		        {
		            uint8_t bIsoSensitivity; //! 0 : execute
		        };
		        DataIso data{};
		        data.bIsoSensitivity = msg->data;
		        bool ret = this->SetCameraCtrl(0x06, 0x16, &data, sizeof(data));

		        if (ret == false){
		        	RCLCPP_ERROR(this->get_logger(), "Camera ISO: communication error");
		        }
	        } else {
	        	RCLCPP_WARN(this->get_logger(), "Camera ISO: value out range");
	        }
			

		}

		void camera_aperture_callback(const std_msgs::msg::Int8::SharedPtr msg)
		{
			/* 
			Aperature Control
				0 : F2.8 (default)
				1 : F3.2
				2 : F3.5
				3 : F4.0
				4 : F4.5
				5 : F5.0
				6 : F5.6
				7 : F6.3
				8 : F7.1
				9 : F8.0
			   10 : F9.0
			   11 : F10.0
			   12 : F11.0
			*/

			RCLCPP_INFO(this->get_logger(), "Camera Aperture: %d", msg->data);

			if ((0 <= msg->data) && (msg->data <= 12)){
			

				struct DataAperture
		        {
		            uint8_t bAperture; //! 0 : execute
		        };
		        DataAperture data{};
		        data.bAperture = msg->data;
		        bool ret = this->SetCameraCtrl(0x06, 0x1b, &data, sizeof(data));

		        if (ret == false){
		        	RCLCPP_ERROR(this->get_logger(), "Camera Aperture: communication error");
		        }
		    } else {

		    	RCLCPP_WARN(this->get_logger(), "Camera Aperture: value out range");
		    }
			

		}

		////////////
		/// Loop ///
		////////////
		void timer_callback() {
			

			cv::Mat bgr;
            if (!f_rgb.empty())
            {

            	cv::cvtColor(f_rgb, bgr, cv::COLOR_RGB2BGR);

            	/// local display
            	if (this->local_play){
	                cv::imshow("local_play", bgr);
                }

                /// publish compressed image by image_transport at /xacti/camera/view topic
                std_msgs::msg::Header hdr;
				sensor_msgs::msg::Image::SharedPtr image_msg;
				image_msg = cv_bridge::CvImage(hdr, "bgr8", bgr).toImageMsg();
				image_pub.publish(image_msg);

				/// write opencv frame to output_video /dev/video30
				if (this->allow_video_out){
					cv::Mat result;
					result = f_rgb.clone();
					size_t written = write(this->output_video, result.data, this->framesize);
					if (written < 0) {
					    RCLCPP_ERROR(this->get_logger(), "ERROR: could not write to output device!\n");

					}
				}
				

				/// check if the last stamp in cb and now stamp is more than 1 second
				/// then it seems that frame is frozen...
				int64_t stamp_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
				int64_t diff = stamp_now - last_frame_stamp;

                if ((diff > 1000) && (this->start_stream_flag == true)){
                	RCLCPP_WARN(this->get_logger(), "Frame seems to be stuck with diff %d", diff);
                	RCLCPP_WARN(this->get_logger(), "Kill node");
                	// RCLCPP_WARN(this->get_logger(), "Try start streaming again");
                	// this->uvcInit(NULL);
                	// this->startStream();
                	exit(1);
                }
            }
            
            char key = (char)cv::waitKey(1);
  
		}

		/// libuvc ///
		uvc_context_t *m_ctx;
	    uvc_device_t *m_dev;
	    uvc_device_handle_t *m_devh;

	    /// opencv ///
	    cv::VideoCapture cap;
	    int m_video_width;
	    int m_video_height;
	    int m_video_fps;

	    /// class parameters ///
	    int camera_position;
	    bool camera_recording = false;
	    bool start_stream_flag = false;

	    /// v4l2loopback write ///
	    bool allow_video_out;
	    int output_video;
	    size_t framesize;
	    
	    /// ROS params ///
	    bool local_play;

		/// Define the object of sub/pub ///
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr take_photo_sub;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gimbal_enable_sub;
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr camera_orientation_sub;
		rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr gimbal_pan_sub;
		rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr gimbal_tilt_sub;
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr camera_focus_mode_sub;
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr camera_focus_sub;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_recording_sub;
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr camera_video_res_sub;
		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr optical_zoom_sub;
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr camera_iso_sub;
		rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr camera_aperture_sub;

		image_transport::Publisher image_pub;

		/// ROS parameters callback ///
		OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[]){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraControl>());
	rclcpp::shutdown();

	return 0;
}