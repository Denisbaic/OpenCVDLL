#pragma once

#define DLL_EXPORT __declspec(dllexport)    //shortens __declspec(dllexport) to DLL_EXPORT

#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/face/facemarkLBF.hpp>

/*
#include <dlib/opencv.h>

#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
*/


//#define DLIB

#ifdef DLIB

dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
dlib::shape_predictor pose_model;
#endif


class FaceDetector {
public:
	explicit FaceDetector();

	bool load(std::string const& face_detection_configuration, std::string const& face_detection_weights);
	/// Detect faces in an image frame
	/// \param frame Image to detect faces in
	/// \return Vector of detected faces
	std::vector<cv::Rect> detect_face_rectangles(const cv::Mat& frame);

private:
	/// Face detection network
	cv::dnn::Net network_;
	/// Input image width
	const int input_image_width_;
	/// Input image height
	const int input_image_height_;
	/// Scale factor when creating image blob
	const double scale_factor_;
	/// Mean normalization values network was trained with
	const cv::Scalar mean_values_;
	/// Face detection confidence threshold
	const float confidence_threshold_;

};



	//CascadeClassifier face_detector;
	FaceDetector face_detector;
	cv::Ptr<cv::face::Facemark> facemark;

	cv::VideoCapture cam;

	// Variable to store a video frame and its grayscale 
	cv::Mat frame, gray;

	//game events
    bool is_need_to_show_b_box = false;	
	bool is_selected_nose_position_for_mouse_control = false;
	cv::Rect mouse_field;
	std::vector<std::vector<cv::Point2f> > landmarks;
	std::vector<cv::Rect> faces;

	cv::Scalar ui_color(255, 0, 0);

	float FacialLandmarks_output[136];

	extern "C" int  DLL_EXPORT InitOpenCV(int cam_index, char* face_detector_config, char* face_detector_weights ,char* face_mark_model_file_path, int mouse_wheel_field_width, int mouse_wheel_field_height);

	extern "C" bool DLL_EXPORT IsEyeOpen(int face_index, bool check_left_eye, float EAR, float& current_ear);

	extern "C" bool DLL_EXPORT IsMouthOpen(int face_index, float MAR);

	extern "C" bool DLL_EXPORT CalculateFacialLandmarks();

	extern "C" void DLL_EXPORT GetFrame(uchar* &  OutputFrame);
	
    extern "C" void DLL_EXPORT StopOpenCV();

	extern "C" void DLL_EXPORT SetIsNeedToShowBBox(bool NewValue);
	extern "C" bool DLL_EXPORT GetIsNeedToShowBBox();

	extern "C" void DLL_EXPORT SetIsSelectedNosePositionForMouseControl(bool NewValue);
	extern "C" bool DLL_EXPORT GetIsSelectedNosePositionForMouseControl();

	extern "C" void DLL_EXPORT SetMouseField(int x, int y, int width, int height);
	extern "C" void DLL_EXPORT GetMouseField(int& x, int& y, int& width, int& height);

	extern "C" bool DLL_EXPORT IsCamOpened();

	extern "C" void DLL_EXPORT GetFrameSize(int& width, int& height);
	extern "C" void DLL_EXPORT ResizeFrame(int& width, int& height);

	extern "C" void DLL_EXPORT GetFacialLandmarks(int face_index, float*& arr_output, int& size);

	extern "C" void DLL_EXPORT SetUIColor(double R, double G, double B);
	extern "C" void DLL_EXPORT GetUIColor(double& R, double& G, double& B);;