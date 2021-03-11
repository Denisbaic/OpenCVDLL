#pragma once

#define DLL_EXPORT __declspec(dllexport)    //shortens __declspec(dllexport) to DLL_EXPORT

#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include "DrawLandmarks.h"

#include <dlib/opencv.h>

#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>



#define DLIB

#ifdef DLIB

dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
dlib::shape_predictor pose_model;
#endif


	CascadeClassifier face_detector;
	Ptr<face::Facemark> facemark;

	VideoCapture cam;

	// Variable to store a video frame and its grayscale 
	Mat frame, gray;

	//game events
    bool is_need_to_show_b_box = false;	
	bool is_selected_nose_position_for_mouse_control = false;
	Rect mouse_field;
	vector< vector<Point2f> > landmarks;
	vector<Rect> faces;

	float FacialLandmarks_output[136];

	extern "C" int  DLL_EXPORT InitOpenCV(int cam_index, char* face_detector_cascade_file_path, char* face_mark_model_file_path, int mouse_wheel_field_width, int mouse_wheel_field_height);

	extern "C" bool DLL_EXPORT IsEyeOpen(int face_index, bool check_left_eye, float EAR);

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
