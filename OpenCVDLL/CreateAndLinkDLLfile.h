#pragma once

#define DLL_EXPORT __declspec(dllexport)    //shortens __declspec(dllexport) to DLL_EXPORT

#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include "drawLandmarks.h"
/*
#include <dlib/opencv.h>

#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

*/

CascadeClassifier faceDetector;
Ptr<face::Facemark> facemark;

VideoCapture cam;

// Variable to store a video frame and its grayscale 
Mat frame, gray;
struct FacialMarks
{
    bool IsLeftEyeClosed = false;
    bool IsRightEyeClosed = false;

    bool IsMouthOpen = false;
} facial_marks;


extern "C" bool DLL_EXPORT getInvertedBool(bool boolState);
extern "C" int DLL_EXPORT getIntPlusPlus(int lastInt);
extern "C" float DLL_EXPORT getCircleArea(float radius);
extern "C" char DLL_EXPORT* getCharArray(char* parameterText);
extern "C" float DLL_EXPORT* getVector4(float x, float y, float z, float w);

    bool temp = false;

    extern "C"  void DLL_EXPORT InvertBool();
    extern "C" bool DLL_EXPORT getBool();
	
    extern "C" int DLL_EXPORT init_opencv(int cam_index, char* FaceDetectorCascadeFilePath, char* FaceMarkModelFilePath);
    extern "C" void DLL_EXPORT show_frame();

    extern "C" void DLL_EXPORT stop_opencv();


    //config
    float const EAR = 0.2f;
    float const MAR = 0.6f;

    int MouseFieldWidth = 200;
    int MouseFieldHeight = 100;

    //game events
    bool IsSelectedNosePositionForMouseControl = false;
    Rect MouseField;

    float euclideanDist(Point2f& p, Point2f& q) {
        Point2f diff = p - q;
        return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
    }

    bool IsEyeOpen(vector<Point2f>& landmarks, bool CheckLeftEye)
    {
        float CurrentEAR;
        if (CheckLeftEye)
        {
            CurrentEAR = (euclideanDist(landmarks[37], landmarks[41]) + euclideanDist(landmarks[38], landmarks[40])) / (2.f * euclideanDist(landmarks[36], landmarks[39]));
        }
        else
        {
            CurrentEAR = (euclideanDist(landmarks[43], landmarks[47]) + euclideanDist(landmarks[44], landmarks[46])) / (2.f * euclideanDist(landmarks[42], landmarks[45]));
        }
        cout << CurrentEAR << endl;
        return CurrentEAR <= EAR;
    }

    bool IsMouthOpen(vector<Point2f>& landmarks)
    {
        float const CurrentMAR = (euclideanDist(landmarks[61], landmarks[67]) +
            euclideanDist(landmarks[62], landmarks[66]) +
            euclideanDist(landmarks[63], landmarks[65])) / (2.f * euclideanDist(landmarks[60], landmarks[64]));
        return 	CurrentMAR >= MAR;
    }

    extern "C" void DLL_EXPORT GetFacialMarks(bool& IsLeftEyeClosed, bool& IsRightEyeClosed, bool& IsMouthOpen);


/*

dlib::image_window win;
dlib::frontal_face_detector detector;
dlib::shape_predictor pose_model;
    extern "C" int DLL_EXPORT init_dlib(int cam_index, char* predictorFilePath);
    extern "C" void DLL_EXPORT show_frame_dlib();

    extern "C" void DLL_EXPORT stop_dlib();
*/