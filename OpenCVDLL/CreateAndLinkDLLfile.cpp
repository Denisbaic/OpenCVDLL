#include "pch.h"

#include "CreateAndLinkDLLfile.h"

inline float euclideanDist(cv::Point2f& p, cv::Point2f& q) {
	cv::Point2f diff = p - q;
    return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
}

inline void DrawMouseRect(cv::Mat& im, cv::Rect& MouseRect)
{
    line(im, { MouseRect.x , MouseRect.y }, { MouseRect.x , MouseRect.y + MouseRect.height }, ui_color);
    line(im, { MouseRect.x , MouseRect.y }, { MouseRect.x + MouseRect.width , MouseRect.y }, ui_color);
    line(im, { MouseRect.x + MouseRect.width , MouseRect.y }, { MouseRect.x + MouseRect.width , MouseRect.y + MouseRect.height }, ui_color);
    line(im, { MouseRect.x , MouseRect.y + MouseRect.height }, { MouseRect.x + MouseRect.width , MouseRect.y + MouseRect.height }, ui_color);
}

inline void DrawMouseDirection(cv::Mat& im, cv::Rect& MouseRect, std::vector<cv::Point2f>& landmarks)
{
	cv::Point2i const MiddlePoint(MouseRect.x + MouseRect.width / 2, MouseRect.y + MouseRect.height / 2);
    line(im, MiddlePoint, landmarks[30], ui_color);
}

void DrawLandmarks(cv::Mat& im, std::vector<cv::Point2f>& landmarks)
{
    for (int i = 0; i < landmarks.size(); i++)
    {
        circle(im, landmarks[i], 3, ui_color, cv::FILLED);
    }
}

void DrawFaceRect(cv::Mat& im, std::vector<cv::Rect>& faces)
{
    for (auto& face : faces)
    {
        line(im, { face.x , face.y }, { face.x , face.y + face.height }, ui_color);
        line(im, { face.x , face.y }, { face.x + face.width , face.y }, ui_color);
        line(im, { face.x + face.width , face.y }, { face.x + face.width , face.y + face.height }, ui_color);
        line(im, { face.x , face.y + face.height }, { face.x + face.width , face.y + face.height }, ui_color);
    }
}


int InitOpenCV(int cam_index, char* face_detector_config, char* face_detector_weights,  char* face_mark_model_file_path,
                 int const mouse_wheel_field_width, int const mouse_wheel_field_height)
{
#ifndef DLIB
    is_selected_nose_position_for_mouse_control = false;
    is_need_to_show_b_box = false;

	
    //IsLastCalculateSuccess = false;
    landmarks.clear();
    faces.clear();
	
    mouse_field.height = mouse_wheel_field_height;
    mouse_field.width = mouse_wheel_field_width;
    //return 4;
    cam.release();
    if (!cam.open(cam_index))
        return 1;

    
    face_detector.load(face_detector_config, face_detector_weights);
    //if (face_detector.empty())
      //  return 2;
	
    //facemark.release();
    facemark = cv::face::FacemarkLBF::create();
    facemark->loadModel(face_mark_model_file_path);
    if (facemark->empty())
        return 3;
#else
    if (cam.open(cam_index) == false)
        return 3;

    
    dlib::deserialize(face_detector_cascade_file_path) >> pose_model;

    return 0;
#endif
    return 0;
}

void LoadDataSet(char* folder)
{
	std::vector<cv::String> fn;
	cv::String const folder_str(folder);
	cv::glob(folder_str +"*.png", fn, false);

	size_t const count = fn.size(); //number of png files in images folder
    for (size_t i = 0; i < count; i++)
        test_faces.push_back(cv::imread(fn[i]));
}

bool IsEyeOpen(int face_index, bool check_left_eye, float EAR,float& current_ear)
{
    if (face_index < 0 || face_index >= landmarks.size() || landmarks[face_index].size() != 68)
        return true;
	
    float CurrentEAR;
    if (check_left_eye)
    {
        CurrentEAR = (euclideanDist(landmarks[face_index][37], landmarks[face_index][41]) + euclideanDist(landmarks[face_index][38], landmarks[face_index][40])) / (2.f * euclideanDist(landmarks[face_index][36], landmarks[face_index][39]));
    }
    else
    {
        CurrentEAR = (euclideanDist(landmarks[face_index][43], landmarks[face_index][47]) + euclideanDist(landmarks[face_index][44], landmarks[face_index][46])) / (2.f * euclideanDist(landmarks[face_index][42], landmarks[face_index][45]));
    }
    current_ear = CurrentEAR;
    //cout << CurrentEAR << endl;
    return CurrentEAR >= EAR;
}

bool IsMouthOpen(int face_index, float MAR, float& current_mar)
{
    if (face_index < 0 || face_index >= landmarks.size() || landmarks[face_index].size()!=68)
        return false;
	
    float const CurrentMAR = (euclideanDist(landmarks[face_index][61], landmarks[face_index][67]) +
        euclideanDist(landmarks[face_index][62], landmarks[face_index][66]) +
        euclideanDist(landmarks[face_index][63], landmarks[face_index][65])) / (2.f * euclideanDist(landmarks[face_index][60], landmarks[face_index][64]));
    current_mar = CurrentMAR;
	return 	CurrentMAR >= MAR;
}

bool IsEyebrowsRaised(int face_index, float BAR, float& current_bar)
{
    //const float lBAR = (euclideanDist(landmarks[face_index][23], landmarks[face_index][43]) + euclideanDist(landmarks[face_index][24], landmarks[face_index][43]) + euclideanDist(landmarks[face_index][25], landmarks[face_index][43]) + euclideanDist(landmarks[face_index][23], landmarks[face_index][44]) + euclideanDist(landmarks[face_index][24], landmarks[face_index][44]) + euclideanDist(landmarks[face_index][25], landmarks[face_index][44])) / (3.f * cv::max(euclideanDist(landmarks[face_index][22], landmarks[face_index][26]), euclideanDist(landmarks[face_index][17], landmarks[face_index][21])));
    //const float rBAR = (euclideanDist(landmarks[face_index][18], landmarks[face_index][37]) + euclideanDist(landmarks[face_index][19], landmarks[face_index][37]) + euclideanDist(landmarks[face_index][20], landmarks[face_index][37]) + euclideanDist(landmarks[face_index][18], landmarks[face_index][38]) + euclideanDist(landmarks[face_index][19], landmarks[face_index][38]) + euclideanDist(landmarks[face_index][20], landmarks[face_index][38])) / (3.f * cv::max(euclideanDist(landmarks[face_index][22], landmarks[face_index][26]), euclideanDist(landmarks[face_index][17], landmarks[face_index][21])));
    //current_bar = cv::max(lBAR, rBAR);
    if (face_index < 0 || face_index >= landmarks.size() || landmarks[face_index].size() != 68)
        return false;
	
    float const lBAR = (euclideanDist(landmarks[face_index][8], landmarks[face_index][22]) + euclideanDist(landmarks[face_index][8], landmarks[face_index][23]) + euclideanDist(landmarks[face_index][8], landmarks[face_index][24]) + euclideanDist(landmarks[face_index][8], landmarks[face_index][25]) + euclideanDist(landmarks[face_index][8], landmarks[face_index][26]))/(4.f* euclideanDist(landmarks[face_index][8], landmarks[face_index][27]));
    float const rBAR = (euclideanDist(landmarks[face_index][8], landmarks[face_index][17]) + euclideanDist(landmarks[face_index][8], landmarks[face_index][18]) + euclideanDist(landmarks[face_index][8], landmarks[face_index][19]) + euclideanDist(landmarks[face_index][8], landmarks[face_index][20]) + euclideanDist(landmarks[face_index][8], landmarks[face_index][21]))/(4.f * euclideanDist(landmarks[face_index][8], landmarks[face_index][27]));
	current_bar = cv::max(lBAR, rBAR);
	return current_bar > BAR;
}

bool CalculateFacialLandmarks()
{
    // Find face
    faces.clear();
	
    // Variable for landmarks. 
	// Landmarks for one face is a vector of points
	// There can be more than one face in the image. Hence, we 
	// use a vector of vector of points. 
    landmarks.clear();
	//TODO move this to another place
	if(!is_test_mode)
	{
		if (!cam.read(frame))
			return false;
	}
    else
    {
        test_faces[test_frame_index].copyTo(frame);
    }
    
    // Convert frame to grayscale because
    // face_detector requires grayscale image.
    //cvtColor(frame, gray, COLOR_BGR2GRAY);

    // Detect faces
    //face_detector.detectMultiScale(gray, faces);
    faces = face_detector.detect_face_rectangles(frame);
    // Run landmark detector
    bool const success = facemark->fit(frame, faces, landmarks);
    return success;
}

void GetFrame(uchar*& OutputFrame)
{
	 // If successful, render the landmarks on the face
	 for (auto& landmark : landmarks)
	 {
	        DrawLandmarks(frame, landmark);
	 }

     if (is_selected_nose_position_for_mouse_control)
     {
	        DrawMouseRect(frame, mouse_field);
     	
     	if(landmarks.size()>0)
     	{
            DrawMouseDirection(frame, mouse_field, landmarks[0]);
     	}	       
     }

     if (is_need_to_show_b_box)
     {
         DrawFaceRect(frame, faces);
     }
     OutputFrame = frame.data;
	
}

void StopOpenCV()
{
    landmarks.clear();
    faces.clear();
	
    is_selected_nose_position_for_mouse_control = false;
    is_need_to_show_b_box = false;
	
    //destroyWindow("Facial Landmark Detection");
    cam.release();
    facemark.release();
}

void SetIsNeedToShowBBox(bool NewValue)
{
    is_need_to_show_b_box = NewValue;
}

bool GetIsNeedToShowBBox()
{
    return is_need_to_show_b_box;
}

void SetIsSelectedNosePositionForMouseControl(bool NewValue)
{
    is_selected_nose_position_for_mouse_control = NewValue;
}

bool GetIsSelectedNosePositionForMouseControl()
{
    return is_selected_nose_position_for_mouse_control;
}

void SetMouseField(int x, int y, int width, int height)
{
    mouse_field = cv::Rect(x, y, width, height);
}

void GetMouseField(int& x, int& y, int& width, int& height)
{
    x = mouse_field.x;
    y = mouse_field.y;
    width = mouse_field.width;
    height = mouse_field.height;
}

bool IsCamOpened()
{
    return cam.isOpened();
}

void GetFrameSize(int& width, int& height)
{
    width = frame.cols;
    height = frame.rows;
}

void ResizeFrame(int width, int height)
{
    cv::resize(frame, frame, cv::Size(width, height));
}

void GetFacialLandmarks(int face_index, float*& arr_output, int& size)
{
	if(face_index < 0 || static_cast<unsigned long long>(face_index) >= landmarks.size() || landmarks[face_index].size()!= 68)
	{
        arr_output = nullptr;
		return;
	}
    size = 136;
	for (int i = 0; i < 68; ++i)
	{
        FacialLandmarks_output[i * 2] = landmarks[face_index][i].x;
        FacialLandmarks_output[i * 2+1] = landmarks[face_index][i].y;
	}
    arr_output = FacialLandmarks_output;
}

void SetUIColor(float R, float G, float B)
{
    ui_color = cv::Scalar(B, G, R);
}

void GetUIColor(float& R, float& G, float& B)
{
    B = ui_color.val[0]; //B
    G = ui_color.val[1]; //G
    R = ui_color.val[2]; //R
}

void SetTestMode(bool IsTestMode)
{
    is_test_mode = IsTestMode;
}

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
    return std::max(lower, std::min(n, upper));
}

void NextFrame(bool GetOppositeFrame)
{
    test_frame_index = clip(GetOppositeFrame ? test_frame_index - 1 : test_frame_index + 1, 0, static_cast<int>(test_faces.size()));
}



/*
int init_dlib(int cam_index, char* predictorFilePath)
{
    if (cam.open(cam_index) == false)
        return 3;

    detector = dlib::get_frontal_face_detector();
    dlib::deserialize(predictorFilePath) >> pose_model;
	
    return 0;
}

void show_frame_dlib()
{
    // Grab a frame
    cv::Mat temp;
    if (!cam.read(temp))
    {
        return;
    }
    // Turn OpenCV's Mat into something dlib can deal with.  Note that this just
    // wraps the Mat object, it doesn't copy anything.  So cimg is only valid as
    // long as temp is valid.  Also don't do anything to temp that would cause it
    // to reallocate the memory which stores the image as that will make cimg
    // contain dangling pointers.  This basically means you shouldn't modify temp
    // while using cimg.
    dlib::cv_image<dlib::bgr_pixel> cimg(temp);

    // Detect faces 
    std::vector<dlib::rectangle> faces = detector(cimg);
    // Find the pose of each face.
    std::vector<dlib::full_object_detection> shapes;

    for (auto& face : faces)
        shapes.push_back(pose_model(cimg, face));

    // Display it all on the screen
    win.clear_overlay();
    win.set_image(cimg);
    win.add_overlay(render_face_detections(shapes));
}

void stop_dlib()
{
    win.close_window();
}
*/

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu
#define FACE_DETECTION_CONFIGURATION "deploy.prototxt"
#define FACE_DETECTION_WEIGHTS "res10_300x300_ssd_iter_140000_fp16.caffemodel"


#include <sstream>
#include <vector>
#include <string>

FaceDetector::FaceDetector() : confidence_threshold_(0.5), input_image_height_(300), input_image_width_(300),
scale_factor_(1.0), mean_values_({ 104., 177.0, 123.0 }) {}

bool FaceDetector::load(std::string const& face_detection_configuration, std::string const& face_detection_weights)
{
    // Note: The varibles MODEL_CONFIGURATION_FILE and MODEL_WEIGHTS_FILE are passed in via cmake
    network_ = cv::dnn::readNetFromCaffe(face_detection_configuration, face_detection_weights);

    return !network_.empty();
}

std::vector<cv::Rect> FaceDetector::detect_face_rectangles(const cv::Mat& frame) {
    cv::Mat input_blob = cv::dnn::blobFromImage(frame, scale_factor_, cv::Size(input_image_width_, input_image_height_),
        mean_values_, false, false);
    network_.setInput(input_blob, "data");
    cv::Mat detection = network_.forward("detection_out");
    cv::Mat detection_matrix(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

    std::vector<cv::Rect> faces;

    for (int i = 0; i < detection_matrix.rows; i++) {
        float confidence = detection_matrix.at<float>(i, 2);

        if (confidence < confidence_threshold_) {
            continue;
        }
        int x_left_bottom = static_cast<int>(detection_matrix.at<float>(i, 3) * frame.cols);
        int y_left_bottom = static_cast<int>(detection_matrix.at<float>(i, 4) * frame.rows);
        int x_right_top = static_cast<int>(detection_matrix.at<float>(i, 5) * frame.cols);
        int y_right_top = static_cast<int>(detection_matrix.at<float>(i, 6) * frame.rows);

        faces.emplace_back(x_left_bottom, y_left_bottom, (x_right_top - x_left_bottom), (y_right_top - y_left_bottom));
    }

    return faces;
}
