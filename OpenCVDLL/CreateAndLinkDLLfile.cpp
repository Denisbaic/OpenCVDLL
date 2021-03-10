#include "pch.h"

#include "CreateAndLinkDLLfile.h"
#include "DrawGameLogic.h"

inline float euclideanDist(Point2f& p, Point2f& q) {
    Point2f diff = p - q;
    return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
}

int InitOpenCV(int cam_index, char* face_detector_cascade_file_path, char* face_mark_model_file_path,
                 int const mouse_wheel_field_width, int const mouse_wheel_field_height)
{
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

    face_detector.load(face_detector_cascade_file_path);
    if (face_detector.empty())
        return 2;
	
    //facemark.release();
    facemark = face::FacemarkLBF::create();
    facemark->loadModel(face_mark_model_file_path);
    if (facemark->empty())
        return 3;

    return 0;
}

bool IsEyeOpen(int face_index, bool check_left_eye, float EAR)
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
    cout << CurrentEAR << endl;
    return CurrentEAR <= EAR;
}

bool IsMouthOpen(int face_index, float MAR)
{
    if (face_index < 0 || face_index >= landmarks.size())
        return false;
	
    float const CurrentMAR = (euclideanDist(landmarks[face_index][61], landmarks[face_index][67]) +
        euclideanDist(landmarks[face_index][62], landmarks[face_index][66]) +
        euclideanDist(landmarks[face_index][63], landmarks[face_index][65])) / (2.f * euclideanDist(landmarks[face_index][60], landmarks[face_index][64]));
    return 	CurrentMAR >= MAR;
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
    if (!cam.read(frame))
        return false;
    

    // Convert frame to grayscale because
    // face_detector requires grayscale image.
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // Detect faces
    face_detector.detectMultiScale(gray, faces);

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
	        DrawMouseDirection(frame, mouse_field, landmarks[0]);
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
    mouse_field = Rect(x, y, width, height);
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

void ResizeFrame(int& width, int& height)
{
    cv::resize(frame, frame, cv::Size(width, height));
}

bool GetFacialLandmarks(int face_index, vector<std::pair<float,float>>& facial_landmarks)
{
	if(face_index < 0 || static_cast<unsigned long long>(face_index) >= facial_landmarks.size())
        return false;
    facial_landmarks.clear();
   for(auto& elem : landmarks[face_index])
   {
       facial_landmarks.emplace_back(elem.x,elem.y);
   }
	return true;
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
