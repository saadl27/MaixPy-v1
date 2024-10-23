#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "py/obj.h"
#include "py/runtime.h"
#include <vector>
#include <cmath>

enum MarkerOrientation {
    CENTER = 0,
    BOTTOM,
    LEFT,
    RIGHT,
    TOP
};

// Function to calculate the orientation based on roll, pitch, and yaw
MarkerOrientation calculate_orientation(double roll, double pitch, double yaw) {
    if (180 - std::abs(roll) > 15) {
        return roll < 0 ? BOTTOM : TOP;
    }
    if (std::abs(pitch) > 15) {
        return pitch < 0 ? RIGHT : LEFT;
    }
    return CENTER;
}

static mp_obj_t detect_position(mp_obj_t img_obj) {
    // Convert MicroPython object to C++ cv::Mat
    // Ensure the img_obj passed is a pointer to a cv::Mat
    cv::Mat* img_ptr = reinterpret_cast<cv::Mat*>(MP_OBJ_TO_PTR(img_obj));
    cv::Mat img = *img_ptr;

    // Convert the image to grayscale
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // Hardcode the dictionary type (e.g., DICT_6X6_250)
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);


    // Hardcode the detector parameters
    cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();

    // Marker detection variables
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<int> markerIds;
    std::vector<cv::Vec3d> rvecs, tvecs;

    // Detect ArUco markers on the grayscale image
    cv::aruco::ArucoDetector detector(dictionary, parameters );
    detector.detectMarkers(gray, markerCorners, markerIds, rejectedCandidates);

    // If no markers detected, return an empty list
    if (markerIds.empty()) {
        return mp_obj_new_list(0, NULL);
    }

    // Camera matrix and distortion coefficients (normally you'd get these from calibration)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    // Marker length (real-world marker size)
    double markerLength = 0.201; // 20.1 cm

    // Estimate the pose of the detected markers
    cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

    // Create a list to store the results (marker ID, position, and orientation)
    mp_obj_t result_list = mp_obj_new_list(markerIds.size(), NULL);

    // Loop through the detected markers and get their position
    for (size_t i = 0; i < markerIds.size(); i++) {
        // Marker ID
        int marker_id = markerIds[i];

        // Translation vector (x, y, z) gives the position of the marker in camera coordinates
        cv::Vec3d translation = tvecs[i];

        // Convert rotation vector to a rotation matrix
        cv::Mat R_ct;
        cv::Rodrigues(rvecs[i], R_ct);

        // Calculate the roll, pitch, and yaw angles
        double sy = std::sqrt(R_ct.at<double>(0, 0) * R_ct.at<double>(0, 0) + R_ct.at<double>(1, 0) * R_ct.at<double>(1, 0));
        bool singular = sy < 1e-6;
        double roll, pitch, yaw;

        if (!singular) {
            roll = std::atan2(R_ct.at<double>(2, 1), R_ct.at<double>(2, 2));
            pitch = std::atan2(-R_ct.at<double>(2, 0), sy);
            yaw = std::atan2(R_ct.at<double>(1, 0), R_ct.at<double>(0, 0));
        } else {
            roll = std::atan2(-R_ct.at<double>(1, 2), R_ct.at<double>(1, 1));
            pitch = std::atan2(-R_ct.at<double>(2, 0), sy);
            yaw = 0;
        }

        // Convert radians to degrees
        roll = roll * 180.0 / CV_PI;
        pitch = pitch * 180.0 / CV_PI;
        yaw = yaw * 180.0 / CV_PI;

        // Calculate orientation based on roll, pitch, and yaw
        MarkerOrientation marker_orientation = calculate_orientation(roll, pitch, yaw);

        // Adjust the marker position based on orientation
        double SIN_30 = 0.5;
        double COS_30 = 0.86602540378;
        switch (marker_orientation) {
            case BOTTOM:
                translation[1] -= 0.10 * COS_30 + 0.10;
                translation[2] -= 0.10 * SIN_30;
                break;
            case LEFT:
                translation[0] -= 0.10 * COS_30 + 0.10;
                translation[2] -= 0.10 * SIN_30;
                break;
            case RIGHT:
                translation[0] += 0.10 * COS_30 + 0.10;
                translation[2] -= 0.10 * SIN_30;
                break;
            case TOP:
                translation[1] += 0.10 * COS_30 + 0.10;
                translation[2] -= 0.10 * SIN_30;
                break;
            default:
                break;
        }

        // Create a list to store the marker ID, position, and orientation
        mp_obj_t marker_info = mp_obj_new_list(5, NULL);
        mp_obj_list_store(marker_info, MP_OBJ_NEW_SMALL_INT(0), mp_obj_new_int(marker_id));
        mp_obj_list_store(marker_info, MP_OBJ_NEW_SMALL_INT(1), mp_obj_new_float(translation[0]));
        mp_obj_list_store(marker_info, MP_OBJ_NEW_SMALL_INT(2), mp_obj_new_float(translation[1]));
        mp_obj_list_store(marker_info, MP_OBJ_NEW_SMALL_INT(3), mp_obj_new_float(translation[2]));
        mp_obj_list_store(marker_info, MP_OBJ_NEW_SMALL_INT(4), mp_obj_new_int(marker_orientation));


        // Add this marker info to the result list
        mp_obj_list_store(result_list, MP_OBJ_NEW_SMALL_INT(i), marker_info);
    }

    // Return the list of marker IDs, their positions, and orientations
    return result_list;
}

// Define global dictionary
MP_DEFINE_CONST_FUN_OBJ_0(aruco_func_detect_position_obj, aruco_func_detect_position);

// Registering the function in the module
STATIC const mp_rom_map_elem_t aruco_module_globals_table[] = {
    {MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_aruco) },
    {MP_OBJ_NEW_QSTR(MP_QSTR_detect_position), (mp_obj_t)&aruco_func_detect_position_obj },
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_aruco_globals_dict,
    aruco_globals_table
);

const mp_obj_module_t aruco_module = {
    .base = {&mp_type_module },
     .globals = (mp_obj_dict_t*)&mp_module_aruco_globals_dict,
};

// Registering the function in MicroPython
MP_REGISTER_MODULE(MP_QSTR_aruco, aruco_module, MODULE_ARUCO_ENABLED);