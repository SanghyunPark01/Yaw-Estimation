/*
------------------------------------------------
                 <Yaw Estimation>

This project is Yaw estimstion for Mobile Robot. 
It use ArUco marker and Intel Realsense d435

Made by Sanghyun Park from South Korea.
GitHub : SanghyunPark01
E-mail : pash0302@naver.com
E-mail : pash0302@gmail.com

If you find error, please contact me.
------------------------------------------------
*/
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>
#include <exception>
#include <string.h>
#include <cmath>

static cv::Mat frame_to_mat(const rs2::frame& f);
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f);
double rad_to_deg(double r);
cv::Vec3f Deproject_2D_to_3D(cv::Point2f pixel, float depth);

class Ar_marker { //marker information
private:
    cv::Vec3f corner_lu, corner_ld, corner_ru, corner_rd, center_WP;
    cv::Vec3f tan_ld, tan_lu, tan_rd, tan_ru;
    double depth_C=0, depth_lu=0, depth_ld=0, depth_ru=0, depth_rd=0;
    std::vector<cv::Point2f> _v_C;
    std::vector<cv::Vec3f> _v_Cv;
    double dyaw=0;
    cv::Vec3f tV(void);
public:
    Ar_marker(){};
    void setting(std::vector<cv::Point2f> corner, cv::Mat depth_value);
    void get_yaw(void);
    double depth(void) {
        return depth_C;
    }
    cv::Vec3f center(void) {
        return center_WP;
    }
    std::vector<cv::Point2f> all_corner(void) {
        return _v_C;
    }
    double yaw(void) {
        return dyaw;
    }
    ~Ar_marker(){};
};
void Ar_marker::setting(std::vector<cv::Point2f> corner, cv::Mat depth_value_img) {
    
    //Marker's Center
    cv::Point2f center_buff(0.f, 0.f);
    std::vector<cv::Point2f> _v_cbuff;//0: lu, 1: ru, 2: rd, 3: ld
    for (const auto& corner_a : corner) {
        center_buff += corner_a;
        _v_cbuff.push_back(corner_a);
    }
    _v_C = _v_cbuff;
    center_buff /= 4.f;
    depth_C = depth_value_img.at<double>((int)center_buff.y, (int)center_buff.x);
    center_WP = Deproject_2D_to_3D(center_buff, depth_C);

    //corner world frame    
    depth_lu= depth_value_img.at<double>((int)_v_cbuff[0].y, (int)_v_cbuff[0].x);
    corner_lu = Deproject_2D_to_3D(_v_cbuff[0], depth_lu);

    depth_ru = depth_value_img.at<double>((int)_v_cbuff[1].y, (int)_v_cbuff[1].x);
    corner_ru = Deproject_2D_to_3D(_v_cbuff[1], depth_ru);

    depth_rd = depth_value_img.at<double>((int)_v_cbuff[2].y, (int)_v_cbuff[2].x);
    corner_rd = Deproject_2D_to_3D(_v_cbuff[2], depth_rd);
    
    depth_ld = depth_value_img.at<double>((int)_v_cbuff[3].y, (int)_v_cbuff[3].x);
    corner_ld = Deproject_2D_to_3D(_v_cbuff[3], depth_ld);

    _v_Cv.push_back(corner_lu);
    _v_Cv.push_back(corner_ru);
    _v_Cv.push_back(corner_rd);
    _v_Cv.push_back(corner_ld);
    //Marker's tangential vector of 4 corner
    tan_ld = (corner_lu - corner_ld).cross((corner_rd - corner_ld));
    tan_lu = (corner_ru - corner_lu).cross((corner_ld - corner_lu));
    tan_rd = (corner_ld - corner_rd).cross((corner_ru - corner_rd));
    tan_ru = (corner_rd - corner_ru).cross((corner_lu - corner_ru));

    /*std::cout << tan_ld << std::endl;
    std::cout << tan_lu << std::endl;
    std::cout << tan_rd << std::endl;
    std::cout << tan_ru << std::endl;*/

    Ar_marker::get_yaw();
}
cv::Vec3f Ar_marker::tV(void) {
    cv::Vec3f ResultV;
    cv::Vec3f Zero_vec(0, 0, 0);
    //filtering
    double dL1, dL2, dL3, dL4;
    dL1 = pow(corner_lu(0) - corner_ld(0), 2) + pow(corner_lu(1) - corner_ld(1), 2) + pow(corner_lu(2) - corner_ld(2), 2);
    dL2 = pow(corner_ld(0) - corner_rd(0), 2) + pow(corner_ld(1) - corner_rd(1), 2) + pow(corner_ld(2) - corner_rd(2), 2);
    dL3 = pow(corner_lu(0) - corner_ru(0), 2) + pow(corner_lu(1) - corner_ru(1), 2) + pow(corner_lu(2) - corner_ru(2), 2);
    dL4 = pow(corner_rd(0) - corner_ru(0), 2) + pow(corner_rd(1) - corner_ru(1), 2) + pow(corner_rd(2) - corner_ru(2), 2);
    
    double dL_avg = (dL1 + dL2 + dL3 + dL4) / 4;
    double threshold = dL_avg * 0.1;
    cv::Vec3f AVG = (tan_lu + tan_ld + tan_ru + tan_rd) / 4;
    //std::cout << "AVG : " << AVG << std::endl;
    if ((dL1 - dL_avg > threshold) || (dL2 - dL_avg > threshold) || (dL3 - dL_avg > threshold) || (dL4 - dL_avg > threshold)||tan_ld==Zero_vec||tan_lu == Zero_vec || tan_ru == Zero_vec || tan_rd == Zero_vec) {
        if (tan_ld == Zero_vec && tan_lu == Zero_vec && tan_ru == Zero_vec && tan_rd == Zero_vec) {
            std::cout << "Trash Marker : Too Close Depth" << std::endl;
            ResultV(0) = 0;
            ResultV(1) = 0;
            ResultV(2) = 1;
        }
        else {
            std::cout << "Trash Marker : Uncorrect Vector" << std::endl;
            int cnt = 0;
            for (int i = 0; i < 4; i++) {
                if (_v_Cv[i] == Zero_vec) {
                    cnt++;
                }
            }
            if (cnt >= 2) {
                std::cout << "Can't calculate" << std::endl;
                ResultV(0) = 0;
                ResultV(1) = 0;
                ResultV(2) = 1;
            }
            else {
                if (tan_ld != Zero_vec) {
                    ResultV = tan_ld;
                }
                else if (tan_lu != Zero_vec) {
                    ResultV = tan_lu;
                }
                else if (tan_rd != Zero_vec) {
                    ResultV = tan_rd;
                }
                else if (tan_ru != Zero_vec) {
                    ResultV = tan_ru;
                }
                else {
                    ResultV(0) = 0;
                    ResultV(1) = 0;
                    ResultV(2) = 1;
                }
            }
        }
    }
    else {
        ResultV = AVG;
    }
    return ResultV;
}
void Ar_marker::get_yaw(void) {
    cv::Vec3f  _A_tanV = Ar_marker::tV();
    std::cout << "tangential vector : " << _A_tanV << std::endl;
    double dDia = sqrt(pow(_A_tanV(0), 2) + pow(_A_tanV(2), 2));
    double dyaw_temp = acos(_A_tanV(2) / dDia);
    if (_A_tanV(0) < 0)dyaw = -dyaw_temp;
    else dyaw = dyaw_temp;
}

int main(int argc, char* argv[]) try
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    rs2::align align_to(RS2_STREAM_COLOR);
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    using namespace cv;
    using namespace std;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    Mat cameraMatrix = Mat::eye(3, 3, CV_64FC1);
    Mat distCoeffs = Mat::zeros(1, 5, CV_64FC1);
    cameraMatrix = (Mat1d(3, 3) << 618.53013, 0., 324.7346, 0., 616.45867, 246.24308, 0., 0., 1.);
    distCoeffs = (Mat1d(1, 5) << 0.123868, -0.281684, -0.002987, 0.000575, 0.);
    double u0 = 324.7346;
    double v0 = 246.24308;

    while (waitKey(1)<0)
    {
        //Get Frame
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        //rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::frameset aligned_set = align_to.process(data);
        rs2::frame depth = aligned_set.get_depth_frame();
        auto image_rgb = frame_to_mat(aligned_set.get_color_frame());
        color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2);
        rs2::frame bw_depth = depth.apply_filter(color_map);
        auto image_depth = frame_to_mat(bw_depth);
        //imshow("rgb", color_mat);
        //imshow("depth", near);

        Mat image_output;
        //image_rgb.copyTo(image_output);
        
        //Calibration
        undistort(image_rgb, image_output, cameraMatrix, distCoeffs);
        //imshow("calib", image_output);
        //imshow("uncalib", image_rgb);
         
        //Align Check

        imshow("depth image", image_depth);
        //imshow("rgb image", image_output);

        Mat depth_value = Mat::zeros(480, 640, CV_64F);

        depth_value = depth_frame_to_meters(pipe, depth);
        //imshow("depth value", depth_value);
        //auto distance_d = depth_value.at<double>(320, 240);//center 320, 240
        //cout << distance_d << endl;
        
        //Get Marker
        vector<int>ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(image_output, dictionary, corners, ids);
        //cout << corners.size() << endl;
        //if marker exist
        if (ids.size() > 0) {

            vector<Ar_marker> _v_marker;
            for (const auto& corner : corners)
            {
                Ar_marker marker_temp;
                marker_temp.setting(corner, depth_value);
                _v_marker.push_back(marker_temp);
            }
            for (int i = 0; i < _v_marker.size(); i++) {
                //cout << "Marker Position : " << _v_marker[i].center() << endl;
                cout << "Yaw : " << rad_to_deg(_v_marker[i].yaw()) << endl << endl;
            }


            //visualize
            for (int i = 0; i < _v_marker.size(); i++) {  
                /*putText(image_output, "LU", Point(_v_marker[i].all_corner()[0].x, _v_marker[i].all_corner()[0].y), 1, 1, Scalar(255, 255, 100));
                putText(image_output, "LD", Point(_v_marker[i].all_corner()[3].x, _v_marker[i].all_corner()[3].y), 1, 1, Scalar(255, 255, 100));
                putText(image_output, "RD", Point(_v_marker[i].all_corner()[2].x, _v_marker[i].all_corner()[2].y), 1, 1, Scalar(255, 255, 100));
                putText(image_output, "RU", Point(_v_marker[i].all_corner()[1].x, _v_marker[i].all_corner()[1].y), 1, 1, Scalar(255, 255, 100));*/
            }
            aruco::drawDetectedMarkers(image_output, corners, ids);
        }else{ 
            putText(image_output, "No Marker !!", Point(25, 60), 1, 5, Scalar(0, 0, 255));
        }
        imshow("result", image_output);
        
    }
    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
static cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
    using namespace cv;
    using namespace rs2;

    Mat dm = frame_to_mat(f);
    dm.convertTo(dm, CV_64F);
    auto depth_scale = pipe.get_active_profile()
        .get_device()
        .first<depth_sensor>()
        .get_depth_scale();
    dm = dm * depth_scale;
    return dm;
}
double rad_to_deg(double r)
{
    double d;
    
    d = r * 180 / 3.14;
    if (d < -180) {
        d += 360;
    }else if(d > 180){
        d -= 360;
    }

    return d;
}
cv::Vec3f Deproject_2D_to_3D(cv::Point2f pixel, float depth) {
    double fx, fy, ppx, ppy;
    fx = 618.530133;
    fy = 616.458672;
    ppx = 324.734598;
    ppy = 246.243079;

    double x_s = (pixel.x - ppx) / fx;
    double y_s = (pixel.y - ppy) / fy;

    cv::Vec3f point;
    point(0) = depth * x_s;
    point(1) = depth * y_s;
    point(2) = depth;

    return point;
}