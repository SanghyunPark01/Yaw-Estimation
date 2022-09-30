//#include <librealsense2/rs.hpp>
//#include <opencv2/opencv.hpp>
//#include <iostream>
//
//int main(int argc, char* argv[]) try
//{
//    // Declare depth colorizer for pretty visualization of depth data
//    rs2::colorizer color_map;
//
//    // Declare RealSense pipeline, encapsulating the actual device and sensors
//    rs2::pipeline pipe;
//    // Start streaming with default recommended configuration
//    pipe.start();
//
//    using namespace cv;
//
//    while (1)
//    {
//        rs2::frameset data_d = pipe.wait_for_frames(); // Wait for next set of frames from the camera
//        rs2::frame depth = data_d.get_depth_frame().apply_filter(color_map);
//        rs2::frameset data_rgb = pipe.wait_for_frames();// Wait for next set of frames from the camera
//        rs2::frame rgb = data_rgb.get_color_frame().apply_filter(color_map);
//
//        // Query frame size (width and height)
//        const int w_depth = depth.as<rs2::video_frame>().get_width();
//        const int h_depth = depth.as<rs2::video_frame>().get_height();
//
//        const int w_rgb = rgb.as<rs2::video_frame>().get_width();
//        const int h_rgb = rgb.as<rs2::video_frame>().get_height();
//
//        // Create OpenCV matrix of size (w,h) from the colorized depth data
//        Mat image_depth(Size(w_depth, h_depth), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
//        Mat image_rgb(Size(w_rgb, h_rgb), CV_8UC3, (void*)rgb.get_data(), Mat::AUTO_STEP);
//        cvtColor(image_rgb, image_rgb,COLOR_BGR2RGB);
//
//        std::cout << "good" << std::endl;
//        // Update the window with new data
//        imshow("depth img", image_depth);
//        imshow("rgb img", image_rgb);
//        waitKey(1);
//    }
//
//    return EXIT_SUCCESS;
//}
//catch (const rs2::error& e)
//{
//    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//    return EXIT_FAILURE;
//}
//catch (const std::exception& e)
//{
//    std::cerr << e.what() << std::endl;
//    return EXIT_FAILURE;
//}
