// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp"          // Include short list of convenience functions for rendering
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <librealsense2/example.hpp>

#include "opencv2/opencv.hpp"
// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
using namespace cv;
using namespace std;
int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	// window app(1280, 720, "RealSense Capture Example");
	// Declare two textures on the GPU, one for color and one for depth
	// texture depth_image, color_image;

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_device_from_file("C:/Users/Nick/Downloads/3.bag");
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	// Start streaming with default recommended configuration
	pipe.start(cfg);
	Mat ir_gray;
	//int thresh = 150;
	RNG rng(12345);
	cv::Mat temp;
	cv::Mat eroded;

	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	while (true) // Application still alive?
	{
		rs2::frameset frames;
		//for (int i = 0; i < 30; i++)
		//{
		//Wait for all configured streams to produce a frame
		frames = pipe.wait_for_frames();
		////}
		rs2::colorizer color_map;
		color_map.set_option(RS2_OPTION_COLOR_SCHEME, 3.0);
		rs2::frame depth = color_map(frames.get_depth_frame());       // Find the color data
		Mat ir(Size(640, 480), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
		/*	for (int i = 0; i < ir.rows; i++)
		for (int j = 0; j < ir.cols; j++)
		if (ir.at<Vec3b>(i, j)[0] > 0 ||
		ir.at<Vec3b>(i, j)[1] > 0 ||
		ir.at<Vec3b>(i, j)[2] > 0)
		{
		ir.at<Vec3b>(i, j)[0] = 255;
		ir.at<Vec3b>(i, j)[1] = 255;
		ir.at<Vec3b>(i, j)[2] = 255;
		}
		*/
		// Apply Histogram Equalization
		//equalizeHist(ir, ir);
		//applyColorMap(ir, ir, COLORMAP_JET);

		//	cv::threshold(ir, ir, 0, 127, cv::THRESH_BINARY);
		cvtColor(ir, ir, COLOR_BGR2GRAY);
		cv::Mat skel(ir.size(), CV_8UC1, cv::Scalar(0));


		bool done;
		do
		{
			cv::erode(ir, eroded, element);
			cv::dilate(eroded, temp, element); // temp = open(img)
			cv::subtract(ir, temp, temp);
			cv::bitwise_or(skel, temp, skel);
			eroded.copyTo(ir);

			done = (cv::countNonZero(ir) == 0);
		} while (!done);
		////cvtColor(ir, ir_gray, COLOR_BGR2GRAY);
		Mat canny_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		/// Detect edges using canny
		Canny(skel, canny_output, 100, 255, 3);
		/// Find contours
		findContours(canny_output, contours, hierarchy, 0, CHAIN_APPROX_SIMPLE, Point(0, 0));

		///// Draw contours

		Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
		for (int i = 0; i< contours.size(); i++)
		{
			Scalar color = Scalar(0, 0, 255);
			drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		}
		// Display the image in GUI
		//namedWindow("1", WINDOW_AUTOSIZE);
		namedWindow("2", WINDOW_AUTOSIZE);
		//imshow("1", ir_gray);
		imshow("2", drawing);

		// Render depth on to the first half of the screen and color on to the second
		//depth_image.render(depth, { 0,               0, app.width() , app.height() });
		//color_image.render(color, { app.width() / 2, 0, app.width() / 2, app.height() });
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}



