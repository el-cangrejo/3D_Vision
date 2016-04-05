#include "kinect.hpp"

int prepare_kinect(int argc, char **argv) {
	bool die(false);
	string filename("snapshot");
	string suffix(".png");
	int i_snap(0),iter(0);

	Mat depthMat(Size(640,480),CV_16UC1);
	Mat depthf (Size(640,480),CV_8UC1);
	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
	Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));

	// The next two lines must be changed as Freenect::Freenect
	// isn't a template but the method createDevice:
	// Freenect::Freenect<MyFreenectDevice> freenect;
	// MyFreenectDevice& device = freenect.createDevice(0);
	// by these two lines:

	Freenect::Freenect freenect;
	MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	namedWindow("depth",CV_WINDOW_AUTOSIZE);
	device.startVideo();
	device.startDepth();
	while (!die) {
		device.getVideo(rgbMat);
		device.getDepth(depthMat);
		cv::imshow("rgb", rgbMat);
		depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
		cv::imshow("depth",depthf);
		char k = cvWaitKey(5);
		if( k == 27 ){
			cvDestroyWindow("rgb");
			cvDestroyWindow("depth");
			std::ostringstream file;
			file << filename << i_snap << suffix;
			cv::imwrite(file.str(),depthMat);
			break;
		}
		if( k == 8 ) {
			std::ostringstream file;
			file << filename << i_snap << suffix;
			cv::imwrite(file.str(),rgbMat);
			i_snap++;
		}
	}

	device.stopVideo();
	device.stopDepth();
	return 0;
}
