//===========================================================================
/*
MIT License

Copyright (c) 2018 Rahul B. Warrier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
//===========================================================================
//===========================================================================
/*! \mainpage Introduction

This project is a C++ implementation of a Cartesian controller that provides 
basic human-machine interface modules for real-time end-effector Cartesian 
position/velocity control of the Kinova Mico2 robot arm using the built-in 
velocity controllers in the Kinova API. The project is written in C++ and 
is intended to be run on Windows 8.1/10 with Visual Studio 2017 (v15). The 
code is distributed under the MIT license for maximum flexibility of use. 
By using this software, you are agreeing to the license. Please read the 
license prior to using this project.
 
This introduction is broken down into the following sections.
- \subpage license
- \subpage install

A note on units:  All units in the library, unless specified, are in SI
(international standard), i.e.: radians, meters, kilograms, etc...
 
 
*/
//---------------------------------------------------------------------------
/*!
\page license Software License

MIT License

Copyright (c) 2018 Rahul B. Warrier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 
*/
//---------------------------------------------------------------------------
/*!
\page install Installation Notes

This project was built in Visual Studio 2017 (Community Edition) and the 
solution file is provided for reference. Additional dependencies need to be 
installed before building this project with the source and library files from
 the dependencies correctly linked to the project. Instructions for setting
 up the project and building it are presented below.

 \section install_dependencies Install Dependencies
 The following are the list of dependencies that are used in this project:
 - \subpage kinova_sdk [to control the Kinova Mico-2 robot arm]
 - \subpage kinect_sdk [to use the Microsoft Kinect sensor for skeleton tracking]
 - \subpage chai3d_sdk [to use the Novint Falcon Haptics controller]

 \subsection kinova_sdk Kinova Mico2 SDK
 1. Download the SDK from the <a href="https://www.kinovarobotics.com/en/knowledge-hub/all-kinova-products">Kinova website</a> and follow the instructions in the documentation to install the 32-bit version of the API.
 2. Set the System Environment Variable KINOVASDK_DIR to the installation directory.

 \subsection kinect_sdk Kinect SDK v1.8
 1. Download and install the Kinect v1.8 SDK from the <a href="https://www.microsoft.com/en-us/download/details.aspx?id=40278">Microsoft website</a> and follow the instructions in the documentation to install the 32-but version of the API.
 2. Check whether the system environment variable KINECTSDK10_DIR is set to the correct installation directory.

 \subsection chai3d_sdk CHAI3D (version 3.2.0)
 1. Download the multiplatform version of the CHAI3D SDK (currently tested with version 3.2.0) from the <a href="http://www.chai3d.org/download/releases">CHAI3D website</a>
 2. Install the dependencies (HDF5 and ZLIB) and build the appropriate Visual Studio Solution that matches the available edition.
 3. Set the system environment variable CHAI3D_DIR to the installation directory

*/
//---------------------------------------------------------------------------
/*!
\page overview Overview

*/
//---------------------------------------------------------------------------
/*!
\page gettingstarted Getting Started

*/
//---------------------------------------------------------------------------
/*!
\page references References

*/
//---------------------------------------------------------------------------
#pragma once
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

// No MFC
#define WIN32_LEAN_AND_MEAN
#define USE_KINOVA true
#define USE_KINECT false
#define USE_TCP true
#define USE_HAPTICS true
#define GNUPLOT false

// Windows Includes
#include <WinSock2.h>
#include <Windows.h>
#include "socketInConnection.h"
#include "CommunicationLayerWindows.h"
#include "CommandLayer.h"
#include <conio.h>
#include "KinovaTypes.h"
#include <iostream>
#include <thread>
#include <cstdlib>
#include <fstream>
#include <string>
#include <sstream>
#include <Xinput.h>
#include <cstdio>
#include <queue>

// Kinect includes
#include <Ole2.h>
#include "NuiApi.h"
#include "NuiImageCamera.h"
#include "NuiSensor.h"
#include "NuiSkeleton.h"

// CHAI3D (Novint Falcon Haptics Device) includes
#include "chai3d.h"

// Standard headers
#include <cmath>
//#define M_PI 3.14
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <valarray> 
#include <cstdio>
#include <array>

/*
// OpenCV includes
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////END OF PREAMBLE/////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _XBOX_CONTROLLER_H_
#define _XBOX_CONTROLLER_H_

// No MFC
#define WIN32_LEAN_AND_MEAN

// Now, the XInput Library
// NOTE: COMMENT THIS OUT IF YOU ARE NOT USING
// A COMPILER THAT SUPPORTS THIS METHOD OF LINKING LIBRARIES
//#pragma comment(lib, "XInput.lib")

// XBOX Controller Class Definition
class CXBOXController
{
private:
	XINPUT_STATE _controllerState;
	int _controllerNum;
public:
	CXBOXController(int playerNumber);
	XINPUT_STATE GetState();
	bool IsConnected();
	void Vibrate(int leftVal = 0, int rightVal = 0);
};

#endif

// Timer class : To find time elapsed
using namespace std::literals::chrono_literals;
class Timer {
private:
	unsigned long begTime;
public:
	void start() {
		begTime = clock();
	}

	double elapsedTime() {
		return ((double)clock() - begTime) / CLOCKS_PER_SEC;
	}

	bool isTimeout(unsigned long seconds) {
		return seconds >= elapsedTime();
	}
};

// Kinova API Class
class KinovaAPIFunctions
{
public:
	//Function pointers to the functions we need
	int(*MyInitAPI)();
	int(*MyCloseAPI)();
	int(*MyStartControlAPI)();
	int(*MyStartForceControl)();
	int(*MyStopForceControl)();
	int(*MyGetClientConfigurations)(ClientConfigurations &config);
	int(*MySetClientConfigurations)(ClientConfigurations config);
	int(*MySendJoystickCommand)(JoystickCommand joystickCommand);
	int(*MyGetGlobalTrajectoryInfo)(TrajectoryFIFO &Response);
	int(*MyRunGravityZEstimationSequence)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE]);
	int(*MySendBasicTrajectory)(TrajectoryPoint command);
	int(*MyGetActualTrajectoryInfo)(TrajectoryPoint &);
	int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
	int(*MySetActiveDevice)(KinovaDevice device);
	int(*MySetGravityOptimalZParam)(double optimalZParams[OPTIMAL_Z_PARAM_SIZE]);
	int(*MySetGravityType)(GRAVITY_TYPE type);
	int(*MyMoveHome)();
	int(*MyInitFingers)();
	int(*MyGetCartesianCommand)(CartesianPosition &);
	int(*MyGetCartesianPosition)(CartesianPosition &);
	int(*MyGetAngularPosition)(AngularPosition &);
	int(*MySetActuatorPID)(unsigned int adress, float P, float I, float D);
	int(*MyGetGripperStatus)(Gripper &);
};

// Kinect Skeleton Tracking
class kinectSkelTrack
{
public:
	int width = 640;
	int height = 480;
	// Kinect variables
	HANDLE depthStream;              // The identifier of the Kinect's RGB Camera
	INuiSensor* sensor;            // The kinect sensor
	
								   // Body tracking variables
	Vector4 skeletonPosition[NUI_SKELETON_POSITION_COUNT];
	
	struct KinectInfo {
		bool startSignal = false;
		bool userFound = false;
		double handPosition = 0;
	};

	// Kinect Functions
	bool initKinect();
	void getSkeletalData();
	KinectInfo getKinectData();
};

/* OpenCV Position Tracking
class OpenCVCamera
{
public:
	cv::VideoCapture cap;
	cv::VideoWriter oVideoWriter;
	std::vector<cv::Vec3f> circles;
	char *windowName;
	float DesiredPos[2], HumanPos[2], RobotPos[2];
	const char *calibFile = "C:/Users/UPCLab2013/Dropbox/data/calibration_data/calib.mat";
	mat_t *mat = Mat_Open(calibFile, MAT_ACC_RDONLY);
	double calibGainXZ[2];
	cv::Mat expt_image;
	float center[2], circlePos[4];
	cv::Rect myROI;
	cv::Mat frame;


	// Initialize camera for writing to file
	bool initVideoCap();
	void flush();
	void centering(double Ts);
	void drawDesiredCircle(float timeLeft, int trial, bool running, int trial_id);
	bool writeVideoToFile(char *filename);
	void getObjectPos(cv::Mat, int flag);
};
*/
// FIR Filter Class
class FIRFilter
{
public:
	/// Filename of FIR Filter weights
	const char *filename = "./lowpass.mat";
	
	/// Buffer length to hold input samples of FIR Filter
#define BUFFER_LEN 96 

	int length = 1;
	double *output = new double[length];

	/// buffer array to hold input samples
	double insamp[BUFFER_LEN]; 
	/// FIR Filter weights
	double *coeffs; 
	/// Cutoff frequency of the FIR Filter in Hz
	double cutoff_freq_hz = 0.2; 

	/// Function to initialize the FIR Filter
	bool firFloatInit(); 
	/// Function to compute the output of the FIR Filter
	double* firFloat(double *input, int length); 
};

// Experiment Class
class Experiment
{
public:
	// Time period
	double T = 10.0, T_calib = 10.0, T_TF = 10.0;
	// Sampling time
	double Ts = 1.0 / 125.0;
	// CONSTANTS
	const double fs_qd_lut = 3000.0;
	const double fs_uc_lut = 3000.0;
	double scale_LUT = 1;
	
	// Lookup Table
	double *LUT;
	int len_LUT;

	// PID Velocity Control
	const float Kp = 2.0f;// , Kd = 0.03f;// , Ki = 0.03f;

	// Experiment methods
	void HoloLensCartesianTeleop(char *argv[], KinovaAPIFunctions kinova);
	
	// Function prototypes for other methods
	double interp_lut(double t, double *y_lut, int L_lut, double Tp_lut, double fs_lut);
	void MovetoStartPos(KinovaAPIFunctions kinova);
	void MoveEndEffectorPos(KinovaAPIFunctions kinova, float xe, float ze);
	bool load_LUT1D(char *filename, char delim);
};

// Novint Falcon Haptics Device Class
class NovintFalconHapticsDevice
{
public:
	// a pointer to the current haptic device
	chai3d::cGenericHapticDevicePtr hapticDevice;
	// a global variable to store the position [m] of the haptic device
	chai3d::cVector3d hapticDevicePosition;
	// haptic thread
	chai3d::cThread* hapticsThread;
	// a haptic device handler
	chai3d::cHapticDeviceHandler* handler;

	double maxforce = 1;
	// position
	chai3d::cVector3d position;
	//orientation
	chai3d::cMatrix3d rotation;
	// linear velocity 
	chai3d::cVector3d linearVelocity;
	// angular velocity
	chai3d::cVector3d angularVelocity;
	// desired position
	chai3d::cVector3d desiredPosition;
	// desired orientation
	chai3d::cMatrix3d desiredRotation;

	// button 0 state (ON/OFF)
	bool isRunning = false;
	bool button0_state = false;
	bool button2_state = false;
	// a flag for using damping (ON/OFF)
	bool useDamping = false;
	// a flag for using force field (ON/OFF)
	bool useForceField = false;
	// initialize the device
	bool InitializeHapticsDevice();
	void UpdateHaptics(void);
	void ResetIC(void);
};