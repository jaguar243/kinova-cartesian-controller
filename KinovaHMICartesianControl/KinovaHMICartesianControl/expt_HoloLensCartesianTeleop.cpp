#include "stdafx.h"
#include "CartesianControl.h"

/* Pre-Processor Directives */
// Training phase
#define TRAIN true
// Testing phase (slow trajectory) 
#define TEST_1 false
// Testing phase (fast trajectory) 
#define TEST_2 false
// Inverse Controller implemented as an FIR Filter
#define INVERSE_CONTROLLER false
// Load human input from CSV file
#define LOAD_UHX_FROM_FILE false
// Load system output from CSV file
#define LOAD_XM_FROM_FILE false

/* Standard headers */
#include <vector>
#include <queue>
#include <future>

// Initialize the Novint Falcon Haptics Controller
NovintFalconHapticsDevice* novintFalcon = new NovintFalconHapticsDevice;

/*
HoloLens Hand Tracking in Cartesian Space (Basic)

Input	: connection = TCP connection to HoloLens for displaying robot configuration (Live)
: kinova = KinovaAPIFunctions object containing API functions to control the robot
Output	: None
*/
void Experiment::HoloLensCartesianTeleop(char *argv[], KinovaAPIFunctions kinova)
{
#if USE_KINOVA
	MovetoStartPos(kinova);
#endif
	// Socket Communication
	int portNb = 0;
	if (argv[0] != NULL)
		portNb = atoi(argv[1]);
	if (portNb == 0)
	{
		printf("Indicate the connection port number as argument! Using default Port number (11000)\n");
#ifdef _WIN32
		Sleep(5000);
#else
		usleep(5000 * 1000);
#endif
		portNb = 11000;
	}
	std::cout << "The Server port is " << portNb;
	CSocketInConnection connection(portNb);
#if TRAIN
	printf("\n\n------------STARTING TRAINING TRIALS----------\n\n");
#endif
#if TEST_1
	printf("\n\n------------STARTING TEST 1 TRIALS----------\n\n");
#endif
#if TEST_2
	printf("\n\n------------STARTING TEST 2 TRIALS----------\n\n");
#endif
	
	printf("\nConnecting to client...\n");
	if (!USE_TCP || connection.connectToClient())
	{
#if USE_TCP
		printf("\nConnected to Hololens!\n");
#endif
#if TRAIN
		std::ofstream data_file("train_data.csv");		// Data file for recording positions
#endif
#if TEST_1
		std::ofstream data_file("test1_data.csv");		// Data file for recording positions
#endif
#if TEST_2
		std::ofstream data_file("test2_data.csv");		// Data file for recording positions
#endif
		
		float stop_signal = 0, start_signal = 0;

		// Construct a kinect object
#if USE_KINECT
		kinectSkelTrack kinect;
		kinectSkelTrack::KinectInfo kin;
		std::cout << "Initializing Kinect API...\n";
		if (!kinect.initKinect())
		{
			std::cout << " Failed!";
			Sleep(5000);
			return;
		}
#endif
		/* HAPTIC DEVICE */
#if USE_HAPTICS
		// Initialize haptics device
		if (!novintFalcon->InitializeHapticsDevice())
		{
			std::cout << "\n\nCould not initialize the Haptics device... exiting!\n";
			Sleep(3000);
			return;
		}
		else
			std::cout << "\n\nFound haptics device!\n";
#endif

#if INVERSE_CONTROLLER
		// Construct a filter object and initialize
		std::cout << "Initializing FIR Filter Module..." << std::endl;
		FIRFilter filterx1, filterx2;
		filterx1.filename = "./ghinv1.csv";
		filterx2.filename = "./ghinv2.csv";
		if (!filterx1.firFloatInit()) {
			std::cout << "Failed to initialize FIR Filter Module 1!" << std::endl;
			Sleep(2000);
			return;
		}
		if (!filterx2.firFloatInit()) {
			std::cout << "Failed to initialize FIR Filter Module 2!" << std::endl;
			Sleep(2000);
			return;
		}
#endif

		// Kinova Position/Command Structures
		CartesianPosition currentPosition;
		CartesianPosition currentCommand;
		AngularPosition kinovaPositionFeedback;			// AngularPosition struct
		TrajectoryPoint v, gripper;

#if (USE_KINOVA)
		// Initialize the Kinova Position/Command Structures
		kinovaPositionFeedback.Actuators.InitStruct();	// Set up the position feedback struct

		v.InitStruct(); gripper.InitStruct();

		v.Position.Type = CARTESIAN_VELOCITY;
		gripper.Position.Type = CARTESIAN_POSITION;

		v.Position.CartesianPosition.X = 0.0f;
		v.Position.CartesianPosition.Y = 0.0f;
		v.Position.CartesianPosition.Z = 0.0f;
		v.Position.CartesianPosition.ThetaX = 0.0f;
		v.Position.CartesianPosition.ThetaY = 0.0f;
		v.Position.CartesianPosition.ThetaZ = 0.0f;
#endif		
		// Array for TCP communication with Hololens
		float send_debug[3]; // isRunning, DesPos[2]  
		float send[2]; //isRunning, DesPos[2], RobPos[2], JointPos[4], currentTime

		// Reference Position
		double DesPos[2] = { 0.0,0.0 }; 
		// Robot end-effector Position
		double RobPos[2] = { 0.0,0.0 };

		// Position PID Control 
		float Kp = 4.0f, Kd = 0.01f;// , Ki = 0.03f;

		std::cout << "*********************************" << std::endl;

		// Initialize the data logging into the CSV file
		data_file << "Time (ms), DesX, RobPosX, Ux, UhX, Act1, Act2, Act3, Act4" << std::endl;

		// Reference starting position of end-effector
		float refPos[2] = { 0.0f,0.0f };
#if (USE_KINOVA)
		kinova.MyGetCartesianPosition(currentPosition);
		refPos[0] = currentPosition.Coordinates.X;
		refPos[1] = currentPosition.Coordinates.Z;
#endif		
		// Input variables
		double uhx = 0.0, ux = 0.0, ucx = 0.0, uhx_filt, xm_filt, *filtp;
		filtp = &uhx_filt;

		// Robot model (first order)
		double tau = 8.0;
		int trial = 1, n_trials = 10;
		
		// Time vector
		Timer t, t_start, t2;
		T = 30.0; // Trial duration (s)
		Ts = 1 / 30.0; // sampling time (s)
		double Ts_LUT = 1 / 100.0; // Lookup table sampling time (s)
		
		// Load Lookup Tables from file for reference trajectory 
		std::vector <double*> xd_LUT; 
		std::vector <double> T_LUT;
		double *uhx_LUT = 0, *xm_LUT = 0;
		scale_LUT = -0.05;
		for (int k = 0; k < n_trials; k++)
		{
			char filename[100];
#if TRAIN
			sprintf(filename, "./qd_sumsines%d.csv", k+1);
#endif
#if TEST_1
			sprintf(filename, "./qd_slow%d.csv", 11);
#endif
#if TEST_2
			sprintf(filename, "./qd_fast%d.csv", 11);
#endif

			if (load_LUT1D(filename, '\n')) xd_LUT.push_back(LUT);
			else
			{
				std::cout << "\nLUT data for qd not loaded... Exiting!\n";
				Sleep(3000);
				return;
			}
			T = (double)(len_LUT * Ts_LUT);
			T_LUT.push_back(T);
		}
		
#if LOAD_UHX_FROM_FILE
		scale_LUT = 1;
		if (load_LUT1D("./uh.csv", '\n')) uhx_LUT = LUT;
		else
		{
			std::cout << "\nLUT data for uh not loaded... Exiting!\n";
			Sleep(3000);
			return;
		}
#endif
#if LOAD_XM_FROM_FILE
		scale_LUT = 1;
		if (load_LUT1D("./xm.csv", '\n')) xm_LUT = LUT;
		else
		{
			std::cout << "\nLUT data for xm not loaded... Exiting!\n";
			Sleep(3000);
			return;
		}
#endif
		
		std::cout << "\n\nTime period of trial: T = " << T << " seconds\n\n";
		
		std::cout << "Starting now...\n" << std::endl;

		// Buffer to hold input
		std::queue<double> buffer_uhx, buffer_xm;

#if GNUPLOT
		// Plotting variables
		std::vector<double> x, y1, y2, y3, y4;
		x.push_back(0); y1.push_back(0); y2.push_back(0); y3.push_back(0); y4.push_back(0);
		Gnuplot g1;
		g1.set_xrange(0, T);
		g1.set_yrange(-0.1, 0.1);
#endif
		// Initialize the program controls
		double currentTime = 0.0; // current time instant (s)
		bool isRunning = false; // boolean whether trial is running
		
		/* WAIT FOR USER START*/
		std::cout << "Waiting for user..." << std::endl;

#if USE_KINECT
		kin = kinect.getKinectData();
		while (!kin.startSignal) {
			kin = kinect.getKinectData();
			//auto res = std::async(&kinectSkelTrack::getKinectData, kinect);
			//kin = res.get();
			if (kin.userFound) {
				if (kin.startSignal) {
					isRunning = !isRunning;
					if (isRunning) std::cout << "\nUser requests Start ... Starting experiment ... \n";
					else std::cout << "\n User requests Pause ... Waiting ...\n";
					send[0] = isRunning;
					Sleep(1000);
					if (USE_TCP && !connection.replyToReceivedData((char*)send, sizeof(send)))
					{
						printf("Failed to send reply.\n");
						return;
					}
				}
			}
		}		
#endif
#if USE_HAPTICS
		//thread.get();
		//std::cout << "\nFirst thread.get() function called...\n";
		while (!novintFalcon->button2_state)//(!novintFalcon.button0_state)
		{
			novintFalcon->ResetIC();
			//std::cout << novintFalcon->button0_state << std::endl;
			//novintFalcon->UpdateHaptics();
			if (novintFalcon->button2_state)
			{
				isRunning = !isRunning;
				if (isRunning) std::cout << "\nUser requests Start ... Starting experiment ... \n";
				else std::cout << "\n User requests Pause ... Waiting ...\n";
				send[0] = isRunning;
				Sleep(1000);
				if (USE_TCP && !connection.replyToReceivedData((char*)send, sizeof(send)))
				{
					printf("Failed to send reply.\n");
					return;
				}
				break;
			}
		}
#endif
		// Set the trial duration (s)
		T = T_LUT.front();
		T_LUT.erase(T_LUT.begin());

		/* Start the experiment */
		t.start();
		while (trial <= n_trials)
		{ //This is the server loop (the robot's control loop):

			/* Uncomment this to receive data through TCP socket */
			/*
			int receivedLength = 0;
			char* received = connection.receiveData(receivedLength);

			if (received != NULL)
			{
				std::cout << receivedLength << " - " << received << " - " << *(float*)received << "\n";
				isRunning = *(float*)received;
			}
			*/

#if USE_KINECT
			kinect.getSkeletalData();
			uhx = ((double)kinect.skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT].x - (double)kinect.skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x) / 2.0f;
#endif
#if USE_HAPTICS
			novintFalcon->UpdateHaptics();
			if (novintFalcon->button0_state)
				uhx = 2.5 * novintFalcon->position.y();
			else
				uhx = 0;
#endif
#if !(USE_HAPTICS || USE_KINECT)
			isRunning = true;
			if (LOAD_UHX_FROM_FILE)
				uhx = interp_lut(currentTime, uhx_LUT, len_LUT, T, 1 / Ts_LUT);
			else
				uhx = 0;
			if (LOAD_XM_FROM_FILE)
				RobPos[0] = interp_lut(currentTime, xm_LUT, len_LUT, T, 1 / Ts_LUT);
			else
				RobPos[0] = 0;
#endif
			// Desired trajectory waypoints
			DesPos[0] = interp_lut(currentTime, xd_LUT[trial - 1], len_LUT, T, 1 / Ts_LUT);

			// Apply the inverse human model to get intent
#if INVERSE_CONTROLLER
			uhx_filt = filterx1.firFloat(&uhx, 1);
			xm_filt = filterx2.firFloat(&RobPos[0], 1);
#else
			uhx_filt = 0;
			xm_filt = 0;
#endif
			// If the trial is not running set input to Robot to zero
			if (!isRunning)
			{
				ux = 0;
				currentTime = 0.0;
			}
			else
			{
				currentTime = t.elapsedTime(); // Current time

				// Stop the trial after trial duration is exceeded
				if (currentTime > T) 
				{
					std::cout << "\nTrial " << trial++ << "complete...\n";

					// Check if number of trials exceeded
					if (trial <= n_trials)
					{
						// reset trial duration based on reference trajectory
						T = T_LUT.front();
						T_LUT.erase(T_LUT.begin());
						t.start();
					}
					else
					{
						// Stop the experiment
						isRunning = false;
						send[0] = isRunning;
						send[1] = 0;
						if (USE_TCP && !connection.replyToReceivedData((char*)send, sizeof(send)))
						{
							printf("Failed to send reply.\n");
							return;
						}
						novintFalcon->ResetIC();
					}
	
					// reset the time counter
					currentTime = 0;
				}
				// Total input
				/* Uncomment to test latency */
				//uhx = DesPos[0];
				ux = uhx;//uhx_filt + xm_filt;
				//std::cout << ux << std::endl;
			}
			
#if (USE_KINOVA)
			// Get current joint position commands
			kinova.MyGetCartesianCommand(currentCommand);
			// Get the current cartesian position of the robot end-effector.
			kinova.MyGetCartesianPosition(currentPosition);
			RobPos[0] = currentPosition.Coordinates.X - refPos[0];
			//RobPos[1] = currentPosition.Coordinates.Z - refPos[1];
			v.Position.CartesianPosition.X = (float)(Kp * (ux + refPos[0] - currentPosition.Coordinates.X));
			//v.Position.CartesianPosition.Z = (float)(Kp * (uz + refPos[1] - currentPosition.Coordinates.Z));
			// Send cartesian velocity commands
			kinova.MySendBasicTrajectory(v);
			// Get the current angular position of the robot joints
			kinova.MyGetAngularPosition(kinovaPositionFeedback);
			// Write Current Cartesian Coordinates to Console
			//std::cout << currentPosition.Coordinates.X << "," << currentPosition.Coordinates.Y << "," << currentPosition.Coordinates.Z << std::endl;
				
			if (isRunning)
			{
				// Log data to file
				data_file << currentTime << "," 
					<< DesPos[0] << "," 
					<< RobPos[0] << "," 
					<< ux << "," 
					<< uhx << ","
					<< kinovaPositionFeedback.Actuators.Actuator1 << ","
					<< kinovaPositionFeedback.Actuators.Actuator2 << ","
					<< kinovaPositionFeedback.Actuators.Actuator3 << ","
					<< kinovaPositionFeedback.Actuators.Actuator4 << ","
					<< std::endl;
			}
			// Uncomment to send data to Hololens
			send[1] = currentTime;
			//send[2] = DesPos[0];
			//send[3] = RobPos[0];
			//connection.replyToReceivedData((char*)send, sizeof(send));
#endif
		}
		std::cout << "\n\n EXPERIMENT COMPLETE... BYE BYE!";
		data_file.close();
	}
	else
	{
		std::cout << "\nCould not connect to UWP Server! Exiting\n";
	}

#ifdef _WIN32
	Sleep(5000);
#else
	usleep(5000 * 1000);
#endif
}