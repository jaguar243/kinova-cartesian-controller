#include "stdafx.h"
#include "CartesianControl.h"

//A handle to the API.
HINSTANCE commandLayer_handle;

int main(int argc, char* argv[])
{
	Experiment experiments;

	//freopen("output.log", "w", stdout);
	//freopen("CON", "w", stdout);

	HWND consoleWindow = GetConsoleWindow();
	//SetWindowPos(consoleWindow, 0, 1275, 725, 0, 0, SWP_NOSIZE | SWP_NOZORDER);

	/*
	HINSTANCE hinst = LoadLibrary(L"C:\\Users\\UPCLab2013\\Dropbox\\1. Udrive\\lab\\1. Robot Arm Research\\Cartesian_Control_Step2D\\KeyBoardLogger\\Debug\\KeyBoardLogger.dll");
	if (hinst == NULL)
	{
	printf("null hinst");
	return 0;
	}
	typedef void(*Install)();
	typedef void(*Uninstall)();

	Install install = (Install)GetProcAddress(hinst, "install");
	Uninstall uninstall = (Uninstall)GetProcAddress(hinst, "uninstall");

	install();
	*/

	/*
	// Test the simulation window
	Experiment experiments;
	experiments.testSimulationWindow();
	return 0;
	*/
	std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << std::endl << std::endl;
	// Load Kinova Library
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");
	bool loadWasSuccesful = false;
	int programResult = 0;
	std::cout << "Loading the Kinova API functions... " << std::endl;
	//We load the functions from the library (Under Windows, use GetProcAddress)
	KinovaAPIFunctions kinova;
	kinova.MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	kinova.MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	kinova.MyStartControlAPI = (int(*)()) GetProcAddress(commandLayer_handle, "StartControlAPI");
	kinova.MyStartForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StartForceControl");
	kinova.MyStopForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StopForceControl");
	kinova.MyGetClientConfigurations = (int(*)(ClientConfigurations &config)) GetProcAddress(commandLayer_handle, "GetClientConfigurations");
	kinova.MySetClientConfigurations = (int(*)(ClientConfigurations config)) GetProcAddress(commandLayer_handle, "SetClientConfigurations");
	kinova.MySendJoystickCommand = (int(*)(JoystickCommand joystickCommand)) GetProcAddress(commandLayer_handle, "SendJoystickCommand");
	kinova.MyGetGlobalTrajectoryInfo = (int(*)(TrajectoryFIFO&)) GetProcAddress(commandLayer_handle, "GetGlobalTrajectoryInfo");
	kinova.MyRunGravityZEstimationSequence = (int(*)(ROBOT_TYPE type, double OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) GetProcAddress(commandLayer_handle, "RunGravityZEstimationSequence");
	kinova.MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	kinova.MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
	kinova.MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
	kinova.MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	kinova.MySetGravityOptimalZParam = (int(*)(double optimalZParam[OPTIMAL_Z_PARAM_SIZE])) GetProcAddress(commandLayer_handle, "SetGravityOptimalZParam");
	kinova.MySetGravityType = (int(*)(GRAVITY_TYPE type)) GetProcAddress(commandLayer_handle, "SetGravityType");
	kinova.MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	kinova.MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
	kinova.MyGetCartesianPosition = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianPosition");
	kinova.MySetActuatorPID = (int(*)(unsigned int, float, float, float)) GetProcAddress(commandLayer_handle, "SetActuatorPID");
	kinova.MyGetAngularPosition = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularPosition");
	kinova.MyGetGripperStatus = (int(*)(Gripper &)) GetProcAddress(commandLayer_handle, "GetGripperStatus");
	//Verify that all functions has been loaded correctly
	if ((kinova.MyInitAPI == NULL) || (kinova.MyCloseAPI == NULL) || (kinova.MySendBasicTrajectory == NULL) ||
		(kinova.MyGetDevices == NULL) || (kinova.MySetActiveDevice == NULL) || (kinova.MyGetCartesianCommand == NULL) ||
		(kinova.MyGetCartesianPosition == NULL) || (kinova.MyMoveHome == NULL) || (kinova.MyInitFingers == NULL) ||
		(kinova.MySetActuatorPID == NULL) || (kinova.MyGetAngularPosition == NULL) || (kinova.MyGetGripperStatus == NULL))
	{
		std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << std::endl;
		programResult = 0;
		Sleep(3000);
	}
	else
	{
		std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << std::endl << std::endl;
#if (!USE_KINOVA)
		if (argv[1] == NULL)
			argv[1] = "11000";
		experiments.HoloLensCartesianTeleop(argv, kinova);
#else
		std::cout << "Initializing the Kinova API...\n";
		int result = (*kinova.MyInitAPI)();                    // Initialize the Kinonva API
		std::cout << "The result of Kinova InitAPI = " << result << "\n";
		if (result == 1) {
			result = (*kinova.MyStartControlAPI)();            // Assume control with Kinova API
			std::cout << "The result of Kinova StartControlAPI = " << result << "\n";
			if (result == 1) {
				result = (*kinova.MyMoveHome)();                   // Go to the init positions
				std::cout << "The result of Kinova MoveHome = " << result << "\n";
				if (result == 1) {
					loadWasSuccesful = true;
					std::cout << "Initialization's result :" << result << std::endl;
					KinovaDevice list[MAX_KINOVA_DEVICE];
					int devicesCount = kinova.MyGetDevices(list, result);
					for (int i = 0; i < devicesCount; i++)
					{
						std::cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << std::endl;
						//Setting the current device as the active device.
						kinova.MySetActiveDevice(list[i]);
						Experiment experiments;

						////////////////////////////////////////////////////////////////////////////
						/* Run this if gripper is changed (for running admittance control)
						///////////////////////////////////////////////////////////////////////////
						double optimalZParam[OPTIMAL_Z_PARAM_SIZE];
						std::string line;
						std::string::size_type sz;
						std::ifstream params_file ("ParametersOptimal_Z.txt");
						if (params_file.is_open())
						for (i = 0; i < OPTIMAL_Z_PARAM_SIZE; i++)
						{
						std::getline(params_file, line);
						optimalZParam[i] = std::stod(line, &sz);
						}
						else
						{
						std::cout << "Parameter file not found... Running GravityZEstimationSequence" << std::endl;
						Sleep(3000);
						return 0;
						//kinova.MyRunGravityZEstimationSequence(MICO_4DOF_SERVICE, optimalZParam);
						}
						kinova.MySetGravityOptimalZParam(optimalZParam);
						kinova.MySetGravityType(OPTIMAL);
						kinova.MyStartForceControl();
						END OF GRAVITY ESTIMATION*/
						//////////////////////////////////////////////////////////////////////////////
						ClientConfigurations config;
						std::cout << "\n*********************************\n";

						std::cout << "\nCurrent Configuration\n";
						kinova.MyGetClientConfigurations(config);
						std::cout << "Name: " << config.ClientName << std::endl;
						std::cout << "Serial: " << config.Serial << std::endl;
						std::cout << "Max Linear Speed: " << config.MaxTranslationVelocity << std::endl;
						std::cout << "Max Angular Speed: " << config.MaxOrientationVelocity << std::endl;
						std::cout << "Max Linear Acceleration: " << config.MaxTranslationAcceleration << std::endl;
						std::cout << "Sensibility: " << config.Sensibility << std::endl;
						std::cout << "Organization: " << config.Organization << std::endl;

						/*
						system("pause");

						// Change the Configuration
						strcpy(config.ClientName, "UPC Lab");
						strcpy(config.Organization, "UW, Seattle");
						config.MaxTranslationVelocity = 0.2f;
						config.MaxOrientationVelocity = 1.0f;
						config.MaxTranslationAcceleration = 5.0f;
						config.Sensibility = 500;

						std::cout << "\n*********************************\n";

						std::cout << "\nNew Configuration\n";
						std::cout << "Name: " << config.ClientName << std::endl;
						std::cout << "Serial: " << config.Serial << std::endl;
						std::cout << "Max Linear Speed: " << config.MaxTranslationVelocity << std::endl;
						std::cout << "Max Angular Speed: " << config.MaxOrientationVelocity << std::endl;
						std::cout << "Max Linear Acceleration: " << config.MaxTranslationAcceleration << std::endl;
						std::cout << "Sensibility: " << config.Sensibility << std::endl;
						std::cout << "Organization: " << config.Organization << std::endl;

						std::cout << "\n*********************************\n";

						kinova.MySetClientConfigurations(config);
						std::cout << "\nModified Configuration\n";
						kinova.MyGetClientConfigurations(config);
						std::cout << "Name: " << config.ClientName << std::endl;
						std::cout << "Serial: " << config.Serial << std::endl;
						std::cout << "Max Linear Speed: " << config.MaxTranslationVelocity << std::endl;
						std::cout << "Max Angular Speed: " << config.MaxOrientationVelocity << std::endl;
						std::cout << "Max Linear Acceleration: " << config.MaxTranslationAcceleration << std::endl;
						std::cout << "Sensibility: " << config.Sensibility << std::endl;
						std::cout << "Organization: " << config.Organization << std::endl;

						system("pause");
						*/
						//experiments.MovetoStartPos(kinova);
						//experiments.testSimulationWindow();

						if (argv[1] == NULL)
							argv[1] = (char*)"11000";
						experiments.HoloLensCartesianTeleop(argv, kinova);

						// At the end of experiment send Robot to Home position
						std::cout << "Send the robot to HOME position" << std::endl;
						kinova.MyMoveHome();
						kinova.MyStopForceControl();
						std::cout << "*********************************\n\n\n";
					}
				}
			}
		}
#endif		
		std::cout << std::endl << "C L O S I N G   A P I\n";
#if (!USE_KINOVA)
		programResult = (*kinova.MyCloseAPI)();
#endif
		programResult = 1;
	}
	FreeLibrary(commandLayer_handle);
	//FreeLibrary(hinst);
	//uninstall();
	return programResult;
	//Sleep(3000);
}