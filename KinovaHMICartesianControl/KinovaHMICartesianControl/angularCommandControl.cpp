#include "stdafx.h"
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "CartesianControl.h"

/*
Move to Starting  Position
*/
void Experiment::MovetoStartPos(KinovaAPIFunctions kinova)
{
	TrajectoryPoint kinovaStartPosition;			// AngularPosition struct
	kinovaStartPosition.Position.Type = ANGULAR_POSITION;

	kinovaStartPosition.Position.Actuators.Actuator1 = 270.00f;// 273.79f;
	kinovaStartPosition.Position.Actuators.Actuator2 = 215.00f;// 204.33f;
	kinovaStartPosition.Position.Actuators.Actuator3 = 90.00f;// 73.68f;
	kinovaStartPosition.Position.Actuators.Actuator4 = 180.0f;// 180.0f;
	kinovaStartPosition.Position.Fingers.Finger1 = 6312.0;
	kinovaStartPosition.Position.Fingers.Finger2 = 6966.0;
	kinova.MySendBasicTrajectory(kinovaStartPosition);
}

/*
Move End-Effector to Desired Position (Angular Control)
*/
void Experiment::MoveEndEffectorPos(KinovaAPIFunctions kinova, float xe, float ze)
{
	ze = ze - 0.2755f;
	float l1 = 0.2900f, l2 = 0.1900f, B = (float)sqrt(xe*xe + ze*ze);
	TrajectoryPoint actuatorPos;			// AngularPosition struct
	actuatorPos.Position.Type = ANGULAR_VELOCITY;

	actuatorPos.Position.Actuators.Actuator1 = 0.0f; // FIXED
	actuatorPos.Position.Actuators.Actuator2 = 270.0f - (float)atan2((double)ze, (double)xe)*180.0f / M_PI - (float)acos((l1*l1 + B*B - l2*l2) / (2 * l1*B))*180.0f / M_PI - 210.1f;
	actuatorPos.Position.Actuators.Actuator3 = 180.0f + acos((l1*l1 + l2*l2 - B*B) / (2 * l1*l2))*180.0f / M_PI - actuatorPos.Position.Actuators.Actuator2 - 101.1f;
	actuatorPos.Position.Actuators.Actuator4 = 0.0f; // FIXED
													 //kinova.MySendBasicTrajectory(actuatorPos);

	std::cout << xe << "," << ze << "," << actuatorPos.Position.Actuators.Actuator2 << "," << actuatorPos.Position.Actuators.Actuator3 << std::endl;
}