#include "stdafx.h"
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include "CartesianControl.h"
#include <future>

//*************************************************************//
// KINECT FUNCTIONS
//*************************************************************//
/*!
Initialize the Kinect sensor for skeleton tracking of the user closest to the sensor.
*/
bool kinectSkelTrack::initKinect() {
	// Get a working kinect sensor
	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
	if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

	// Initialize sensor
	sensor->NuiInitialize(
		NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
		| NUI_INITIALIZE_FLAG_USES_COLOR
		| NUI_INITIALIZE_FLAG_USES_SKELETON);
	sensor->NuiSkeletonTrackingEnable(
		NULL,
		NUI_SKELETON_FRAME_FLAG_SEATED_SUPPORT_ENABLED | NUI_SKELETON_QUALITY_CLIPPED_BOTTOM
	);
	sensor->NuiImageStreamSetImageFrameFlags(
		NULL,
		NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE
	);
	sensor->NuiCameraElevationSetAngle(10);
	return true;
}

/*!
Get the skeletal data from the current frame
*/
void kinectSkelTrack::getSkeletalData() {
	const NUI_TRANSFORM_SMOOTH_PARAMETERS somewhatLatentParams =
	{ 0.5f, 0.1f, 0.5f, 0.1f, 0.1f };
	NUI_SKELETON_FRAME skeletonFrame = { 0 };
	if (sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0) {
		sensor->NuiTransformSmooth(&skeletonFrame, &somewhatLatentParams);
		// Process skeletal frame
		// Loop over all sensed skeletons
		for (int z = 0; z < NUI_SKELETON_COUNT; ++z) {
			const NUI_SKELETON_DATA& skeleton = skeletonFrame.SkeletonData[z];
			// Check the state of the skeleton
			if (skeleton.eTrackingState == NUI_SKELETON_TRACKED) {
				// Get skeleton data
				// For the first tracked skeleton
				{
					// Copy the joint positions into our array
					for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
						skeletonPosition[i] = skeleton.SkeletonPositions[i];
						if (skeleton.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_NOT_TRACKED) {
							skeletonPosition[i].w = 0;
						}
					}
					return; // Only take the data for one skeleton
				}
			}
		}
	}
}

/*! Auxiliary function if multithreading the Kinect sensor */
kinectSkelTrack::KinectInfo kinectSkelTrack::getKinectData()
{
	KinectInfo ret;
	getSkeletalData();
	ret.startSignal = false;
	if (skeletonPosition[0].w > 0)
	{
		ret.userFound = true;
		// Check for start signal
		if ((float)skeletonPosition[NUI_SKELETON_POSITION_HAND_LEFT].y >
			(float)skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_LEFT].y) {
			ret.startSignal = true;		
		}

		// Get raw hand position relative to shoulder joint from Kinect
		ret.handPosition = ((double)skeletonPosition[NUI_SKELETON_POSITION_HAND_RIGHT].x - (double)skeletonPosition[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x) / 2.0f;

	}
	else
		ret.userFound = false;

	return ret;
}

//*************************************************************//
// NOVINT FALCON FUNCTIONS
//*************************************************************//
//NovintFalconHapticsDevice* taskptr = new NovintFalconHapticsDevice;

//void wrapperUpdateHaptics(void);
/*!
Initialize the Novint Falcon Haptics device
*/
bool NovintFalconHapticsDevice::InitializeHapticsDevice() {
	// create a haptic device handler
	handler = new chai3d::cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);

	// open a connection to haptic device
	hapticDevice->open();

	// calibrate device (if necessary)
	hapticDevice->calibrate();

	// if the device has a gripper, enable the gripper to simulate a user switch
	hapticDevice->setEnableGripperUserSwitch(true);

	// Set reference position
	desiredPosition.set(0.0, 0.0, 0.0);
	desiredRotation.identity();

	return true;
}

/*!
Reset the Haptics Device and apply a feedback force towards the center of the joystick
*/
void NovintFalconHapticsDevice::ResetIC(void) {
	// Reset to 0 position
	desiredPosition.set(0, 0, 0);
	// read the button 2 state
	hapticDevice->getUserSwitch(2, button2_state);
	// read position 
	hapticDevice->getPosition(position);
	// read orientation
	hapticDevice->getRotation(rotation);
	// read linear velocity
	hapticDevice->getLinearVelocity(linearVelocity);
	// read angular velocity
	hapticDevice->getAngularVelocity(angularVelocity);
	// read gripper position
	double gripperAngle;
	hapticDevice->getGripperAngleRad(gripperAngle);
	// read gripper angular velocity
	double gripperAngularVelocity;
	hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);
	/////////////////////////////////////////////////////////////////////
	// COMPUTE AND APPLY FORCES
	/////////////////////////////////////////////////////////////////////
	// variables for forces
	chai3d::cVector3d force(0, 0, 0);
	chai3d::cVector3d torque(0, 0, 0);
	double gripperForce = 0.0;
	chai3d::cHapticDeviceInfo info = hapticDevice->getSpecifications();
	double Kv = 0.5 * info.m_maxLinearDamping;
	// apply force field
	if (useForceField)
	{
		// compute linear force
		// ForceField Constant
		double Kp = 350;//175; // [N/m]
		chai3d::cVector3d forceField = Kp * (desiredPosition - position) - Kv * linearVelocity;
		if (position.distance(desiredPosition) > 0.005)
		{
			//forceField.normalize();
			//forceField.mul(maxforce);
			force.add(forceField);
		}
		//std::cout << position.distance(desiredPosition) << std::endl;

		// compute angular torque
		double Kr = 0.05; // [N/m.rad]
		chai3d::cVector3d axis;
		double angle;
		chai3d::cMatrix3d deltaRotation = chai3d::cTranspose(rotation) * desiredRotation;
		deltaRotation.toAxisAngle(axis, angle);
		torque = rotation * ((Kr * angle) * axis);
	}

	// apply damping term
	if (useDamping)
	{
		// compute linear damping force
		chai3d::cVector3d forceDamping = -Kv * linearVelocity;
		force.add(forceDamping);

		// compute angular damping force
		double Kvr = 1.0 * info.m_maxAngularDamping;
		chai3d::cVector3d torqueDamping = -Kvr * angularVelocity;
		torque.add(torqueDamping);

		// compute gripper angular damping force
		double Kvg = 1.0 * info.m_maxGripperAngularDamping;
		gripperForce = gripperForce - Kvg * gripperAngularVelocity;
	}
	// send computed force, torque, and gripper force to haptic device
	hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
}

/*!
Update the current position of the Haptics joystick
*/
void NovintFalconHapticsDevice::UpdateHaptics(void) {
	// read position 
	hapticDevice->getPosition(position);
	// read the button 0 state
	hapticDevice->getUserSwitch(0, button0_state);
	/*
	// read orientation
	hapticDevice->getRotation(rotation);
	// read linear velocity
	hapticDevice->getLinearVelocity(linearVelocity);
	// read angular velocity
	hapticDevice->getAngularVelocity(angularVelocity);
	// read gripper position
	double gripperAngle;
	hapticDevice->getGripperAngleRad(gripperAngle);
	// read gripper angular velocity
	double gripperAngularVelocity;
	hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);
	/////////////////////////////////////////////////////////////////////
	// COMPUTE AND APPLY FORCES
	/////////////////////////////////////////////////////////////////////
	// variables for forces
	chai3d::cVector3d force(0, 0, 0);
	chai3d::cVector3d torque(0, 0, 0);
	double gripperForce = 0.0;
	chai3d::cHapticDeviceInfo info = hapticDevice->getSpecifications();
	double Kv = 0.5 * info.m_maxLinearDamping;
	// apply force field
	if (useForceField)
	{
		// compute linear force
		// ForceField Constant
		double Kp = 175; // [N/m]
		chai3d::cVector3d forceField = Kp * (desiredPosition - position) - Kv * linearVelocity;
		if (position.distance(desiredPosition) > 0.005)
		{
			//forceField.normalize();
			//forceField.mul(maxforce);
			force.add(forceField);
		}
		//std::cout << position.distance(desiredPosition) << std::endl;

		// compute angular torque
		double Kr = 0.05; // [N/m.rad]
		chai3d::cVector3d axis;
		double angle;
		chai3d::cMatrix3d deltaRotation = chai3d::cTranspose(rotation) * desiredRotation;
		deltaRotation.toAxisAngle(axis, angle);
		torque = rotation * ((Kr * angle) * axis);
	}

	// apply damping term
	if (useDamping)
	{
		// compute linear damping force
		chai3d::cVector3d forceDamping = -Kv * linearVelocity;
		force.add(forceDamping);

		// compute angular damping force
		double Kvr = 1.0 * info.m_maxAngularDamping;
		chai3d::cVector3d torqueDamping = -Kvr * angularVelocity;
		torque.add(torqueDamping);

		// compute gripper angular damping force
		double Kvg = 1.0 * info.m_maxGripperAngularDamping;
		gripperForce = gripperForce - Kvg * gripperAngularVelocity;
	}
	// send computed force, torque, and gripper force to haptic device
	hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
	*/
}