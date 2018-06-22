#include "NiTE.h"
#include "util.h"
#include <windows.h>

#define EXPORT extern "C" __declspec(dllexport)
#define FACTOR_CONFIANZA .5
#define MIN_DEG_shoulright_THROTTLE 40.
#define MIN_DEG_YAW_RIGHT 35
#define MIN_DEG_YAW_LEFT 45
#define MIN_DEG_PITCH_DOWN 25
#define MIN_DEG_PITCH_UP 40
#define VARIACION_THROTTLE_PERMITIDA 0.0
#define DISTANCE_MAX_DISARM 100

float ultimo_throttle = 1000;
nite::UserId idPrimerUsuario;

float min_throttle = 0;
float max_throttle = 0;

float throttle = 1000;
float yaw = 0;
float pitch = 0;
float roll = 0;

bool finish = false;

/// <summary>
/// Exported function to receive the throttle value
/// </summary>
/// <returns>throttle</returns>
EXPORT float GetThrottle() {
	return throttle;
}

/// <summary>
/// Exported function to receive the yaw value
/// </summary>
/// <returns>yaw</returns>
EXPORT float GetYaw() {
	return yaw;
}

/// <summary>
/// Exported function to receive the pitch value
/// </summary>
/// <returns>pitch</returns>
EXPORT float GetPitch() {
	return pitch;
}

/// <summary>
/// Exported funciton to receive the pitch value
/// </summary>
/// <returns>roll</returns>
EXPORT float GetRoll() {
	return roll;
}

/// <summary>
/// Exported function to be able to stop the program externaly
/// </summary>
EXPORT void Stop() {
	finish = true;
}

/// <summary>
/// Indicates if the left arm is in the right position to send a valid throttle value
/// </summary>
/// <returns>true if it is on the right position</returns>
bool is_valid_throttle(nite::SkeletonJoint shoulright_left, nite::SkeletonJoint elbow_left) {
	// solo con x e y
	nite::Point3f shoulright = nite::Point3f(shoulright_left.getPosition().x, shoulright_left.getPosition().y, 0);
	nite::Point3f elbow = nite::Point3f(elbow_left.getPosition().x, elbow_left.getPosition().y, 0);

	nite::Point3f aux = nite::Point3f();
	aux.x = shoulright.x;
	aux.y = elbow.y;
	aux.z = 0;

	float a = euclidean_distance(elbow, shoulright);
	float c = euclidean_distance(aux, elbow);
	float sinalpha = c / a;

	float alpha = rad_to_deg(asin(sinalpha));

	return alpha > MIN_DEG_shoulright_THROTTLE;
}

/// <summary>
/// Calculates the throttle value. Triangulates the left elbow and hand to get the angle between them.
/// </summary>
/// <returns>throttle value</returns>
float calculate_throttle(nite::SkeletonJoint hand_left, nite::SkeletonJoint elbow_left) {
	// solo con z e y
	int signo = 1;
	nite::Point3f hand = nite::Point3f(0, hand_left.getPosition().y, hand_left.getPosition().z);
	nite::Point3f elbow = nite::Point3f(0, elbow_left.getPosition().y, elbow_left.getPosition().z);

	nite::Point3f aux = nite::Point3f();
	aux.x = 0;
	aux.y = elbow.y;
	aux.z = hand.z;

	if (hand.y < elbow.y) {
		signo = -1;
	}

	float a = euclidean_distance(elbow, hand);
	float c = euclidean_distance(aux, hand);
	float sinalpha = c / a;
	return signo*rad_to_deg(asin(sinalpha));
}

/// <summary>
/// Calculates the pitch value to send to the UAV based on the angle between right hand and elbow.
/// </summary>
/// <returns>-1 to go back, 1  to go forward and 0 to keep straight</returns>
float calculate_pitch(nite::SkeletonJoint hand_right, nite::SkeletonJoint elbow_right) {
	bool UP = false;
	nite::Point3f hand = nite::Point3f(0, hand_right.getPosition().y, hand_right.getPosition().z);
	nite::Point3f elbow = nite::Point3f(0, elbow_right.getPosition().y, elbow_right.getPosition().z);

	nite::Point3f aux = nite::Point3f();
	aux.x = 0;
	aux.y = elbow.y;
	aux.z = hand.z;

	UP = hand.y > elbow.y;

	float a = euclidean_distance(elbow, hand);
	float c = euclidean_distance(aux, hand);
	float sinalpha = c / a;
	float deg = rad_to_deg(asin(sinalpha));

	if (UP) {
		if (deg > MIN_DEG_PITCH_UP) {
			return 1;
		}
	}
	else {
		if (deg > MIN_DEG_PITCH_DOWN) {
			return -1;
		}
	}

	return 0;

}

/// <summary>
/// Calculates the roll value based on the position of the right hand and right elbow
/// </summary>
/// <returns>-1 to go left, 1  to go right and 0 to keep straight</returns>
float calculate_roll(nite::SkeletonJoint hand_right, nite::SkeletonJoint elbow_right) {
	nite::Point3f hand = nite::Point3f(hand_right.getPosition().x, 0, hand_right.getPosition().z);
	nite::Point3f elbow = nite::Point3f(elbow_right.getPosition().x, 0, elbow_right.getPosition().z);

	nite::Point3f aux = nite::Point3f();
	aux.x = hand.x;
	aux.y = 0;
	aux.z = elbow.z;

	float a = euclidean_distance(hand, aux);
	float c = euclidean_distance(hand, elbow);
	float sinalpha = a / c;
	float deg = rad_to_deg(asin(sinalpha));

	// left (opening arm)
	if (hand.x < elbow.x) {
		if (deg < MIN_DEG_YAW_LEFT) {
			return -1;
		}
	}
	// right (flexing arm)
	else {
		if (deg < MIN_DEG_YAW_RIGHT) {
			return 1;
		}
	}

	// straight
	return 0;
}

/// <summary>
/// Checks if both hands are close enought to send a disarm command to the UAV
/// </summary>
/// <returns>true if disarm is required</returns>
bool desarmar(nite::SkeletonJoint hand_left, nite::SkeletonJoint hand_right) {
	nite::Point3f left = nite::Point3f(hand_left.getPosition().x, hand_left.getPosition().y, hand_left.getPosition().z);
	nite::Point3f right = nite::Point3f(hand_right.getPosition().x, hand_right.getPosition().y, hand_right.getPosition().z);

	float dist = euclidean_distance(left, right);
	//printf("\nDIST: %f", dist);
	return dist<DISTANCE_MAX_DISARM;
}

/// <summary>
/// Calculates the yaw value based on the position of the left hand and right elbow
/// </summary>
/// <returns>-1 to turn left, 1  to turn right and 0 to keep straight</returns>
float calculate_yaw(nite::SkeletonJoint hand_left, nite::SkeletonJoint elbow_left) {
	nite::Point3f hand = nite::Point3f(hand_left.getPosition().x, 0, hand_left.getPosition().z);
	nite::Point3f elbow = nite::Point3f(elbow_left.getPosition().x, 0, elbow_left.getPosition().z);

	nite::Point3f aux = nite::Point3f();
	aux.x = hand.x;
	aux.y = 0;
	aux.z = elbow.z;

	float a = euclidean_distance(hand, aux);
	float c = euclidean_distance(hand, elbow);
	float sinalpha = a / c;
	float deg = rad_to_deg(asin(sinalpha));

	// LEFT (abrir arm)
	if (hand.x < elbow.x) {
		if (deg < MIN_DEG_YAW_LEFT) {
			return -1;
		}
	}
	// RIGHT (flexionar arm)
	else {
		if (deg < MIN_DEG_YAW_RIGHT) {
			return 1;
		}
	}

	// straigth arm
	return 0;
}

/// <summary>
/// Closes the kinect sensor stream
/// </summary>
void close() {
	nite::NiTE::shutdown();
}

/// <summary>
/// Initializes kinect sensor, tracks user skeleton and sends commands to UAV until disarm is required or until key is pressed.
/// </summary>
/// <returns>-1 if error, 1 when finishes</returns>
EXPORT int Initialize(float maxthrottle, float minthrottle)
{
	// set minimum and maximum throttle
	max_throttle = maxthrottle;
	min_throttle = minthrottle;

	// Kinect sensor initialization
	printf("Initializing tracking...");
	nite::UserTracker tracker;
	nite::Status state;
	bool savedUser = false;
	finish = false;

	nite::NiTE::initialize();
	state = tracker.create();
	if (state != nite::STATUS_OK)
	{
		printf("Error.\nKinect sensor could not be openned. Check whether OpenNi2 y NiTE2 are copied on $Output and project path.");
		return -1;
	}
	printf(" ok!");
	// Fin inicializacion de Kinect

	printf("\n\nTrying to detect a person...");

	nite::UserTrackerFrameRef frame;
	while (!key_pressed()&& !finish)
	{
		state = tracker.readFrame(&frame);
		if (state != nite::STATUS_OK)
		{
			printf("Frame error\n");
			continue;
		}

		const nite::Array<nite::UserData>& users = frame.getUsers();

		// If the user leaves, stop the uav
		if (users.getSize() == 0) {
			throttle = 1000;
		}

		for (int i = 0; i < users.getSize(); ++i)
		{

			const nite::UserData& user = users[i];

			if (user.isNew())
			{
				tracker.startSkeletonTracking(user.getId());
			}
			else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
			{

				// throttle
				const nite::SkeletonJoint& hand_left = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
				const nite::SkeletonJoint& elbow_left = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
				const nite::SkeletonJoint& shoulright_left = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);


				// yaw
				const nite::SkeletonJoint&  hand_right = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
				const nite::SkeletonJoint&  elbow_right = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);

				// roll
				const nite::SkeletonJoint& cabeza = user.getSkeleton().getJoint(nite::JOINT_HEAD);
				const nite::SkeletonJoint& carighta_left = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
				const nite::SkeletonJoint& carighta_right = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);

				bool disarm_command = false;

				if (hand_right.getPositionConfidence() > .5 && hand_left.getPositionConfidence() > .5) {
					disarm_command = desarmar(hand_left, hand_right);
				}

				if (hand_right.getPositionConfidence() > .5 && elbow_right.getPositionConfidence() > .5) {
					pitch = calculate_pitch(hand_right, elbow_right);
				}

				if (hand_right.getPositionConfidence() > .5 && elbow_right.getPositionConfidence() > .5 /*&& throttle_valido(shoulright_left, elbow_left)*/) {
					roll = calculate_roll(hand_right, elbow_right);
				}

				if (hand_left.getPositionConfidence() > .5 && elbow_left.getPositionConfidence() > .5 && is_valid_throttle(shoulright_left, elbow_left)) {
					yaw = calculate_yaw(hand_left, elbow_left);
				}

				if (hand_left.getPositionConfidence() > .5 && elbow_left.getPositionConfidence() > .5 /*&& throttle_valido(shoulright_left, elbow_left)*/) {
					throttle = ang_to_throttle(calculate_throttle(hand_left, elbow_left),min_throttle,max_throttle);
					/*float resta = fabs(throttle - ultimo_throttle);
					if (resta > ultimo_throttle*VARIACION_THROTTLE_PERMITIDA) {
						ultimo_throttle = throttle;
					}
					else {
						throttle = ultimo_throttle;
					}*/

				}

				if (disarm_command) {
					throttle = 0;
					finish = true;
					break;
				}

				printf("%f %f %f %f\n", throttle, yaw, pitch, roll);
			}
		}

	}


	nite::NiTE::shutdown();
	return 1;
}
