#ifndef _NITE_SAMPLE_UTILITIES_H_
#define _NITE_SAMPLE_UTILITIES_H_

#include <stdio.h>
#include <OpenNI.h>
#include <NiTE.h>
#include <math.h>

#define PI 3.141592653589793238463

#ifdef WIN32
#include <conio.h>
int key_pressed()
{
	return (int)_kbhit();
}

#else // linux

#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>

/// <summary>
/// Detects a key pressed
/// </summary>
/// <returns>1 if pressed, else 0</returns>
int key_pressed()
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	// don't echo and don't wait for ENTER
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

	// make it non-blocking (so we can check without waiting)
	if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK))
	{
		return 0;
	}

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf))
	{
		return 0;
	}

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}
#endif // WIN32

/// <summary>
/// Calculates the euclidean distance between two 3d points
/// </summary>
/// <returns>distance between two points</returns>
float euclidean_distance(nite::Point3f a, nite::Point3f b) {
	return sqrt((pow(a.x - b.x, 2)) + (pow(a.y - b.y, 2)) + (pow(a.z - b.z, 2)));
}

/// <summary>
/// Transforms radians to degrees
/// </summary>
/// <returns>equivalence in degrees of @rad radians</returns>
float rad_to_deg(float rad) {
	return rad * (180.0 / PI);
}

/// <summary>
/// Transforms the elbow angle to a right throttle value. Angle is limited by the following range: [-80,80]
/// </summary>
/// <returns>throttle value</returns>
float ang_to_throttle(float ang,float min_throttle,float max_throttle) {
	float norm = ang;
	//if (norm < -80) {
	//	norm = -80.0;
	//}
	//else if (norm > 80) {
	//	norm = 80.0;
	//}

	//float res = ((norm * 1000) / 160) + 1500;
	float res = ((norm*min_throttle) / 200) + ((max_throttle + min_throttle) / 2);
	if (res < min_throttle) {
		return min_throttle;
	}

	if (res > max_throttle) {
		return max_throttle;
	}
	//if (res < min_throttle) {
	//	res = min_throttle;
	//}
	//if (res > max_throttle) {
	//	res = max_throttle;
	//}


	return res;
}

#endif // _NITE_SAMPLE_UTILITIES_H_
#pragma once
