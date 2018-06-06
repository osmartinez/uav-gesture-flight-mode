#ifndef _NITE_SAMPLE_UTILITIES_H_
#define _NITE_SAMPLE_UTILITIES_H_

#include <stdio.h>
#include <OpenNI.h>
#include <NiTE.h>
#include <math.h>

#define PI 3.141592653589793238463

#ifdef WIN32
#include <conio.h>
int wasKeyboardHit()
{
	return (int)_kbhit();
}

#else // linux

#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
int wasKeyboardHit()
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

float distancia_euclidea(nite::Point3f a, nite::Point3f b) {
	return sqrt((pow(a.x - b.x, 2)) + (pow(a.y - b.y, 2)) + (pow(a.z - b.z, 2)));
}

float rad_to_deg(float rad) {
	return rad * (180.0 / PI);
}

float ang_to_throttle(float ang) {
	float norm = ang;
	if (norm < -80) {
		norm = -80.0;
	}
	else if (norm > 80) {
		norm = 80.0;
	}

	float res = ((norm * 1000) / 160) + 1500;
	if (res < 1050) {
		return 1000;
	}
	if (res > 1920) {
		return 2000;
	}

	return res;
}

char* val_to_char(int val) {
	if (val == 1) {
		return "1550";
	}
	else if (val == -1) {
		return "1450";
	}

	return "1500";
}

#endif // _NITE_SAMPLE_UTILITIES_H_
#pragma once
