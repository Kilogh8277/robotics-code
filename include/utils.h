#ifndef __UTILS__H
#define __UTILS__H

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <algorithm>

#include "kinematics.h"

// Custom math functions
double norm(double* vec, int length);
void normalize(double*vec, int length);
void cross(const double* a, const double* b, double* c);

// void transpose(double* mat, int size[2]);
void transpose(const double* matIn, double* matOut, int size[2]);
void matrixMultiply(const double* a, const int sizeA[2], const double* b, const int sizeB[2], double* c);

// Rotation matrix functions
void rotx(double c, double* R);
void rotx_deg(double c, bool isDegBool, double* R);
void roty(double c, double* R);
void roty_deg(double c, bool isDegBool, double* R);
void rotz(double c, double* R);
void rotz_deg(double c, bool isDegBool, double* R);
void rotx(double c, struct Transform* R);
void roty(double c, struct Transform* R);
void rotz(double c, struct Transform* R);
void displacex(double c, struct Transform* d);
void displacey(double c, struct Transform* d);
void displacez(double c, struct Transform* d);
void dor(const double* Rd, const double* Ra, double* dr);

#endif // __UTILS__H