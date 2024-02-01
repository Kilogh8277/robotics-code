#ifndef __TRANSFORM__H
#define __TRANSFORM__H

#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>

#include "utils.h"

#define joint_length 50

// Structure to represent a 3D vector
struct Vector3 {
    double x, y, z;
};

// Structure to represent a 3D rotation (Euler angles)
struct Rotation3 {
    double x, y, z;
};

struct Transform {
    double transform[16] = {1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1};
};

// Structure to represent joint information
struct JointInfo;

void parseURDF(char* urdf_file);

JointInfo* findJointByName(std::string nameToFind);

void GetOffset(char* bodyName, double* offset);

void getTransform(const double* q, const char* body, double* transform);

void GetRotationMatrixFromTransform(const double* transform, double* rotMat, double* distVec);

void SetTransformFromRotMat(double* rotMat, double* distVec, Transform* transform);

void RotXYZ(double* rotationMat, const double* rpy);

void InitTransforms();

#endif // __TRANSFORM__H