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

struct Jacobian {
    double jacobian[6*joint_length] = {0};
};

// Structure to represent joint information
struct JointInfo {
    // Parameters of this joint
    char name[50] = {0};                // Name of this joint
    char link_name[50] = {0};           // Name of the associated link
    char type[10] = {0};                // Type of joint - fixed, revolute, or prismatic
    double mass = 0;                    // Mass of the link
    double inertia[6] = {0};            // Inertia of the link
    double com[3] = {0};                // Center of mass of the link
    double origin_xyz[3];               // Translation of this joint relative to the parent (in parent frame) in meters
    double origin_rpy[3];               // Orientation of this joint relative to the parent (in parent frame) in radians
    double axis[3] = {0, 0, 0};         // The axis of rotation or displacement
    char parent_link_name[50] = {0};    // The name of the parent link
    JointInfo* parent_link = NULL;      // Pointer to the parent link
    JointInfo* child_links[5] = {NULL}; // Assuming a maximum of 5 child links (adjust as needed)
    int numChildren = 0;                // Keep track of the number of children so we can allocate from the top
    uint8_t actuated = 0, actuator = 0; // Keep track if this joint is actuated and where it is in the input array
    size_t name_size = 0;               // The size of the name to ensure string comparison occurs appropriately

    // Important joint information
    Transform transform;                // The transform of this joint
    Jacobian jacobian;                  // The jacobian matrix of this joint
    void (*actuationFunc)(double, Transform*);  // Function pointer to the actuation function for this joint
};

void initializeMemory(char* urdfPath);

void freeMemory();

void parseURDF(char* urdf_file);

JointInfo* findJointByName(std::string nameToFind);

void GetOffset(char* bodyName, double* offset);

void getTransform(const double* q, const char* body, double* transform, double currTimeStep);

void TransformFromTo(const double* q, const char* source, const char* target, double* transform, double currTimeStep);

void updateTransformTree(const double* q);

void GetRotationMatrixFromTransform(const double* transform, double* rotMat, double* distVec);

void SetTransformFromRotMat(double* rotMat, double* distVec, Transform* transform);

void InvertTransform(Transform* T);

void GetJacobianForBody(const double* q, char* bodyName);

#endif // __TRANSFORM__H