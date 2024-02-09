#ifndef __KINEMATICS__H
#define __KINEMATICS__H

#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>

#include "utils.h"

#define joint_length 50

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
    size_t link_name_size = 0;          // The size of the link name to ensure string comparison occurs appropriately

    // Important joint information
    Transform transform;                // The transform of this joint
    Jacobian jacobian;                  // The jacobian matrix of this joint
    void (*actuationFunc)(double, Transform*);  // Function pointer to the actuation function for this joint
};

void initializeMemory(void);

void freeMemory();

void TransformFromTo(const signed char* urdfpath, const int urdflen, const double* q, const signed char* source, const signed char* target, double* transform, double currTimeStep);

void GetJacobianForBody(const signed char* urdfpath, const int urdflen, const double* q, const signed char* bodyName, double currTimeStep, double* jacobian);

#endif // __KINEMATICS__H