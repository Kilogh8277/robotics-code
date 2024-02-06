#include "kinematics.h"

struct JointInfo* joints;
static int initialized = 0;
int *num_joints = (int *)calloc(1, sizeof(int));
double *mostRecentTimeStep = (double *)calloc(1, sizeof(double));

/*******************************************************************************************
Function to allocate memory for the array of JointInfo structures
*******************************************************************************************/
void initializeMemory(char* urdfPath) {
    if (!initialized) {
        // Allocate memory for the joints array
        joints = (JointInfo*)calloc(joint_length, sizeof(JointInfo));

        // Check if the allocation was successful
        if (joints == NULL) {
            fprintf(stderr, "Error: Failed to allocate memory for joints array.\n");
            exit(EXIT_FAILURE);
        }

        num_joints = (int *)calloc(1, sizeof(int));

        parseURDF(urdfPath);
        initialized = 1;
    }
}

/*******************************************************************************************
Function to deallocate memory -- Important in Simulink because the OS does not release the 
allocated memory until MATLAB is closed
*******************************************************************************************/
void freeMemory() {
    if (initialized) {
        free(joints);
        free(num_joints);
        free(mostRecentTimeStep);
        initialized = 0;
    }
}

/*******************************************************************************************
Find the joint by name
If no joint has this name, return NULL, otherwise return the joint
*******************************************************************************************/
JointInfo* findJointByName(std::string nameToFind) {
    JointInfo* currJoint;
    for (int i = 0; i < 50; i++) {
        currJoint = &joints[i];
        if (currJoint->name[0] > 0 && !std::strncmp(nameToFind.c_str(), currJoint->name, currJoint->name_size)) {
            return currJoint;
        }
    }
    return NULL;
}

/*******************************************************************************************
Get the translation offset relative to the parent frame of the specified body
*******************************************************************************************/
void GetOffset(char* bodyName, double* offset) {
    for (int i = 0; i < 50; i++) {
        if (!std::strncmp(joints[i].name, bodyName, joints[i].name_size)) {
            memcpy(offset, joints[i].origin_xyz, 3*sizeof(double));
            break;
        }
    }
}

/*******************************************************************************************
Get the rotation matrix and translation vector from the provided transform (row-major)
*******************************************************************************************/
void GetRotationMatrixFromTransform(const double* transform, double* rotMat, double* distVec) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rotMat[3*i+j] = transform[4*i+j];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 3; j <= 3; j++) {
            distVec[i] = transform[4*i+j];
        }
    }
}

/*******************************************************************************************
Set the transform with the rotation matrix and translation vector (row-major)
*******************************************************************************************/
void SetTransformFromRotMat(double* rotMat, double* distVec, Transform* transform) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            transform->transform[4*i+j] = rotMat[3*i+j];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 3; j <= 3; j++) {
            transform->transform[4*i+j] = distVec[i];
        }
    }
}

/*******************************************************************************************
Calculate the transform FROM source TO target
*******************************************************************************************/
void TransformFromTo(char* urdfpath, const double* q, const char* source, const char* target, double* transform, double currTimeStep) {
    int sourceIndex = 0, targetIndex = 0;
    double sourceRotMat[9] = {0}, sourceDistVec[3] = {0},
           targetRotMat[9] = {0}, targetDistVec[3] = {0};
    Transform sourceTransform, targetTransform;
    for (int i = 0; i < joint_length; i++) {
        if (!std::strncmp(joints[i].name, source, joints[i].name_size)) {
            getTransform(q, source, &sourceTransform.transform[0], currTimeStep);
            sourceIndex = i;
        }
        if (!std::strncmp(joints[i].name, target, joints[i].name_size)) {
            getTransform(q, target, &targetTransform.transform[0], currTimeStep);
            targetIndex = i;
        }
        if (sourceIndex > 0 && targetIndex > 0) {
            break;
        }
    }
    if (sourceIndex == 0 || targetIndex == 0) {
        fprintf(stderr, "ERROR! Source and/or target not able to update.\n");
        fprintf(stderr, "Source: %s\n", source);
        fprintf(stderr, "Target: %s\n", target);
    }
    else {
        int transformSize[] = {4, 4};
        InvertTransform(&targetTransform);
        matrixMultiply(&targetTransform.transform[0], transformSize, &sourceTransform.transform[0], transformSize, &transform[0]);
    }
}

/*******************************************************************************************
Update the transforms with the current joint configuration, then retrieve the specified 
transform
*******************************************************************************************/
void getTransform(const double* q, const char* body, double* transform, double currTimeStep) {
    if (*mostRecentTimeStep != currTimeStep) {
        updateTransformTree(q);
        *mostRecentTimeStep = currTimeStep;
    }
    for (int i = 0; i < *num_joints; i++) {
        if (!std::strncmp(body, joints[i].name, joints[i].name_size)) {
            memcpy(transform, &joints[i].transform.transform[0], sizeof(Transform));
            return;
        }
    }
}

/*******************************************************************************************
Update the transforms throughout the JointInfo structure
*******************************************************************************************/
void updateTransformTree(const double* q) {
    // Variables for this transform and the parent transform
    Transform currTransform; double rotMat[9] = {0}, distVec[3] = {0};
    Transform parentTransform; double parentRotMat[9], parentDistVec[3];
    
    // Variables for the x, y, and z rotations, along with the sequential rotation matrices (X-Y-Z rotations)
    double rotX[9] = {0}, rotY[9] = {0}, rotZ[9] = {0}, 
            rotMatX[9] = {0}, rotMatXY[9] = {0}, rotMatXYZ[9] = {0}; 
    
    // Size of each type of matrix and vector for matrix multiplication
    int rotSize[] = {3, 3}, transformSize[] = {4, 4};

    // Go through each joint
    for (int i = 0; i < *num_joints; i++) {
        // Set each variable to zeros
        memset(rotX, 0, 9*sizeof(double)); memset(rotY, 0, 9*sizeof(double)); memset(rotZ, 0, 9*sizeof(double));
        memset(rotMatX, 0, 9*sizeof(double)); memset(rotMatXY, 0, 9*sizeof(double)); memset(rotMatXYZ, 0, 9*sizeof(double));
        memset(distVec, 0, 3*sizeof(double));

        // Initiailize currTransform to identity
        Transform currTransform;

        // Get the parent link -- if it's NULL, use an identity matrix
        if (joints[i].parent_link != NULL) {
            parentTransform = joints[i].parent_link->transform;
        }
        else {
            Transform parentTransform;
        }

        // Retrieve the rotation matrix and distance vector from the current matrix
        GetRotationMatrixFromTransform(currTransform.transform, rotMat, distVec);
        GetRotationMatrixFromTransform(parentTransform.transform, parentRotMat, parentDistVec);

        // Rotate through the origin roll-pitch-yaw angles and assign to the variables
        rotx(joints[i].origin_rpy[0], rotX);
        roty(joints[i].origin_rpy[1], rotY);
        rotz(joints[i].origin_rpy[2], rotZ);

        // Multiply the rotation matrices together to get one rotation matrix rotMatXYZ
        matrixMultiply(rotMat, rotSize, rotX, rotSize, rotMatX);
        matrixMultiply(rotMatX, rotSize, rotY, rotSize, rotMatXY);
        matrixMultiply(rotMatXY, rotSize, rotZ, rotSize, rotMatXYZ);

        // Get the origin displacement and assign to temp variable
        double temp[3] = {0}; int vecSize[] = {3, 1};
        temp[0] = joints[i].origin_xyz[0];
        temp[1] = joints[i].origin_xyz[1];
        temp[2] = joints[i].origin_xyz[2];

        transpose(rotMatXYZ, rotMat, rotSize);
        // matrixMultiply(rotMatXYZ, rotSize, temp, vecSize, distVec);
        Transform result = {0};
        SetTransformFromRotMat(rotMat, temp, &currTransform);
        if (joints[i].actuated > 0) {
            joints[i].actuationFunc(-q[joints[i].actuator], &currTransform);
        }

        matrixMultiply(&parentTransform.transform[0], transformSize, &currTransform.transform[0], transformSize, &result.transform[0]);
        
        memcpy(&joints[i].transform, &result, sizeof(Transform));
    }
}

void InvertTransform(Transform* T) {
    double rotMatIn[9] = {0}, rotMatOut[9] = {0}, distVecIn[3] = {0}, distVecOut[3] = {0};
    int rotMatSize[] = {3, 3};
    GetRotationMatrixFromTransform(T->transform, rotMatIn, distVecIn);
    transpose(rotMatIn, rotMatOut, rotMatSize);
    double temp[3] = {0};
    int distVecSize[3] = {3, 1};
    temp[0] = -distVecIn[0];
    temp[1] = -distVecIn[1];
    temp[2] = -distVecIn[2];
    matrixMultiply(rotMatOut, rotMatSize, temp, distVecSize, distVecOut);
    SetTransformFromRotMat(rotMatOut, distVecOut, T);
}

void CalculateJacobianColumn(JointInfo thisJoint, double* jacobian) {
    Transform thisBodyTransform; double rotMat[9] = {0}, distVec[3] = {0}, rotMatAxis[3] = {0}, crossVec[3] = {0};
    uint8_t actuator_index = thisJoint.actuator;
    thisBodyTransform   =   thisJoint.transform;
    GetRotationMatrixFromTransform(&thisBodyTransform.transform[0], rotMat, distVec);
    if (thisJoint.axis[0] == 1) {
        rotMatAxis[0] = rotMat[0]; rotMatAxis[1] = rotMat[3]; rotMatAxis[2] = rotMat[6];
    }
    else if (thisJoint.axis[1] == 1) {
        rotMatAxis[0] = rotMat[1]; rotMatAxis[1] = rotMat[4]; rotMatAxis[2] = rotMat[7];
    }
    else if (thisJoint.axis[2] == 1) {
        rotMatAxis[0] = rotMat[2]; rotMatAxis[1] = rotMat[5]; rotMatAxis[2] = rotMat[8];
    }
    else {
        fprintf(stderr, "GetJacobianForBody: ERROR! A joint appears to be actuated but no axis is defined\n");
        fprintf(stderr, "GetJacobianForBody: Joint: %s\n", thisJoint.name);
        return;
    }
    cross(distVec, rotMatAxis, crossVec);
    jacobian[6*actuator_index + 0] = rotMatAxis[0];
    jacobian[6*actuator_index + 1] = rotMatAxis[1];
    jacobian[6*actuator_index + 2] = rotMatAxis[2];
    jacobian[6*actuator_index + 3] = crossVec[0];
    jacobian[6*actuator_index + 4] = crossVec[1];
    jacobian[6*actuator_index + 5] = crossVec[2];
}

void GetJacobianForBody(const double* q, char* bodyName, double currTimeStep, double* jacobian) {
    if (*mostRecentTimeStep != currTimeStep) {
        updateTransformTree(q);
        *mostRecentTimeStep = currTimeStep;
    }
    JointInfo thisJoint;
    for (int i = 0; i < *num_joints; i++) {
        if (!std::strncmp(joints[i].name, bodyName, joints[i].name_size)) {
            thisJoint = joints[i];
            if (thisJoint.actuated > 0) {
                CalculateJacobianColumn(thisJoint, &jacobian[0]);
            }
                do  {
                    thisJoint = *thisJoint.parent_link;
                    if (thisJoint.actuated > 0) {
                        CalculateJacobianColumn(thisJoint, &jacobian[0]);
                    }
                } while (thisJoint.parent_link != NULL);
            return;
        }

    }
}

// Function to parse URDF file and extract joint information
void parseURDF(char* urdf_file) {
    std::ifstream file(urdf_file);
    std::string line;
    int joint_num = 0, num_actuators = 0;

    while (std::getline(file, line)) {
        // If this is the definition of a joint
        if (line.find("<joint") != std::string::npos) {
            JointInfo *newJoint = &joints[joint_num];

            while (line.find("</joint") == std::string::npos) {
                // Get the name of the joint and assign to the newJoint
                std::size_t name_pos = line.find("name=\"");
                if (name_pos != std::string::npos) {
                    name_pos += 6;  // We want to start after name="
                    std::size_t name_end = line.find("\"", name_pos);
                    memcpy(&newJoint->name, line.substr(name_pos, name_end - name_pos).c_str(), name_end - name_pos);
                    newJoint->name_size = (size_t)(name_end - name_pos);
                }

                // The type should be on the same line
                std::size_t type_pos = line.find("type=\"");
                if (type_pos != std::string::npos) {
                    type_pos += 6;  // We want to start after type="
                    std::size_t type_end = line.find("\"", type_pos);
                    memcpy(&newJoint->type, line.substr(type_pos, type_end - type_pos).c_str(), type_end - type_pos);
                    if(!std::strncmp(newJoint->type, "revolute", 8)) {
                        newJoint->actuated = 1;
                        newJoint->actuator = num_actuators++;
                        
                    }
                    else if(!std::strncmp(newJoint->type, "prismatic", 9)) {
                        newJoint->actuated = 1;
                        newJoint->actuator = num_actuators++;
                    }
                }

                if (line.find("<origin") != std::string::npos) {
                    std::size_t xyz_pos = line.find("xyz=\"") + 5;
                    std::size_t xyz_end = line.find("\"", xyz_pos);
                    std::string xyz_str = line.substr(xyz_pos, xyz_end - xyz_pos);

                    std::stringstream xyz_stream(xyz_str);
                    xyz_stream >> newJoint->origin_xyz[0] >> newJoint->origin_xyz[1] >> newJoint->origin_xyz[2];
                    
                    std::size_t rpy_pos = line.find("rpy=\"") + 5;
                    std::size_t rpy_end = line.find("\"", rpy_pos);
                    std::string rpy_str = line.substr(rpy_pos, rpy_end - rpy_pos);

                    std::stringstream rpy_stream(rpy_str);
                    rpy_stream >> newJoint->origin_rpy[0] >> newJoint->origin_rpy[1] >> newJoint->origin_rpy[2];
                }
                if (line.find("<parent") != std::string::npos) {
                    std::size_t parent_pos = line.find("link=\"") + 6;
                    std::size_t parent_end = line.find("\"", parent_pos);
                    std::string parent_name = line.substr(parent_pos, parent_end - parent_pos);
                    
                    newJoint->parent_link = findJointByName(parent_name);
                    if (newJoint->parent_link != NULL) {
                        int parentChildNums = newJoint->parent_link->numChildren;
                        newJoint->parent_link->child_links[parentChildNums] = newJoint;
                        newJoint->parent_link->numChildren++;
                    }
                }   
                if (line.find("<axis") != std::string::npos) {
                    std::size_t axis_pos = line.find("xyz=\"") + 5;
                    std::size_t axis_end = line.find("\"", axis_pos);
                    std::string axis_str = line.substr(axis_pos, axis_end - axis_pos);

                    std::stringstream axis_stream(axis_str);
                    axis_stream >> newJoint->axis[0] >> newJoint->axis[1] >> newJoint->axis[2];

                    if (!std::strncmp(newJoint->type, (char *)"revolute", 8)) {
                        if (newJoint->axis[0]) {
                            newJoint->actuationFunc = &rotx;
                        }
                        if (newJoint->axis[1]) {
                            newJoint->actuationFunc = &roty;
                        }
                        if (newJoint->axis[2]) {
                            newJoint->actuationFunc = &rotz;
                        }
                    }
                    else if (!std::strncmp(newJoint->type, (char *)"prismatic", 9)) {
                        if (newJoint->axis[0]) {
                            newJoint->actuationFunc = &displacex;
                        }
                        if (newJoint->axis[1]) {
                            newJoint->actuationFunc = &displacey;
                        }
                        if (newJoint->axis[2]) {
                            newJoint->actuationFunc = &displacez;
                        }
                    }
                }
                // Retrieve the next line
                std::getline(file, line);
            }
            // Once </joint> is found, increment the number of joints in this URDF
            joint_num++;
        }
        // else if (line.find("<link") != std::string::npos) {
        //     while (line.find("</link>") == std::string::npos) {

        //         if (line.find("<inertial") != std::string::npos) {
        //             while (line.find("</inertial") == std::string::npos) {

        //             }
        //         }
        //     }
        // }
    }
    // This makes sure the URDF specified opened a valid file
    if (joint_num == 0) {
        fprintf(stderr, "ERROR! No DOFs were found in URDF. Check the location of the URDF.\n");
        freeMemory();
        exit(-1);
    }
    // Set the value of the global variable
    *num_joints = joint_num;
}