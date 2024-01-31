#include "transform.h"

// Structure to represent joint information
struct JointInfo {
    // Parameters of this joint
    char name[50] = {0};
    char type[10] = {0};
    double origin_xyz[3];
    double origin_rpy[3];
    double axis[3] = {0, 0, 0};
    JointInfo* parent_link = NULL;
    JointInfo* child_links[5] = {NULL}; // Assuming a maximum of 5 child links (adjust as needed)
    int numChildren = 0;

    // Important joint information
    Transform transform;
};

struct JointInfo joints[joint_length];
int num_joints = 0, num_actuators = 0;
uint8_t initialized = 0;

// void calculateTransform(Orientation rotation, Vector3 displacement, double* transform) {
//     transform[0] = rotation[0]; transform[1] = rotation[1]; transform[2] = rotation[2];  transform[3] = displacement[0];
//     transform[4] = rotation[3]; transform[5] = rotation[4]; transform[6] = rotation[5];  transform[7] = displacement[1];
//     transform[8] = rotation[6]; transform[9] = rotation[7]; transform[10] = rotation[8]; transform[11] = displacement[2];
//     transform[12] = 0;          transform[13] = 0;          transform[14] = 0;           transform[15] = 1;
// }

JointInfo* findJointByName(std::string nameToFind) {
    JointInfo* currJoint;
    for (int i = 0; i < 50; i++) {
        currJoint = &joints[i];
        // printf("Current joint: %s\n", currJoint->name);
        // printf("Name to find : %s\n", nameToFind.c_str());
        if (currJoint->name[0] > 0 && !std::strcmp(nameToFind.c_str(), currJoint->name)) {
            return currJoint;
        }
    }
    return NULL;
}

void GetOffset(char* bodyName, double* offset) {
    for (int i = 0; i < 50; i++) {
        if (!std::strcmp(joints[i].name, bodyName)) {
            memcpy(offset, joints[i].origin_xyz, 3*sizeof(double));
            break;
        }
    }
}

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

void InitTransforms() {
    if (initialized == 0) {
        Transform currTransform; double rotMat[9] = {0}, distVec[3] = {0};
        Transform parentTransform; double parentRotMat[9], parentDistVec[3];
        
        memcpy(&currTransform, &joints[0].transform, sizeof(Transform));
        GetRotationMatrixFromTransform(currTransform.transform, rotMat, distVec);
        double rotX[9] = {0}, rotY[9] = {0}, rotZ[9] = {0}, 
               rotMatX[9] = {0}, rotMatXY[9] = {0}, rotMatXYZ[9] = {0}; 
        int rotSize[] = {3, 3}, transformSize[] = {4, 4};
        RotXYZ(&rotMat[0], joints[0].origin_rpy);
        rotx(joints[0].origin_rpy[0], rotX);
        roty(joints[0].origin_rpy[1], rotY);
        rotz(joints[0].origin_rpy[2], rotZ);
        
        matrixMultiply(rotMat, rotSize, rotX, rotSize, rotMatX);
        matrixMultiply(rotMatX, rotSize, rotY, rotSize, rotMatXY);
        matrixMultiply(rotMatXY, rotSize, rotZ, rotSize, rotMatXYZ);
        distVec[0] = -joints[0].origin_xyz[0];
        distVec[1] = -joints[0].origin_xyz[1];
        distVec[2] = -joints[0].origin_xyz[2];
        SetTransformFromRotMat(rotMatXYZ, distVec, &joints[0].transform);
        for (int i = 1; i < num_joints; i++) {
            memset(rotX, 0, 9*sizeof(double)); memset(rotY, 0, 9*sizeof(double)); memset(rotZ, 0, 9*sizeof(double));
            memset(rotMatX, 0, 9*sizeof(double)); memset(rotMatXY, 0, 9*sizeof(double)); memset(rotMatXYZ, 0, 9*sizeof(double));
            memset(distVec, 0, 3*sizeof(double));
            currTransform = joints[i].transform; parentTransform = joints[i].parent_link->transform;
            GetRotationMatrixFromTransform(currTransform.transform, rotMat, distVec);
            GetRotationMatrixFromTransform(parentTransform.transform, parentRotMat, parentDistVec);
            rotx(joints[i].origin_rpy[0], rotX);
            roty(joints[i].origin_rpy[1], rotY);
            rotz(joints[i].origin_rpy[2], rotZ);
            matrixMultiply(rotMat, rotSize, rotX, rotSize, rotMatX);
            matrixMultiply(rotMatX, rotSize, rotY, rotSize, rotMatXY);
            matrixMultiply(rotMatXY, rotSize, rotZ, rotSize, rotMatXYZ);
            double temp[3] = {0}; int vecSize[] = {3, 1};
            temp[0] = -joints[i].origin_xyz[0];
            temp[1] = -joints[i].origin_xyz[1];
            temp[2] = -joints[i].origin_xyz[2];
            matrixMultiply(rotMatXYZ, rotSize, temp, vecSize, distVec);
            Transform result = {0};
            SetTransformFromRotMat(rotMatXYZ, distVec, &currTransform);
            matrixMultiply(&currTransform.transform[0], transformSize, &parentTransform.transform[0], transformSize, &result.transform[0]);
            memcpy(&joints[i].transform, &result, sizeof(Transform));
        }
        initialized = 1;
    }
}

void TransformFromTo(const char* source, const char* target) {
    Transform sourceTransform, targetTransform;
    for (int i = 0; i < joint_length; i++) {
        if (!std::strcmp(joints[i].name, source)) {
            sourceTransform = joints[i].transform;
        }
        if (!std::strcmp(joints[i].name, target)) {
            targetTransform = joints[i].transform;
        }
    }
}

void getTransform(const char* body, double* transform) {
    for (int i = 0; i < joint_length; i++) {
        if (!std::strcmp(body, joints[i].name)) {
            memcpy(transform, &joints[i].transform.transform[0], sizeof(Transform));
            return;
        }
    }
}

void RotXYZ(double* rotationMat, const double* rpy) {
    rotx(rpy[0], rotationMat);
    roty(rpy[1], rotationMat);
    rotz(rpy[2], rotationMat);
}

// Function to parse URDF file and extract joint information
void parseURDF(char* urdf_file) {
    std::ifstream file(urdf_file);
    std::string line;
    int joint_num = 0;

    while (std::getline(file, line)) { // Assuming a maximum of 100 joints (adjust as needed)
        if (line.find("<joint") != std::string::npos) {
            JointInfo *newJoint = &joints[joint_num];

            std::size_t name_pos = line.find("name=\"") + 6;
            std::size_t name_end = line.find("\"", name_pos);
            memcpy(&newJoint->name, line.substr(name_pos, name_end - name_pos).c_str(), name_end - name_pos);

            std::size_t type_pos = line.find("type=\"") + 6;
            std::size_t type_end = line.find("\"", type_pos);
            memcpy(&newJoint->type, line.substr(type_pos, type_end - type_pos).c_str(), type_end - type_pos);
            if(!std::strcmp(newJoint->type, "revolute")) {
                num_actuators++;
            }

            while (line.find("</joint") == std::string::npos) {
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
                }
                std::getline(file, line);
            }
            joint_num++;
        }
    }
    num_joints = joint_num;
}