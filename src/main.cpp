#include "transform.h"

extern JointInfo joints[joint_length];
extern int num_joints;

int main(int argc, char** argv) {
    // Specify your URDF file path
    char urdf_file[] = "/workspaces/robotics_code/urdf/alpha4_LC_FB_XYZ.urdf";
    
    // Parse URDF file and extract joint information
    parseURDF(urdf_file);

    double transform[16] = {1, 2, 3, 4,
                            5, 6, 7, 8,
                            9, 10, 11, 12,
                            13, 14, 15, 16};
    double rotMat[9] = {0};
    double distVec[3] = {0};

    GetRotationMatrixFromTransform(transform, rotMat, distVec);

    double q[16] = {0};
    InitTransforms();



    getTransform((char *)"LeftAnkleFlex", q);

    for (int i = 0; i < 16; i++) {
        printf("%0.5f ", q[i]);
        if ((i+1) % 4 == 0) {
            printf("\n");
        }
    }
    return 0;
}