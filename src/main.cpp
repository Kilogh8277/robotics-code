#include "kinematics.h"
#include "stdint.h"

extern JointInfo* joints;
extern int* num_joints;

int main(int argc, char** argv) {
    char urdf_file[] = "urdf/XT_PRU_noHands.urdf";
    int name_size = 24;
    initializeMemory();
    parseURDF(urdf_file, name_size);

    double q[16] = {0};
    Transform output = {0};

    q[0] = 1.0; q[1] = 2.0; q[3] = 3.0;

    TransformFromTo(urdf_file, name_size, q, (char *)"LeftShoulderFlex", (char *)"RightWristFlex", &output.transform[0], 0.002);

    for (int i = 0; i < 16; i++) {
        printf("%0.5f ", output.transform[i]);
        if ((i+1) % 4 == 0) {
            printf("\n");
        }
    }

    char bodyName[] = "LeftElbowFlex";
    int body_index = 0;
    for (int i = 0; i < *num_joints; i++) {
        if (!std::strncmp(joints[i].name, bodyName, joints[i].name_size)) {
            body_index = i;
            break;
        }
    }
    
    GetJacobianForBody(urdf_file, name_size, q, bodyName, 0.002, &joints[body_index].jacobian.jacobian[0]);

    for (int i = 0; i < *num_joints; i++) {
        for (int j = 0; j < 6; j++) {
            printf("%0.5f ", joints[body_index].jacobian.jacobian[6*i + j]);
        }
        printf("\n");
    }

    freeMemory();
    return 0;
}