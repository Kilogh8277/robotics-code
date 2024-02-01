#include "utils.h"

/*******************************************************************************************
Calculate the length of the vector vec
*******************************************************************************************/
double norm(double* vec, int length) {
    double sum = 0;
    for (int i = 0; i < length; i++) {
        sum += vec[i] * vec[i];
    }
    sum = sqrt(sum);
    return sum;
}

/*******************************************************************************************
Calculate the 2-norm of a vector vec
*******************************************************************************************/
void normalize(double*vec, int length) {
    double vNorm;
    vNorm = norm(vec, length);

    if (vNorm != 0) {
        for (int i = 0; i < length; i++) {
            vec[i] = vec[i] / vNorm;
        }
    }
}

/*******************************************************************************************
Calculate the matrix multiplication of a * b, output c (row-wise functions)
*******************************************************************************************/
void matrixMultiply(const double* a, const int sizeA[2], const double* b, const int sizeB[2], double* c) {
    int r1 = sizeA[0], c1 = sizeA[1], r2 = sizeB[0], c2 = sizeB[1];

    if (c1 != r2) {
        fprintf(stderr, "ERROR! Matrix dimensions do not match -- columns of A must match rows of B --> A * B\n");
        fprintf(stderr, "The dimensions are A: [%d %d] and B: [%d %d]\n", r1, c1, r2, c2);
        exit(-1);
    }

    int val = 0;
    for (int i = 0; i < r1; i++) {
        for (int j = 0; j < c2; j++) {
            c[val] = 0;
            for (int k = 0; k < c1; k++) {
                c[val] += a[r1*i+k] * b[c2*k+j];
            }
            val++;
            // std::cout << c[r1*i+j] << " ";
        }
        // std::cout << std::endl;
    }
}

// void transpose(double* mat, int size[2]) {
//     int row = size[0], col = size[1];
//     double transposedTemp[row*col];

//     for (int i = 0; i < col; i++) {
//         for (int j = 0; j < row; j++) {
//             transposedTemp[col*i+j] = mat[row*j+i];
//         }
//     }

//     for (int i = 0; i < col; i++) {
//         for (int j = 0; j < row; j++) {
//             mat[col*i+j] = transposedTemp[col*i+j];
//         }
//     }
// }

void transpose(const double* matIn, double* matOut, int size[2]) {
    int row = size[0], col = size[1];

    for (int i = 0; i < col; i++) {
        for (int j = 0; j < row; j++) {
            matOut[col*i+j] = matIn[row*j+i];
        }
    }
}

/*******************************************************************************************
Calculate the cross product of a X b, output c
*******************************************************************************************/
void cross(const double* a, const double* b, double* c) {
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

/*******************************************************************************************
Calculate the rotation matrix about the x-axis c radians
*******************************************************************************************/
void rotx(double c, double* R) {
    R[0] = 1;   R[1] = 0;       R[2] = 0;
    R[3] = 0;   R[4] = cos(c);  R[5] = sin(c);
    R[6] = 0;   R[7] = -sin(c);  R[8] = cos(c);
}

/*******************************************************************************************
Calculate the rotation matrix about the x-axis c degrees (if isDegBool = true) or c radians
(if isDegBool = false)
*******************************************************************************************/
void rotx_deg(double c, bool isDegBool, double* R) {
    if (isDegBool) {
        c = c * M_PI / 180;
    }
    rotx(c, R);
}

/*******************************************************************************************
Calculate the rotation matrix about the y-axis c radians
*******************************************************************************************/
void roty(double c, double* R) {
    R[0] = cos(c);  R[1] = 0;   R[2] = -sin(c);
    R[3] = 0;       R[4] = 1;   R[5] = 0;
    R[6] = sin(c); R[7] = 0;   R[8] = cos(c);
}

/*******************************************************************************************
Calculate the rotation matrix about the y-axis c degrees (if isDegBool = true) or c radians
(if isDegBool = false)
*******************************************************************************************/
void roty_deg(double c, bool isDegBool, double* R) {
    if (isDegBool) {
        c = c * M_PI / 180;
    }
    roty(c, R);
}

/*******************************************************************************************
Calculate the rotation matrix about the z-axis c radians
*******************************************************************************************/
void rotz(double c, double* R) {
    R[0] = cos(c); R[1] = sin(c);  R[2] = 0;
    R[3] = -sin(c); R[4] = cos(c);   R[5] = 0;
    R[6] = 0;      R[7] = 0;        R[8] = 1;
}

/*******************************************************************************************
Calculate the rotation matrix about the z-axis c degrees (if isDegBool = true) or c radians
(if isDegBool = false)
*******************************************************************************************/
void rotz_deg(double c, bool isDegBool, double* R) {
    if (isDegBool) {
        c = c * M_PI / 180;
    }
    rotz(c, R);
}

/*******************************************************************************************
Displace a 3-vector in the x-direction by c meters
*******************************************************************************************/
void displacex(const double c, double* d) {
    d[0] += c;
}

/*******************************************************************************************
Displace a 3-vector in the y-direction by c meters
*******************************************************************************************/
void displacey(const double c, double* d) {
    d[1] += c;
}

/*******************************************************************************************
Displace a 3-vector in the z-direction by c meters
*******************************************************************************************/
void displacez(const double c, double* d) {
    d[2] += c;
}

/*******************************************************************************************
Differential orthogonal rotation (rotation matrix between
two rotation matrices that accounts for wrapping)
*******************************************************************************************/
void dor(const double* Rd, const double* Ra, double* dr) {
    double Sr[9], Rd_Ra_diff[9], Ra_transpose[9];

    for (int i = 0; i < 9; i++) {
        Rd_Ra_diff[i] = Rd[i] - Ra[i];
    }
    int rotMatSize[] = {3, 3};
    transpose(Ra, Ra_transpose, rotMatSize);

    matrixMultiply(Rd_Ra_diff, rotMatSize, Ra_transpose, rotMatSize, Sr);

    dr[0] = (Sr[7] - Sr[5]) / 2;
    dr[1] = (Sr[2] - Sr[6]) / 2;    dr[2] = (Sr[3] - Sr[1]) / 2;
}