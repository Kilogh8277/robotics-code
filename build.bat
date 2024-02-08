@echo off
IF [%1] EQU [] echo Please provide the distribution of MATLAB you wish to use. For example, to use R2023a, type 'build.bat 23a'.
IF [%1] NEQ [] call:SOURCE %1

GOTO:EOF


:SOURCE
    call C:\ProgramData\MATLAB\SupportPackages\R20%1\toolbox\slrealtime\target\supportpackage\qnx710\qnxsdp-env.bat
    mkdir build
    cd build
    cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../qnxToolchain.cmake ..
    make
    cd ..