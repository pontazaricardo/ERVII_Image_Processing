# ERVII_Image_Processing

## Introduction
The main idea of this file is to present the analysis, design, problems and implementation of a c++ program instance that could control the ERVII (ER7) arm in order to detect several different objects and manipulate them in such a way that could be staffed in a pyramid ordered by size.

## Goal

This project's main goal is stated as \emph{Given a set of randomly picked objects, you need to locate these objects through a given camera and pick and place these objects as a stack.} With this idea in mind, the logic of the processes needed to solve this were stated as
1. Obtain images from camera.
2. Work on camera calibration.
3. Work on the inverse kinematics (when solvable)
4. Design and implement a c++ program to communicate with the ER7 arm via ACL.

## Camera software and calibration

In order to use the camera's images and make an analysis of the objects captured by it, we used *openCV* libraries that already have several different image processing characteristics inside their methods.

## Datastructure

The *main()* clause in our c++ implementation is stated as follows:
```c++
int main(){
    hCom = rs232_open("COM1");
    if(hCom == 0)
        exit(-1);
    //Initialize arm
    initializeArm();
    //Make camera calibration and obtain objects in obtainObjectsAndNumbers()
    er7Actions(obtainObjectsAndNumbers());
    return 0;
}
```
