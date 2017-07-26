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
The camera administration is made under the *obtainObjectsAndNumbers()* method, which is a 
customized object structure called *objectsAndNumbers*, where both the structures involved and the abstract constructor are stated as follow:

```c++
struct object{
    //properties
    double area;
    double phi;
    double gyrumAngle;
    double principalAngle;
    Point2f centroid;
};

struct objectsAndNumbers{
    //properties
    vector<object> vectorObject;
    int numberOfObjects;
    //Constructor
    objectsAndNumbers(vector<object> vectorObjectInput,int numberOfObjectsInput): 
         vectorObject(vectorObjectInput), numberOfObjects(numberOfObjectsInput){ }
};
```
and the interface stated as

```c++
objectsAndNumbers obtainObjectsAndNumbers();
```

The *obtainObjectsAndNumbers()* returns an abstract object made of two objects, a *vector<object>* *vectorObject* which is the list of the already analyzed objects, and a *int* *numberOfObjects* which contains the number of objects obtained.

### Note
From the above code, we can see that all the necessary characteristics are inside the *objectsAndNumbers* abstract structure. In order to access the properties of the *n*-th object, the path that is needed is just
```c++
&obtainObjectsAndNumbers().vectorObject[n].area;
&obtainObjectsAndNumbers().vectorObject[n].phi;
&obtainObjectsAndNumbers().vectorObject[n].gyrumAngle;
&obtainObjectsAndNumbers().vectorObject[n].principalAngle;
&obtainObjectsAndNumbers().vectorObject[n].centroid;
```

and to access the number of objects is just needed
```c++
&obtainObjectsAndNumbers().numberOfObjects;
```
Because the *vectorObject[n]* requires an specific number to be accessed, is was easier to carry with the number of objects from the previous method and add it as an object in the structure than calculating from the *vectorObject[]* itself.

These abstract objects were necessary to be created in order to save and use in a more order way the objects' analysis results obtained from the camera analysis.

