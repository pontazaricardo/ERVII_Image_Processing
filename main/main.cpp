#include "RS232.h"
#include "ER7.h"
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "math.h"
#include <stdio.h> 

using namespace std;
using namespace cv;

int speedArm = 50;
HANDLE hCom;
#define pi acos(-1.0)

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
	objectsAndNumbers(vector<object> vectorObjectInput,int numberOfObjectsInput): vectorObject(vectorObjectInput), numberOfObjects(numberOfObjectsInput){ }
};

//Header
void initializeArm();
Point2f matrixTransformation(Point2f inputPoint);
objectsAndNumbers obtainObjectsAndNumbers();
void sendInstructionToER7(char* s1);
void sendInstructionToER7(char* s1, char* s2);
void sendInstructionToER7(char* s1, int i1);//NOTE: s1 must have %d
void sendReceiveInstructionToER7(int i1, char* s1);
void sendReceiveInstructionToER7(char* s1, int i1, char* reply);//NOTE: s1 DOES NOT NEED %d
void sendInstructionToER7(char* s1, int i1, char* s2, int i2); //NOTE: s2 needs space before and after: eg: " z ", " r "
double phiCalculation(Point2f virtualPosition);
bool tryToReach(int x, int y, int z, int p, int r, int objectNumber);
void er7Actions(objectsAndNumbers objectsAndNumbersInput);
void gotoPointNew(int x, int y, int z, int p, int r, int objectNumber);
//Ends header

//int main(int argc, char ** argv){
int main(){
	//TODO Look for general form of main
	hCom = rs232_open("COM1");
	if(hCom == 0)
		exit(-1);
	//Initialize arm
	initializeArm();
	//
	er7Actions(obtainObjectsAndNumbers());
	return 0;
}



void initializeArm(){
	//Open hand
	sendInstructionToER7("OPEN");
	//Sends to H0
	Sleep(1000);
	sendInstructionToER7("MOVE H0");
	Sleep(1000);
	//Sets speed
	sendInstructionToER7("SPEED %d", speedArm);
	//Move to position out of camera
		//Deletes point
	sendInstructionToER7("DELP C0");
	sendInstructionToER7("YES");
		//Recreates point
	sendInstructionToER7("DEFP C0");
	sendInstructionToER7("TEACH C0");
		//TODO: Set values of coordinates for point out of camera
	sendInstructionToER7("0");//Value X //10,000
	sendInstructionToER7("0");//Value Y //1000
	sendInstructionToER7("0");//Value Z //1000
	sendInstructionToER7("0");//Value P //1000
	sendInstructionToER7("0");//Value R //7000
	//Moves to point
	sendInstructionToER7("MOVE C0");
	//Teaches Up
		//Deletes point
	sendInstructionToER7("DELP UP");
	sendInstructionToER7("YES");
		//Defines point
	sendInstructionToER7("DEFP UP");
		//Teaches point
	sendInstructionToER7("TEACHR UP");
	sendInstructionToER7("0");//x
	sendInstructionToER7("0");//y
	sendInstructionToER7("2000");//z //TODO: Check correct z value
	sendInstructionToER7("0");//p
	sendInstructionToER7("0");//r

	//Teaches Up
		//Deletes point
	sendInstructionToER7("DELP DOWN");
	sendInstructionToER7("YES");
		//Defines point
	sendInstructionToER7("DEFP DOWN");
		//Teaches point
	sendInstructionToER7("TEACHR DOWN");
	sendInstructionToER7("0");//x
	sendInstructionToER7("0");//y
	sendInstructionToER7("-2000");//z //TODO: Check correct z value
	sendInstructionToER7("0");//p
	sendInstructionToER7("0");//r
}

double phiCalculation(Point2f virtualPosition){
    //Calculation of hypotenuse
	double hypotenuse=sqrt(virtualPosition.x*virtualPosition.x+virtualPosition.y*virtualPosition.y);
	return asin(353/hypotenuse)+asin(virtualPosition.y/hypotenuse);
}

objectsAndNumbers obtainObjectsAndNumbers(){
	//Open camera
	VideoCapture videoCapture(0);
	if(!videoCapture.isOpened()){
		exit (-1);//TODO: Find return
	}
	Mat realWorldImage;
	namedWindow("Real world",1);
	cout << "Press a key in order to start analysis" << endl;
	//Waits for the user to press a key and start analysis
	while(1){
		Mat frameRealWorldImage;
		videoCapture >> frameRealWorldImage;
		cvtColor(frameRealWorldImage, realWorldImage, CV_BGR2GRAY);
		imshow("Real world image", realWorldImage);
		if(waitKey(30)>=0) break;
	}
	videoCapture.release();
	//Modification of image in copy
	Mat resultRealWorldImage;
	resultRealWorldImage = realWorldImage.clone();
	//Apply blur, binarization and dilatation
	blur(resultRealWorldImage, resultRealWorldImage, Size(6,6));
	threshold(resultRealWorldImage, resultRealWorldImage, 80, 255, CV_THRESH_BINARY);
	erode(resultRealWorldImage,resultRealWorldImage, Mat(6,6,0));
	dilate(resultRealWorldImage,resultRealWorldImage, Mat(6,6,0));
	//Finding contours process begins
		//Objects for managing obtained information of contours
	vector<vector<Point> > foundContours;
	vector<Vec4i> outputArrayHierarchy;
		//Finding contour methods
	Canny(resultRealWorldImage, resultRealWorldImage, 100, 255, 3); //Applies Canny edge detector and produces the edge map.
	findContours(resultRealWorldImage, foundContours, outputArrayHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));//INPUTS: InputOutputArray image, OutputArrayOfArrays contours,OutputArray hierarchy, int mode, int method, Point offset=Point()
	//Finding contours process ends
	//Finding of moments
	//Related structure creation
		//Creation of foundMoments vector<Moments>
	vector<Moments> foundMoments(foundContours.size());
		//Creation of structure to get centroid information using moments
	vector<Point2f> momentCentroid(foundContours.size());
		//Creation of structure for principal angle
	vector<double> principalAngleVector(foundContours.size());
		//Creation of offset value
	double offset = 2.5/180.0*pi;
		//Creation of objects to be returned
	vector<object> objectVectorToReturn(foundContours.size()/2);
	//Start cycles:
		//Cycle 1: Calculation of the moments
	for(int i0=0; i0<foundContours.size(); i0++){
		foundMoments[i0] = moments(foundContours[i0],false);
	}
		//Cycle 2: Calculation of the centroids
	for( int i1=0; i1<foundContours.size(); i1++ ){
		momentCentroid[i1] = Point2f(foundMoments[i1].m10/foundMoments[i1].m00, foundMoments[i1].m01/foundMoments[i1].m00);
	}
	Mat realWorldImageClone = realWorldImage.clone();
		//Cycle 3: Calculation of the principle angle
	for(int i2=0; i2<foundContours.size(); i2++){
		principalAngleVector[i2] = (atan2(2*foundMoments[i2].mu11,foundMoments[i2].mu20-foundMoments[i2].mu02))/2;
	}
		//Cycle 4: Transformation
	for(int i3=0; i3<foundContours.size()/2; i3++){
			objectVectorToReturn[i3].area = foundMoments[i3*2].m00;
			objectVectorToReturn[i3].centroid = matrixTransformation(momentCentroid[i3*2]);
			objectVectorToReturn[i3].gyrumAngle = principalAngleVector[i3*2];
			objectVectorToReturn[i3].principalAngle = principalAngleVector[i3*2]+pi/2;
			objectVectorToReturn[i3].phi = phiCalculation(objectVectorToReturn[i3].centroid)-offset;
			//Cout the obtained values for the objects
			cout << "================================" << endl;
			cout << "Object no:[" << i3 << "]" << endl;
			cout << "Centroids:" << endl;
			cout << "Undistorsioned:[" << momentCentroid[i3*2].x << "," << momentCentroid[i3*2].y << "]" << endl; // Camera centroid
			cout << "Real world:[" << objectVectorToReturn[i3].centroid.x << "," << objectVectorToReturn[i3].centroid.y << "]" << endl; //ER7 centroid
			cout << "Principal angle:[" << objectVectorToReturn[i3].principalAngle/pi*180 << "]" << endl;
			cout << "Gyrum angle:[" << objectVectorToReturn[i3].gyrumAngle/pi*180 << "]" <<endl;
			cout << "Phi:[" << objectVectorToReturn[i3].phi/pi*180 << "]" << endl;
			cout << "================================" << endl;
	}
	cout << "Total of detected objects:["<< foundContours.size()/2 <<"]" << endl;
	//Creation of objectsAndNumbers structure
	objectsAndNumbers objectsAndNumbersReturn(objectVectorToReturn,foundContours.size()/2); //Number of objects: foundContours.size()/2
	//Send of created objectsAndNumbers structure
	return objectsAndNumbersReturn;
}


Point2f matrixTransformation(Point2f inputPoint){
	Point2f pointToReturn;
	//Transformation matrix
	double transformationMatrix[3][3] = {{0.1,7.3,0.0},{7.6,-0.2,0.1},{3570.7,-2429.7,718.9}};//TODO: check values
	//Definition of point extension in order to be multiplied
	double inputPointExtended[3] = {inputPoint.x, inputPoint.y, 1};
	pointToReturn.x = 0.0;
	pointToReturn.y = 0.0;
	//Matrix multiplication
		//Cycle one: column 0
	for(int i0=0; i0<3;i0++){
		pointToReturn.x += inputPointExtended[i0] * transformationMatrix[i0][0];
	}
		//Cycle two: column 1
	for(int i1=0; i1<3;i1++){
		pointToReturn.y += inputPointExtended[i1] * transformationMatrix[i1][1];
	}
	return pointToReturn;
}

bool tryToReach(int x, int y, int z, int p, int r, int objectNumber){
	//Creation of objects needed to verify if the point is reachable
	char verifyPointer[30];
	char bufReply[6][50];
	//NOTE: TMP points are going to be used
		//Deletes TMP(N) point
	sendInstructionToER7("DELP TMP%d",objectNumber);
	sendInstructionToER7("YES");
		//Redefines TMP(N) point
	sendInstructionToER7("DEFP TMP%d",objectNumber);
		//Assigns HERE to TEMP(N)
	sendInstructionToER7("HERE TMP%d",objectNumber);
		//Teaches TMP(N) and keeps track of the answer of the console in bufReply
	sendReceiveInstructionToER7("TEACH TMP%d",objectNumber,bufReply[0]);
	sendReceiveInstructionToER7(x,bufReply[1]);
	sendReceiveInstructionToER7(y,bufReply[2]);
	sendReceiveInstructionToER7(z,bufReply[3]);
	sendReceiveInstructionToER7(p,bufReply[4]);
	sendReceiveInstructionToER7(r,bufReply[5]); //GOT FINAL ANSWER FOR R
	//If final answer is BAD POINT COORDINATES, means that the object is out of working space
	char *finalAnswer=strstr(bufReply[5],"Bad");
	if(finalAnswer==NULL){
		return true;
	}
	//TRY to confirm if 'BAD POINT COORDINATES' is gotten
	memcpy(verifyPointer,finalAnswer,9);//TODO: VERIFY badpoint
	verifyPointer[9]='\0';
	if(strcmp(verifyPointer,"Bad point")==0){
		return false;
	}
	return true;
}


void er7Actions(objectsAndNumbers objectsAndNumbersInput){
	//Consruction of necessary structures
		//Number of objects to be working on
	int numerOfObjects = objectsAndNumbersInput.numberOfObjects;
		//Objects
			//Object with maximum area
				//Biggest object index
	int biggestObject;
				//Maximum area detected
	double maximumObjectArea = 0;
				//Index for baseObject
	int baseObjectIndex;
			//Missing coordinates
				//z coordinate
	int z=1400;
				//height
	int heightOfObject=0;
				//roll coordinate
	int rollOfObject=0;

	sendInstructionToER7("OPEN");
	Sleep(1000);
	//SetDefaultJoint(hCom, "ORIG", 5116, -353, 4354, -700, -201,speedArm);
	SetDefaultJoint(hCom, "ORIG", 0, -11244, 6580, -22437, 0,speedArm);
	//Look for the biggest area object
	while(numerOfObjects>0){
		maximumObjectArea=0;
		for(int i0=0; i0<objectsAndNumbersInput.numberOfObjects; i0++){
			if(objectsAndNumbersInput.vectorObject[i0].area>maximumObjectArea) {
				biggestObject=i0;
				maximumObjectArea = objectsAndNumbersInput.vectorObject[i0].area;
			}
		}
		//Override biggest area object area parameter
		objectsAndNumbersInput.vectorObject[biggestObject].area = 0.0;
		if(heightOfObject == 0){//GET INSIDE FOR BIGGEST OBJECT
			rollOfObject = -201+((-1)*objectsAndNumbersInput.vectorObject[biggestObject].phi-objectsAndNumbersInput.vectorObject[biggestObject].gyrumAngle)/pi*(180*10);
			//Try to see if the object is in the working area, if it is, teaches it to grab it, if not, throws an error
			if(!tryToReach(objectsAndNumbersInput.vectorObject[biggestObject].centroid.x, objectsAndNumbersInput.vectorObject[biggestObject].centroid.y, z, -900, rollOfObject,numerOfObjects)){
				 cout << "[OBJECT IS OUT OF WORKING AREA]" << endl;
			}else{
				//Change coordinates for object
				heightOfObject++;
				baseObjectIndex=biggestObject;
				cout << "[Object inside working area; base selected:" << biggestObject << "]"<<endl;
			}
			numerOfObjects--; //Gets one object out of the list
		}else{//BASE IS ALREADY SELECTED
			//Calculates the coordinates for object to be picked up
				//Z: 900 (before 1400)
			z = 900;
			rollOfObject = -201+((-1)*objectsAndNumbersInput.vectorObject[biggestObject].phi-objectsAndNumbersInput.vectorObject[biggestObject].gyrumAngle)/pi*(180*10);
			if(!tryToReach(objectsAndNumbersInput.vectorObject[biggestObject].centroid.x, objectsAndNumbersInput.vectorObject[biggestObject].centroid.y, z, -900, rollOfObject,numerOfObjects)){
				//The object is out of the working area
				cout << "[OBJECT IS OUT OF WORKING AREA]" << endl;
			}else{
				//Object is inside of working area
				cout << "[Object inside working area]" << endl;
				//cout << "---0---" << endl;
				z+=390;
				sendInstructionToER7("SETPVC TMP%d",numerOfObjects ," z %d", z);
				Sleep(1000);
				sendInstructionToER7("MOVE TMP%d",numerOfObjects);
				Sleep(1000);
				//cout << "---1---" << endl;
				z-=390;
				sendInstructionToER7("SETPVC TMP%d",numerOfObjects ," z %d", z);
				Sleep(1000);
				sendInstructionToER7("MOVE TMP%d",numerOfObjects);
				//cout << "---2---" << endl;
				Sleep(1000);
				sendInstructionToER7("CLOSE");
				Sleep(1000);
				z+=390*(heightOfObject+1);
				rollOfObject=-201+((-1)*objectsAndNumbersInput.vectorObject[baseObjectIndex].phi-objectsAndNumbersInput.vectorObject[baseObjectIndex].gyrumAngle)/pi*(180*10);
				sendInstructionToER7("SETPVC TMP%d",numerOfObjects," z %d", z);
				Sleep(1000);
				sendInstructionToER7("SETPVC TMP%d",numerOfObjects," r %d", rollOfObject);
				Sleep(1000);
				sendInstructionToER7("MOVE TMP%d",numerOfObjects);
				Sleep(1000);
				//cout << "---3---" << endl;
				gotoPointNew(objectsAndNumbersInput.vectorObject[baseObjectIndex].centroid.x, objectsAndNumbersInput.vectorObject[baseObjectIndex].centroid.y, z, -900, rollOfObject,numerOfObjects);
				Sleep(1000);
				z-=385;
				sendInstructionToER7("SETPVC TMP%d",numerOfObjects," z %d", z);
				Sleep(1000);
				sendInstructionToER7("MOVE TMP%d",numerOfObjects);
				Sleep(1000);
				sendInstructionToER7("OPEN");
				Sleep(1000);
				z+=500;
				//cout << "---4---" << endl;
				sendInstructionToER7("SETPVC TMP%d",numerOfObjects," z %d", z);
				Sleep(1000);
				sendInstructionToER7("MOVE TMP%d",numerOfObjects);
				Sleep(1000);
				sendInstructionToER7("MOVE ORIG");
				heightOfObject++;
			}
			numerOfObjects--;
		}
	}
}

void sendInstructionToER7(char* s1){
	rs232_sendCmd(hCom, s1);
}

void sendInstructionToER7(char* s1, char* s2){
	char buf[100];
	sprintf(buf, s1, s2);
	rs232_sendCmd(hCom, buf);
}

void sendInstructionToER7(char* s1, int i1){//NOTE: s1 must have %d
	char buf[100];
	sprintf(buf, s1, i1);
	rs232_sendCmd(hCom, buf);
}

void sendReceiveInstructionToER7(int i1, char* s1){
	char buf[50];
	sprintf(buf, "%d", i1);
	rs232_sendCmd(hCom, buf, s1);
}

void sendReceiveInstructionToER7(char* s1, int i1, char* reply){
	char buf[50];
	sprintf(buf,s1,i1);
	rs232_sendCmd(hCom, buf, reply);
}

void sendInstructionToER7(char* s1, int i1, char* s2, int i2){//NOTE: s1 must have %d
	char buf1[50];
	char buf2[50];
	sprintf(buf1,s1,i1);
	sprintf(buf2,s2,i2);
	strcat(buf1,buf2);
	rs232_sendCmd(hCom, buf1);
}


void gotoPointNew(int x, int y, int z, int p, int r, int objectNumber){
	sendInstructionToER7("DEFP TMP%d",objectNumber); // 382, -85, 402, -85, -20
	sendInstructionToER7("HERE TMP%d",objectNumber);
	sendInstructionToER7("SETPVC TMP%d",objectNumber," X %d", x);
	sendInstructionToER7("SETPVC TMP%d",objectNumber," Y %d", y);
	sendInstructionToER7("SETPVC TMP%d",objectNumber," Z %d", z);
	sendInstructionToER7("SETPVC TMP%d",objectNumber," P %d", p);	
	sendInstructionToER7("SETPVC TMP%d",objectNumber," R %d", r);
	sendInstructionToER7("SPEED %d",30);
	sendInstructionToER7("MOVE TMP%d",objectNumber);
	Sleep(1000);
}