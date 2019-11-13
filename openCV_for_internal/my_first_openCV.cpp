#include <iostream>
#include <cmath>
#include <vector>
#include <Windows.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
//#include "serial_port.h"


using namespace cv;
using namespace std;


void img_process(const Mat& img_input, Mat& img_output, const InputArray& lower_bound, const InputArray& upper_bound)
{
	Mat kernel = getStructuringElement(MORPH_RECT, Size(10, 10));

	blur(img_input, img_output, Size(10, 10));
	morphologyEx(img_output, img_output, MORPH_OPEN, kernel);
	morphologyEx(img_output, img_output, MORPH_CLOSE, kernel);

	inRange(img_output, lower_bound, upper_bound, img_output);
}

bool is_square(vector<Point>& polygon, const int& lower_size, const int& upper_size)
{
	if (/*polygon.size() == 4 && */fabs( contourArea(polygon) ) > lower_size && fabs( contourArea(polygon) ) < upper_size/* && isContourConvex(polygon) */) return true;
	else return false;
}

void find_squares(const Mat& img_input, vector<vector<Point>>& squares, vector<Point>& centers, const int& lower_size, const int& upper_size)
{
	vector<vector<Point>> contours;
	vector<Point> polygon;
	vector<Vec4i>hierarchy;
	findContours(img_input, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

	for (int num = 0; num < contours.size(); num++)
	{
		approxPolyDP(contours[num], polygon, 3, true);
		if ( is_square(polygon, lower_size, upper_size) )
		{
			squares.push_back(polygon);
			Moments moment = moments(polygon , true);
			centers.push_back( Point( moment.m10 / moment.m00, moment.m01 / moment.m00) );
		}
	}
}

void draw(Mat& img_input, vector<vector<Point>> squares, vector<Point> centers)
{
	drawContours(img_input, squares, -1, Scalar(0, 0, 255), 1, 8);

	for (int num = 0; num < centers.size(); num++)
	{
		circle(img_input, centers[num], 1, Scalar(0, 0, 255));
		cout << "( " << centers.back().x << " , " << centers.back().y << " )" << endl;
	}
}

void openSerialPortandSetTimeouts(HANDLE& hComm, char commPortName[], const DWORD& BaudRate, const BYTE& ByteSize, const BYTE& Parity, const BYTE& StopBits)
{
	hComm = CreateFile(commPortName/*PortNum*/, GENERIC_READ | GENERIC_WRITE/*Read/Write*/, 0/*No Sharing*/, NULL/*No Security*/, OPEN_EXISTING/*Open existing port only*/, 0/* Non Overlapped I/O */, NULL/*Null for Comm Devices*/);
	if (hComm == INVALID_HANDLE_VALUE)
	{
		cerr << endl << "Cannot open port" << commPortName << endl;
		return;
	}
	else cout << endl << "Port " << commPortName << " Opened." << endl;

	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hComm, &dcbSerialParams))
	{
		cerr << endl << "Cannot GetCommState() for port " << commPortName << endl;
		return;
	}

	dcbSerialParams.BaudRate = BaudRate;
	dcbSerialParams.ByteSize = ByteSize;
	dcbSerialParams.StopBits = StopBits;
	dcbSerialParams.Parity = Parity;
	if (!SetCommState(hComm, &dcbSerialParams))
	{
		cerr << endl << "Cannot set DCB Structure for port " << commPortName << endl;
		return;
	}
	else
	{
		cout << endl << "Successfully set DCB for port " << commPortName << endl
			 << endl << "Baudrate : " << BaudRate << endl
			 << endl << "ByteSize : " << static_cast<int>(ByteSize) << endl
			 << endl << "Parity : " << static_cast<int>(Parity) << endl
			 << endl << "StopBits : " << static_cast<int>(StopBits) << endl;
	}

	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 0;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	if (SetCommTimeouts(hComm, &timeouts)) cout << endl << "Comm Timeouts set successfully for port " << commPortName << endl;
	else cerr << endl << "cannot set hComm timeouts for port " << commPortName << endl;
}

void setPortRead(HANDLE& hComm, char commPortName[])
{
	DWORD dwEventMask;
	if (!SetCommMask(hComm, EV_RXCHAR)) cerr << endl << "Cannot set CommMask for port " << commPortName << endl;
	else cout << endl << "Setting CommMask successfull and Waiting for Data Reception for port " << commPortName << endl;

	if (!WaitCommEvent(hComm, &dwEventMask, NULL)) cerr << endl << "Cannot set WaitCommEvent() for port " << commPortName << endl;
	else cout << endl << "Successfully set WaitCommEvent for port " << commPortName << endl;
}

void readPort(HANDLE& hComm, char& readChar)
{
	DWORD ReadSize;
	if (ReadFile(hComm, &readChar, sizeof(readChar), &ReadSize, NULL))
	{
		cout << endl << static_cast<int>(readChar) << endl;
	}
}

int main(int argc, char** argv)
{
	HANDLE hCommForRead, hCommForWrite;
	char commPortNameRead[] = "\\\\.\\COM5", commPortNameWrite[] = "\\\\.\\COM5";

	char  readChar;
	float writefloat = 0.123;
	DWORD writtenFloatSize;

	openSerialPortandSetTimeouts(hCommForRead, commPortNameRead, CBR_115200, 8, EVENPARITY, ONESTOPBIT);
//	openSerialPort(hCommForWrite, commPortNameWrite, CBR_9600, 8, NOPARITY, ONESTOPBIT);


	cout << endl << WriteFile(hCommForRead, &writefloat ,sizeof(writefloat),&writtenFloatSize,NULL) << endl;

	setPortRead(hCommForRead, commPortNameRead);
	
	while (true)
	{
		readPort(hCommForRead, readChar);
	}


//	CloseHandle(hComm);//Closing the Serial Port
	
	
/*	enum scanning_color { red, blue, green, yellow, debug }color;

	namedWindow("Color", WINDOW_AUTOSIZE);
	namedWindow("Size", WINDOW_AUTOSIZE);

	int LowB = 0;
	int HighB = 255;

	int LowG = 0;
	int HighG = 255;

	int LowR = 0;
	int HighR = 255;

	int LowS = 500;
	int HighS = 1000;

	createTrackbar("LowB", "Color", &LowB, 255);
	createTrackbar("HighB", "Color", &HighB, 255);

	createTrackbar("LowG", "Color", &LowG, 255);
	createTrackbar("HighG", "Color", &HighG, 255);

	createTrackbar("LowR", "Color", &LowR, 255);
	createTrackbar("HighR", "Color", &HighR, 255);

	createTrackbar("LowS", "Size", &LowS, 2000);
	createTrackbar("HighS", "Size", &HighS, 100000);

	
	//Open camera.
	VideoCapture cap;
	cap.open(0);
	if (!cap.isOpened())
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	
	while (true)
	{
		Mat img_input, img_output;
		vector<vector<Point>> squares;
		vector<Point> centers;

		//Capture frame to img_input.
		if (!cap.read(img_input))
		{
			cout << "Cannot read a frame from the camera." << endl;
			break;
		}

		color = debug;

		switch (color)
		{
		case red:
			img_process(img_input, img_output, Scalar(0, 0, 162), Scalar(47, 102, 255));
			break;
		case blue:
			img_process(img_input, img_output, Scalar(39, 33, 0), Scalar(155, 125, 53));
			break;
		case green:
			img_process(img_input, img_output, Scalar(0, 20, 0), Scalar(47, 86, 42));
			break;
		case yellow:
			img_process(img_input, img_output, Scalar(0,106,158) , Scalar(53, 185, 243));
			break;
		case debug:
			img_process(img_input, img_output, Scalar(LowB,LowG,LowR) , Scalar(HighB,HighG,HighR) );
			break;
		}

		find_squares(img_output, squares, centers, LowS , HighS);

		draw(img_input, squares, centers);

		imshow("input", img_input);
		imshow("output", img_output);

		waitKey(10);
	}
	return 0;
*/
}