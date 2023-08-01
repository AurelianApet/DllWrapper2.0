// DLWrapper.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include <iostream>

#include "yolo_v2_class.h"
#include "opencv2/core/cuda.hpp"
using namespace cv;
using namespace std;

string cfg_file = "air.cfg";				  //yolo model cfg file 
string weight_file = "air.weights";	      //yolo model 	weights file 
Detector *pDetector = NULL;

void Log(string logstring)
{
	FILE* fstream = fopen("D:\\dlwrapper.log", "a+");
	if (fstream != NULL)
	{
		fwrite(logstring.data(), logstring.size(), 1, fstream);
		fwrite("\r\n", 2, 1, fstream);
	}
	fclose(fstream);
}

extern "C" {
	// return value 0 = success, 1 = already initialized, -1 = failed.
	_declspec(dllexport) int Initialize()
	{
		if (pDetector != NULL)
		{
			return 1;
		}
		CHAR lpszPath[1024];
		GetCurrentDirectoryA(1024, lpszPath);
		strcat_s(lpszPath, "\\");
		string FullPath = lpszPath;
		FullPath += cfg_file;

		std::ifstream ifile(FullPath.c_str());
		if (!ifile) {
			Log(FullPath + " doesn't exist.");
			return -1;
		}

		FullPath = lpszPath;
		FullPath += weight_file;
		std::ifstream iweightfile(FullPath.c_str());
		if (!iweightfile) {
			Log(FullPath + " doesn't exist.");
			return -1;
		}

		pDetector = new Detector(cfg_file, weight_file);
		if (pDetector == NULL)
		{
			return -1;
		}

		return 0;

	}
	_declspec(dllexport) int Detect_test(unsigned char* data, int height, int width, void* detectResult)
	{
		RECT* rc = new RECT[3];
		rc[0].left = 10;
		rc[0].top = 20;
		rc[0].right = 30;
		rc[0].bottom = 40;
		rc[1].left = 50;
		rc[1].top = 60;
		rc[1].right = 70;
		rc[1].bottom = 80;
		rc[2].left = 30;
		rc[2].top = 40;
		rc[2].right = 50;
		rc[2].bottom = 60;
		//detectResult = new RECT[3];

		memcpy(detectResult, rc, 3*sizeof(RECT));
		return 3;
	}
	_declspec(dllexport) int Detect(unsigned char* data, int height, int width, void* detectResult)
	{
		if (pDetector != NULL)
		{
			Log("Detector didn't initialize.");
			return 0;
		}
		Log("Detecting ...");
		float zscale = 0.3;
		Mat iyframe;

		Mat constructed (height, width, CV_8UC4, data);

		//이미지를 90도 회전
		cv::rotate(constructed, constructed, ROTATE_90_COUNTERCLOCKWISE);


		//이미지 사이즈를 zscale배로 작게한다.
		cv::resize(constructed, iyframe, Size(width*zscale, height*zscale));

		vector<bbox_t> result;
		vector<RECT> air_rects;
		int lap_f = 0;

		result = pDetector->detect(iyframe);
		for (int j = 0; j < result.size(); j++)
		{
			if (result[j].obj_id == 4)//비행기인 경우
			{
				RECT rc; 
				rc.left = (LONG)result[j].x / zscale;
				rc.top = (LONG)result[j].y / zscale;
				rc.right = (LONG)result[j].w / zscale;
				rc.bottom = (LONG)result[j].h / zscale;
				air_rects.push_back(rc);
				lap_f++;
			}
			if (lap_f == 0 && (result[j].obj_id == 63 || result[j].obj_id == 62 || result[j].obj_id == 73 || result[j].obj_id == 67))
			{ //모니터, TV, 랩톱, 셀폰
				Mat cropped_monitor(iyframe, Rect(result[j].x, result[j].y, result[j].w, result[j].h));
				vector<bbox_t> air_plane_result;
				air_plane_result = pDetector->detect(cropped_monitor);
				for (int i = 0; i < air_plane_result.size(); i++)
				{
					if (air_plane_result[i].obj_id == 4)
					{
						RECT rc;
						rc.left = (LONG)(result[j].x+air_plane_result[j].x) / zscale;
						rc.top = (LONG)(result[j].y+air_plane_result[j].y) / zscale;
						rc.right = (LONG)air_plane_result[j].w / zscale;
						rc.bottom = (LONG)air_plane_result[j].h / zscale;
						air_rects.push_back(rc);
					}
				}
			}
		}

		int air_count = air_rects.size();
		if (air_count == 0)
			return 0;

		//최대로 20개까지 인식하도록....
		if (air_count > 20)
			air_count = 20;

		char msg[1024]; sprintf_s(msg, "Detected all %d", air_count);
		string logmessage = msg;
		Log(msg);
		memcpy(detectResult, air_rects.data(), air_count*sizeof(RECT));
		return air_count;
	}
}