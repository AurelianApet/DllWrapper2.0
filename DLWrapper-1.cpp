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
	FILE* fstream = fopen("dlwrapper.log", "a+");
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
			//Log(FullPath + " doesn't exist.");
			return -1;
		}

		FullPath = lpszPath;
		FullPath += weight_file;
		std::ifstream iweightfile(FullPath.c_str());
		if (!iweightfile) {
			//Log(FullPath + " doesn't exist.");
			return -1;
		}


		int nSize = sizeof(RECT);
		CHAR msg[1024];
		sprintf_s(msg, "Rect size = %d", nSize);
		Log(msg);
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

		memcpy(detectResult, rc, 1*sizeof(RECT));
		return 1;
	}
	Rect rotateRectBack(Rect rect, int k, int wd, int ht)
	{
		Point p0 = Point(rect.x, rect.y);
		Point p3 = Point(rect.x + rect.width, rect.y);
		Point p2 = Point(rect.x + rect.width, rect.y + rect.height);
		Point p1 = Point(rect.x, rect.y + rect.height);

		Rect converted_rect;

		if (k == 0)
		{
			converted_rect = rect;

		}
		if (k == 1)
		{
			converted_rect.x = p3.y;
			converted_rect.y = wd - p3.x;
			converted_rect.width = rect.height;
			converted_rect.height = rect.width;
		}
		if (k == 2)
		{
			converted_rect.x = wd - p2.x;
			converted_rect.y = ht - p2.y;
			converted_rect.width = rect.width;
			converted_rect.height = rect.height;

		}
		if (k == 3)
		{
			converted_rect.x = ht - p1.y;
			converted_rect.y = p1.x;
			converted_rect.width = rect.height;
			converted_rect.height = rect.width;

		}

		return converted_rect;
	}
	_declspec(dllexport) int Detect(unsigned char* data, int height, int width, void* detectResult)
	{
		float zscale = 0.4;
		Mat frame(height, width, CV_8UC4, data);
		Mat iyframe;
		float camera_scale = 0.0;
		int rs_width = 0;
		int rs_height = 0;
		int rotate_flag = 0;
		int n_r_key = 0;
		int f_count = 1;
		bool detect_yes = true;
		vector<RECT> air_rects;
		if (pDetector == NULL)
		{
			//Log("Detector didn't initialize.");
			return 0;
		}
		//Log("Detecting ...");

		n_r_key = 2;
		
		rotate_flag = n_r_key % 3;
		try {
			if (rotate_flag == 1)
			{
				cv::rotate(frame, frame, ROTATE_90_CLOCKWISE);
			}
			if (rotate_flag == 2)
			{
				cv::rotate(frame, frame, ROTATE_90_COUNTERCLOCKWISE);
			}
			int f_width = frame.size().width;
			int f_height = frame.size().height;
			camera_scale = f_width*1.0 / f_height;
			if (f_width > f_height)
			{
				rs_width = f_width;
				rs_height = f_height;
			}
			if (f_height > f_width)
			{
				rs_width = f_height * (camera_scale);
				rs_height = f_height;
			}

			Rect ROI = Rect(0, 0, f_width, f_height);
			//Mat image;
			//cv::rotate(frame, frame, ROTATE_90_COUNTERCLOCKWISE);
			Mat image(rs_height, rs_width, CV_8UC3, Scalar(0, 0, 0));
			//cv::resize(frame, image, Size(rs_width, rs_height));
			frame.copyTo(image(ROI));
			//cv::rotate(image, image, ROTATE_90_CLOCKWISE);
			//cv::imwrite("rotate.jpg", image);
			cv::resize(image, iyframe, Size(int(image.size().width*zscale), int(image.size().height*zscale)));
			int lap_f = 0;
			try
			{
				vector<bbox_t> result;
				vector<bbox_t> air_plane_result;
				result = pDetector->detect(iyframe);
				//imwrite("frame.jpg", iyframe);
				for (int j = 0; j < result.size(); j++)
				{
					if (result[j].obj_id == 4)
					{

						Rect boundrect = Rect(result[j].x, result[j].y, result[j].w, result[j].h);
						int x = boundrect.x / zscale;
						int y = boundrect.y / zscale;
						int width = boundrect.width / zscale;
						int height = boundrect.height / zscale;
						RECT rc;
						rc.left = (LONG)(x);
						rc.top = (LONG)(y);
						rc.right = (LONG)(x + width);
						rc.bottom = (LONG)(y + height);
						air_rects.push_back(rc);
						lap_f++;
					}
					if (lap_f == 0 && (result[j].obj_id == 63 || result[j].obj_id == 62 || result[j].obj_id == 73 || result[j].obj_id == 67))
					{

						Mat cropped_monitor(iyframe, Rect(result[j].x, result[j].y, result[j].w, result[j].h));
						//imshow("test", cropped_monitor);
						//imwrite("cropped_monitor.jpg", cropped_monitor);
						Mat rotated_monitor_mat;
						for (int k = 0; k < 4; k++)
						{
							if (k == 0)
							{
								rotated_monitor_mat = cropped_monitor.clone();
							}
							else
							{
								cv::rotate(cropped_monitor, rotated_monitor_mat, k - 1);
							}

							air_plane_result = pDetector->detect(rotated_monitor_mat);

							for (int i = 0; i < air_plane_result.size(); i++)
							{

								if (air_plane_result[i].obj_id == 4)
								{

									//cv::imshow("ccc", rotated_monitor_mat);
									Rect tmp = Rect(air_plane_result[i].x, air_plane_result[i].y, air_plane_result[i].w, air_plane_result[i].h);
									Rect rotated_back_rect = rotateRectBack(tmp, k, rotated_monitor_mat.size().width, rotated_monitor_mat.size().height);
									int air_x = result[j].x + rotated_back_rect.x;
									int air_y = result[j].y + rotated_back_rect.y;

									int x = int(air_x / zscale);
									int y = int(air_y / zscale);
									int w = int(rotated_back_rect.width / zscale);
									int h = int(rotated_back_rect.height / zscale);

									RECT rc;
									rc.left = (LONG)x;
									rc.top = (LONG)y;
									rc.right = (LONG)(x + w);
									rc.bottom = (LONG)(y + h);
									air_rects.push_back(rc);
									lap_f++;
									
								}

							}

						}

					}
				}
			}
			catch (Exception e)
			{	}

		}
		catch (Exception e)
		{	}
		
		int air_count = air_rects.size();
		if (air_count == 0)
			return 0;

		//최대로 20개까지 인식하도록....
		if (air_count > 20)
			air_count = 20;

		//char msg[1024]; sprintf_s(msg, "Detected all %d", air_count);
		//string logmessage = msg;
		//Log(logmessage);
		RECT* tmp = (RECT*)detectResult;
		for (int i = 0; i < air_count; i++)
		{
			tmp[i].left = air_rects[i].left;
			tmp[i].top = air_rects[i].top;
			tmp[i].right = air_rects[i].right;
			tmp[i].bottom = air_rects[i].bottom;
		}
//		memcpy((unsigned char*)detectResult, (unsigned char*)air_rects.data(), air_count*sizeof(RECT));
		return air_count;
	}
}