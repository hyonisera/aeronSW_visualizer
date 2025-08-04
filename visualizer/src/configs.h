#pragma once

//INIT
#define INIT_CAM_RADIUS     5.0f   //초기 카메라 위치 : 중심으로부터 거리
#define INIT_CAM_VANGLE     10.0f   //초기 카메라 위치 : 수직각도 // degree
#define INIT_CAM_HANGLE     50.0f   //초기 카메라 위치 : 수평각도 // degree

//window
#define WINDOW_WIDTH    1280        //창 너비
#define WINDOW_HEIGHT   720         //창 높이

// Constants
#define ORBIT_SPEED         0.01f   // 궤도 회전 속도
#define TRANSLATION_SPEED   0.5f    // 이동 속도
#define ZOOM_SPEED          1.0f    // 줌 속도
#define LINE_THICKNESS      2.0f    // 선 두께
#define CAMERA_SPEED        0.1f

// Scene set
#define GRID_NUM        16
#define GRID_Z_OFFSET   -0.01f
#define GRID_COEFFI     2

// video configure
#define VIDEO_SPEED     1                       //배속
#define VIDEO_SPEED_CONTROL_RESOLUTION  0.05f   //조절배속
#define VIDEO_SPEED_COEFFI_MAX  30.0f            //최대계수
#define VIDEO_SPEED_COEFFI_MIN  0.005f            //최소계수

// panorama window
#define PANORAMA_WINDOW_WIDTH   2048
#define PANORAMA_WINDOW_HEIGHT  512
#define ZOOM                    2

// dataset
#define COCODATASET 1


#include <chrono>
#include <unordered_map>
#include <string>
#include <iostream>

class MyTimer{
public:
    MyTimer(){
        start_time = std::chrono::high_resolution_clock::now();
        end_time = std::chrono::high_resolution_clock::now();
    }

    void start(){
        start_time = std::chrono::high_resolution_clock::now();
    }

    void end(){
        end_time = std::chrono::high_resolution_clock::now();
    }

    double elapsed_ms(){
        return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();    // 밀리초 값을 정수로 반환
    }

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;     // high_resolution_clock 기준의 한 시점
    std::chrono::time_point<std::chrono::high_resolution_clock> end_time;

};


#if COCODATASET
static const std::unordered_map<int, std::string> objId_to_label = {
    {0, "person"},          {1, "bicycle"},     {2, "car"},            {3, "motorcycle"},     {4, "airplane"},
    {5, "bus"},             {6, "train"},       {7, "truck"},          {8, "boat"},           {9, "traffic light"},
    {10, "fire hydrant"},   {11, "stop sign"},  {12, "parking meter"}, {13, "bench"},         {14, "bird"},
    {15, "cat"},            {16, "dog"},        {17, "horse"},         {18, "sheep"},         {19, "cow"},
    {20, "elephant"},       {21, "bear"},       {22, "zebra"},         {23, "giraffe"},       {24, "backpack"},
    {25, "umbrella"},       {26, "handbag"},    {27, "tie"},           {28, "suitcase"},      {29, "frisbee"},
    {30, "skis"},           {31, "snowboard"},  {32, "sports ball"},   {33, "kite"},          {34, "baseball bat"},
    {35, "baseball glove"}, {36, "skateboard"}, {37, "surfboard"},     {38, "tennis racket"}, {39, "bottle"},
    {40, "wine glass"},     {41, "cup"},        {42, "fork"},          {43, "knife"},         {44, "spoon"},
    {45, "bowl"},           {46, "banana"},     {47, "apple"},         {48, "sandwich"},      {49, "orange"},
    {50, "broccoli"},       {51, "carrot"},     {52, "hot dog"},       {53, "pizza"},         {54, "donut"},
    {55, "cake"},           {56, "chair"},      {57, "couch"},         {58, "potted plant"},  {59, "bed"},
    {60, "dining table"},   {61, "toilet"},     {62, "tv"},            {63, "laptop"},        {64, "mouse"},
    {65, "remote"},         {66, "keyboard"},   {67, "cell phone"},    {68, "microwave"},     {69, "oven"},
    {70, "toaster"},        {71, "sink"},       {72, "refrigerator"},  {73, "book"},          {74, "clock"},
    {75, "vase"},           {76, "scissors"},   {77, "teddy bear"},    {78, "hair drier"},    {79, "toothbrush"}
};
#endif