# aeronSW_visualizer
- 라이다 + 객체인식 데이터 시각화 (OpenGL)
- 데모 및 디버깅 용도
- ubuntu 20.04, cmake 3.26

## Requirements
glut, glfw, glew, glm 라이브러리
```
$ sudo apt install freeglut3-dev libglu1-mesa-dev mesa-common-dev
$ sudo apt install mesa-utils
$ sudo apt install libglew-dev
$ sudo apt isntall libglfw3-dev libglfw3
$ sudo apt install libglm-dev
```

## Project Tree
```
aeronSW_visualizer
├── data
│   └── uam_data
│       ├── detectinfo_yy_MM_dd_HH_mm_ss.udd
│       ...
│       ├── lidar_0_yy_MM_dd_HH_mm_ss.uld
│       ...
└── visualizer
    ├── CMakeLists.txt
    └── src
        ├── binary_utils.cpp
        ├── binary_utils.h
        ├── camera.cpp
        ├── camera.h
        ├── configs.h
        ├── main.cpp
        ├── space.cpp
        └── space.h
```

## Build
```
$ cd visualizer
$ mkdir build && cd build
$ cmake ..
$ make
```

## Run
```
$ ./aeronSW_visualizer [start_time] [end_time]
```
- time format: yy-MM-dd-HH-mm-ss

## Functions
1. 프레임 이동</p>
    1.1 Timeline 방식
    - 라이다 + 객체인식 통합 데이터 시간순 벡터
    - 키보드 o: Timeline 방식으로 전환 (mode 전환시 시간 동기화)
    - 키보드 u: 이전 프레임
    - 키보드 i: 다음 프레임
    - 처음 실행시 Timeline 방식으로 작동

    1.2 Binary Search 방식
    - 객체인식 데이터 시간에 근접한 라이다 데이터 시간 매칭
    - 키보드 l: Binary Search 방식으로 전환 (mode 전환시 시간 동기화)
    - 키보드 j: 이전 프레임
    - 키보드 k: 다음 프레임

2. 데이터 정보
    - 스페이스바: 해당 프레임 데이터 정보 출력

3. 3D 화면
    - 마우스 휠: 확대 축소
    - 마우스 드래그: 화면 회전
    - 키보드 w, a, s, d: 화면 이동

4. 비디오
    - 키보드 p: 정지 / 시작
    - 키보드 page up / down: 속도 증가 / 감소