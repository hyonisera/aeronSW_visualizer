#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <thread>
#include <mutex>
#include <atomic>
#include <sstream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>


#define LIDAR_ID 0
#define MAX_TIME_DIFF   100      // 100ms

// 저장된 라이다 구조체
typedef struct lidar_data_pack{
    float x, y, z;
    uint32_t reflectivity;
    int cluster_id;
    //TODO : lidar data - signal,,,,
}LidarData;

typedef struct lidar_binary_data{
    uint64_t time; // 시간
    uint32_t num; // point 수
    std::vector<LidarData> lidar_data;
}LidarBinary;

// 탐지된 객체 구조체
typedef struct obj_data_pack{
    int obj_id;
    float nearest_x, nearest_y, nearest_z;
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    float distance;
    float size;
}ObjData;

typedef struct obj_binary_data{
    uint64_t time; // 시간
    uint32_t num; // dectect_obj 수
    std::vector<ObjData> obj_data; // 1차원 벡터로 ObjData 저장
}ObjBinary;


enum class DataType { LIDAR, OBJECT };

struct UnifiedData {
    DataType type;
    size_t index;
};


class BinaryUtils {
public:
    // LidarBinary를 바이너리 파일에서 읽기
    static bool load_lidar_binary(const std::string& filename, std::vector<LidarBinary>& out_data_list);

    // obj Binary 파일
    static bool load_obj_binary(const std::string& filename, std::vector<ObjBinary>& out_data_list);

    // 이진탐색 함수
    int findClosestLidarFrame(uint64_t obj_time, const std::vector<LidarBinary>& lidar_list);
};
