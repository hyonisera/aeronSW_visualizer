
#include "binary_utils.h"


// // Unix time (ms) → yy_mm_dd_hh_mm_ss_ms 형태 문자열로 변환 (KST 기준)
// std::string unixTimeToFormattedString(int64_t unix_ms) {
//     // 초 단위로 분리
//     time_t unix_sec = unix_ms / 1000;
//     int ms = unix_ms % 1000;

//     // UTC 기준 tm 구조체 가져오기
//     std::tm *utc_tm = std::gmtime(&unix_sec);

//     // KST(+9시간) 적용
//     utc_tm->tm_hour += 9;
//     mktime(utc_tm); // 시간 오버플로우 보정 (예: 24시 → 다음날 0시)

//     // 스트링 스트림으로 포맷팅
//     std::ostringstream oss;
//     oss << std::setfill('0') 
//         << std::setw(2) << (utc_tm->tm_year + 1900) % 100 << "_"
//         << std::setw(2) << (utc_tm->tm_mon + 1) << "_"
//         << std::setw(2) << utc_tm->tm_mday << "_"
//         << std::setw(2) << utc_tm->tm_hour << "_"
//         << std::setw(2) << utc_tm->tm_min << "_"
//         << std::setw(2) << utc_tm->tm_sec << "_"
//         << std::setw(3) << ms;

//     return oss.str();
// }

// LidarBinary를 바이너리 파일에서 읽기
bool BinaryUtils::load_lidar_binary(const std::string& filename, std::vector<LidarBinary>& out_data_list) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        std::cerr << "Failed to open Lidar file: " << filename << std::endl;   
        return false;
    }

    while(ifs.peek() != EOF) {      // 모든 파일 읽기
        LidarBinary data;
        
        ifs.read(reinterpret_cast<char*>(&data.time), sizeof(data.time));
        ifs.read(reinterpret_cast<char*>(&data.num), sizeof(data.num));

        if(!data.num){
            std::cout << "Lidar data is empty." << std::endl;
            ifs.close();
            return true;
        }

        // resize로 메모리 할당해야 실행됨
        // data.lidar_data.resize(1); // ex) lidar_data[LIDAR_ID]={{,,,,,},{,,,,,}} 
        data.lidar_data.resize(data.num); // resize하는 이유 : 동적인 벡터크기를 지정하여 안전하게 접근/저장하기 위함.

        ifs.read(reinterpret_cast<char*>(data.lidar_data.data()), data.num * sizeof(LidarData));    // data() 데이터 처음부터 읽기

        std::cout << "[LOADED] time: " << data.time << ", num: " << data.num << std::endl;
        // std::cout << "Formatted time: " << unixTimeToFormattedString(data.time) << std::endl;

        out_data_list.push_back(data);
    }

    ifs.close();
    return true;
}


std::streamsize getFileSize(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary | std::ios::ate);
    if (!ifs) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return -1;
    }

    // 파일 끝으로 이동 후 위치 반환
    return ifs.tellg(); // 현재 위치 = 파일 끝 = 전체 크기 (바이트 단위)
}

// ObjBinary를 바이너리 파일에서 읽기
bool BinaryUtils::load_obj_binary(const std::string& filename, std::vector<ObjBinary>& out_data_list) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        std::cerr << "Failed to open ObjectInfo file: " << filename << std::endl;   
        return false;
    }

    // std::streamsize size = getFileSize(filename);
    // std::cout << filename << " Opened!" << std::endl;
    // std::cout << "file" << filename << "size : " << size <<std::endl;

    while(ifs.peek() != EOF) { 
        ObjBinary data;
        ifs.read(reinterpret_cast<char*>(&data.time), sizeof(data.time));
        ifs.read(reinterpret_cast<char*>(&data.num), sizeof(data.num));


        
        // std::cout << "======>> data.num : " << data.num << std::endl;
        // printf("time / num : %08x / %08x / %08x\n", *((int*)&(data.time)), *((int*)&(data.time)+1), data.num);
        if(data.num == 0) {
            std::cout << "[EMPTY] time: " << data.time << " -> ObjectInfo data is empty." << std::endl;    
            continue;
        }

        // 필수: 비정상 크기 방어 (예: 너무 큰 값이나 음수 cast된 경우)
        if(data.num > 100000) {
            std::cerr << "[ERROR] Unreasonably large object count: " << data.num << std::endl;
            printf("time / num : %08x / %08x / %08x\n", *((int*)&(data.time)), *((int*)&(data.time)+1), data.num);
            break;
        }

        // resize로 메모리 할당해야 실행됨
        // data.obj_data.resize(1); // ex) lidar_data[LIDAR_ID]={{,,,,,},{,,,,,}} 
        data.obj_data.resize(data.num); // resize하는 이유 : 동적인 벡터크기를 지정하여 안전하게 접근/저장하기 위함.

        ifs.read(reinterpret_cast<char*>(data.obj_data.data()), data.num * sizeof(ObjData));    // data() 데이터 처음부터 읽기
        
        // std::cout << "read data size : " << data.num * sizeof(ObjData) << std::endl;
        // std::cout << "============== test ==============" << std::endl;

        // for(auto obj : data.obj_data){
        //     // std::cout << "obj.nearest_x : " << obj.nearest_x << std::endl;
        //     // std::cout << "obj.nearest_y : " << obj.nearest_y << std::endl;
        //     // std::cout << "obj.nearest_z : " << obj.nearest_z << std::endl;
        //     std::cout << "\nobj.id: " << obj.obj_id << std::endl; 
        //     std::cout << "obj.min_x : " << obj.min_x << std::endl;
        //     std::cout << "obj.min_y : " << obj.min_y << std::endl;
        //     std::cout << "obj.min_z : " << obj.min_z << std::endl;
        //     std::cout << "obj.max_x : " << obj.max_x << std::endl;
        //     std::cout << "obj.max_y : " << obj.max_y << std::endl;
        //     std::cout << "obj.max_z : " << obj.max_z << std::endl;
            
        // }

        // std::cout << "==================================" << std::endl;

        std::cout << "[LOADED] time: " << data.time << ", num: " << data.num << std::endl;
        // std::cout << "Formatted time: " << unixTimeToFormattedString(data.time) << std::endl;

        out_data_list.push_back(data);
    
    }
    ifs.close();
    return true;
}

// void BinaryUtils::mergeDataByTime(const std::vector<LidarBinary>& lidar_list, const std::vector<ObjBinary>& obj_list) {
//     size_t i=0, j=0;
//     while(i < lidar_list.size() && j < obj_list.size()) {
//         if(lidar_list[i].time <= obj_list[j].time) {
            
//         }
//     }
// }


// lidar_time과 가장 가까운 ObjBinary.time을 가진 인덱스를 이진탐색으로 찾기
int BinaryUtils::findClosestObjectFrame(uint64_t lidar_time, const std::vector<ObjBinary>& obj_list) {
    // left, right 이진탐색 범위(처음엔 전체 벡터)
    int left = 0;
    int right = obj_list.size() - 1;
    int best_idx = -1;
    uint64_t min_diff = std::numeric_limits<uint64_t>::max();   // 가장 작은 시간 차이를 저장 (초기엔 최대값)

    while(left <= right) {
        int mid = (left + right) / 2;       // 리스트를 절반씩 나누어 탐색
        uint64_t obj_time = obj_list[mid].time;
        uint64_t diff = (lidar_time > obj_time) ? (lidar_time - obj_time) : (obj_time - lidar_time);

        // 지금까지 본 값 중 가장 시간 차이가 작은 인덱스를 갱신
        if(diff < min_diff) {
            min_diff = diff;
            best_idx = mid;
        }
        // 오름차순 정렬을 가정, 현재 obj_time이 lidar_time보다 작으면 오른쪽 탐색
        if(obj_time < lidar_time) {
            left = mid + 1;     // 오른쪽으로 이동
        } else {
            right = mid - 1;    // 왼쪽으로 이동
        }
    }
    return best_idx;
}



// void LidarBinaryUtils::clearField(){
//     field.clear();
//     std::vector<glm::vec3>().swap(field);

//     color_field.clear();
//     std::vector<std::pair<glm::vec3, uint32_t>>().swap(color_field);
// }


// void LidarBinaryUtils::drawPoints(){
//     float color_sense;
    
//     for(auto &point: color_field){
//         color_sense = point.second / 26.54f;
//         glm::vec3 color(0.0f, 0.973f - color_sense, 0.364f + color_sense);
//         space.addPoint(point.first, color);
//     }
// }