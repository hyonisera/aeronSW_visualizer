
#include "binary_utils.h"


// LidarBinary를 바이너리 파일에서 읽기
bool BinaryUtils::load_lidar_binary(const std::string& filename, std::vector<LidarBinary>& out_data_list) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        std::cerr << "Failed to open Lidar file: " << filename << std::endl;   
        return false;
    }

    int total_count = 0;

    while(ifs.peek() != EOF) {      // 모든 파일 읽기
        LidarBinary data;
        ++total_count;
        
        ifs.read(reinterpret_cast<char*>(&data.time), sizeof(data.time));
        ifs.read(reinterpret_cast<char*>(&data.num), sizeof(data.num));

        if(data.num == 0 || data.time == 0) {
            std::cout << "[SKIPPED] Lidar idx: "<< total_count - 1 << ", time: " << data.time << ", num: " << data.num << std::endl;
            continue;
        }

        if(data.time < 1600000000000ULL || data.time > 1900000000000ULL) {
            std::cerr << "[SKIPPED] Unrealistic lidar time at idx: " << total_count - 1 << ", time: " << data.time << std::endl;
            continue;
        }

        // resize로 메모리 할당해야 실행됨
        data.lidar_data.resize(data.num); // resize하는 이유 : 동적인 벡터크기를 지정하여 안전하게 접근/저장하기 위함.

        ifs.read(reinterpret_cast<char*>(data.lidar_data.data()), data.num * sizeof(LidarData));    // data() 데이터 처음부터 읽기

        if(ifs.gcount() != static_cast<std::streamsize>(data.num * sizeof(LidarData))) {
            std::cerr << "[ERROR] Incomplete lidar data read" << std::endl;
            break;
        }

        out_data_list.push_back(data);
        std::cout << "[LOADED] idx: " << total_count - 1 << ", time: " << data.time << ", num: " << data.num << std::endl;
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

    int total_count = 0;

    while(ifs.peek() != EOF) { 
        ObjBinary data;
        ++total_count;

        ifs.read(reinterpret_cast<char*>(&data.time), sizeof(data.time));
        // if(ifs.gcount() != sizeof(data.time)) {
        //     std::cerr << "[ERROR] Failed to read time at idx: " << total_count - 1 << std::endl;
        //     break;
        // }
        ifs.read(reinterpret_cast<char*>(&data.num), sizeof(data.num));
        // if(ifs.gcount() != sizeof(data.num)) {
        //     std::cerr << "[ERROR] Failed to read num at idx: " << total_count - 1 << std::endl;
        //     break;
        // }


        // std::cout << "======>> data.num : " << data.num << std::endl;
        // printf("time / num : %08x / %08x / %08x\n", *((int*)&(data.time)), *((int*)&(data.time)+1), data.num);
        if(data.num == 0 || data.time == 0) {
            std::cout << "[SKIPPED] Obj idx: "<< total_count - 1 << ", time: " << data.time << ", num: " << data.num << std::endl;
            continue;
        }

        if(data.time < 1600000000000ULL || data.time > 1900000000000ULL) {
            std::cerr << "[SKIPPED] Unrealistic obj time at idx: " << total_count - 1 << ", time: " << data.time << std::endl;
            continue;
        }

        // 필수: 비정상 크기 방어 (예: 너무 큰 값이나 음수 cast된 경우)
        if(data.num > 100000) {
            std::cerr << "[ERROR] Unreasonably large object count at idx: " << total_count - 1 << ", num: " << data.num << std::endl;
            printf("time / num : %08x / %08x / %08x\n", *((int*)&(data.time)), *((int*)&(data.time)+1), data.num);
            break;
        }

        // resize로 메모리 할당해야 실행됨
        data.obj_data.resize(data.num); // resize하는 이유 : 동적인 벡터크기를 지정하여 안전하게 접근/저장하기 위함.

        ifs.read(reinterpret_cast<char*>(data.obj_data.data()), data.num * sizeof(ObjData));    // data() 데이터 처음부터 읽기

        if(ifs.gcount() != static_cast<std::streamsize>(data.num * sizeof(ObjData))) {
            std::cerr << "[ERROR] Incomplete obj data read" << std::endl;
            break;
        }

        out_data_list.push_back(data);
        std::cout << "[LOADED] idx: " << total_count - 1 << ", time: " << data.time << ", num: " << data.num << std::endl;
    }

    ifs.close();
    return true;
}


// 객체 시간에 가장 가까운 라이다 프레임 인덱스를 찾는 이진탐색 함수
int BinaryUtils::findClosestLidarFrame(uint64_t obj_time, const std::vector<LidarBinary>& lidar_list) {
    int left = 0;
    int right = lidar_list.size() - 1;
    int best_idx = -1;
    uint64_t min_diff = std::numeric_limits<uint64_t>::max();

    while(left <= right) {
        int mid = (left + right) / 2;
        uint64_t lidar_time = lidar_list[mid].time;
        uint64_t diff = (obj_time > lidar_time) ? (obj_time - lidar_time) : (lidar_time - obj_time);

        if(diff < min_diff) {
            min_diff = diff;
            best_idx = mid;
        }
        if(lidar_time < obj_time) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }
    // 최대 허용 시간 차(100ms) 이내일 때만 유효한 인덱스 반환
    if(min_diff <= MAX_TIME_DIFF) {
        return best_idx;
    } else {
        return -1;
    }
}
