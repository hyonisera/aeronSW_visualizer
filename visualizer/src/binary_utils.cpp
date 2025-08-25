
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
    

    while(ifs && ifs.peek() != EOF) { 
        ObjBinary data{};
        ++total_count;

        
        ifs.read(reinterpret_cast<char*>(&data.time), sizeof(data.time));
        if (!ifs) {
            std::cerr << "[ERROR] Failed to read time at idx " << (total_count - 1) << "\n";
            break;
        }

        ifs.read(reinterpret_cast<char*>(&data.num), sizeof(data.num));
        if (!ifs) {
            std::cerr << "[ERROR] Failed to read num at idx " << (total_count - 1) << "\n";
            break;
        }

        // 헤더 요약 출력
        std::cout << "\n[HEADER] idx=" << (total_count - 1)
                  << " time=" << data.time
                  << " num=" << static_cast<unsigned>(data.num) << "\n";


        // std::cout << "======>> data.num : " << data.num << std::endl;
        // printf("time / num : %08x / %08x / %08x\n", *((int*)&(data.time)), *((int*)&(data.time)+1), data.num);
        if(data.num == 0 || data.time == 0) {
            std::cout << "[SKIPPED] Obj idx: "<< total_count - 1 << ", time: " << data.time << ", num: " << static_cast<unsigned>(data.num) << "\n";
            continue;
        }

        if(data.time < 1600000000000ULL || data.time > 1900000000000ULL) {
            std::cerr << "[SKIPPED] Unrealistic obj time at idx: " << total_count - 1 << ", time: " << data.time << "\n";
            continue;
        }

        // 필수: 비정상 크기 방어 (예: 너무 큰 값이나 음수 cast된 경우)
        if(data.num > 100000) {
            std::cerr << "[ERROR] Unreasonably large object count at idx: " << total_count - 1 << ", time: " << data.time << ", num: " << data.num << std::endl;
            printf("time / num : %08x / %08x / %08x\n", *((int*)&(data.time)), *((int*)&(data.time)+1), data.num);
            // break;
            // std::cerr << "[WARN] Unusually large num=" << static_cast<unsigned>(data.num)
            //           << " at idx " << (total_count - 1)
            //           << " (writer/reader 포맷 불일치 가능성 점검)\n";
        }

        // resize로 메모리 할당해야 실행됨
        data.obj_data.resize(data.num); // resize하는 이유 : 동적인 벡터크기를 지정하여 안전하게 접근/저장하기 위함.

        const std::streamsize obj_bytes =
            static_cast<std::streamsize>(data.num) * static_cast<std::streamsize>(sizeof(ObjData));
        ifs.read(reinterpret_cast<char*>(data.obj_data.data()), obj_bytes);    // data() 데이터 처음부터 읽기

        if (!ifs) {
            std::cerr << "[ERROR] Incomplete ObjData block at idx " << (total_count - 1)
                      << " (wanted " << obj_bytes << " bytes)\n";
            break;
        }

        for (int i = 0; i < static_cast<int>(data.num); ++i) {
            const ObjData& o = data.obj_data[i];
            std::cout << "  [ObjData " << i << "] id=" << o.obj_id
                      << " near=(" << o.nearest_x << "," << o.nearest_y << "," << o.nearest_z << ")"
                      << " min=("  << o.min_x << "," << o.min_y << "," << o.min_z << ")"
                      << " max=("  << o.max_x << "," << o.max_y << "," << o.max_z << ")"
                      << " dist="  << o.distance
                      << " size="  << o.size << "\n";
        }


        // uint32_t outer_size = 1;
        // data.box_converted_regions.resize(outer_size);

        // for(uint32_t i = 0; i < outer_size; ++i) {
        //     uint32_t inner_size = 4;
        //     data.box_converted_regions[i].resize(inner_size);
        //     ifs.read(reinterpret_cast<char*>(data.box_converted_regions[i].data()), inner_size * sizeof(glm::vec3));
        // }

        // uint32_t regions_count = 1;     // 객체 하나
        
        // ifs.read(reinterpret_cast<char*>(&regions_count), sizeof(regions_count));
        // if(!ifs) {
        //     std::cerr << "[ERROR] Failed to read regions_count at idx: " << (total_count - 1) << std::endl;
        //     break;
        // }
        // if(regions_count > 100000) {
        //     std::cerr << "[ERROR] Unreasonably large regions_count at idx: " << (total_count - 1) << ", regions_count: " << regions_count << std::endl;
        //     break;
        // }

        // ifs.read(reinterpret_cast<char*>(&data.outer_size), sizeof(data.outer_size));
        // ifs.read(reinterpret_cast<char*>(&data.inner_size), sizeof(data.inner_size));

        // data.region_block.regions.clear();
        // data.region_block.regions.resize(data.outer_size);

        // const size_t floats_per_region = static_cast<size_t>(data.inner_size) * 3;
        // std::vector<float> buf;
        // buf.resize(floats_per_region);

        // for(uint32_t r = 0; r < data.outer_size; ++r) {
        //     const auto bytes = static_cast<std::streamsize>(floats_per_region * sizeof(float));
        //     ifs.read(reinterpret_cast<char*>(buf.data()), bytes);

        //     auto& region = data.region_block.regions[r];
        //     region.resize(data.inner_size);

        //     for(uint32_t p = 0; p < data.inner_size; ++p) {
        //         float x = buf[p * 3 + 0];
        //         float y = buf[p * 3 + 1];
        //         float z = buf[p * 3 + 2];

        //         region[p] = Float3{x, y, z};
        //     }
        //     // uint32_t points_count = 4;
        //     // ifs.read(reinterpret_cast<char*>(&points_count), sizeof(points_count));
        //     // if(!ifs) {
        //     //     std::cerr << "[ERROR] Failed to read points_count (region " << r << ") at idx: " << (total_count - 1) << std::endl;
        //     //     ok = false; break;
        //     // }
        //     // if(points_count > 4096) {
        //     //     std::cerr << "[ERROR] Unreasonably large points_count (" << points_count << ") at region " << r << ", idx: " << (total_count - 1) << std::endl;
        //     //     ok = false; break;
        //     // }

        //     // data.region_block.regions[r].resize(points_count);
        //     // for(uint32_t p = 0; p < points_count; ++p) {
        //     //     float xyz[3];
        //     //     ifs.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
        //     //     if(!ifs) {
        //     //         std::cerr << "[ERROR] Incomplete point read (region " << r << ", point " << p << ") at idx: " << (total_count - 1) << std::endl;
        //     //         break;
        //     //     }
        //     //     data.region_block.regions[r][p] = glm::vec3(xyz[0], xyz[1], xyz[2]);
        //     // }
        //     // if(!ifs) break;

        //     // float xyz[12];
        //     // ifs.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
        //     // if(!ifs) {
        //     //     std::cerr << "[ERROR] Incomplete FOV points (region " << r << ") at idx: " << (total_count - 1) << std::endl;
        //     //     break;
        //     // }
        //     // data.box_converted_regions[r].resize(4);
        //     // for(int k = 0; k < 4; ++k) {
        //     //     data.box_converted_regions[r][k] = glm::vec3(xyz[3*k + 0], xyz[3*k + 1], xyz[3*k + 2]);
        //     // }
        // }

        uint8_t outer_size;
        ifs.read(reinterpret_cast<char*>(&outer_size), sizeof(outer_size));

        if (!ifs) {
            std::cerr << "[ERROR] Failed to read outer_size at idx " << (total_count - 1) << "\n";
            break;
        }
        std::cout << "BCR: outer_size=" << static_cast<unsigned>(outer_size) << "\n";

        std::cout << "outer_size: " << outer_size << "byte: " << sizeof(outer_size)<<std::endl;


                    //메모리상의 값 출력 엔디언 확인
        unsigned char* p = reinterpret_cast<unsigned char*>(&outer_size);
        for(size_t i=0; i<sizeof(outer_size); i++){
            std::cout << "0x" <<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<int>(p[i]) << " ";
        }
        std::cout << std::dec<<std::endl;


        // std::vector<std::vector<glm::vec3>> box_converted_regions(outer_size);
        data.bcr.clear();
        data.bcr.resize(outer_size); // 2차원 벡터로 BCR 저장

        int printed_points = 0;
        for (uint32_t i = 0; i < outer_size; ++i) {
            uint8_t inner_size = 0;
            ifs.read(reinterpret_cast<char*>(&inner_size), sizeof(inner_size));
            if (!ifs) {
                std::cerr << "[ERROR] Failed to read inner_size (region " << i
                          << ") at idx " << (total_count - 1) << "\n";
                break;
            }

            std::cout << "  region[" << i << "], inner_size: " << static_cast<unsigned>(inner_size) << "\n";
            // if (inner_size == 0 || inner_size > 32) {
            //     std::cerr << "  [WARN] suspicious inner_size=" << static_cast<unsigned>(inner_size)
            //               << " (보통 4 기대). 파일 작성 쪽 포맷 확인 권장.\n";
            // }

            data.bcr[i].resize(inner_size);

            for (uint32_t j = 0; j < inner_size; ++j) {
                // glm::vec3 point;
                // ifs.read(reinterpret_cast<char*>(&point), sizeof(glm::vec3));

                // data.bcr[i][j] = point;

                // std::cout << "    point[" << j << "] = ("
                //         << point.x << ", "
                //         << point.y << ", "
                //         << point.z << ")\n";

                float xyz[3];
                ifs.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
                if (!ifs) {
                    std::cerr << "[ERROR] Incomplete vec3 at region " << i << ", point " << j
                              << " (wanted " << sizeof(xyz) << " bytes)\n";
                    break;
                }
                data.bcr[i][j] = glm::vec3(xyz[0], xyz[1], xyz[2]);

                std::cout << "    p[" << j << "] = ("
                          << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << ")\n";
            }
            if (!ifs) break;
        }
        if (!ifs) break;


        std::cout << "sizeof(ObjData)=" << sizeof(ObjData) << " bytes\n";

        

       



        // if(ifs.gcount() != static_cast<std::streamsize>(data.num * sizeof(ObjData))) {
        //     std::cerr << "[ERROR] Incomplete obj data read" << std::endl;
        //     break;
        // }

        out_data_list.push_back(std::move(data));
        // std::cout << "[LOADED] idx: " << total_count - 1 << ", time: " << data.time << ", num: " << data.num << std::endl;
        std::cout << "[LOADED] idx=" << (total_count - 1)
                  << " time=" << out_data_list.back().time
                  << " num="  << static_cast<unsigned>(out_data_list.back().num)
                  << " regions=" << out_data_list.back().bcr.size() << "\n";

        // std::cout << "outer_size: " << data.outer_size << ", inner_size: " << data.inner_size << "\n";
        // for(size_t r = 0; r < data.region_block.regions.size(); ++r) {
        //     std::cout << " Region " << r << " (" << data.region_block.regions[r].size() << " points):\n";
        //     for(size_t p = 0; p < data.region_block.regions[r].size(); ++p) {
        //         const Float3& pt = data.region_block.regions[r][p];
        //         std::cout << "      Point " << p << ": (" << pt.x << ", " << pt.y << ", " << pt.z << ")\n";
        //     }
        // }

        
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
