
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <GL/glut.h>
#include <iostream>
#include <string>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <chrono>

#include "camera.h"
#include "space.h"
#include "configs.h"
#include "binary_utils.h"

// #include "opencv2/opencv.hpp"

#define PRINT_VAL       0
#define TIMELINE        1
#define BINARY_SEARCH   2
#define INTEGRATION     1
int mode = TIMELINE;
#define FRAME_RATE      5      // 초당 프레임 수(Hz)

// Globals
Camera camera;
Space space;

bool leftMousePressed = false;
double lastMouseX = 0.0, lastMouseY = 0.0;
float horizontalAngle = glm::radians(INIT_CAM_HANGLE * 360.0f / 100.0f), verticalAngle = glm::radians(INIT_CAM_VANGLE * 360.0f / 100.0f); // 카메라의 회전 각도
float orbitRadius = INIT_CAM_RADIUS;                           // 궤도 반지름

// screen control
// 0 : play // 1 : pause // 2 : prev frame // 3 : next frame
int video_control = 0;
float video_speed_coeffi = 1.0f;
float prev_speed_coeffi = video_speed_coeffi;



// camera position offset
glm::vec3 cameraPosition(0.0f, 0.0f, 0.0f);
glm::vec3 cameraTarget(0.0f, 0.0f, 0.0f);
glm::vec3 up(0.0f, 1.0f, 0.0f);


float x = orbitRadius * glm::cos(verticalAngle) * glm::cos(horizontalAngle) + cameraPosition.x;
float y = orbitRadius * glm::cos(verticalAngle) * glm::sin(horizontalAngle) + cameraPosition.y;
float z = orbitRadius * glm::sin(verticalAngle) + cameraPosition.z;

int obj_frame=0;

int print_current = 1;
int print_current_lidar = 1;
int print_current_obj = 1;

// argument callback function
void printHowToUse();
// void printUsage();
// void printErr(int index);
std::string getFileExtension(const std::string& fileName);
std::string removeFileExtension(const std::string& fileName);

// Function Prototypes
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
void setupViewport(const GLuint width, const GLuint height);


int main(int argc, char* argv[]) {

    /**** Option argument parsing (with no external lib) ****/

    // if(argc < 0){
    //     printHowToUse();
    //     printUsage();
    // }

    //======================== argument parsing ============================
    if(argc < 2) {
        printHowToUse();
        return -1;
    }

    std::string uam_data_path = "../../data/uam_data";

    // 디렉토리 존재 확인
    if(!std::filesystem::exists(uam_data_path)) {
        std::cerr << "UAM data directory is not found: " << uam_data_path << std::endl;
        return 1;
    }
    
    std::string start_time = argv[1];
    std::string end_time = argv[2];

    std::vector<std::string> get_lidar_filename;      // 불러온 파일들 벡터
    std::vector<std::string> get_detectinfo_filename;

    for(const auto& entry : std::filesystem::directory_iterator(uam_data_path)) {
        std::string filename = entry.path().filename().string();  // 파일 이름만 추출

        // std::cout << "Processing file: " << filename << std::endl;
        
        if(getFileExtension(filename) == "uld") {
            std::string lidar_start_time = "lidar_0_" + start_time;
            std::string lidar_end_time = "lidar_0_" + end_time;

            std::string _filename = removeFileExtension(filename);

            if(_filename >= lidar_start_time && _filename <= lidar_end_time) {
                get_lidar_filename.push_back(uam_data_path + "/" + filename);
            }
        }
        else if(getFileExtension(filename) == "udd") {
            std::string detectinfo_start_time = "detectinfo_" + start_time;
            std::string detectinfo_end_time = "detectinfo_" + end_time;

            std::string _filename = removeFileExtension(filename);

            if(_filename >= detectinfo_start_time && _filename <= detectinfo_end_time) {
                get_detectinfo_filename.push_back(uam_data_path + "/" + filename);
            }
        }
        else {
            // printErr(3);
            std::cout << "Error: file extension is not .uld or .udd" << std::endl;
            return 0;
        }
    }

    std::sort(get_lidar_filename.begin(), get_lidar_filename.end());    // 불러온 파일들 시간순 정렬
    std::sort(get_detectinfo_filename.begin(), get_detectinfo_filename.end());



    //======================== read binary file ============================

    // read binary files
    BinaryUtils utils;
    std::vector<LidarBinary> lidar_loaded_bin;    // 여러 프레임
    std::vector<ObjBinary> obj_loaded_bin;

    // auto load_start = std::chrono::high_resolution_clock::now();
    for(const std::string& name : get_lidar_filename) {
        std::cout << "Loaded filename: " << name << std::endl;

#if PRINT_VAL
        lidar_loaded_bin.clear();
#endif

        if(utils.load_lidar_binary(name, lidar_loaded_bin)) {
            // auto load_end = std::chrono::high_resolution_clock::now();
            // auto load_duration = std::chrono::duration_cast<std::chrono::microseconds>(load_end - load_start).count();
            // std::cout << "Load time: " << load_duration << " us" << std::endl;
#if PRINT_VAL
            for(const auto& data : lidar_loaded_bin) {
                
                std::cout << "Loaded binary time: " << data.time << std::endl;
                std::cout << "Loaded num: " << data.num << std::endl;
                int i=0;
                for(const auto& _data : data.lidar_data) {
                    
                    std::cout << i << " x: " << _data.x
                                    << ", y: " << _data.y
                                    << ", z: " << _data.z
                                    << ", reflectivity: " << _data.reflectivity
                                    << ", cluster_id: " << _data.cluster_id
                                    << std::endl;
                    i++;
                }
            }
#endif
            std::cout << "Lidar data loaded" << std::endl;
        }
    }

    for(const std::string& name : get_detectinfo_filename) {
        std::cout << "Loaded filename: " << name << std::endl;

#if PRINT_VAL
        obj_loaded_bin.clear();
#endif

        if(utils.load_obj_binary(name, obj_loaded_bin)) {

#if PRINT_VAL
            for(const auto& data : obj_loaded_bin) {
                
                std::cout << "Loaded binary time: " << data.time << std::endl;
                std::cout << "Loaded num: " << data.num << std::endl;
                int i=0;
                for(const auto& _data : data.obj_data) {
                    
                    std::cout << i << " obj_id: " << _data.obj_id
                                    << ", nearest_x: " << _data.nearest_x
                                    << ", nearest_y: " << _data.nearest_y
                                    << ", nearest_z: " << _data.nearest_z
                                    << ", min_x: " << _data.min_x
                                    << ", min_y: " << _data.min_y
                                    << ", min_z: " << _data.min_z
                                    << ", max_x: " << _data.max_x
                                    << ", max_y: " << _data.max_y
                                    << ", max_z: " << _data.max_z
                                    << ", distance: " << _data.distance
                                    << ", size: " << _data.size
                                    << std::endl;
                    i++;
                }
            }
#endif
            std::cout << "ObjectInfo data loaded" << std::endl;
        }
    }


    // // parser
    // for (int i=0; i<argc; i++){
    //     std::string arg = argv[i];
    //     if(arg == "-h" || arg == "--help"){
    //         printUsage();
    //     }else if((arg == "-d" || arg == "--data") ){
    //         if(1 + i < argc){
    //             i++;
    //             std::string arg2 = argv[i];
    //             if(arg2 == "-p"){ // file type = pcap type : -d -p [pcapfile] [json meta file]

    //                 if(2 + i < argc){
    //                     lidar_data_type = 1;
    //                     lidar_filename = argv[i+1];
    //                     lidar_metadata_filename = argv[i+2];
                        
    //                     if(getFileExtension(lidar_filename) != "pcap" ||    // uld
    //                         getFileExtension(lidar_metadata_filename) != "json"){   // udd
    //                         printErr(3);
    //                         return 0;
    //                     }
                        
    //                     i+=2;
                        
    //                 }else{
    //                     printErr(3); // -d -p error print
    //                 }
                    

    //             }else{
    //                 printErr(2);
    //                 return 0;
    //             }
    //         }else{
    //             printErr(2); // -d error print
    //             return 0;
    //         }
    //     }
    // }



    /**** time util ****/
    int ms_per_frame = 1000 / FRAME_RATE * VIDEO_SPEED;

    /**** Initialize GLFW ****/ 
    glutInit(&argc, argv);  // GLUT 초기화(glutBitmapCharacter()는 GLUT 내부 초기화가 되어 있어야 사용가능)

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Orbiting Camera", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    setupViewport(WINDOW_WIDTH, WINDOW_HEIGHT);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    /**** Set callbacks ****/ 
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);


    /**** Main loop ****/ 

    std::cout << "==================================================" << std::endl;
    std::cout << "Lidar frame num: " << lidar_loaded_bin.size() << std::endl;      // 라이다 프레임 총 개수
    std::cout << "Detection data set num: " << obj_loaded_bin.size() << std::endl;  // 객체 인식 데이터셋 총 개수
    std::cout << "==================================================" << std::endl;


    //========================= 라이다+객체인식 데이터 시간 매핑 ===========================

    // std::map<int, std::vector<ObjBinary>> obj_to_frame;

    // for(const auto& obj : obj_loaded_bin) {
    //     int lidar_idx = utils.findClosestFrame(obj.time, lidar_loaded_bin);
    //     if(lidar_idx == -1) continue;

    //     const auto& lidar_frame = lidar_loaded_bin[lidar_idx];
    // }

    // // 라이다+객체인식 통합 데이터 시간순 화면 출력
    // LidarBinary recent_lidar;
    // ObjBinary recent_obj;

    // if(!lidar_loaded_bin.empty()) {
    //     recent_lidar = lidar_loaded_bin[0];
    // }
    // else {
    //     std::cerr << "[Error] Lidar data is empty!\n";
    //     return -1;
    // }
    // if(!obj_loaded_bin.empty()) {
    //     recent_obj = obj_loaded_bin[0];
    // }
    // else {
    //     std::cerr << "[Error] Object Detection data is empty!\n";
    //     return -1;
    // }

    // //========================= 1. 라이다+객체인식 통합 데이터 시간순 화면 출력 ============================
    // std::vector<UnifiedData> timeline;
    // size_t timeline_idx = 0;
    // const UnifiedData& current_time = timeline[timeline_idx];
    // //=========================  ============================


// #if TIMELINE
//     std::vector<UnifiedData> timeline;
//     size_t lidar_idx = 0;       // 벡터 순회 변수는 size_t 선언하는 것이 좋음
//     size_t obj_idx = 0;
//     MyTimer timer;

//     while(lidar_idx < lidar_loaded_bin.size() || obj_idx < obj_loaded_bin.size()) {
//         if(obj_loaded_bin[obj_idx].time <= lidar_loaded_bin[lidar_idx].time) {
//             timeline.push_back({DataType::OBJECT, obj_idx++});
//         }
//         else {
//             timeline.push_back({DataType::LIDAR, lidar_idx++});
//         }
//     }

//     for(size_t i=0; i<timeline.size(); ++i) {
//         const UnifiedData& entry = timeline[i];
//         if(entry.type == DataType::LIDAR) {
//             std::cout << "[" << i << "] lid idx = " << entry.index << ", time = " << lidar_loaded_bin[entry.index].time << std::endl;
//         }
//         else {
//             std::cout << "[" << i << "] obj idx = " << entry.index << ", time = " << obj_loaded_bin[entry.index].time << std::endl;
//         }
//     }

//     size_t timeline_idx = 0;
// #endif

// #if BINARY_SEARCH
//     size_t _lidar_idx = 0;
//     MyTimer timer;
// #endif

#if INTEGRATION
    std::vector<UnifiedData> timeline;
    size_t lidar_idx = 0;
    size_t obj_idx = 0;
    MyTimer timer;

    while(lidar_idx < lidar_loaded_bin.size() || obj_idx < obj_loaded_bin.size()) {
        if(obj_loaded_bin[obj_idx].time <= lidar_loaded_bin[lidar_idx].time) {
            timeline.push_back({DataType::OBJECT, obj_idx++});
        }
        else {
            timeline.push_back({DataType::LIDAR, lidar_idx++});
        }
    }

    for(size_t i=0; i<timeline.size(); ++i) {
        const UnifiedData& entry = timeline[i];
        if(entry.type == DataType::LIDAR) {
            std::cout << "[" << i << "] lid idx = " << entry.index << ", time = " << lidar_loaded_bin[entry.index].time << std::endl;
        }
        else {
            std::cout << "[" << i << "] obj idx = " << entry.index << ", time = " << obj_loaded_bin[entry.index].time << std::endl;
        }
    }

    size_t timeline_idx = 0;
#endif

    // 한 프레임씩 while문
    while (!glfwWindowShouldClose(window)) {
        // Clear screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /**** draw screen ****/

        // init space
        space.clearLines();
        space.clearBoxes();
        space.drawGrid();
        

        int new_ms_per_frame = ms_per_frame * video_speed_coeffi;
        if(prev_speed_coeffi != video_speed_coeffi) {
            prev_speed_coeffi = video_speed_coeffi;
            std::cout << "video speed changed! : coefficient = " << video_speed_coeffi << std::endl;
        }

#if INTEGRATION
        timer.end();
        if(mode == TIMELINE) {
            if((timer.elapsed_ms() >= new_ms_per_frame) && (video_control == 0)) {
                if(timeline_idx + 1 < timeline.size()) {
                    ++timeline_idx;
                    timer.start();
                }
            }
            else if(video_control == 2) {
                if(timeline_idx > 0) {
                    --timeline_idx;
                    std::cout << "\n[key U] timeline_idx = " << timeline_idx << std::endl;
                }
                video_control = 1;
            }
            else if(video_control == 3) {
                if(timeline_idx + 1 < timeline.size()) {
                    ++timeline_idx;
                    std::cout << "\n[key I] timeline_idx = " << timeline_idx << std::endl;
                }
                video_control = 1;
            }


            const UnifiedData& current = timeline[timeline_idx];
            
            if(current.type == DataType::OBJECT) {
                const ObjBinary& current_obj = obj_loaded_bin[current.index];
                space.clearObjPoints();
                space.objIntoSpace(current_obj);
                if(print_current == 0) {
                    std::string time_str = space.formatUnixTime(current_obj.time);
                    std::cout << "\n[Object] idx = " << current.index << ", time = " << current_obj.time << " (" << time_str << "), num = " << current_obj.num << std::endl;
                
                    std::cout << "obj_id = [ ";
                    for(size_t i = 0; i < current_obj.obj_data.size(); ++i) {
                        std::cout << current_obj.obj_data[i].obj_id;
                        if(i != current_obj.obj_data.size() - 1) std::cout << ", ";
                    }
                    std::cout << " ]" << std::endl;

                    for(size_t i = 0; i < current_obj.obj_data.size(); ++i) {
                        const auto& obj = current_obj.obj_data[i];

                        std::string label = "unknown";
                        auto it = objId_to_label.find(obj.obj_id);
                        if(it != objId_to_label.end()) {
                            label = it->second;
                        }

                        std::cout << "id=[" << obj.obj_id << "] " << label 
                                    << "/ nearest(" << obj.nearest_x << ", " << obj.nearest_y << ", " << obj.nearest_z
                                    << "), min(" << obj.min_x << ", " << obj.min_y << ", " << obj.min_z
                                    << "), max(" << obj.max_x << ", " << obj.max_y << ", " << obj.max_z
                                    << "), distance=" << obj.distance 
                                    << ", size=" << obj.size << std::endl;
                    }
                    print_current = 1;
                }
            }
            else {
                const LidarBinary& current_lidar = lidar_loaded_bin[current.index];
                space.clearLidarPoints();
                space.lidarIntoSpace(current_lidar);
                if(print_current == 0) {
                    std::string time_str = space.formatUnixTime(current_lidar.time);
                    std::cout << "\n[Lidar] idx = " << current.index << ", time = " << current_lidar.time << " (" << time_str << "), num = " << current_lidar.num << std::endl;
                    
                    print_current = 1;
                }
            }
        }
        
        //============================================================================
        else if(mode == BINARY_SEARCH) {
            if((timer.elapsed_ms() >= new_ms_per_frame) && (video_control == 0)) {
                if(lidar_idx + 1 < lidar_loaded_bin.size()) {
                    ++lidar_idx;
                    timer.start();
                }
            }
            else if(video_control == 4) {
                if(lidar_idx > 0) {
                    --lidar_idx;
                    std::cout << "\n[key J] lidar_idx = " << lidar_idx << std::endl;
                }
                video_control = 1;
            }
            else if(video_control == 5) {
                if(lidar_idx + 1 < lidar_loaded_bin.size()) {
                    ++lidar_idx;
                    std::cout << "\n[key K] lidar_idx = " << lidar_idx << std::endl;
                }
                video_control = 1;
            }

            const LidarBinary& current_lidar = lidar_loaded_bin[lidar_idx];
            space.clearLidarPoints();
            space.lidarIntoSpace(current_lidar);

            if(print_current_lidar == 0) {
                std::string time_str = space.formatUnixTime(current_lidar.time);
                std::cout << "\n[Lidar] idx = " << lidar_idx << ", time = " << current_lidar.time << " (" << time_str << "), num = " << current_lidar.num << std::endl;
                
                print_current_lidar = 1;
            }
            
            int obj_idx = utils.findClosestObjectFrame(current_lidar.time, obj_loaded_bin);
            if(obj_idx != -1) {     // 이진탐색 결과가 유효한 인덱스인지
                space.clearObjPoints();
                space.objIntoSpace(obj_loaded_bin[obj_idx]);

                const ObjBinary& current_obj = obj_loaded_bin[obj_idx];
                if(print_current_obj == 0) {
                    std::string time_str = space.formatUnixTime(current_obj.time);
                    std::cout << "\n[Object] idx = " << obj_idx << ", time = " << current_obj.time << " (" << time_str << "), num = " << current_obj.num << std::endl;

                    std::cout << "obj_id = [ ";
                    for(const auto& obj : current_obj.obj_data) {
                        std::cout << obj.obj_id << ", ";
                    }
                    std::cout << " ]" << std::endl;

                    for(size_t i = 0; i < current_obj.obj_data.size(); ++i) {
                        const auto& obj = current_obj.obj_data[i];

                        std::string label = "unknown";
                        auto it = objId_to_label.find(obj.obj_id);
                        if(it != objId_to_label.end()) {
                            label = it->second;
                        }

                        std::cout << "id=[" << obj.obj_id << "] " << label 
                                    << "/ nearest(" << obj.nearest_x << ", " << obj.nearest_y << ", " << obj.nearest_z
                                    << "), min(" << obj.min_x << ", " << obj.min_y << ", " << obj.min_z
                                    << "), max(" << obj.max_x << ", " << obj.max_y << ", " << obj.max_z
                                    << "), distance=" << obj.distance 
                                    << ", size=" << obj.size << std::endl;
                    }
                    print_current_obj = 1;
                }
            }
            else {
                std::cout << "[Object] Closest frame not found for lidar time = " << current_lidar.time << std::endl;
            }
        }

        // mode 전환시 시간 동기화
        if(video_control == 6) {        // key O -> TIMELINE 모드로 전환
            if(mode != TIMELINE) {
                uint64_t lidar_time = lidar_loaded_bin[lidar_idx].time;

                // timeline에서 lidar_time과 가장 가까운 index 찾기
                size_t closest_idx = 0;
                uint64_t min_diff = std::numeric_limits<uint64_t>::max();
                for(size_t i = 0; i < timeline.size(); ++i) {
                    uint64_t t = (timeline[i].type == DataType::LIDAR) ? 
                                    lidar_loaded_bin[timeline[i].index].time : obj_loaded_bin[timeline[i].index].time;
                    uint64_t diff = (lidar_time > t) ? lidar_time - t : t - lidar_time;
                    if(diff < min_diff) {
                        min_diff = diff;
                        closest_idx = i;
                    }
                }
                timeline_idx = closest_idx;
                mode = TIMELINE;
                std::cout << "\nSwitched to TIMELINE mode (synced)" << std::endl;
            }
            video_control = 1;
        }

        if(video_control == 7) {        // key L -> BINARY SEARCH 모드로 전환
            if(mode != BINARY_SEARCH) {
                uint64_t current_time;
                if(timeline[timeline_idx].type == DataType::LIDAR) {
                    current_time = lidar_loaded_bin[timeline[timeline_idx].index].time;
                } else {
                    current_time = obj_loaded_bin[timeline[timeline_idx].index].time;
                }

                // lidar에서 current_time과 가장 가까운 index 찾기
                size_t closest_idx = 0;
                uint64_t min_diff = std::numeric_limits<uint64_t>::max();
                for(size_t i = 0; i < lidar_loaded_bin.size(); ++i) {
                    uint64_t diff = (lidar_loaded_bin[i].time > current_time) ? 
                                        lidar_loaded_bin[i].time - current_time : current_time - lidar_loaded_bin[i].time;
                    if(diff < min_diff) {
                        min_diff = diff;
                        closest_idx = i;
                    }
                }
                lidar_idx = closest_idx;
                mode = BINARY_SEARCH;
                std::cout << "\nSwitched to BINARY SEARCH mode (synced)" << std::endl;
            }
            video_control = 1;
        }
#endif

// #if TIMELINE
//         timer.end();
//         if((timer.elapsed_ms() >= new_ms_per_frame) && (video_control == 0)) {
//             if(timeline_idx + 1 < timeline.size()) {
//                 ++timeline_idx;
//                 timer.start();
//             }
//         }
//         else if(video_control == 2) {
//             if(timeline_idx > 0) {
//                 --timeline_idx;
//                 std::cout << "[key Q] timeline_idx = " << timeline_idx << std::endl;
//             }
//             video_control = 1;
//         }
//         else if(video_control == 3) {
//             if(timeline_idx + 1 < timeline.size()) {
//                 ++timeline_idx;
//                 std::cout << "[key E] timeline_idx = " << timeline_idx << std::endl;
//             }
//             video_control = 1;
//         }


//         const UnifiedData& current = timeline[timeline_idx];
        
//         if(current.type == DataType::OBJECT) {
//             const ObjBinary& current_obj = obj_loaded_bin[current.index];
//             space.clearObjPoints();
//             space.objIntoSpace(current_obj);
//             if(print_current == 0){
//                 std::cout << "[Object] idx = " << current.index << ", time = " << current_obj.time << ", num = " << current_obj.num << std::endl;
            
//                 std::cout << "obj_id = [ ";
//                 for(size_t i = 0; i < current_obj.obj_data.size(); ++i) {
//                     std::cout << current_obj.obj_data[i].obj_id;
//                     if(i != current_obj.obj_data.size() - 1) std::cout << ", ";
//                 }
//                 std::cout << " ]" << std::endl;
//                 print_current = 1;
//             }
            
//         }
//         else {
//             const LidarBinary& current_lidar = lidar_loaded_bin[current.index];
//             space.clearLidarPoints();
//             space.lidarIntoSpace(current_lidar);
//             if(print_current == 0) {
//                 std::cout << "[Lidar] idx = " << current.index << ", time = " << current_lidar.time << ", num = " << current_lidar.num << std::endl;
//                 print_current = 1;
//             }
            
//         }
// #endif
        
// #if BINARY_SEARCH
//         timer.end();
//         if((timer.elapsed_ms() >= new_ms_per_frame) && (video_control == 0)) {
//             if(_lidar_idx + 1 < lidar_loaded_bin.size()) {
//                 ++_lidar_idx;
//                 timer.start();
//             }
//         }
//         else if(video_control == 4) {
//             if(_lidar_idx > 0) {
//                 --_lidar_idx;
//                 std::cout << "[key J] lidar_idx = " << _lidar_idx << std::endl;
//             }
//             video_control = 1;
//         }
//         else if(video_control == 5) {
//             if(_lidar_idx + 1 < lidar_loaded_bin.size()) {
//                 ++_lidar_idx;
//                 std::cout << "[key K] lidar_idx = " << _lidar_idx << std::endl;
//             }
//             video_control = 1;
//         }

//         const LidarBinary& current_lidar = lidar_loaded_bin[_lidar_idx];
//         space.clearLidarPoints();
//         space.lidarIntoSpace(current_lidar);

//         if(print_current_lidar == 0) {
//             std::cout << "[Lidar] idx = " << _lidar_idx << ", time = " << current_lidar.time << ", num = " << current_lidar.num << std::endl;
//             print_current_lidar = 1;
//         }
        
//         int obj_idx = utils.findClosestObjectFrame(current_lidar.time, obj_loaded_bin);
//         if(obj_idx != -1) {     // 이진탐색 결과가 유효한 인덱스인지
//             space.clearObjPoints();
//             space.objIntoSpace(obj_loaded_bin[obj_idx]);

//             const ObjBinary& current_obj = obj_loaded_bin[obj_idx];
//             if(print_current_obj == 0) {
//                 std::cout << "[Object] idx = " << obj_idx << ", time = " << current_obj.time << ", num = " << current_obj.num << std::endl;

//                 std::cout << "obj_id = [ ";
//                 for(const auto& obj : current_obj.obj_data) {
//                     std::cout << obj.obj_id << ", ";
//                 }
//                 std::cout << " ]" << std::endl;
//                 print_current_obj = 1;
//             }
//         }
//         else {
//             std::cout << "[Object] Closest frame not found for lidar time = " << current_lidar.time << std::endl;
//         }
// #endif

        // // frame init
        // static int frame_index;
        // _cloud.clearField();

        // // set frame
        // _cloud.setField(frame_index);
        
        // //======================================= print_ver_0.2 ==============================================

        // std::cout << "obj frame : " << obj_frame << std::endl;

        // for(auto obj_dat : obj_loaded_bin[obj_frame].obj_data){
        //     std::cout << "class : " << obj_dat.obj_id << std::endl;
        //     if(obj_dat.obj_id == 56){
        //         std::cout << "@@@@@@@@@2Find Chair!\n";
        //         std::cout << "min_x: " << obj_dat.min_x
        //                 << ", min_y: " << obj_dat.min_y
        //                 << ", min_z: " << obj_dat.min_z
        //                 << ", max_x: " << obj_dat.max_x
        //                 << ", max_y: " << obj_dat.max_y
        //                 << ", max_z: " << obj_dat.max_z << std::endl;
        //         space.addPoint(glm::vec3(obj_dat.min_x, obj_dat.min_y, obj_dat.min_z), glm::vec3(1.0f, 0.0f, 0.0f));
        //         space.addPoint(glm::vec3(obj_dat.max_x, obj_dat.max_y, obj_dat.max_z), glm::vec3(1.0f, 0.0f, 1.0f));
        //         space.addBox(glm::vec3(obj_dat.min_x, obj_dat.min_y, obj_dat.min_z), glm::vec3(obj_dat.max_x, obj_dat.max_y, obj_dat.max_z), glm::vec3(1.0f, 0.0f, 0.0f));
        //         space.addLine(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(obj_dat.nearest_x, obj_dat.nearest_y, obj_dat.nearest_z), 1.0f, 1.0f, 0.0f);
        //     }
        // }

        // space.lidarIntoSpace(lidar_loaded_bin, frame_index);

        // if(obj_frame > obj_loaded_bin.size() - 1) obj_frame --;

        // //======================================= print_ver_0.2 ==============================================

        // if(prev_frame_index != frame_index) {
            // space.clearPoints();
            // std::cout << frame_index << "th frame\n";
            // prev_frame_index = frame_index;

            // // 마지막 프레임이면 stay
            // if(frame_index == lidar_loaded_bin.size()) {
            //     frame_index = lidar_loaded_bin.size() - 1;
            // }

            // std::cout << "Lidar data size: " << lidar_loaded_bin[frame_index].lidar_data.size() << std::endl;
            // std::cout << "ObjectInfo data size: " << obj_loaded_bin[frame_index].obj_data.size() << std::endl;

            // ObjBinary* objdat = &(obj_loaded_bin[frame_index]);


            // for(auto obj_dat : obj_loaded_bin[frame_index].obj_data){
            //     std::cout << "class : " << obj_dat.obj_id << std::endl;
            //     if(obj_dat.obj_id == 56){
            //         std::cout << "@@@@@@@@@2Find Chair!\n";
            //         space.addBox(glm::vec3(obj_dat.min_x, obj_dat.min_y, obj_dat.min_z), glm::vec3(obj_dat.max_x, obj_dat.max_y, obj_dat.max_z), glm::vec3(1.0f, 0.0f, 0.0f));
            //         space.addLine(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(obj_dat.nearest_x, obj_dat.nearest_y, obj_dat.nearest_z), 1.0f, 1.0f, 0.0f);
            //     }
            // }

            // space.lidarIntoSpace(lidar_loaded_bin, frame_index);
            // frame_index++;
            // space.ObjIntoSpace(obj_loaded_bin, frame_index);
        // }

        // Update camera position based on spherical coordinates
        float x = orbitRadius * glm::cos(verticalAngle) * glm::cos(horizontalAngle) + cameraPosition.x;
        float y = orbitRadius * glm::cos(verticalAngle) * glm::sin(horizontalAngle) + cameraPosition.y;
        float z = orbitRadius * glm::sin(verticalAngle) + cameraPosition.z;
        camera.setPosition(glm::vec3(x, y, z));
        camera.setTarget(cameraTarget);

        // Apply camera view matrix
        glm::mat4 view = camera.getViewMatrix();
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(&view[0][0]);

        // Render scene
        space.render();
        // space.renderBillboards(camera.getPosition());       // 카메라 위치 전달

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

// void printErr(int index){

//     std::cout << "[Occur the Error!!]" << std::endl;
//     if(index == 2){ // Error Number 2 : wrong argument with option -d
//         std::cout << "Option -d or --data need the additional option [filename]" << std::endl;
//         std::cout << "Usage : ./aeronSW-visualizer -d [Lidar Data filename]" << std::endl;
//         std::cout << "or check the arguments : ./aeronSW-visualizer -h" << std::endl;
//     }else if(index == 3){ // Error Number 3 : wron argumnet with option -d -p
//         std::cout << "Option -d -p need the additional options [pcap filename] [json filename]" << std::endl;
//         std::cout << "Usage : ./aeronSW-visualizer -d -p [lidar_filename.pcap] [metadata_filename.json]" << std::endl;
//     }
// }

void printHowToUse(){
    std::cout << "./aeronSW-visualizer [start time] [end time]" << std::endl;
}

// void printUsage(){
//     std::cout << "[Arguments]" << std::endl;
//     std::cout << "-h, --help : Show you the Usage" << std::endl;
//     std::cout << "-d, --data [option] [lidar filename] [metadata filename]\n"
//                             "\t\t\t\t: Data file path that you want to show" << std::endl;
//     std::cout << "-d -p [pcap_filename] [json_filename]" << std::endl;
// }

std::string getFileExtension(const std::string& fileName) {
    // 마지막 '.'의 위치를 찾음
    size_t dotPos = fileName.rfind('.');
    if (dotPos == std::string::npos) {
        // '.'이 없다면 확장자가 없는 경우
        return "";
    }
    // '.' 이후의 문자열 반환
    return fileName.substr(dotPos + 1);
}

std::string removeFileExtension(const std::string& fileName) {
    size_t lastDot = fileName.find_last_of('.');
    if(lastDot == std::string::npos) {
        // '.'이 없다면 확장자가 없다고 간주하고 원본 파일명 그대로 반환
        return fileName;
    }
    // 파일명 문자열의 처음부터 '.' 직전까지의 부분 문자열을 잘라서 반환
    return fileName.substr(0, lastDot);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        glm::vec3 cameraDirection(glm::cos(horizontalAngle),
                                  glm::sin(horizontalAngle),
                                  0.0f);
        glm::vec3 cameraOrthoDirection(-glm::sin(horizontalAngle),
                                       glm::cos(horizontalAngle),
                                       0.0f);
        if (key == GLFW_KEY_W) {                            //w
            cameraPosition -= CAMERA_SPEED * cameraDirection; // 앞으로 이동
            cameraTarget -= CAMERA_SPEED * cameraDirection; // 앞으로 이동
        } else if (key == GLFW_KEY_S) {                     //s
            cameraPosition += CAMERA_SPEED * cameraDirection; // 뒤로 이동
            cameraTarget += CAMERA_SPEED * cameraDirection; // 뒤로 이동
        } else if (key == GLFW_KEY_A) {                     //a
            cameraPosition -= CAMERA_SPEED * cameraOrthoDirection; // 왼쪽 이동
            cameraTarget -= CAMERA_SPEED * cameraOrthoDirection; // 왼쪽 이동
        } else if (key == GLFW_KEY_D) {                     //d
            cameraPosition += CAMERA_SPEED * cameraOrthoDirection; // 오른쪽 이동
            cameraTarget += CAMERA_SPEED * cameraOrthoDirection; // 오른쪽 이동
        } else if (key == GLFW_KEY_ESCAPE) {                //esc
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        } else if (key == GLFW_KEY_P) {                     //p
            if(video_control != 1)
                video_control = 1;                          //pause
            else
                video_control = 0;                          //play
        } else if (key == GLFW_KEY_O) {                     //o
            video_control = 6;                             //TIMELINE mode
        } else if (key == GLFW_KEY_L) {                     //l
            video_control = 7;                             //BINARY SEARCH mode
        } else if (key == GLFW_KEY_U) {                     //u
            mode = TIMELINE;
            video_control = 2;                              //previous frame (timeline_idx)
        } else if (key == GLFW_KEY_I) {                     //i
            mode = TIMELINE;
            video_control = 3;                              //next frame (timeline_idx)
        } else if (key == GLFW_KEY_J) {                     //j
            mode = BINARY_SEARCH;
            video_control = 4;                              //previous frame (binary search with object)
        } else if (key == GLFW_KEY_K) {                     //k
            mode = BINARY_SEARCH;
            video_control = 5;                              //next frame (binary search with object)
        } else if(key == GLFW_KEY_SPACE){                   //space bar
            print_current = 0;                              //print current data info
            print_current_lidar = 0;
            print_current_obj = 0;
        } else if (key == GLFW_KEY_PAGE_DOWN){              //page down
            if (video_speed_coeffi < VIDEO_SPEED_COEFFI_MAX)
                video_speed_coeffi += VIDEO_SPEED_CONTROL_RESOLUTION; //video speed down
        } else if (key == GLFW_KEY_PAGE_UP){                //page up
            if (video_speed_coeffi > VIDEO_SPEED_COEFFI_MIN)
                video_speed_coeffi -= VIDEO_SPEED_CONTROL_RESOLUTION; //video speed up
        }
    }
}



void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        leftMousePressed = (action == GLFW_PRESS);
    }
}


void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    if (leftMousePressed) {
        double dx = xpos - lastMouseX;
        double dy = ypos - lastMouseY;

        horizontalAngle += (float)(dx * ORBIT_SPEED);
        verticalAngle -= (float)(dy * ORBIT_SPEED);
        

        // 상하 회전 제한 (90도 범위 유지)
        if (verticalAngle > glm::radians(89.0f)) verticalAngle = glm::radians(89.0f);
        if (verticalAngle < glm::radians(-89.0f)) verticalAngle = glm::radians(-89.0f);


    }
    lastMouseX = xpos;
    lastMouseY = ypos;

}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    orbitRadius -= (float)(yoffset * ZOOM_SPEED);
    if (orbitRadius < 1.0f) orbitRadius = 1.0f; // 최소 거리 제한
}

void setupViewport(const GLuint width, const GLuint height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, static_cast<double>(width) / height, 0.1, 100.0);
}