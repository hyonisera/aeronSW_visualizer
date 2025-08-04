
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

#define WINDOW          1
#define PRINT_VAL       0
#define TIMELINE        1
#define BINARY_SEARCH   2
int mode = TIMELINE;
#define FRAME_RATE      5      // 초당 프레임 수(Hz)

// Globals
Camera camera;
Space space;
BinaryUtils binaryutils;

bool leftMousePressed = false;
double lastMouseX = 0.0, lastMouseY = 0.0;
float horizontalAngle = glm::radians(INIT_CAM_HANGLE * 360.0f / 100.0f), verticalAngle = glm::radians(INIT_CAM_VANGLE * 360.0f / 100.0f); // 카메라의 회전 각도
float orbitRadius = INIT_CAM_RADIUS;                           // 궤도 반지름

// screen control
// 0 : play // 1 : pause
// 6 : timeline mode // 2 : timeline prev frame // 3 : timeline next frame
// 7 : binary search mode // 4 : binary search prev frame // 5 : binary search next frame
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

int print_current = 1;
int print_current_lidar = 1;
int print_current_obj = 1;

// argument callback function
void printHowToUse();
std::string getFileExtension(const std::string& fileName);
std::string removeFileExtension(const std::string& fileName);

// Function Prototypes
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
void setupViewport(const GLuint width, const GLuint height);


int main(int argc, char* argv[]) {

    //============================= argument parsing =================================
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
            std::cout << "Error: file extension is not .uld or .udd" << std::endl;
            return 0;
        }
    }

    std::sort(get_lidar_filename.begin(), get_lidar_filename.end());    // 불러온 파일들 시간순 정렬
    std::sort(get_detectinfo_filename.begin(), get_detectinfo_filename.end());


    //============================= read binary files =================================
    BinaryUtils utils;
    std::vector<LidarBinary> lidar_loaded_bin;    // 여러 프레임
    std::vector<ObjBinary> obj_loaded_bin;

    for(const std::string& name : get_lidar_filename) {
        std::cout << "Loaded filename: " << name << std::endl;
#if PRINT_VAL
        lidar_loaded_bin.clear();
#endif
        if(utils.load_lidar_binary(name, lidar_loaded_bin)) {
#if PRINT_VAL
            for(const auto& data : lidar_loaded_bin) {

                std::string time_str = space.formatUnixTime(data.time);

                std::cout << "Loaded binary time: " << data.time << " / " << time_str << std::endl;
                std::cout << "Loaded num: " << data.num << std::endl;
                // int i=0;
                // for(const auto& _data : data.lidar_data) {
                    
                //     std::cout << i << " x: " << _data.x
                //                     << ", y: " << _data.y
                //                     << ", z: " << _data.z
                //                     << ", reflectivity: " << _data.reflectivity
                //                     << ", cluster_id: " << _data.cluster_id
                //                     << std::endl;
                //     i++;
                // }
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

                std::string time_str = space.formatUnixTime(data.time);
                
                std::cout << "Loaded binary time: " << data.time << " / " << time_str << std::endl;
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


    // 이진탐색 방식 - 객체 기준 매칭 테이블 생성
    std::vector<int> obj_to_lidar_mapping;
    if(!lidar_loaded_bin.empty() && !obj_loaded_bin.empty()) {
        // 모든 객체 프레임에 대해 가장 가까운 라이다 프레임 인덱스를 매핑
        obj_to_lidar_mapping.resize(obj_loaded_bin.size());
        for(size_t i = 0; i < obj_loaded_bin.size(); ++i) {
            obj_to_lidar_mapping[i] = binaryutils.findClosestLidarFrame(obj_loaded_bin[i].time, lidar_loaded_bin);
        }
        std::cout << "Object-to-Lidar mapping created successfully" << std::endl;
    }

    // 타임라인 방식
    std::vector<UnifiedData> timeline;
    size_t lidar_idx = 0;
    size_t obj_idx = 0;
    MyTimer timer;

    while(lidar_idx < lidar_loaded_bin.size() || obj_idx < obj_loaded_bin.size()) {
        if(obj_loaded_bin[obj_idx].time <= lidar_loaded_bin[lidar_idx].time) {
            if(obj_loaded_bin[obj_idx].time == 0) {     // 빈 파일(num=0, time=0) 예외처리
                ++obj_idx;
                continue;
            }
            timeline.push_back({DataType::OBJECT, obj_idx++});
        }
        else {
            timeline.push_back({DataType::LIDAR, lidar_idx++});
        }
    }

    for(size_t i=0; i<timeline.size(); ++i) {
        const UnifiedData& entry = timeline[i];
        if(entry.type == DataType::LIDAR) {
            std::string time_str = space.formatUnixTime(lidar_loaded_bin[entry.index].time);
            std::cout << "[" << i << "] lid idx = " << entry.index << ", time = " << lidar_loaded_bin[entry.index].time << " / " << time_str << std::endl;
        }
        else {
            std::string time_str = space.formatUnixTime(obj_loaded_bin[entry.index].time);
            std::cout << "[" << i << "] obj idx = " << entry.index << ", time = " << obj_loaded_bin[entry.index].time << " / " << time_str << std::endl;
        }
    }

    size_t timeline_idx = 0;


#if WINDOW
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


        timer.end();
        //============================= TIMELINE MODE =================================
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
            
            if(current.type == DataType::LIDAR) {
                const LidarBinary& current_lidar = lidar_loaded_bin[current.index];
                space.clearLidarPoints();
                space.clearObjPoints();     // obj box 그린 후 다음 프레임 넘어갔을 때 lidar 데이터 차례에 min, max, nearest 점들이 남아있는 것 삭제
                space.lidarIntoSpace(current_lidar);

                if(print_current == 0) {
                    std::string time_str = space.formatUnixTime(current_lidar.time);
                    std::cout << "\n[Lidar] idx = " << current.index << ", time = " << current_lidar.time << " (" << time_str << "), num = " << current_lidar.num << std::endl;
                    
                    print_current = 1;
                }
            }
            else {
                const ObjBinary& current_obj = obj_loaded_bin[current.index];

                // 이전 프레임으로 갈 때 timeline 반대방향 기준 직전 라이다 데이터에 객체인식 데이터가 입혀지는 문제 수정
                // timeline[timeline_idx]가 OBJECT 타입일 때 가장 가까운 라이다 프레임 인덱스를 찾아 lidar 데이터 먼저 띄우고 그 위에 obj 올림
                int lidar_to_display = -1;      // 대응되는 라이다 인덱스 저장
                for(int i = timeline_idx - 1; i >= 0; --i) {
                    if(timeline[i].type == DataType::LIDAR) {
                        const LidarBinary& lid = lidar_loaded_bin[timeline[i].index];
                        if(lid.time <= current_obj.time) {
                            lidar_to_display = timeline[i].index;
                            break;
                        }
                    }
                }
                if(lidar_to_display != -1) {
                    space.clearLidarPoints();
                    space.lidarIntoSpace(lidar_loaded_bin[lidar_to_display]);
                }

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
        }
        
        //============================= BINARY SEARCH MODE =================================
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

            // 객체 기준 매칭: 현재 라이다 프레임에 매칭된 객체 프레임 찾기
            bool obj_found = false;
            for(size_t i = 0; i < obj_to_lidar_mapping.size(); ++i) {
                if(obj_to_lidar_mapping[i] == static_cast<int>(lidar_idx)) {
                    space.clearObjPoints();
                    space.objIntoSpace(obj_loaded_bin[i]);

                    const ObjBinary& current_obj = obj_loaded_bin[i];
                    if(print_current_obj == 0) {
                        std::string time_str = space.formatUnixTime(current_obj.time);
                        std::cout << "\n[Object] idx = " << i << ", time = " << current_obj.time << " (" << time_str << "), num = " << current_obj.num << std::endl;

                        std::cout << "obj_id = [ ";
                        for(const auto& obj : current_obj.obj_data) {
                            std::cout << obj.obj_id << ", ";
                        }
                        std::cout << " ]" << std::endl;

                        for(size_t j = 0; j < current_obj.obj_data.size(); ++j) {
                            const auto& obj = current_obj.obj_data[j];

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
                    obj_found = true;
                    break;      // 하나의 객체 프레임만 매칭되므로
                }
            }

            if(!obj_found) {
                space.clearObjPoints();     // 라이다 데이터만 화면에 출력될 때 이전의 객체인식 점이 남아있는 것 제거
                if(print_current_obj == 0) {
                    std::cout << "[Object] No object frame mapped to lidar frame " << lidar_idx << std::endl;
                    print_current_obj = 1;
                }
            }
        }

        //============================= 모드 전환시 시간 동기화 =================================
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
#endif

    return 0;
}


void printHowToUse(){
    std::cout << "Usage: ./aeronSW_visualizer [start_time] [end_time]\nTime Format: yy-MM-dd-HH-mm-ss" << std::endl;
}


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