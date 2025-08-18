#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <iostream>
#include <vector>

// image sensor configure (camera)
// C270 HD 웹캠: 대각선 시야(FOV) 55도      1280x720
#define LENS_HFOV       48.81   // degree // 수평 시야각
#define LENS_VFOV       28.63   // degree // 수직 시야각
#define LENS_ZOOM       5       // 화면 상의 카메라 시야를 보여주는 선 길이를 늘리기 위함

struct CameraConfig {
    int id;                 // 카메라 ID
    glm::vec3 position;     // 라이다 기준 상대 위치 (x, y, z)
    glm::vec3 direction;    // 카메라가 바라보는 방향
    glm::vec3 color;
    // 시야각과 줌은 전역 상수로 사용
};

// 다중 카메라 설정
inline std::vector<CameraConfig> cameras = {
    {0, glm::vec3(0.0f, -0.1f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f)},     // yellow
    {1, glm::vec3(0.0f, 0.1f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f)},
    // {2, glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(1.0f, 1.0f, 0.0f)}
};

class LensView{
public:
    LensView(glm::vec3 front, glm::vec3 h, glm::vec3 w, glm::vec3 V){
        this->front = front;
        this->h = h;
        this->w = w;
        this->V = V;

        // must be kept this order
        // 렌즈 시야의 4개 모서리 벡터 생성
        glm::vec3 viewvec0 = front + h + w;     // 우상단
        glm::vec3 viewvec1 = front + h - w;     // 좌상단
        glm::vec3 viewvec2 = front - h - w;     // 좌하단
        glm::vec3 viewvec3 = front - h + w;     // 우하단

        this->lens_view_vector.push_back(viewvec0);
        this->lens_view_vector.push_back(viewvec1);
        this->lens_view_vector.push_back(viewvec2);
        this->lens_view_vector.push_back(viewvec3);
    };
    
    std::vector<float> transposeMatrix(const std::vector<float>& matrix, int rows, int cols) {
        std::vector<float> transposed(cols * rows);

        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                transposed[j * rows + i] = matrix[i * cols + j]; // (i, j) -> (j, i)
            }
        }
        return transposed;
    }

    glm::vec3 front;
    glm::vec3 h;
    glm::vec3 w;
    glm::vec3 V;

    std::vector<glm::vec3> lens_view_vector;
};