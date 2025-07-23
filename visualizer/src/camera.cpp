
#include "camera.h"

Camera::Camera()
    : position(5.0f, 0.0f, 0.0f), target(0.0f, 0.0f, 0.0f), up(0.0f, 0.0f, 1.0f) {}

void Camera::setPosition(const glm::vec3& pos) {
    position = pos;
}

void Camera::setTarget(const glm::vec3& tgt) {
    target = tgt;
}

void Camera::setUp(const glm::vec3& upDirection) {
    up = upDirection;
}

glm::vec3 Camera::getPosition() const {
    return position;
}

glm::vec3 Camera::getTarget() const {
    return target;
}

glm::vec3 Camera::getUp() const {
    return up;
}

glm::mat4 Camera::getViewMatrix() const {
    return glm::lookAt(position, target, up);
}