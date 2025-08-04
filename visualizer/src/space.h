#pragma once

#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <GL/glut.h>
#include <cmath>        // for sin, cos, M_PI
#include <tuple>
#include <iostream>
#include <unordered_map>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "configs.h"
#include "binary_utils.h"


typedef struct _box{
    glm::vec3 point;
    glm::vec3 xway;
    glm::vec3 yway;
    glm::vec3 zway;
    glm::vec3 color;
}Box;

struct RenderTextInfo {
    glm::vec3 color;
    int obj_id;
    std::string label;
    glm::vec3 min_coord;
    glm::vec3 max_coord;
    float distance;
};

class Space {
public:
    void addLidarPoint(const glm::vec3& point);
    void addLidarPoint(const glm::vec3& point, const glm::vec3& color);

    void addObjPoint(const glm::vec3& point, const glm::vec3& color);

    void clearLidarPoints();
    void clearObjPoints();

    void addLine(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color);
    void clearLines();

    // utils for box
    void addBox(const glm::vec3& point, const glm::vec3& xway, 
                const glm::vec3& yway, const glm::vec3& zway, const glm::vec3& color);
    void addBox(const glm::vec3& start_point, const glm::vec3& end_point, const glm::vec3& color);
    void clearBoxes();
    
    void drawGrid();
    void render() const;

    void lidarIntoSpace(const LidarBinary& index);
    void objIntoSpace(const ObjBinary& index);

    void getDynamicColorById(int obj_id, glm::vec3& out_color);
    void resetColorMap();

    std::string colorToString(const glm::vec3& color) const;
    void renderText2D(const std::string& text, float x, float y, const glm::vec3& color) const;

    std::string formatUnixTime(uint64_t unixtime) const;
    

    std::vector<std::pair<glm::vec3, glm::vec3>> lidar_points; //point, color
    std::vector<std::pair<glm::vec3, glm::vec3>> obj_points;
    //start point, end point, R, G, B
    std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3>> lines; 
    std::vector<Box> boxes;

    std::vector<RenderTextInfo> render_text_list;
};