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

// struct Billboard {
//     glm::vec3 position;
//     std::string texture_path;
//     float size = 0.5f;
// };

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
    // void addLine(const glm::vec3& start, const glm::vec3& end,
    //             const GLfloat R, const GLfloat G, const GLfloat B);
    void addLine(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color);
    void clearLines();
    // void addObj(const glm::vec3& point, const glm::vec3& xway, 
    //             const glm::vec3& yway, const glm::vec3& zway);

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
    
    // std::string getLabelFromId(int obj_id);
    // std::string getLabelTexturePath(int obj_id);

    // void addLabelBillboard(const glm::vec3& pos, const std::string& textture_path);
    // void renderBillboards(const glm::vec3& cameraPos);

    // GLuint loadTexture(const std::string& path);
    // void clearTextureCache();

// private:
    //time series points
    // std::vector<std::pair<int, std::vector<glm::vec3>>> ts_points; 
    std::vector<std::pair<glm::vec3, glm::vec3>> lidar_points; //point, color
    std::vector<std::pair<glm::vec3, glm::vec3>> obj_points;
    //start point, end point, R, G, B
    std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3>> lines; 
    std::vector<Box> boxes;

    // std::vector<Billboard> billboards;


    // std::vector<std::pair<glm::vec3, std::string>> render_text_list;
    std::vector<RenderTextInfo> render_text_list;
};