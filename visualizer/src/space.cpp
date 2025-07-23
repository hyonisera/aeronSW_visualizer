
#include "space.h"

#define DYNAMIC 1
#define STATIC  0
#define COCODATASET 1

#define BILLBOARD   0

// #define STB_IMAGE_IMPLEMENTATION
// #include "stb_image.h"

extern float orbitRadius;

std::string vecToStr(const glm::vec3& v) {
    char buff[100];
    snprintf(buff, sizeof(buff), "%.4f, %.4f, %.4f", v.x, v.y, v.z);
    return std::string(buff);
}

std::string Space::formatUnixTime(uint64_t unixtime) const{
    std::time_t sec_time = static_cast<std::time_t>(unixtime / 1000);
    uint64_t millis = static_cast<uint64_t>(unixtime % 1000);   // ms
    std::tm* tm_ptr = std::localtime(&sec_time);

    std::ostringstream oss;
    oss << std::put_time(tm_ptr, "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << millis;  // ms
    return oss.str();
}


void Space::addLidarPoint(const glm::vec3& point) {      // point(x, y, z) 위치에 빨간색 점
    std::pair<glm::vec3, glm::vec3> _point;
    _point.first = point;
    _point.second = glm::vec3(1.0f, 0.0f, 0.0f); //default = red
    lidar_points.push_back(_point);
}

void Space::addLidarPoint(const glm::vec3& point, const glm::vec3& color){   // point(x, y, z) 위치에 내가 지정한 색 점
    std::pair<glm::vec3, glm::vec3> _point;
    _point.first = point;
    _point.second = color;
    lidar_points.push_back(_point);
}

void Space::addObjPoint(const glm::vec3& point, const glm::vec3& color) {
    std::pair<glm::vec3, glm::vec3> _point;
    _point.first = point;
    _point.second = color;
    obj_points.push_back(_point);
}

void Space::clearLidarPoints(){
    lidar_points.clear();
    std::vector <std::pair<glm::vec3, glm::vec3>>().swap(lidar_points);
    // std::vector <std::pair<glm::vec3, glm::vec3>>() -> 빈 벡터 / 빈 벡터와 points를 교환해서 할당된 메모리 전부 해제
}

void Space::clearObjPoints(){
    obj_points.clear();
    std::vector <std::pair<glm::vec3, glm::vec3>>().swap(obj_points);
}

// void Space::addLine(const glm::vec3& start, const glm::vec3& end,
//             const GLfloat R, const GLfloat G, const GLfloat B) {
//     lines.emplace_back(start, end, R, G, B);
// }

void Space::addLine(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color) {
    lines.emplace_back(start, end, color);
}

void Space::clearLines(){
    lines.clear();
    std::vector <std::tuple<glm::vec3, glm::vec3, glm::vec3>>().swap(lines);
}

void Space::addBox(const glm::vec3& point, const glm::vec3& xway, 
                const glm::vec3& yway, const glm::vec3& zway, const glm::vec3& color){
    Box _box;
    _box.point = point;
    _box.xway = xway;
    _box.yway = yway;
    _box.zway = zway;
    _box.color = color;
    boxes.push_back(_box);
}

void Space::addBox(const glm::vec3& start_point, const glm::vec3& end_point, const glm::vec3& color){
    addBox(start_point, glm::vec3(end_point.x-start_point.x, 0.0f, 0.0f), 
                        glm::vec3(0.0f, end_point.y-start_point.y, 0.0f),
                        glm::vec3(0.0f, 0.0f, end_point.z-start_point.z), color);
}

void Space::clearBoxes(){
    boxes.clear();
    std::vector <Box>().swap(boxes);
}

// void Space::addObj(const glm::vec3& point, const glm::vec3& xway, 
//                 const glm::vec3& yway, const glm::vec3& zway){

//     lines.emplace_back(point, point + xway, 1.0f, 0.0f, 0.0f);
//     lines.emplace_back(point, point + yway, 0.0f, 1.0f, 0.0f);
//     lines.emplace_back(point, point + zway, 1.0f, 1.0f, 0.0f);
// }

// Helper function to draw a circle
void drawCircle(const glm::vec3& center, float radius, int segments, const glm::vec3& color) {
    glBegin(GL_TRIANGLE_FAN);       // 삼각형 팬이 원처럼 보이도록
    glColor3f((GLfloat)color.x, (GLfloat)color.y, (GLfloat)color.z);  // 동그라미 색상 (빨간색)

    glVertex3f(center.x, center.y, center.z);  // 중심점

    for (int i = 0; i <= segments; ++i) {
        float theta = 2.0f * M_PI * float(i) / float(segments);  // 각도
        float x = radius * cosf(theta);
        float y = radius * sinf(theta);
        glVertex3f(center.x + x, center.y + y, center.z);
    }
    glEnd();
}

void Space::drawGrid(){
    std::vector<std::pair<glm::vec3, glm::vec3>> gridpoints;

    for(int i=-GRID_NUM; i<GRID_NUM+1; i++){
        std::pair<glm::vec3, glm::vec3> line0_points;
        line0_points.first = glm::vec3(GRID_COEFFI*orbitRadius*i/GRID_NUM, -GRID_COEFFI*orbitRadius, GRID_Z_OFFSET);
        line0_points.second = glm::vec3(GRID_COEFFI*orbitRadius*i/GRID_NUM, GRID_COEFFI*orbitRadius, GRID_Z_OFFSET);
        gridpoints.push_back(line0_points);

        std::pair<glm::vec3, glm::vec3> line1_points;
        line1_points.first = glm::vec3(-GRID_COEFFI*orbitRadius, GRID_COEFFI*orbitRadius*i/GRID_NUM, GRID_Z_OFFSET);
        line1_points.second = glm::vec3(GRID_COEFFI*orbitRadius, GRID_COEFFI*orbitRadius*i/GRID_NUM, GRID_Z_OFFSET);
        gridpoints.push_back(line1_points);
    }

    for(const auto &points : gridpoints){
        const glm::vec3 color(0.6f, 0.6f, 0.6f);    //grid: gray line
        addLine(points.first, points.second, color);
    }
}

void Space::render() const {
    // Draw circles for points
    float lidar_radius = 0.01f;  // 동그라미 크기 설정
    int segments = 30;     // 동그라미 세그먼트 (정확도)
    for (const auto& point : lidar_points) {
        drawCircle(point.first, lidar_radius, segments, point.second);
    }

    float obj_radius = 0.01f;
    int obj_segments = 30;
    for(const auto& point : obj_points) {
        drawCircle(point.first, obj_radius, obj_segments, point.second);
    }

    // Draw lines
    for (const auto& line : lines) {
        glLineWidth(LINE_THICKNESS);  // 선 두께 설정
        glBegin(GL_LINES);
        
        glColor3f(std::get<2>(line).x, std::get<2>(line).y, std::get<2>(line).z);  // 선 색상
        glVertex3f(std::get<0>(line).x, std::get<0>(line).y, std::get<0>(line).z);  // 시작점
        glVertex3f(std::get<1>(line).x, std::get<1>(line).y, std::get<1>(line).z);  // 끝점
        glEnd();
    }

    // Draw boxes
    for (const auto& box : boxes) {
        glLineWidth(LINE_THICKNESS);
        glBegin(GL_LINES);
        glColor3f(box.color.x, box.color.y, box.color.z);

        float x = box.point.x;
        float y = box.point.y;
        float z = box.point.z;

        // std::cout << "Box: ( " << x << ", " << y << ", " << z << " )\n";
        
        glVertex3f(x, y, z);        glVertex3f(x + box.xway.x, y + box.xway.y, z + box.xway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.yway.x, y + box.yway.y, z + box.yway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.zway.x, y + box.zway.y, z + box.zway.z);
        
        x = box.point.x + box.xway.x + box.yway.x;
        y = box.point.y + box.xway.y + box.yway.y;
        z = box.point.z + box.xway.z + box.yway.z;

        glVertex3f(x, y, z);        glVertex3f(x - box.xway.x, y - box.xway.y, z - box.xway.z);
        glVertex3f(x, y, z);        glVertex3f(x - box.yway.x, y - box.yway.y, z - box.yway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.zway.x, y + box.zway.y, z + box.zway.z);

        x = box.point.x + box.yway.x + box.zway.x;
        y = box.point.y + box.yway.y + box.zway.y;
        z = box.point.z + box.yway.z + box.zway.z;

        glVertex3f(x, y, z);        glVertex3f(x - box.yway.x, y - box.yway.y, z - box.yway.z);
        glVertex3f(x, y, z);        glVertex3f(x - box.zway.x, y - box.zway.y, z - box.zway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.xway.x, y + box.xway.y, z + box.xway.z);

        x = box.point.x + box.zway.x + box.xway.x;
        y = box.point.y + box.zway.y + box.xway.y;
        z = box.point.z + box.zway.z + box.xway.z;

        glVertex3f(x, y, z);        glVertex3f(x - box.zway.x, y - box.zway.y, z - box.zway.z);
        glVertex3f(x, y, z);        glVertex3f(x - box.xway.x, y - box.xway.y, z - box.xway.z);
        glVertex3f(x, y, z);        glVertex3f(x + box.yway.x, y + box.yway.y, z + box.yway.z);

        glEnd();
    }

    #if BILLBOARD
    // 좌측 상단 객체인식 box color별 label 2D text
    std::unordered_map<std::string, glm::vec3> label_color_map;
    for(const auto& [color, label] : render_text_list) {
        label_color_map[label] = color;     // 한 프레임에 color 중복 제거
    }

    int y_offset = 20;
    for(const auto& [label, color] : label_color_map) {
        std::string display = colorToString(color) + ": " + label;
        renderText2D(display, 10, y_offset, color);
        y_offset += 20;
    }
    #endif

    int y_offset = 20;

    for(const auto& info : render_text_list) {
        std::string line1 = colorToString(info.color) + ": " + info.label + " (id=" + std::to_string(info.obj_id) + ")";
        std::string line2 = "min(" + vecToStr(info.min_coord) + ")";
        std::string line3 = "max(" + vecToStr(info.max_coord) + ")";
        std::string line4 = "distance=" + std::to_string(info.distance);

        renderText2D(line1, 10, y_offset, info.color); y_offset += 20;
        renderText2D(line2, 10, y_offset, info.color); y_offset += 20;
        renderText2D(line3, 10, y_offset, info.color); y_offset += 20;
        renderText2D(line4, 10, y_offset, info.color); y_offset += 20;
    }
}


void Space::lidarIntoSpace(const LidarBinary& index) {
    const auto& _points = index.lidar_data;

    for(size_t i = 0; i < _points.size(); ++i) {
        glm::vec3 point(_points[i].x, _points[i].y, _points[i].z);
        float color_sense = _points[i].reflectivity / 26.54f;
        glm::vec3 color(0.0f, 0.973f - color_sense, 0.364f + color_sense);
        addLidarPoint(point, color);
    }
}

void Space::objIntoSpace(const ObjBinary& index) {
    render_text_list.clear();

    const auto& _points = index.obj_data;

    for(size_t i = 0; i < _points.size(); ++i) {
        if(_points[i].obj_id == -1) {
            continue;
        }
        glm::vec3 point_nearest(_points[i].nearest_x, _points[i].nearest_y, _points[i].nearest_z);
        glm::vec3 point_min(_points[i].min_x, _points[i].min_y, _points[i].min_z);
        glm::vec3 point_max(_points[i].max_x, _points[i].max_y, _points[i].max_z);

        #if BILLBOARD
        std::string texture_path = getLabelTexturePath(_points[i].obj_id);
        // 박스 위에 라벨 위치
        glm::vec3 labelPos((_points[i].min_x + _points[i].max_x) * 0.5f, 
                            (_points[i].min_y + _points[i].max_y) * 0.5f, _points[i].max_z + 0.3f);
        addLabelBillboard(labelPos, texture_path);
        #endif

        #if STATIC
        glm::vec3 red(1.0f, 0.0f, 0.0f);
        glm::vec3 yellow(1.0f, 1.0f, 0.0f);
        glm::vec3 pink(1.0f, 0.75f, 0.8f);
        glm::vec3 orange(1.0f, 0.5f, 0.0f);
        glm::vec3 purple(0.5f, 0.0f, 1.0f);
        glm::vec3 gray(0.5f, 0.5f, 0.5f);
        #endif

        #if DYNAMIC
        glm::vec3 color;
        getDynamicColorById(_points[i].obj_id, color);
        addObjPoint(point_nearest, color);
        addObjPoint(point_min, color);
        addObjPoint(point_max, color);
        addBox(point_min, point_max, color);
        addLine(glm::vec3(0.0f, 0.0f, 0.0f), point_nearest, color);
        #endif


        auto it = objId_to_label.find(_points[i].obj_id);
        std::string label = (it != objId_to_label.end()) ? it->second : "unknown";

        render_text_list.push_back({
            color,
            _points[i].obj_id,
            label,
            point_min,
            point_max,
            _points[i].distance
        });

        #if STATIC
        if(index.obj_data[i].obj_id == 56) {        // chair
            addObjPoint(point_nearest, red);
            addObjPoint(point_min, red);
            addObjPoint(point_max, red);
            addBox(point_min, point_max, red);
            addLine(glm::vec3(0.0f, 0.0f, 0.0f), point_nearest, red);
        }
        if(index.obj_data[i].obj_id == 73) {       // book
            addObjPoint(point_nearest, yellow);
            addObjPoint(point_min, yellow);
            addObjPoint(point_max, yellow);
            addBox(point_min, point_max, yellow);
            addLine(glm::vec3(0.0f, 0.0f, 0.0f), point_nearest, yellow);
        }
        else if(index.obj_data[i].obj_id == 62) {       // tv
            addObjPoint(point_nearest, pink);
            addObjPoint(point_min, pink);
            addObjPoint(point_max, pink);
            addBox(point_min, point_max, pink);
            addLine(glm::vec3(0.0f, 0.0f, 0.0f), point_nearest, pink);
        }
        else if(index.obj_data[i].obj_id == 66) {       // keyboard
            addObjPoint(point_nearest, orange);
            addObjPoint(point_min, orange);
            addObjPoint(point_max, orange);
            addBox(point_min, point_max, orange);
            addLine(glm::vec3(0.0f, 0.0f, 0.0f), point_nearest, orange);
        }
        else if(index.obj_data[i].obj_id == 63) {       // laptop
            addObjPoint(point_nearest, purple);
            addObjPoint(point_min, purple);
            addObjPoint(point_max, purple);
            addBox(point_min, point_max, purple);
            addLine(glm::vec3(0.0f, 0.0f, 0.0f), point_nearest, purple);
        }
        #endif
    } 
}

#if DYNAMIC
// 이미 어떤 obj_id에 색이 할당되었는지 기억하는 해시 맵, 같은 obj_id가 다시 들어오면 이전에 할당한 색을 그대로 재사용
static std::unordered_map<int, glm::vec3> objId_to_color;   // int obj_id, glm::vec3 color
static int next_color_idx = 0;

static const std::vector<glm::vec3> color_table = {
    {1.0f, 0.0f, 0.0f},         // red #ff0000
    {1.0f, 0.65f, 0.0f},        // orange #ffa500
    {1.0f, 1.0f, 0.0f},         // yellow #ffff00
    {1.0f, 0.75f, 0.8f},        // pink #ffc0cb
    {0.58f, 0.0f, 0.83f},       // dark violet #9400d3
    {1.0f, 0.0f, 1.0f},         // magenta #ff00ff
    {0.0f, 1.0f, 1.0f}          // aqua #00ffff
};

// obj_id마다 고유한 색상을 동적 매핑
void Space::getDynamicColorById(int obj_id, glm::vec3& out_color) {
    // obj_id가 이미 등록된 경우, 저장된 색상(it->second)을 out_color에 복사하고 리턴
    auto it = objId_to_color.find(obj_id);
    if(it != objId_to_color.end()) {
        out_color = it->second;
        return;
    }

    // 새로운 obj_id에 색상 할당
    glm::vec3 color = color_table[next_color_idx % color_table.size()];
    objId_to_color[obj_id] = color;
    out_color = color;
    ++next_color_idx;
}

void Space::resetColorMap() {
    objId_to_color.clear();
    next_color_idx = 0;
}
#endif

// 좌측 상단 2D text 고정
std::string Space::colorToString(const glm::vec3& color) const{
    if(color == glm::vec3(1.0f, 0.0f, 0.0f)) return "red";
    if(color == glm::vec3(1.0f, 0.65f, 0.0f)) return "orange";
    if(color == glm::vec3(1.0f, 1.0f, 0.0f)) return "yellow";
    if(color == glm::vec3(1.0f, 0.75f, 0.8f)) return "pink";
    if(color == glm::vec3(0.58f, 0.0f, 0.83f)) return "dark violet";
    if(color == glm::vec3(1.0f, 0.0f, 1.0f)) return "magenta";
    if(color == glm::vec3(0.0f, 1.0f, 1.0f)) return "aqua";
    return "custom";
}

void Space::renderText2D(const std::string& text, float x, float y, const glm::vec3& color) const{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glColor3f(color.r, color.g, color.b);
    glRasterPos2f(x, WINDOW_HEIGHT - y);    // 좌측 상단 기준   (0, 0)이 좌하단
    for(char c : text) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}



#if BILLBOARD
std::string Space::getLabelFromId(int obj_id) {
    static std::unordered_map<int, std::string> id_to_label = {
        {0, "person"},          {1, "bicycle"},     {2, "car"},            {3, "motorcycle"},     {4, "airplane"},
        {5, "bus"},             {6, "train"},       {7, "truck"},          {8, "boat"},           {9, "traffic light"},
        {10, "fire hydrant"},   {11, "stop sign"},  {12, "parking meter"}, {13, "bench"},         {14, "bird"},
        {15, "cat"},            {16, "dog"},        {17, "horse"},         {18, "sheep"},         {19, "cow"},
        {20, "elephant"},       {21, "bear"},       {22, "zebra"},         {23, "giraffe"},       {24, "backpack"},
        {25, "umbrella"},       {26, "handbag"},    {27, "tie"},           {28, "suitcase"},      {29, "frisbee"},
        {30, "skis"},           {31, "snowboard"},  {32, "sports ball"},   {33, "kite"},          {34, "baseball bat"},
        {35, "baseball glove"}, {36, "skateboard"}, {37, "surfboard"},     {38, "tennis racket"}, {39, "bottle"},
        {40, "wine glass"},     {41, "cup"},        {42, "fork"},          {43, "knife"},         {44, "spoon"},
        {45, "bowl"},           {46, "banana"},     {47, "apple"},         {48, "sandwich"},      {49, "orange"},
        {50, "broccoli"},       {51, "carrot"},     {52, "hot dog"},       {53, "pizza"},         {54, "donut"},
        {55, "cake"},           {56, "chair"},      {57, "couch"},         {58, "potted plant"},  {59, "bed"},
        {60, "dining table"},   {61, "toilet"},     {62, "tv"},            {63, "laptop"},        {64, "mouse"},
        {65, "remote"},         {66, "keyboard"},   {67, "cell phone"},    {68, "microwave"},     {69, "oven"},
        {70, "toaster"},        {71, "sink"},       {72, "refrigerator"},  {73, "book"},          {74, "clock"},
        {75, "vase"},           {76, "scissors"},   {77, "teddy bear"},    {78, "hair drier"},    {79, "toothbrush"}
    };
    auto it = id_to_label.find(obj_id);
    return (it != id_to_label.end()) ? it->second : "unknown";
}

// obj_id -> texture_path 매핑
std::string Space::getLabelTexturePath(int obj_id) {
    static std::unordered_map<int, std::string> label_map = {
        {56, "../../label/chair.png"}
    };
    auto it = label_map.find(obj_id);
    if (it != label_map.end()) return it -> second;
    return "";
}

// 박스 위 위치 계산 후 addLabelBillboard 호출
void Space::addLabelBillboard(const glm::vec3& pos, const std::string& texture_path) {
    if(!texture_path.empty()) {
        Billboard b;
        b.position = pos;
        b.texture_path = texture_path;
        billboards.push_back(b);
    }
}

// renderBillboards()에서 카메라를 향하도록 평면 텍스처 렌더링
void Space::renderBillboards(const glm::vec3& cameraPos) {
    for(const auto& b : billboards) {
        glm::vec3 dir = glm::normalize(cameraPos - b.position);
        float angle = atan2(dir.y, dir.x);

        GLuint texID = loadTexture(b.texture_path);

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texID);
        glColor3f(1.0f, 1.0f, 1.0f);

        glPushMatrix();
        glTranslatef(b.position.x, b.position.y, b.position.z + 0.5f);  // 박스 위로 살짝 올림
        glRotatef(glm::degrees(angle), 0.0f, 0.0f, 1.0f);   // 카메라 방향으로 회전

        float w = b.size;
        float h = b.size * 0.3f;    // 텍스트는 긴 직사각형

        glBegin(GL_QUADS);
        glTexCoord2f(0, 0); glVertex3f(-w, -h, 0);
        glTexCoord2f(1, 0); glVertex3f(w, -h, 0);
        glTexCoord2f(1, 1); glVertex3f(w, h, 0);
        glTexCoord2f(0, 1); glVertex3f(-w, h, 0);
        glEnd();

        glPopMatrix();
        glDisable(GL_TEXTURE_2D);
    }
}


// 텍스처 로딩(텍스터가 여러 개라면 캐싱 필요)
static std::unordered_map<std::string, GLuint> textureCache;

GLuint Space::loadTexture(const std::string& path) {
    // 캐시에 존재하면 반환
    if(textureCache.count(path)) {
        return textureCache[path];
    }

    int width, height, channels;
    stbi_set_flip_vertically_on_load(true); // OpenGL 좌표계 기준 이미지 뒤집기

    // 이미지 로드(RGBA 강제 변환)
    unsigned char* image = stbi_load(path.c_str(), &width, &height, &channels, STBI_rgb_alpha);
    if(!image) {
        std::cerr << "Failed to load texture: " << path << std::endl;
        return 0;
    }

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // 텍스처 데이터 업로드
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);

    // 텍스처 파라미터 설정
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);   // 축소 시 선형 보간
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);   // 확대 시 선형 보간
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);    // 가장자리에서 반복 없음
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    stbi_image_free(image);     // 메모리 해제

    textureCache[path] = textureID;
    return textureID;
}

void Space::clearTextureCache() {
    for(auto& pair : textureCache) {
        glDeleteTextures(1, &pair.second);
    }
    textureCache.clear();
}
#endif