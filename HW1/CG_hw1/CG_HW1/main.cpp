#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <windows.h>
#include <GL/freeglut.h>

struct Pt {
    float x, y;
};
int winW = 800, winH = 600;

enum class DrawMode { Point, Line, Polyline, Circle, Freehand, Text };
DrawMode mode = DrawMode::Line;

struct Stroke { // 存圖形的資料結構
    std::vector<Pt> pts;
    float r = 0, g = 0, b = 0;
    DrawMode md = DrawMode::Point;
    float lineW = 2.f;
    float pointSize = 5.f;
    bool fill = false;
    std::string text;
    int fontId = 2;
};

std::vector<Stroke> strokes;
Stroke current;

float curColor[3] = { 1, 0, 0 };
float curLineWidth = 2.f;
float curPointSize = 5.f;
bool curFill = false;
int curFontId = 2;

bool drawing = false;

// ---- 儲存的東西 ----
static std::vector<unsigned char> savedBuf;  // RGB
static int savedW = 0, savedH = 0;
static bool hasSaved = false;
static bool autoSaveOn = false;  // 這個壞了
static bool canvasDirty = true;  // 這個壞了
static int snapshotId = 1;

// ---- Grid 的東西 ----
static bool showGrid = true;
static int gridStep = 50;

// 麻煩
void setOrtho2D(int w, int h) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, w, 0, h);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
static void* getFont(int id) {
    switch (id) {
    case 0:
        return GLUT_BITMAP_8_BY_13;
    case 1:
        return GLUT_BITMAP_9_BY_15;
    case 2:
        return GLUT_BITMAP_HELVETICA_18;
    case 3:
        return GLUT_BITMAP_TIMES_ROMAN_24;
    default:
        return GLUT_BITMAP_HELVETICA_18;
    }
}
static inline void drawTextAt(void* font, float x, float y, const std::string& s) {
    glRasterPos2f(x, y);
    for (unsigned char c : s) {
        if (c >= 32 && c <= 126) // ASCII code
            glutBitmapCharacter(font, c);
    }
}
static inline bool farEnough(const Pt& a, const Pt& b, float eps = 1.2f) { // 主要是 freehand 那邊要判
    float dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy >= eps * eps;
}
Pt screenToWorld(int x, int y) { return Pt{ (float)x, (float)(winH - y) }; }

// ---- 畫 grid ----
static void drawGrid() {
    if (!showGrid || gridStep <= 0)
        return;
    glDisable(GL_LINE_SMOOTH);

    glColor3f(0.9f, 0.9f, 0.9f);
    glLineWidth(1.f);
    glBegin(GL_LINES);
    for (int x = 0; x <= winW; x += gridStep) {
        glVertex2f((float)x, 0.f);
        glVertex2f((float)x, (float)winH);
    }
    for (int y = 0; y <= winH; y += gridStep) {
        glVertex2f(0.f, (float)y);
        glVertex2f((float)winW, (float)y);
    }
    glEnd();

    glColor3f(0.75f, 0.75f, 0.75f);
    glLineWidth(2.f);
    glBegin(GL_LINES);
    glVertex2f(0.f, 0.f);
    glVertex2f((float)winW, 0.f);
    glVertex2f(0.f, 0.f);
    glVertex2f(0.f, (float)winH);
    glEnd();

    glColor3f(0.5f, 0.5f, 0.5f);
    void* font = GLUT_BITMAP_8_BY_13;
    for (int x = gridStep * 2; x < winW; x += gridStep * 2)
        drawTextAt(font, (float)x + 2.f, 2.f, std::to_string(x));
    for (int y = gridStep * 2; y < winH; y += gridStep * 2)
        drawTextAt(font, 2.f, (float)y + 2.f, std::to_string(y));
}

// ---- 畫一筆 Stroke ----
static inline void drawStroke(const Stroke& s) {
    glColor3f(s.r, s.g, s.b);
    switch (s.md) {
    case DrawMode::Point: {
        glPointSize(s.pointSize);
        glBegin(GL_POINTS);
        for (auto& p : s.pts)
            glVertex2f(p.x, p.y);
        glEnd();
    } break;
    case DrawMode::Line: {
        if (s.pts.size() >= 2) {
            glLineWidth(s.lineW);
            glBegin(GL_LINES);
            glVertex2f(s.pts.front().x, s.pts.front().y);
            glVertex2f(s.pts.back().x, s.pts.back().y);
            glEnd();
        }
    } break;
    case DrawMode::Polyline: // 這裡要配合 motion 跟 mouse，搞了一下
    case DrawMode::Freehand: {
        if (s.pts.size() >= 2) {
            glLineWidth(s.lineW);
            glBegin(GL_LINE_STRIP);
            for (auto& p : s.pts)
                glVertex2f(p.x, p.y);
            glEnd();
        }
    } break;
    case DrawMode::Circle: {
        if (s.pts.size() >= 2) {
            Pt c = s.pts.front(), q = s.pts.back();
            float r = std::hypot(q.x - c.x, q.y - c.y);
            const int N = 64;
            if (s.fill) {
                glBegin(GL_TRIANGLE_FAN);
                glVertex2f(c.x, c.y);
                for (int i = 0; i <= N; ++i) {
                    float a = 2.f * 3.1415926f * i / N; // pi 徑
                    glVertex2f(c.x + r * std::cos(a), c.y + r * std::sin(a));
                }
                glEnd();
            }
            glLineWidth(s.lineW);
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < N; ++i) {
                float a = 2.f * 3.1415926f * i / N; // pi 徑
                glVertex2f(c.x + r * std::cos(a), c.y + r * std::sin(a));
            }
            glEnd();
        }
    } break;
    case DrawMode::Text: {
        if (!s.pts.empty() && !s.text.empty()) {
            drawTextAt(getFont(s.fontId), s.pts.front().x, s.pts.front().y, s.text);
        }
    } break;
    }
}

// 這邊就確保他的大小是對的，這裡搞了一下
static void ensureSavedBufSize() {
    if (savedW != winW || savedH != winH) {
        savedW = winW;
        savedH = winH;
        savedBuf.assign(savedW * savedH * 3, 255);
        hasSaved = false;
    }
}
static void captureFrontToSaved() {  // 只在沒有暫存筆畫時呼叫
    ensureSavedBufSize();
    glReadBuffer(GL_FRONT);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glFinish();  // 確保繪製完成
    glReadPixels(0, 0, savedW, savedH, GL_RGB, GL_UNSIGNED_BYTE, savedBuf.data());
    hasSaved = true;
}
static bool savePPM(const std::string& filename) {
    if (!hasSaved || savedW <= 0 || savedH <= 0)
        return false;
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs)
        return false;
    ofs << "P6\n" << savedW << " " << savedH << "\n255\n";
    int stride = savedW * 3;
    for (int row = savedH - 1; row >= 0; --row) { // 這裡要小心 OpenGL 的座標是反的
        ofs.write(reinterpret_cast<const char*>(savedBuf.data() + row * stride), stride);
    }
    return true;
}
static std::string nextSnap() {
    std::ostringstream oss;
    oss << "snapshot_" << std::setfill('0') << std::setw(3) << snapshotId++ << ".ppm";
    return oss.str();
}

// ---- GLUT 回叫 ----
void display() {
    // 每幀流程：清空 -> 畫格線 -> 畫既有筆畫 -> 畫暫存筆畫
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();

    drawGrid();

    for (auto& s : strokes)
        drawStroke(s);
    if (!current.pts.empty())
        drawStroke(current);

    glutSwapBuffers();

    // 這裡壞了
    if (autoSaveOn && canvasDirty && current.pts.empty() && !drawing) {
        captureFrontToSaved();
        canvasDirty = false;
    }
}
void reshape(int w, int h) {
    winW = w;
    winH = h;
    glViewport(0, 0, w, h);
    setOrtho2D(w, h); // 這在搞我
    ensureSavedBufSize();
    glutPostRedisplay();
}

void startNewStroke(DrawMode md, const Pt& p) { // 當開始拖曳的時候，要搞點事
    drawing = true;
    current = Stroke{};
    current.r = curColor[0];
    current.g = curColor[1];
    current.b = curColor[2];
    current.md = md;
    current.lineW = curLineWidth;
    current.pointSize = curPointSize;
    current.fill = curFill;
    current.fontId = curFontId;
    current.pts.push_back(p);
    if (md == DrawMode::Line || md == DrawMode::Circle)
        current.pts.push_back(p);
    if (md == DrawMode::Text)
        current.text.clear();
}

void finishCurrentIfNeeded(bool pushEvenIfEmpty = false) {
    bool pushed = false;
    if (current.md == DrawMode::Polyline) {
        if (current.pts.size() >= 2) {
            // 去掉最後一個臨時的點
            if (current.pts.size() >= 2) {
                auto& a = current.pts[current.pts.size() - 1], & b = current.pts[current.pts.size() - 2];
                if (a.x == b.x && a.y == b.y)
                    current.pts.pop_back();
            }
            if (current.pts.size() >= 2) {
                strokes.push_back(current);
                pushed = true;
            }
        }
        current.pts.clear();
    }
    else if (current.md == DrawMode::Text) {
        if (!current.text.empty() || pushEvenIfEmpty) {
            strokes.push_back(current);
            pushed = true;
        }
        current.pts.clear();
        current.text.clear();
    }
    else {
        if (!current.pts.empty()) {
            strokes.push_back(current);
            pushed = true;
            current.pts.clear();
        }
    }
    drawing = false;
    if (pushed) {
        canvasDirty = true;
    }  // 有存才變髒
    glutPostRedisplay();
}

void keyboard(unsigned char key, int, int) {
    if (key == 27) {
        glutLeaveMainLoop();
        return;
    }
    if (key == 13 || key == '\n') {  // 完成 polyline 或 text
        if (current.md == DrawMode::Polyline && !current.pts.empty()) {
            finishCurrentIfNeeded();
            return;
        }
        if (current.md == DrawMode::Text && !current.pts.empty()) {
            if (!current.text.empty())
                strokes.push_back(current), canvasDirty = true;
            current.pts.clear();
            current.text.clear();
            glutPostRedisplay();
            return;
        }
    }
    // 痾痾阿阿換行的偵測要放在 text 之前
    if (current.md == DrawMode::Text && !current.pts.empty()) {
        if (key == 8 || key == 127) {
            if (!current.text.empty())
                current.text.pop_back();
        }
        else if (key >= 32 && key <= 126) { // ASCII code
            current.text.push_back((char)key);
        }
        glutPostRedisplay();
        return;
    }
    if (key == 'c' || key == 'C') {
        strokes.clear();
        current.pts.clear();
        current.text.clear();
        canvasDirty = true;
        glutPostRedisplay();
        return;
    }
    if (key == 'g' || key == 'G') {
        showGrid = !showGrid;
        glutPostRedisplay();
        return;
    }
    //if (key == 'a' || key == 'A') { // 壞了別理他
        //autoSaveOn = !autoSaveOn;
        //return;
    //}
    if (key == 's' || key == 'S') {
        // 若剛存過，savedBuf 已是最新；否則就先抓一張看看
        if (!hasSaved || canvasDirty) {
            if (current.pts.empty() && !drawing) {
                captureFrontToSaved();
                canvasDirty = false;
            }
        }
        std::string fn = nextSnap();
        if (savePPM(fn))
            std::cout << "Saved: " << fn << "\n";
        else
            std::cerr << "Save failed.\n";
        return;
    }
}

void mouse(int button, int state, int x, int y) {
    if (button != GLUT_LEFT_BUTTON)
        return;
    Pt p = screenToWorld(x, y); // ㄦㄦㄦㄦㄦ ㄦ 好煩
    if (state == GLUT_DOWN) {
        if (mode == DrawMode::Polyline) {
            if (current.pts.empty() || current.md != DrawMode::Polyline) {
                drawing = true;
                current = Stroke{};
                current.r = curColor[0];
                current.g = curColor[1];
                current.b = curColor[2];
                current.md = DrawMode::Polyline;
                current.lineW = curLineWidth;
                current.pointSize = curPointSize;
                current.fill = curFill;
                current.fontId = curFontId;
                current.pts.push_back(p);
                current.pts.push_back(p);
            }
            else {
                current.pts.back() = p;
                current.pts.push_back(p);
                drawing = true;
            }
        }
        else if (mode == DrawMode::Text) {
            startNewStroke(DrawMode::Text, p);
            drawing = false;  // 啊ㄚㄚㄚㄚ 
            glutPostRedisplay();
        }
        else {
            startNewStroke(mode, p);
        }
    }
    else if (state == GLUT_UP) {
        if (mode == DrawMode::Polyline || mode == DrawMode::Text) {
            glutPostRedisplay();
        }
        else {
            drawing = false;
            if (!current.pts.empty()) {
                if ((current.md == DrawMode::Line || current.md == DrawMode::Circle) && current.pts.size() == 1)
                    current.pts.push_back(p);
                if (current.md != DrawMode::Polyline && current.md != DrawMode::Text) {
                    strokes.push_back(current);
                    current.pts.clear();
                    canvasDirty = true;  // 已存
                }
                glutPostRedisplay();
            }
        }
    }
}

void motion(int x, int y) {
    if (!drawing)
        return;
    Pt p = screenToWorld(x, y);
    if (current.md == DrawMode::Freehand) {
        if (current.pts.empty())
            current.pts.push_back(p);
        else if (farEnough(current.pts.back(), p))
            current.pts.push_back(p);
    }
    else if (current.md == DrawMode::Polyline) {
        if (current.pts.size() >= 2)
            current.pts.back() = p;
    }
    else if (current.md == DrawMode::Point) {
        current.pts.push_back(p);
    }
    else if (current.md == DrawMode::Line || current.md == DrawMode::Circle) {
        if (current.pts.size() == 1)
            current.pts.push_back(p);
        else
            current.pts.back() = p;
    }
    glutPostRedisplay();
}

// ---- 給 manual 的一坨功能編號 ----
enum MenuEntry {
    M_POINT = 1,
    M_LINE,
    M_POLYLINE,
    M_CIRCLE,
    M_FREEHAND,
    M_TEXT,
    C_RED = 101,
    C_GREEN,
    C_BLUE,
    C_BLACK,
    LW_1 = 201,
    LW_2,
    LW_4,
    LW_8,
    PS_3 = 301,
    PS_5,
    PS_9,
    PS_13,
    F_OUTLINE = 401,
    F_FILL,
    FONT_8x13 = 501,
    FONT_9x15,
    FONT_HELV18,
    FONT_TIMES24,
    ADV_AUTOSAVE_ON = 601,
    ADV_AUTOSAVE_OFF,
    ADV_SAVE_NOW,
    ADV_GRID_ON,
    ADV_GRID_OFF,
    ADV_GRID_STEP25,
    ADV_GRID_STEP50,
    ADV_GRID_STEP100
};
void menuMain(int item) { // 從 manual 切換不同的模式
    switch (item) {
    case M_POINT:
        mode = DrawMode::Point;
        break;
    case M_LINE:
        mode = DrawMode::Line;
        break;
    case M_POLYLINE:
        mode = DrawMode::Polyline;
        break;
    case M_CIRCLE:
        mode = DrawMode::Circle;
        break;
    case M_FREEHAND:
        mode = DrawMode::Freehand;
        break;
    case M_TEXT:
        mode = DrawMode::Text;
        break;
    }
}
void menuColor(int item) { // manual 改變顏色
    if (item == C_RED) {
        curColor[0] = 1;
        curColor[1] = 0;
        curColor[2] = 0;
    }
    if (item == C_GREEN) {
        curColor[0] = 0;
        curColor[1] = 1;
        curColor[2] = 0;
    }
    if (item == C_BLUE) {
        curColor[0] = 0;
        curColor[1] = 0;
        curColor[2] = 1;
    }
    if (item == C_BLACK) {
        curColor[0] = 0;
        curColor[1] = 0;
        curColor[2] = 0;
    }
    glutPostRedisplay();
}
void menuLineWidth(int item) { // manual 改變線的粗細
    if (item == LW_1)
        curLineWidth = 1.f;
    if (item == LW_2)
        curLineWidth = 2.f;
    if (item == LW_4)
        curLineWidth = 4.f;
    if (item == LW_8)
        curLineWidth = 8.f;
    if (!current.pts.empty()) {
        current.lineW = curLineWidth;
        glutPostRedisplay();
    }
}
void menuPointSize(int item) { // manual 改變點的大小
    if (item == PS_3)
        curPointSize = 3.f;
    if (item == PS_5)
        curPointSize = 5.f;
    if (item == PS_9)
        curPointSize = 9.f;
    if (item == PS_13)
        curPointSize = 13.f;
    if (!current.pts.empty()) {
        current.pointSize = curPointSize;
        glutPostRedisplay();
    }
}
void menuFill(int item) { // manual 改變圓是否要填滿
    if (item == F_OUTLINE)
        curFill = false;
    if (item == F_FILL)
        curFill = true;
    if (!current.pts.empty()) {
        current.fill = curFill;
        glutPostRedisplay();
    }
}
void menuFont(int item) { // manual 改變字體
    if (item == FONT_8x13)
        curFontId = 0;
    if (item == FONT_9x15)
        curFontId = 1;
    if (item == FONT_HELV18)
        curFontId = 2;
    if (item == FONT_TIMES24)
        curFontId = 3;
    if (!current.pts.empty() && current.md == DrawMode::Text) {
        current.fontId = curFontId;
        glutPostRedisplay();
    }
}
void menuAdvanced(int item) { // manual 改其他東西
    switch (item) {
    case ADV_AUTOSAVE_ON:
        autoSaveOn = true;
        break;
    case ADV_AUTOSAVE_OFF:
        autoSaveOn = false;
        break;
    case ADV_SAVE_NOW: {
        if (current.pts.empty() && !drawing) {
            captureFrontToSaved();
            canvasDirty = false;
        }
        std::string fn = nextSnap();
        if (savePPM(fn))
            std::cout << "Saved: " << fn << "\n";
        else
            std::cerr << "Save failed.\n";
    } break;
    case ADV_GRID_ON:
        showGrid = true;
        glutPostRedisplay();
        break;
    case ADV_GRID_OFF:
        showGrid = false;
        glutPostRedisplay();
        break;
    case ADV_GRID_STEP25:
        gridStep = 25;
        glutPostRedisplay();
        break;
    case ADV_GRID_STEP50:
        gridStep = 50;
        glutPostRedisplay();
        break;
    case ADV_GRID_STEP100:
        gridStep = 100;
        glutPostRedisplay();
        break;
    }
}
void buildMenus() { // manual 在這裡
    int colorMenu = glutCreateMenu(menuColor);
    glutAddMenuEntry("Red", C_RED);
    glutAddMenuEntry("Green", C_GREEN);
    glutAddMenuEntry("Blue", C_BLUE);
    glutAddMenuEntry("Black", C_BLACK);

    int lwMenu = glutCreateMenu(menuLineWidth);
    glutAddMenuEntry("1 px", LW_1);
    glutAddMenuEntry("2 px", LW_2);
    glutAddMenuEntry("4 px", LW_4);
    glutAddMenuEntry("8 px", LW_8);

    int psMenu = glutCreateMenu(menuPointSize);
    glutAddMenuEntry("3 px", PS_3);
    glutAddMenuEntry("5 px", PS_5);
    glutAddMenuEntry("9 px", PS_9);
    glutAddMenuEntry("13 px", PS_13);

    int fillMenu = glutCreateMenu(menuFill);
    glutAddMenuEntry("Outline", F_OUTLINE);
    glutAddMenuEntry("Fill", F_FILL);

    int fontMenu = glutCreateMenu(menuFont);
    glutAddMenuEntry("Bitmap 8x13", FONT_8x13);
    glutAddMenuEntry("Bitmap 9x15", FONT_9x15);
    glutAddMenuEntry("Helvetica 18", FONT_HELV18);
    glutAddMenuEntry("Times Roman 24", FONT_TIMES24);

    int advMenu = glutCreateMenu(menuAdvanced);
    //glutAddMenuEntry("Auto Save ON", ADV_AUTOSAVE_ON); // 壞了不要理他
    //glutAddMenuEntry("Auto Save OFF", ADV_AUTOSAVE_OFF);
    glutAddMenuEntry("Save Now (PPM)", ADV_SAVE_NOW);
    glutAddMenuEntry("----------------", 0);
    glutAddMenuEntry("Grid ON", ADV_GRID_ON);
    glutAddMenuEntry("Grid OFF", ADV_GRID_OFF);
    glutAddMenuEntry("Grid Step 25", ADV_GRID_STEP25);
    glutAddMenuEntry("Grid Step 50", ADV_GRID_STEP50);
    glutAddMenuEntry("Grid Step 100", ADV_GRID_STEP100);

    int mainMenu = glutCreateMenu(menuMain);
    glutAddMenuEntry("Point", M_POINT);
    glutAddMenuEntry("Line", M_LINE);
    glutAddMenuEntry("Polyline (Enter to finish)", M_POLYLINE);
    glutAddMenuEntry("Circle", M_CIRCLE);
    glutAddMenuEntry("Freehand", M_FREEHAND);
    glutAddMenuEntry("Text (Enter to finish)", M_TEXT);
    glutAddSubMenu("Color", colorMenu);
    glutAddSubMenu("Line Width", lwMenu);
    glutAddSubMenu("Point Size", psMenu);
    glutAddSubMenu("Fill Mode", fillMenu);
    glutAddSubMenu("Text Font", fontMenu);
    glutAddSubMenu("Advanced", advMenu);

    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(winW, winH);
    glutCreateWindow("Drawing Panel");

    glClearColor(1, 1, 1, 1);
    setOrtho2D(winW, winH);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);

    buildMenus();

    glutMainLoop();
    return 0;
}
