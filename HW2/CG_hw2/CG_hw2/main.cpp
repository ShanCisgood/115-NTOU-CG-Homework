/***************************************************
 * Hw2: A car in a scene
 *   - Floor with ramps and flats
 *   - A car (body + 4 wheels)
 *   - Some simple buildings
 *   - Car can move forward/backward, turn left/right
 *   - Avoid obstacles (buildings, map bounds)
 *   - Go uphill/downhill (height from ramps)
 *
 * Controls:
 *   W/S : forward / backward
 *   A/D : turn left / right
 *   R   : reset car to start
 *   1/2 : polygon mode fill/line
 *   V   : Standby mode
 *   Q   : quit
 *
 * Coordinate system:
 *   Floor is x-z plane, y is vertical
 ***************************************************/
#include <GL/glut.h>
#include <math.h>
#include <stdio.h>

#include <cstdlib>
#include <ctime>

#define PI 3.14159265358979323846

 /*==================== Global State ====================*/
static int gWinW = 1000, gWinH = 800;
static int gPolyModeFill = 1;

/* World bounds: opposite corners (0,0,0) and (100,0,100). */
static const float WORLD_MIN_X = 0.0f, WORLD_MAX_X = 100.0f;
static const float WORLD_MIN_Z = 0.0f, WORLD_MAX_Z = 100.0f;

/* ---- Camera control ---- */
static int cam_edit_mode = 0; /* 0: driving (default), 1: camera edit */
static float cam_eye_x = 95.0f, cam_eye_y = 70.0f, cam_eye_z = 95.0f;
static float cam_ctr_x = 45.0f, cam_ctr_y = 0.0f, cam_ctr_z = 45.0f;
static float cam_move_speed = 2.0f; /* movement per key press in camera-edit mode */

/* ---- Car state ---- */
static float car_x = 10.0f;
static float car_z = 10.0f;
static float car_y = 0.0f;       /* sampled from terrain */
static float car_dir_deg = 0.0f; /* 0 means +x direction */
static float car_speed = 0.6f;   /* movement per key press */
static float car_turn = 6.0f;    /* degrees per key press */

/* Car size (for simple collision & placement) */
static const float CAR_HALF_L = 4.0f; /* half length in local +x/-x */
static const float CAR_HALF_W = 2.0f; /* half width  in local +z/-z */
static const float CAR_BODY_H = 2.0f;

/* GLU quadrics */
static GLUquadric* gCyl = NULL, * gSphere = NULL;
static const float EPS = 0.06f; /* 原本 0.03f -> 0.06f */

// ---- Screensaver mode (bouncing viewport) ----
static bool screensaver_mode = false;
static int vp_x = 0, vp_y = 0, vp_w = 0, vp_h = 0;
static float vp_vx = 540.0f, vp_vy = 540.0f;        // 每秒像素速度
static unsigned int rng_seed = 9487u;               // 用來決定初始角落與方向
extern int gWinW, gWinH;

/*==================== Simple Terrain ====================*
 * Floor tiles are at y = -2.5 as baseline
 */
static const float BASE_Y = -2.5f;

typedef struct {
    float x0, z0, x1, z1;
    float riseY;
    int alongX;       /* 1: 高度沿 X 線性變化 (xmin -> xmax); 0: 沿 Z (zmin -> zmax) */
    float cr, cg, cb; /* 顏色 */
} Ramp;

/* 幾個簡單斜坡: 矩形區域，沿 X 或 Z 線性升高 */
static Ramp gRamps[] = {
    /*  x0   z0   x1   z1   rise   alongX   r     g     b  */
    {15.0f, 15.0f, 35.0f, 25.0f, 10.0f, 1, 0.90f, 0.55f, 0.20f}, /* 橘色 */
    {55.0f, 20.0f, 70.0f, 35.0f, 6.0f,  0, 0.30f, 0.80f, 0.70f}, /* 藍綠 */
    {30.0f, 60.0f, 50.0f, 80.0f, 8.0f,  1, 0.70f, 0.60f, 0.95f}, /* 淡紫 */
};
static const int gNumRamps = sizeof(gRamps) / sizeof(gRamps[0]);

/* 高度查詢: baseline + 若落入某個 ramp，就做線性內插升高 */
static float terrain_height(float x, float z) {
    float y = BASE_Y;
    for (int i = 0; i < gNumRamps; ++i) {
        Ramp r = gRamps[i];
        float xmin = fminf(r.x0, r.x1), xmax = fmaxf(r.x0, r.x1);
        float zmin = fminf(r.z0, r.z1), zmax = fmaxf(r.z0, r.z1);
        if (x >= xmin && x <= xmax && z >= zmin && z <= zmax) {
            if (r.alongX) {
                float t = (x - xmin) / (xmax - xmin); /* 0..1 */
                y += r.riseY * t;
            }
            else {
                float t = (z - zmin) / (zmax - zmin); /* 0..1 */
                y += r.riseY * t;
            }
        }
    }
    return y;
}

/* 大概的坡度  */
static void terrain_gradient(float x, float z, float* dydx, float* dydz) {
    const float eps = 0.25f;
    float yx1 = terrain_height(x + eps, z);
    float yx0 = terrain_height(x - eps, z);
    float yz1 = terrain_height(x, z + eps);
    float yz0 = terrain_height(x, z - eps);
    *dydx = (yx1 - yx0) / (2.0f * eps);
    *dydz = (yz1 - yz0) / (2.0f * eps);
}

/*==================== Buildings / Obstacles ====================*
 * 幾棟簡單長方體建築，拿來當障礙物；用 AABB 做簡單碰撞避免。
 */
typedef struct {
    float x0, z0, x1, z1;
    float h;
} Building;

static Building gBlds[] = {
    {40.0f, 0.0f,  50.0f, 10.0f, 20.0f}, // 50
    {70.0f, 55.0f, 85.0f, 75.0f, 25.0f},
    {18.0f, 40.0f, 26.0f, 55.0f, 15.0f},
};
static const int gNumBlds = sizeof(gBlds) / sizeof(gBlds[0]);

/* 檢查新位置是否「碰到建築物的水平投影」；給一點保險邊界 */
static int hit_building_xy(float x, float z, float inflate) {
    for (int i = 0; i < gNumBlds; ++i) {
        float xmin = fminf(gBlds[i].x0, gBlds[i].x1) - inflate;
        float xmax = fmaxf(gBlds[i].x0, gBlds[i].x1) + inflate;
        float zmin = fminf(gBlds[i].z0, gBlds[i].z1) - inflate;
        float zmax = fmaxf(gBlds[i].z0, gBlds[i].z1) + inflate;
        if (x >= xmin && x <= xmax && z >= zmin && z <= zmax)
            return 1;
    }
    return 0;
}

/*==================== Drawing Helpers ====================*/
static void ensure_quadrics(void) {
    if (!gCyl) {
        gCyl = gluNewQuadric();
        gluQuadricDrawStyle(gCyl, GLU_FILL);
        gluQuadricNormals(gCyl, GLU_SMOOTH);
    }
    if (!gSphere) {
        gSphere = gluNewQuadric();
        gluQuadricDrawStyle(gSphere, GLU_FILL);
        gluQuadricNormals(gSphere, GLU_SMOOTH);
    }
}

static void draw_unit_cube(void) {
    /* 一個 1x1x1 立方體，中心在原點 */
    static const GLfloat v[8][3] = {
        {-0.5f, -0.5f, -0.5f},
        {0.5f,  -0.5f, -0.5f},
        {0.5f,  0.5f,  -0.5f},
        {-0.5f, 0.5f,  -0.5f},
        {-0.5f, -0.5f, 0.5f },
        {0.5f,  -0.5f, 0.5f },
        {0.5f,  0.5f,  0.5f },
        {-0.5f, 0.5f,  0.5f }
    };
    static const GLuint f[6][4] = {
        {0, 1, 2, 3},
        {4, 5, 6, 7},
        {0, 4, 5, 1},
        {1, 5, 6, 2},
        {2, 6, 7, 3},
        {0, 3, 7, 4}
    };
    static const GLfloat c[6][3] = {
        {0.8f, 0.4f, 0.4f},
        {0.4f, 0.8f, 0.4f},
        {0.4f, 0.4f, 0.8f},
        {0.8f, 0.8f, 0.4f},
        {0.8f, 0.4f, 0.8f},
        {0.4f, 0.8f, 0.8f}
    };
    for (int i = 0; i < 6; ++i) {
        glColor3fv(c[i]);
        glBegin(GL_POLYGON);
        glVertex3fv(v[f[i][0]]);
        glVertex3fv(v[f[i][1]]);
        glVertex3fv(v[f[i][2]]);
        glVertex3fv(v[f[i][3]]);
        glEnd();
    }
}

static void draw_floor_tiles(void) {
    /* 10x10 tiles, 覆蓋 0..100 */
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            if ((i + j) % 2 == 0)
                glColor3f(1.0f, 0.8f, 0.8f);
            else
                glColor3f(0.1f, 0.1f, 0.7f);
            float x0 = i * 10.0f, x1 = (i + 1) * 10.0f;
            float z0 = j * 10.0f, z1 = (j + 1) * 10.0f;
            /* 讓每塊磚都在當地高度 BASE_Y */
            glBegin(GL_POLYGON);
            glVertex3f(x0, BASE_Y - EPS, z0);
            glVertex3f(x0, BASE_Y - EPS, z1);
            glVertex3f(x1, BASE_Y - EPS, z1);
            glVertex3f(x1, BASE_Y - EPS, z0);

            glEnd();
        }
    }
}

/* 斜面畫斜線紋 (視覺指示): 沿 X 的斜面 (位於 z = zmax) */
static void draw_slope_hatching_alongX(float xmin, float xmax, float zmax, float y0, float y1, int stripes) {
    glLineWidth(1.5f);
    glBegin(GL_LINES);
    for (int i = 0; i <= stripes; ++i) {
        float t = (float)i / (float)stripes; /* 0..1 */
        float x = xmin + t * (xmax - xmin);
        float y = y0 + t * (y1 - y0);
        glVertex3f(x, y, zmax);
        glVertex3f(x, y + 0.4f, zmax);
    }
    glEnd();
}

/* 斜面畫斜線紋: 沿 Z 的斜面 (位於 x = xmax) */
static void draw_slope_hatching_alongZ(float zmin, float zmax, float xmax, float y0, float y1, int stripes) {
    glLineWidth(1.5f);
    glBegin(GL_LINES);
    for (int i = 0; i <= stripes; ++i) {
        float t = (float)i / (float)stripes;
        float z = zmin + t * (zmax - zmin);
        float y = y0 + t * (y1 - y0);
        glVertex3f(xmax, y, z);
        glVertex3f(xmax, y + 0.4f, z);
    }
    glEnd();
}

static void draw_ramps(void) {
    GLboolean cull_save = glIsEnabled(GL_CULL_FACE);
    GLboolean blend_save = glIsEnabled(GL_BLEND);
    GLboolean po_fill_save = glIsEnabled(GL_POLYGON_OFFSET_FILL);

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glDisable(GL_POLYGON_OFFSET_FILL);

    for (int i = 0; i < gNumRamps; ++i) {
        const Ramp r = gRamps[i];
        const float xmin = fminf(r.x0, r.x1), xmax = fmaxf(r.x0, r.x1);
        const float zmin = fminf(r.z0, r.z1), zmax = fmaxf(r.z0, r.z1);

        const float y0 = BASE_Y + EPS;            // 與地板錯開
        const float y1 = BASE_Y + r.riseY + EPS;  // 頂端

        const float R = r.cr, G = r.cg, B = r.cb;
        const float Rdim = R * 0.7f, Gdim = G * 0.7f, Bdim = B * 0.7f;

        if (r.alongX) {
            /* -------- 沿 X 升高: 斜頂在 z = zmax -------- */

            /* 底面 (固定高度 y0) */
            glColor3f(Rdim, Gdim, Bdim);
            glBegin(GL_QUADS);
            glVertex3f(xmin, y0, zmin);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmax, y0, zmin);
            glEnd();

            /* 背面牆 (頂端 x = xmax，從 y0 連到 y1) */
            glBegin(GL_QUADS);
            glVertex3f(xmax, y0, zmin);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmax, y1, zmin);
            glEnd();

            /* 側面牆 (z = zmin，從底面連到背面) */
            glBegin(GL_QUADS);
            glVertex3f(xmin, y0, zmin);
            glVertex3f(xmax, y0, zmin);
            glVertex3f(xmax, y1, zmin);
            glVertex3f(xmin, y0, zmin);
            glEnd();

            /* 斜頂面 (z = zmax，y 隨 x 線性增加) */
            glColor3f(R, G, B);
            glBegin(GL_TRIANGLES);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmax, y1, zmax);

            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmin, y0, zmax);
            glEnd();

            glBegin(GL_TRIANGLES);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmin, y0, zmax);
            glEnd();

            // 兩個三角形
            glBegin(GL_TRIANGLES);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glEnd();
            glBegin(GL_TRIANGLES);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmin, y0, zmax);
            glEnd();

            glColor3f(R, G, B);
            glBegin(GL_TRIANGLES);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glEnd();
            glBegin(GL_TRIANGLES);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmin, y0, zmax + 0.00001f);  // 微小擾動，避免共線退化
            glEnd();

            /* 外框與斜線紋 */
            glLineWidth(3.0f);
            glColor3f(0, 0, 0);
            glBegin(GL_LINE_LOOP);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmin, y0, zmax);
            glEnd();
            glColor3f(0, 0, 0);
            draw_slope_hatching_alongX(xmin, xmax, zmax, y0, y1, 10);
        }
        else {
            /* -------- 沿 Z 升高: 斜頂在 x = xmax -------- */

            /* 底面 */
            glColor3f(Rdim, Gdim, Bdim);
            glBegin(GL_QUADS);
            glVertex3f(xmin, y0, zmin);
            glVertex3f(xmax, y0, zmin);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmin, y0, zmax);
            glEnd();

            /* 背面牆 (高端 z = zmax) */
            glBegin(GL_QUADS);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmin, y0, zmax);
            glEnd();

            /* 側面牆 (x = xmin) */
            glBegin(GL_QUADS);
            glVertex3f(xmin, y0, zmin);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmin, y1, zmax);
            glVertex3f(xmin, y0, zmin);
            glEnd();

            /* 斜頂面 (x = xmax) */
            glColor3f(R, G, B);
            glBegin(GL_TRIANGLES);
            glVertex3f(xmax, y0, zmin);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glEnd();
            glBegin(GL_TRIANGLES);
            glVertex3f(xmax, y0, zmin);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmax, y0, zmin + 0.00001f);
            glEnd();

            /* 外框與斜線紋 */
            glLineWidth(3.0f);
            glColor3f(0, 0, 0);
            glBegin(GL_LINE_LOOP);
            glVertex3f(xmax, y0, zmin);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmax, y0, zmin);
            glEnd();
            glColor3f(0, 0, 0);
            draw_slope_hatching_alongZ(zmin, zmax, xmax, y0, y1, 10);
        }
    }

    if (cull_save)
        glEnable(GL_CULL_FACE);
    else
        glDisable(GL_CULL_FACE);
    if (blend_save)
        glEnable(GL_BLEND);
    else
        glDisable(GL_BLEND);
    if (po_fill_save)
        glEnable(GL_POLYGON_OFFSET_FILL);
    else
        glDisable(GL_POLYGON_OFFSET_FILL);
    glLineWidth(1.0f);
}

static void draw_buildings(void) {
    for (int i = 0; i < gNumBlds; ++i) {
        float xmin = fminf(gBlds[i].x0, gBlds[i].x1);
        float xmax = fmaxf(gBlds[i].x0, gBlds[i].x1);
        float zmin = fminf(gBlds[i].z0, gBlds[i].z1);
        float zmax = fmaxf(gBlds[i].z0, gBlds[i].z1);
        float y0 = terrain_height(0.5f * (xmin + xmax), 0.5f * (zmin + zmax));
        float h = gBlds[i].h;

        /* 用縮放後的單位方塊畫出一棟建築 */
        glPushMatrix();
        glTranslatef(0.5f * (xmin + xmax), y0 + h * 0.5f, 0.5f * (zmin + zmax));
        glScalef((xmax - xmin), h, (zmax - zmin));
        glColor3f(0.75f, 0.75f, 0.75f);
        draw_unit_cube();
        glPopMatrix();
    }
}

static void draw_world_axes_at(float x, float y, float z, float scale) {
    ensure_quadrics();
    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(scale, scale, scale);
    /* origin sphere */
    glColor3f(1, 1, 1);
    gluSphere(gSphere, 0.5, 12, 12);
    /* Z (blue) */
    glColor3f(0, 0, 1);
    gluCylinder(gCyl, 0.1, 0.1, 4.0, 12, 1);
    /* Y (green) */
    glPushMatrix();
    glRotatef(-90, 1, 0, 0);
    glColor3f(0, 1, 0);
    gluCylinder(gCyl, 0.1, 0.1, 4.0, 12, 1);
    glPopMatrix();
    /* X (red) */
    glPushMatrix();
    glRotatef(90, 0, 1, 0);
    glColor3f(1, 0, 0);
    gluCylinder(gCyl, 0.1, 0.1, 4.0, 12, 1);
    glPopMatrix();
    glPopMatrix();
}

static void draw_car(void) {
    ensure_quadrics();

    /* 車身 */
    glPushMatrix();
    glScalef(CAR_HALF_L * 2.0f, CAR_BODY_H, CAR_HALF_W * 2.0f); // 長 * 高 * 寬
    glColor3f(0.9f, 0.2f, 0.2f);
    draw_unit_cube();
    glPopMatrix();

    /* 四顆輪胎 */
    glColor3f(0.1f, 0.1f, 0.1f);
    float wy = -CAR_BODY_H * 0.5f + 0.8f; // 輪胎中心高度 (讓它看起來卡在地面上)
    float dx = CAR_HALF_L - 1.0f;
    float dz = CAR_HALF_W + 0.6f;

    glPushMatrix();
    glTranslatef(-dx, wy, dz);
    glutSolidTorus(0.5, 1.0, 24, 24);
    glPopMatrix(); // 左前
    glPushMatrix();
    glTranslatef(-dx, wy, -dz);
    glutSolidTorus(0.5, 1.0, 24, 24);
    glPopMatrix(); // 右前
    glPushMatrix();
    glTranslatef(dx, wy, dz);
    glutSolidTorus(0.5, 1.0, 24, 24);
    glPopMatrix(); // 左後
    glPushMatrix();
    glTranslatef(dx, wy, -dz);
    glutSolidTorus(0.5, 1.0, 24, 24);
    glPopMatrix(); // 右後

    draw_world_axes_at(0.0f, CAR_BODY_H * 0.6f, 0.0f, 0.6f);
}

/*==================== GLUT Callbacks ====================*/
static void setup_camera_and_projection(void) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-100.0, 100.0, -100.0, 100.0, -200.0, 200.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(cam_eye_x, cam_eye_y, cam_eye_z, cam_ctr_x, cam_ctr_y, cam_ctr_z, 0.0, 1.0, 0.0);
}

static void my_init(void) {
    glClearColor(0, 0, 0, 1);
    glEnable(GL_DEPTH_TEST);
    ensure_quadrics();
}

static void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(vp_x, vp_y, (vp_w > 0 ? vp_w : gWinW), (vp_h > 0 ? vp_h : gWinH));
    setup_camera_and_projection();

    // Polygon mode
    if (gPolyModeFill)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // 畫地板磚＋斜坡＋建築
    draw_floor_tiles();
    draw_ramps();
    draw_buildings();

    draw_world_axes_at(5.0f, terrain_height(5.0f, 5.0f) + 1.0f, 5.0f, 1.2f);

    // 車子的姿態: 位置 (car_x, car_y, car_z) + 朝向 car_dir_deg
    // 根據地形梯度給 pitch (前後傾)與 roll (左右傾)一點點角度
    float dydx = 0.0f, dydz = 0.0f;
    terrain_gradient(car_x, car_z, &dydx, &dydz);

    /* 先算車頭方向 (forward) 與右手方向 (right) (都在水平面) */
    float yaw = car_dir_deg * 0.01745329252f;
    float fx = -cosf(yaw), fz = -sinf(yaw);  // 你之前就有的前向
    float rx = sinf(yaw), rz = cosf(yaw);    // 你之前就有的右向

    /* 將地形梯度 (dydx, dydz) 投影到車體座標: 
       pitch ← 沿 forward 的坡度；roll ← 沿 right 的坡度 (取負較自然) */
    float slope_forward = dydx * fx + dydz * fz;
    float slope_right = dydx * rx + dydz * rz;

    float pitch_deg = atanf(slope_forward) * 180.0f / PI;
    float roll_deg = atanf(slope_right) * 180.0f / PI;

    const float sample_half = 3.0f;  // 前後各取 3 單位
    const float xF = car_x + fx * sample_half, zF = car_z + fz * sample_half;
    const float xB = car_x - fx * sample_half, zB = car_z - fz * sample_half;

    pitch_deg = atan2f(terrain_height(xF, zF) - terrain_height(xB, zB), 2.0f * sample_half) * 57.29578f;

    const float sample_half_w = 2.0f;  // 左右各取 2 單位
    const float xR = car_x + rx * sample_half_w, zR = car_z + rz * sample_half_w;
    const float xL = car_x - rx * sample_half_w, zL = car_z - rz * sample_half_w;

    roll_deg = atan2f(terrain_height(xR, zR) - terrain_height(xL, zL), 2.0f * sample_half_w) * 57.29578f;

    static bool has_prev = false;
    static float prev_x = 0.0f, prev_z = 0.0f;

    if (!has_prev) {  // 第一次進來先建立上一幀位置
        prev_x = car_x;
        prev_z = car_z;
        has_prev = true;
    }

    // 本幀位移向量
    const float dx = car_x - prev_x;
    const float dz = car_z - prev_z;

    const float v_forward = dx * fx + dz * fz;

    //if (v_forward < 0.0f) {
    //    // 倒退: 把 pitch 反號
    //    pitch_deg = pitch_deg;
    //}

    // 更新上一幀位置
    prev_x = car_x;
    prev_z = car_z;

    /* 套用旋轉: Yaw => Pitch => Roll */
    glPushMatrix();
    glTranslatef(car_x, car_y, car_z);
    glRotatef(car_dir_deg, 0, 1, 0);
    glRotatef(pitch_deg, rx, 0.0f, rz);  // pitch around local Right = (rx,0,rz)
    glRotatef(-roll_deg, fx, 0.0f, fz);  // roll  around local Forward = (fx,0,fz)
    draw_car();
    glPopMatrix();

    glutSwapBuffers();
}

static int point_in_rect(float x, float z, float xmin, float xmax, float zmin, float zmax) {
    return (x >= xmin && x <= xmax && z >= zmin && z <= zmax);
}

static int can_enter_ramp(const Ramp* r, float px, float pz, float nx, float nz) {
    const float TOL = 1.2f;

    float xmin = fminf(r->x0, r->x1), xmax = fmaxf(r->x0, r->x1);
    float zmin = fminf(r->z0, r->z1), zmax = fmaxf(r->z0, r->z1);

    auto in_rect = [&](float x, float z, float tol) {
        return (x >= xmin - tol && x <= xmax + tol && z >= zmin - tol && z <= zmax + tol);
        };

    int prev_in = in_rect(px, pz, 0.0f);
    int next_in = in_rect(nx, nz, 0.0f);

    if (!next_in)
        return 1; // 沒打算進這個坡: 不干涉
    if (prev_in)
        return 1; // 已在坡內: 允許移動

    // 只限制「必須從低邊那一側進入」，但放寬角度與 dx/dz 
    if (r->alongX) {
        int from_low_side = (px <= xmin + TOL) &&
            (nx >= xmin - TOL) &&
            (nz >= zmin - TOL && nz <= zmax + TOL);
        return from_low_side ? 1 : 0;
    }
    else {
        // z 大概等於 zmin
        int from_low_side = (pz <= zmin + TOL) && (nz >= zmin - TOL) && (nx >= xmin - TOL && nx <= xmax + TOL);
        return from_low_side ? 1 : 0;
    }
}

static void try_move(float step_forward) {
    float rad = car_dir_deg * (PI / 180.0f);

    // 正確的世界前進向量: (+cos(rad), 0, -sin(rad))
    float nx = car_x + step_forward * cosf(rad);
    float nz = car_z + step_forward * (-sinf(rad));

    const float margin = 1.5f;
    if (nx < WORLD_MIN_X + margin || nx > WORLD_MAX_X - margin)
        return;
    if (nz < WORLD_MIN_Z + margin || nz > WORLD_MAX_Z - margin)
        return;

    float inflate = fmaxf(CAR_HALF_L, CAR_HALF_W) + 0.8f;
    if (hit_building_xy(nx, nz, inflate))
        return;

    // ramp 單面可爬的檢查
    for (int i = 0; i < gNumRamps; ++i) {
        if (!can_enter_ramp(&gRamps[i], car_x, car_z, nx, nz))
            return;
    }

    car_x = nx;
    car_z = nz;
    car_y = terrain_height(car_x, car_z) + CAR_BODY_H * 0.5f + 0.4f;
}

static void enter_screensaver_mode() {
    screensaver_mode = true;

    // 讓整個 3D 畫面縮一點
    vp_w = (int)(gWinW * 0.70);
    vp_h = (int)(gWinH * 0.70);

    // 隨機挑一個角落當起點
    rng_seed = (unsigned)time(nullptr);
    int corner = rng_seed % 4;  // 0: LL, 1: LR, 2: UR, 3: UL
    if (corner == 0) {
        vp_x = 0;
        vp_y = 0;
    }
    else if (corner == 1) {
        vp_x = gWinW - vp_w;
        vp_y = 0;
    }
    else if (corner == 2) {
        vp_x = gWinW - vp_w;
        vp_y = gWinH - vp_h;
    }
    else {
        vp_x = 0;
        vp_y = gWinH - vp_h;
    }

    // 隨機水平/垂直方向
    vp_vx = ((rng_seed >> 1) & 1) ? +540.0f : -540.0f;
    vp_vy = ((rng_seed >> 2) & 1) ? +540.0f : -540.0f;
}

static void leave_screensaver_mode() {
    screensaver_mode = false;
    vp_x = 0;
    vp_y = 0;
    vp_w = gWinW;
    vp_h = gWinH;  // 回到全視窗
}

/* W/S/A/D、R、1/2、Q */
static void on_keyboard(unsigned char key, int x, int y) {
    (void)x;
    (void)y;
    switch (key) {
    case 'Q':
    case 'q':
        exit(0);
        break;
    case '1':
        gPolyModeFill = 1;
        glutPostRedisplay();
        break;
    case '2':
        gPolyModeFill = 0;
        glutPostRedisplay();
        break;
    case 'w':
    case 'W':
        if (screensaver_mode)
            return;
        if (!cam_edit_mode) {
            try_move(+car_speed);
        }
        else {
            /* Camera forward */
            float fx = cam_ctr_x - cam_eye_x;
            float fz = cam_ctr_z - cam_eye_z;
            float len = sqrtf(fx * fx + fz * fz);
            if (len < 1e-6f) {
                fx = -1.0f;
                fz = 0.0f;
                len = 1.0f;
            }
            fx /= len;
            fz /= len;
            cam_eye_x += fx * cam_move_speed;
            cam_ctr_x += fx * cam_move_speed;
            cam_eye_z += fz * cam_move_speed;
            cam_ctr_z += fz * cam_move_speed;
        }
        glutPostRedisplay();
        break;
    case 's':
    case 'S':
        if (screensaver_mode)
            return;
        if (!cam_edit_mode) {
            try_move(-car_speed);
        }
        else {
            float fx = cam_ctr_x - cam_eye_x;
            float fz = cam_ctr_z - cam_eye_z;
            float len = sqrtf(fx * fx + fz * fz);
            if (len < 1e-6f) {
                fx = -1.0f;
                fz = 0.0f;
                len = 1.0f;
            }
            fx /= len;
            fz /= len;
            cam_eye_x -= fx * cam_move_speed;
            cam_ctr_x -= fx * cam_move_speed;
            cam_eye_z -= fz * cam_move_speed;
            cam_ctr_z -= fz * cam_move_speed;
        }
        glutPostRedisplay();
        break;
    case 'a':
    case 'A':
        if (screensaver_mode)
            return;
        if (!cam_edit_mode) {
            car_dir_deg += car_turn;
            if (car_dir_deg >= 360)
                car_dir_deg -= 360;
        }
        else {
            /* Camera strafe left */
            float fx = cam_ctr_x - cam_eye_x;
            float fz = cam_ctr_z - cam_eye_z;
            float len = sqrtf(fx * fx + fz * fz);
            if (len < 1e-6f) {
                fx = -1.0f;
                fz = 0.0f;
                len = 1.0f;
            }
            float rx = -fz / len, rz = fx / len; // left = -right
            cam_eye_x -= rx * cam_move_speed;
            cam_ctr_x -= rx * cam_move_speed;
            cam_eye_z -= rz * cam_move_speed;
            cam_ctr_z -= rz * cam_move_speed;
        }
        glutPostRedisplay();
        break;
    case 'd':
    case 'D':
        if (screensaver_mode)
            return;
        if (!cam_edit_mode) {
            car_dir_deg -= car_turn;
            if (car_dir_deg < 0)
                car_dir_deg += 360;
        }
        else {
            float fx = cam_ctr_x - cam_eye_x;
            float fz = cam_ctr_z - cam_eye_z;
            float len = sqrtf(fx * fx + fz * fz);
            if (len < 1e-6f) {
                fx = -1.0f;
                fz = 0.0f;
                len = 1.0f;
            }
            float rx = -fz / len, rz = fx / len;
            cam_eye_x += rx * cam_move_speed;
            cam_ctr_x += rx * cam_move_speed;
            cam_eye_z += rz * cam_move_speed;
            cam_ctr_z += rz * cam_move_speed;
        }
        glutPostRedisplay();
        break;
    case 'r':
    case 'R':
        if (screensaver_mode)
            return;
        if (!cam_edit_mode) {
            car_x = 10.0f;
            car_z = 10.0f;
            car_dir_deg = 0.0f;
            car_y = terrain_height(car_x, car_z);
        }
        else {
            cam_eye_x = 95.0f;
            cam_eye_y = 70.0f;
            cam_eye_z = 95.0f;
            cam_ctr_x = 45.0f;
            cam_ctr_y = 0.0f;
            cam_ctr_z = 45.0f;
        }
        glutPostRedisplay();
        break;
    case 'c':
    case 'C':
        cam_edit_mode ^= 1; // 取反
        glutPostRedisplay();
        break;
    case 'v':
    case 'V':
        if (screensaver_mode)
            leave_screensaver_mode();
        else
            enter_screensaver_mode();
        break;
    default:
        break;
    }
}

void idle() {
    static int last_ms = glutGet(GLUT_ELAPSED_TIME);
    int now = glutGet(GLUT_ELAPSED_TIME);
    float dt = (now - last_ms) / 1000.0f;
    last_ms = now;

    if (screensaver_mode) {
        // 更新位置
        vp_x += (int)(vp_vx * dt);
        vp_y += (int)(vp_vy * dt);

        // 碰撞邊界就反彈
        if (vp_x <= 0) {
            vp_x = 0;
            vp_vx = fabsf(vp_vx);
        }
        if (vp_y <= 0) {
            vp_y = 0;
            vp_vy = fabsf(vp_vy);
        }
        if (vp_x + vp_w >= gWinW) {
            vp_x = gWinW - vp_w;
            vp_vx = -fabsf(vp_vx);
        }
        if (vp_y + vp_h >= gWinH) {
            vp_y = gWinH - vp_h;
            vp_vy = -fabsf(vp_vy);
        }

        // 縮放
        float s = 0.70f + 0.02f * sinf(now * 0.003f);
        vp_w = (int)(gWinW * s);
        vp_h = (int)(gWinH * s);
    }

    glutPostRedisplay();
}

static void on_reshape(int w, int h) {
    gWinW = w;
    gWinH = h;
    if (!screensaver_mode) {
        vp_x = 0;
        vp_y = 0;
        vp_w = w;
        vp_h = h;
    }
    glViewport(vp_x, vp_y, (vp_w > 0 ? vp_w : gWinW), (vp_h > 0 ? vp_h : gWinH));
    glutPostRedisplay();
}

/*==================== main ====================*/
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(gWinW, gWinH);
    glutCreateWindow("Car Scene");

    my_init();

    /* 初始車高 */
    car_y = terrain_height(car_x, car_z);

    glutDisplayFunc(display);
    glutReshapeFunc(on_reshape);
    glutKeyboardFunc(on_keyboard);
    glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}
