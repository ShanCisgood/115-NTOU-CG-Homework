#include <GL/glut.h>
#include <math.h>
#include <stdio.h>
#include <cstdlib>
#include <ctime>

#define PI 3.14159265358979323846f
#define DEG2RAD(x) ((x) * (PI / 180.0f))

int gWinW = 1000, gWinH = 800;
int gPolyModeFill = 1;

const float WORLD_MIN_X = 0.0f, WORLD_MAX_X = 100.0f;
const float WORLD_MIN_Z = 0.0f, WORLD_MAX_Z = 100.0f;

int cam_edit_mode = 0;
float cam_eye_x = 95.0f, cam_eye_y = 70.0f, cam_eye_z = 95.0f;
float cam_ctr_x = 45.0f, cam_ctr_y = 0.0f, cam_ctr_z = 45.0f;
float cam_up_x = 0.0f, cam_up_y = 1.0f, cam_up_z = 0.0f;
float cam_move_speed = 2.0f; 
float cam_rotate_deg = 3.0f; 

float cam_fov_y = 60.0f;
float cam_near = 1.0f;
float cam_far = 400.0f;
float gLastAspect = 1.0f; 

float gZoom = 1.0f; 
const float ZOOM_MIN = 0.3f;
const float ZOOM_MAX = 3.0f;

float car_x = 10.0f;
float car_z = 10.0f;
float car_y = 0.0f; 
float car_dir_deg = 0.0f; 
float car_speed = 0.6f; 
float car_turn = 6.0f;

const float CAR_HALF_L = 4.0f;
const float CAR_HALF_W = 2.0f;
const float CAR_BODY_H = 2.0f;

GLUquadric* gCyl = NULL;
GLUquadric* gSphere = NULL;
const float EPS = 0.06f;

bool screensaver_mode = false;
int vp_x = 0, vp_y = 0, vp_w = 0, vp_h = 0;
float vp_vx = 540.0f, vp_vy = 540.0f;
unsigned int rng_seed = 9487u; 

enum ProjectionMode { PROJ_ORTHO_X = 0, PROJ_ORTHO_Y = 1, PROJ_ORTHO_Z = 2, PROJ_PERSPECTIVE = 3, PROJ_FOUR_VIEW = 4 };
int gProjMode = PROJ_PERSPECTIVE;

bool gDirLightOn = true;
float gDirAzimDeg = 45.0f;
float gDirElevDeg = 35.0f; 
float gDirColor[3] = { 1.0f, 0.98f, 0.90f };
float gDirIntensity = 0.85f;
int   gDirPreset = 0;

bool  gPointLightOn = true;
float gPointPos[3] = { 50.0f, 85.0f, 50.0f };
float gPointColor[3] = { 1.0f, 1.0f, 1.0f };
float gPointIntensity = 0.9f;

bool  gCarLightOn = true; 
int   gCarLightCount = 2;  
float gCarLightColor[3] = { 1.0f, 1.0f, 0.92f };
float gCarLightIntensity = 1.2f;
float gCarLightCutoff = 18.0f;  
float gCarLightExponent = 20.0f;
float gCarLightYawOffset = 0.0f;
float gCarLightPitch = -8.0f;   

const float LIGHT_COLOR_STEP = 0.05f;
const float LIGHT_INT_STEP = 0.10f;
const float LIGHT_ANGLE_STEP = 2.0f;

struct Vec3 {
    float x, y, z;
};

Vec3 make_vec3(float x, float y, float z) {
    Vec3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}
Vec3 v_add(const Vec3& a, const Vec3& b) { return make_vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
Vec3 v_sub(const Vec3& a, const Vec3& b) { return make_vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
Vec3 v_scale(const Vec3& a, float s) { return make_vec3(a.x * s, a.y * s, a.z * s); }
float v_dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 v_cross(const Vec3& a, const Vec3& b) {
    return make_vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
Vec3 v_normalize(const Vec3& a) {
    float len = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    if (len < 1e-6f)
        return make_vec3(0.0f, 0.0f, 0.0f);
    return make_vec3(a.x / len, a.y / len, a.z / len);
}

Vec3 v_rotate_axis(const Vec3& v, const Vec3& k_unit, float angle) {
    float c = cosf(angle);
    float s = sinf(angle);
    Vec3 term1 = v_scale(v, c);
    Vec3 term2 = v_scale(v_cross(k_unit, v), s);
    Vec3 term3 = v_scale(k_unit, v_dot(k_unit, v) * (1.0f - c));
    return v_add(v_add(term1, term2), term3);
}

Vec3 v_from3(float x, float y, float z) { return make_vec3(x, y, z); }

Vec3 tri_normal(const Vec3& a, const Vec3& b, const Vec3& c) {
    Vec3 n = v_cross(v_sub(b, a), v_sub(c, a));
    return v_normalize(n);
}
void glNormalVec(const Vec3& n) { glNormal3f(n.x, n.y, n.z); }

void set_material_ground() {
    GLfloat spec[4] = { 0,0,0,1 };
    GLfloat amb[4] = { 0.18f,0.18f,0.18f,1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
}
void set_material_car_shiny() {
    GLfloat spec[4] = { 0.95f,0.95f,0.95f,1 };
    GLfloat amb[4] = { 0.10f,0.10f,0.10f,1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 90.0f);
}
void set_material_glossy() {
    GLfloat spec[4] = { 0.45f,0.45f,0.45f,1 };
    GLfloat amb[4] = { 0.15f,0.15f,0.15f,1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 35.0f);
}
void set_material_ramp() {
    GLfloat spec[4] = { 0.20f,0.20f,0.20f,1 };
    GLfloat amb[4] = { 0.12f,0.12f,0.12f,1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 18.0f);
}

void push_emission(float r, float g, float b) {
    GLfloat emi[4] = { r, g, b, 1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
}
void pop_emission() {
    GLfloat emi0[4] = { 0,0,0,1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi0);
}

void setup_lights_in_current_view() {
    glEnable(GL_LIGHTING);

    GLfloat globalAmb[4] = { 0.16f, 0.16f, 0.16f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmb);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    if (gDirLightOn) glEnable(GL_LIGHT0); else glDisable(GL_LIGHT0);
    {
        float az = DEG2RAD(gDirAzimDeg);
        float el = DEG2RAD(gDirElevDeg);
        GLfloat pos[4] = { cosf(el) * cosf(az), sinf(el), cosf(el) * sinf(az), 0.0f };

        GLfloat amb[4] = { 0.05f * gDirIntensity * gDirColor[0],
                           0.05f * gDirIntensity * gDirColor[1],
                           0.05f * gDirIntensity * gDirColor[2], 1.0f };
        GLfloat dif[4] = { gDirIntensity * gDirColor[0],
                           gDirIntensity * gDirColor[1],
                           gDirIntensity * gDirColor[2], 1.0f };
        GLfloat spe[4] = { 0.9f * gDirIntensity * gDirColor[0],
                           0.9f * gDirIntensity * gDirColor[1],
                           0.9f * gDirIntensity * gDirColor[2], 1.0f };

        glLightfv(GL_LIGHT0, GL_POSITION, pos);
        glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
        glLightfv(GL_LIGHT0, GL_SPECULAR, spe);
    }

    if (gPointLightOn) glEnable(GL_LIGHT1); else glDisable(GL_LIGHT1);
    {
        GLfloat pos[4] = { gPointPos[0], gPointPos[1], gPointPos[2], 1.0f };
        GLfloat dif[4] = { gPointIntensity * gPointColor[0],
                           gPointIntensity * gPointColor[1],
                           gPointIntensity * gPointColor[2], 1.0f };
        GLfloat spe[4] = { dif[0], dif[1], dif[2], 1.0f };
        GLfloat amb[4] = { 0,0,0,1 };

        glLightfv(GL_LIGHT1, GL_POSITION, pos);
        glLightfv(GL_LIGHT1, GL_AMBIENT, amb);
        glLightfv(GL_LIGHT1, GL_DIFFUSE, dif);
        glLightfv(GL_LIGHT1, GL_SPECULAR, spe);

        glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(GL_LIGHT1, GL_LINEAR_ATTENUATION, 0.010f);
        glLightf(GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.0015f);
    }

    float yaw = car_dir_deg * (PI / 180.0f);
    Vec3 fwd = make_vec3(cosf(yaw), 0.0f, -sinf(yaw));
    Vec3 right = make_vec3(sinf(yaw), 0.0f, cosf(yaw));
    Vec3 up = make_vec3(0.0f, 1.0f, 0.0f);

    float frontOff = CAR_HALF_L + 0.8f;
    float sideOff = CAR_HALF_W - 0.4f;
    float yOff = -CAR_BODY_H * 0.15f;

    Vec3 carPos = make_vec3(car_x, car_y, car_z);

    Vec3 Lpos = v_add(v_add(carPos, v_scale(fwd, frontOff)), v_scale(right, +sideOff));
    Vec3 Rpos = v_add(v_add(carPos, v_scale(fwd, frontOff)), v_scale(right, -sideOff));
    Lpos.y += yOff;
    Rpos.y += yOff;

    Vec3 dir = fwd;
    if (fabsf(gCarLightYawOffset) > 1e-4f) dir = v_rotate_axis(dir, up, DEG2RAD(gCarLightYawOffset));
    Vec3 pitchAxis = v_normalize(v_cross(up, dir));
    dir = v_rotate_axis(dir, pitchAxis, DEG2RAD(gCarLightPitch));
    dir = v_normalize(dir);

    auto setup_spot = [&](GLenum lightId, const Vec3& pos, bool enable) {
        if (enable) glEnable(lightId); else glDisable(lightId);

        GLfloat p4[4] = { pos.x, pos.y, pos.z, 1.0f };
        GLfloat dif[4] = { gCarLightIntensity * gCarLightColor[0],
                           gCarLightIntensity * gCarLightColor[1],
                           gCarLightIntensity * gCarLightColor[2], 1.0f };
        GLfloat spe[4] = { dif[0], dif[1], dif[2], 1.0f };
        GLfloat amb[4] = { 0,0,0,1 };
        GLfloat sdir[3] = { dir.x, dir.y, dir.z };

        glLightfv(lightId, GL_POSITION, p4);
        glLightfv(lightId, GL_SPOT_DIRECTION, sdir);
        glLightf(lightId, GL_SPOT_CUTOFF, gCarLightCutoff);
        glLightf(lightId, GL_SPOT_EXPONENT, gCarLightExponent);

        glLightfv(lightId, GL_AMBIENT, amb);
        glLightfv(lightId, GL_DIFFUSE, dif);
        glLightfv(lightId, GL_SPECULAR, spe);

        glLightf(lightId, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(lightId, GL_LINEAR_ATTENUATION, 0.020f);
        glLightf(lightId, GL_QUADRATIC_ATTENUATION, 0.0025f);
        };

    bool e2 = gCarLightOn && (gCarLightCount >= 1);
    bool e3 = gCarLightOn && (gCarLightCount >= 2);

    setup_spot(GL_LIGHT2, Lpos, e2);
    setup_spot(GL_LIGHT3, Rpos, e3);
}

void ensure_quadrics(void) {
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

void draw_light_markers() {
    ensure_quadrics();

    if (gPointLightOn) {
        glPushMatrix();
        glTranslatef(gPointPos[0], gPointPos[1], gPointPos[2]);
        glColor3f(gPointColor[0], gPointColor[1], gPointColor[2]);
        push_emission(gPointIntensity * gPointColor[0], gPointIntensity * gPointColor[1], gPointIntensity * gPointColor[2]);
        glutSolidSphere(1.2, 18, 18);
        pop_emission();
        glPopMatrix();
    }

    if (gCarLightOn) {
        float yaw = car_dir_deg * (PI / 180.0f);
        Vec3 fwd = make_vec3(cosf(yaw), 0.0f, -sinf(yaw));
        Vec3 right = make_vec3(sinf(yaw), 0.0f, cosf(yaw));

        float frontOff = CAR_HALF_L + 0.8f;
        float sideOff = CAR_HALF_W - 0.4f;
        float yOff = -CAR_BODY_H * 0.15f;

        Vec3 carPos = make_vec3(car_x, car_y, car_z);
        Vec3 Lpos = v_add(v_add(carPos, v_scale(fwd, frontOff)), v_scale(right, +sideOff));
        Vec3 Rpos = v_add(v_add(carPos, v_scale(fwd, frontOff)), v_scale(right, -sideOff));
        Lpos.y += yOff;
        Rpos.y += yOff;

        auto draw_one = [&](const Vec3& p) {
            glPushMatrix();
            glTranslatef(p.x, p.y, p.z);
            glColor3f(gCarLightColor[0], gCarLightColor[1], gCarLightColor[2]);
            push_emission(gCarLightIntensity * gCarLightColor[0], gCarLightIntensity * gCarLightColor[1], gCarLightIntensity * gCarLightColor[2]);
            glutSolidSphere(0.8, 16, 16);
            pop_emission();
            glPopMatrix();
            };

        if (gCarLightCount >= 1) draw_one(Lpos);
        if (gCarLightCount >= 2) draw_one(Rpos);
    }
}

const float BASE_Y = -2.5f;

typedef struct {
    float x0, z0, x1, z1;
    float riseY;
    int alongX;
    float cr, cg, cb;
} Ramp;

Ramp gRamps[] = {
    /*  x0   z0   x1   z1   rise   alongX   r     g     b  */
    {15.0f, 15.0f, 35.0f, 25.0f, 10.0f, 1, 0.90f, 0.55f, 0.20f}, /* 橘色 */
    {55.0f, 20.0f, 70.0f, 35.0f, 6.0f,  0, 0.30f, 0.80f, 0.70f}, /* 藍綠 */
    {30.0f, 60.0f, 50.0f, 80.0f, 8.0f,  1, 0.70f, 0.60f, 0.95f}, /* 淡紫 */
};
const int gNumRamps = sizeof(gRamps) / sizeof(gRamps[0]);

float terrain_height(float x, float z) {
    float y = BASE_Y;
    for (int i = 0; i < gNumRamps; ++i) {
        Ramp r = gRamps[i];
        float xmin = fminf(r.x0, r.x1), xmax = fmaxf(r.x0, r.x1);
        float zmin = fminf(r.z0, r.z1), zmax = fmaxf(r.z0, r.z1);
        if (x >= xmin && x <= xmax && z >= zmin && z <= zmax) {
            if (r.alongX) {
                float t = (x - xmin) / (xmax - xmin);
                y += r.riseY * t;
            }
            else {
                float t = (z - zmin) / (zmax - zmin); 
                y += r.riseY * t;
            }
        }
    }
    return y;
}

void terrain_gradient(float x, float z, float* dydx, float* dydz) {
    const float eps = 0.25f;
    float yx1 = terrain_height(x + eps, z);
    float yx0 = terrain_height(x - eps, z);
    float yz1 = terrain_height(x, z + eps);
    float yz0 = terrain_height(x, z - eps);
    *dydx = (yx1 - yx0) / (2.0f * eps);
    *dydz = (yz1 - yz0) / (2.0f * eps);
}

typedef struct {
    float x0, z0, x1, z1;
    float h;
} Building;

Building gBlds[] = {
    {40.0f, 0.0f,  50.0f, 10.0f, 20.0f},
    {70.0f, 55.0f, 85.0f, 75.0f, 25.0f},
    {18.0f, 40.0f, 26.0f, 55.0f, 15.0f},
};
const int gNumBlds = sizeof(gBlds) / sizeof(gBlds[0]);

int hit_building_xy(float x, float z, float inflate) {
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


void draw_unit_cube(void) {
    /* 一個 1x1x1 立方體，中心在原點 (含法向量) */
    const GLfloat v[8][3] = {
        {-0.5f, -0.5f, -0.5f},
        { 0.5f, -0.5f, -0.5f},
        { 0.5f,  0.5f, -0.5f},
        {-0.5f,  0.5f, -0.5f},
        {-0.5f, -0.5f,  0.5f},
        { 0.5f, -0.5f,  0.5f},
        { 0.5f,  0.5f,  0.5f},
        {-0.5f,  0.5f,  0.5f}
    };

    struct Face { int a, b, c, d; float nx, ny, nz; };
    const Face faces[6] = {
        {0,1,2,3,  0, 0,-1}, // -Z
        {4,5,6,7,  0, 0, 1}, // +Z
        {0,4,5,1,  0,-1, 0}, // -Y
        {3,2,6,7,  0, 1, 0}, // +Y
        {1,5,6,2,  1, 0, 0}, // +X
        {0,3,7,4, -1, 0, 0}, // -X
    };

    for (int i = 0; i < 6; ++i) {
        glNormal3f(faces[i].nx, faces[i].ny, faces[i].nz);
        glBegin(GL_QUADS);
        glVertex3fv(v[faces[i].a]);
        glVertex3fv(v[faces[i].b]);
        glVertex3fv(v[faces[i].c]);
        glVertex3fv(v[faces[i].d]);
        glEnd();
    }
}

void draw_floor_tiles(void) {
    /* 10x10 tiles, 覆蓋 0..100 (Diffuse, no specular) */
    set_material_ground();
    glNormal3f(0.0f, 1.0f, 0.0f);

    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            if ((i + j) % 2 == 0)
                glColor3f(1.0f, 0.8f, 0.8f);
            else
                glColor3f(0.1f, 0.1f, 0.7f);

            float x0 = i * 10.0f, x1 = (i + 1) * 10.0f;
            float z0 = j * 10.0f, z1 = (j + 1) * 10.0f;

            glBegin(GL_QUADS);
            glVertex3f(x0, BASE_Y - EPS, z0);
            glVertex3f(x1, BASE_Y - EPS, z0);
            glVertex3f(x1, BASE_Y - EPS, z1);
            glVertex3f(x0, BASE_Y - EPS, z1);
            glEnd();
        }
    }
}

/* 斜面畫斜線紋 (視覺指示): 沿 X 的斜面 (位於 z = zmax) */
void draw_slope_hatching_alongX(float xmin, float xmax, float zmax, float y0, float y1, int stripes) {
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
void draw_slope_hatching_alongZ(float zmin, float zmax, float xmax, float y0, float y1, int stripes) {
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

void draw_ramps(void) {
    // Draw ramps as wedge solids with correct normals (smooth shading enabled globally)
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
        const float y1 = BASE_Y + r.riseY + EPS;  // 頂端高度

        set_material_ramp();
        glColor3f(r.cr, r.cg, r.cb);

        // --- Bottom (optional; mainly for completeness) ---
        {
            Vec3 a = v_from3(xmin, y0, zmin), b = v_from3(xmax, y0, zmin), c = v_from3(xmax, y0, zmax);
            glNormalVec(tri_normal(a, b, c));
            glBegin(GL_QUADS);
            glVertex3f(xmin, y0, zmin);
            glVertex3f(xmax, y0, zmin);
            glVertex3f(xmax, y0, zmax);
            glVertex3f(xmin, y0, zmax);
            glEnd();
        }

        if (r.alongX) {
            // Height increases from x=xmin (y0) -> x=xmax (y1)
            // cout <<
            // --- Top slope plane ---
            Vec3 a = v_from3(xmin, y0, zmin);
            Vec3 b = v_from3(xmin, y0, zmax);
            Vec3 c = v_from3(xmax, y1, zmax);
            glNormalVec(tri_normal(a, b, c));
            glBegin(GL_QUADS);
            glVertex3f(xmin, y0, zmin);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmax, y1, zmin);
            glEnd();

            // --- High end wall (x=xmax) ---
            {
                Vec3 p0 = v_from3(xmax, y0, zmin), p1 = v_from3(xmax, y0, zmax), p2 = v_from3(xmax, y1, zmax);
                glNormalVec(tri_normal(p0, p1, p2));
                glBegin(GL_QUADS);
                glVertex3f(xmax, y0, zmin);
                glVertex3f(xmax, y0, zmax);
                glVertex3f(xmax, y1, zmax);
                glVertex3f(xmax, y1, zmin);
                glEnd();
            }

            // --- Side triangles (z=zmin / z=zmax) ---
            {
                // z=zmin
                Vec3 p0 = v_from3(xmin, y0, zmin), p1 = v_from3(xmax, y0, zmin), p2 = v_from3(xmax, y1, zmin);
                glNormalVec(tri_normal(p0, p1, p2));
                glBegin(GL_TRIANGLES);
                glVertex3f(xmin, y0, zmin);
                glVertex3f(xmax, y0, zmin);
                glVertex3f(xmax, y1, zmin);
                glEnd();

                // z=zmax
                Vec3 q0 = v_from3(xmin, y0, zmax), q1 = v_from3(xmax, y1, zmax), q2 = v_from3(xmax, y0, zmax);
                glNormalVec(tri_normal(q0, q1, q2));
                glBegin(GL_TRIANGLES);
                glVertex3f(xmin, y0, zmax);
                glVertex3f(xmax, y1, zmax);
                glVertex3f(xmax, y0, zmax);
                glEnd();
            }

            // --- Outline + hatching on slope edge (visual cue) ---
            glDisable(GL_LIGHTING);
            glLineWidth(2.0f);
            glColor3f(0, 0, 0);
            glBegin(GL_LINE_LOOP);
            glVertex3f(xmin, y0, zmax);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmax, y1, zmin);
            glVertex3f(xmin, y0, zmin);
            glEnd();
            glColor3f(0, 0, 0);
            draw_slope_hatching_alongX(xmin, xmax, zmax, y0, y1, 10);
            glLineWidth(1.0f);
            glEnable(GL_LIGHTING);
        }
        else {
            // Height increases from z=zmin (y0) -> z=zmax (y1)

            // --- Top slope plane ---
            Vec3 a = v_from3(xmin, y0, zmin);
            Vec3 b = v_from3(xmax, y0, zmin);
            Vec3 c = v_from3(xmax, y1, zmax);
            glNormalVec(tri_normal(a, b, c));
            glBegin(GL_QUADS);
            glVertex3f(xmin, y0, zmin);
            glVertex3f(xmax, y0, zmin);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmin, y1, zmax);
            glEnd();

            // --- High end wall (z=zmax) ---
            {
                Vec3 p0 = v_from3(xmin, y0, zmax), p1 = v_from3(xmax, y0, zmax), p2 = v_from3(xmax, y1, zmax);
                glNormalVec(tri_normal(p0, p1, p2));
                glBegin(GL_QUADS);
                glVertex3f(xmin, y0, zmax);
                glVertex3f(xmax, y0, zmax);
                glVertex3f(xmax, y1, zmax);
                glVertex3f(xmin, y1, zmax);
                glEnd();
            }

            // --- Side triangles (x=xmin / x=xmax) ---
            {
                // x=xmin
                Vec3 p0 = v_from3(xmin, y0, zmin), p1 = v_from3(xmin, y1, zmax), p2 = v_from3(xmin, y0, zmax);
                glNormalVec(tri_normal(p0, p1, p2));
                glBegin(GL_TRIANGLES);
                glVertex3f(xmin, y0, zmin);
                glVertex3f(xmin, y1, zmax);
                glVertex3f(xmin, y0, zmax);
                glEnd();

                // x=xmax
                Vec3 q0 = v_from3(xmax, y0, zmin), q1 = v_from3(xmax, y0, zmax), q2 = v_from3(xmax, y1, zmax);
                glNormalVec(tri_normal(q0, q1, q2));
                glBegin(GL_TRIANGLES);
                glVertex3f(xmax, y0, zmin);
                glVertex3f(xmax, y0, zmax);
                glVertex3f(xmax, y1, zmax);
                glEnd();
            }

            // --- Outline + hatching ---
            glDisable(GL_LIGHTING);
            glLineWidth(2.0f);
            glColor3f(0, 0, 0);
            glBegin(GL_LINE_LOOP);
            glVertex3f(xmax, y0, zmin);
            glVertex3f(xmax, y1, zmax);
            glVertex3f(xmin, y1, zmax);
            glVertex3f(xmin, y0, zmin);
            glEnd();
            glColor3f(0, 0, 0);
            draw_slope_hatching_alongZ(zmin, zmax, xmax, y0, y1, 10);
            glLineWidth(1.0f);
            glEnable(GL_LIGHTING);
        }
    }

    if (cull_save) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
    if (blend_save) glEnable(GL_BLEND); else glDisable(GL_BLEND);
    if (po_fill_save) glEnable(GL_POLYGON_OFFSET_FILL); else glDisable(GL_POLYGON_OFFSET_FILL);
}


void draw_buildings(void) {
    set_material_glossy();
    for (int i = 0; i < gNumBlds; ++i) {
        float xmin = fminf(gBlds[i].x0, gBlds[i].x1);
        float xmax = fmaxf(gBlds[i].x0, gBlds[i].x1);
        float zmin = fminf(gBlds[i].z0, gBlds[i].z1);
        float zmax = fmaxf(gBlds[i].z0, gBlds[i].z1);
        float y0 = terrain_height(0.5f * (xmin + xmax), 0.5f * (zmin + zmax));
        float h = gBlds[i].h;

        glPushMatrix();
        glTranslatef(0.5f * (xmin + xmax), y0 + h * 0.5f, 0.5f * (zmin + zmax));
        glScalef((xmax - xmin), h, (zmax - zmin));
        glColor3f(0.78f, 0.78f, 0.78f);
        draw_unit_cube();
        glPopMatrix();
    }
}


void draw_world_axes_at(float x, float y, float z, float scale) {
    ensure_quadrics();

    GLboolean lit = glIsEnabled(GL_LIGHTING);
    glDisable(GL_LIGHTING);

    glPushMatrix();
    glTranslatef(x, y, z);
    glScalef(scale, scale, scale);

    // origin sphere
    glColor3f(1, 1, 1);
    gluSphere(gSphere, 0.5, 12, 12);

    // Z (blue)
    glColor3f(0, 0, 1);
    gluCylinder(gCyl, 0.1, 0.1, 4.0, 12, 1);

    // Y (green)
    glPushMatrix();
    glRotatef(-90, 1, 0, 0);
    glColor3f(0, 1, 0);
    gluCylinder(gCyl, 0.1, 0.1, 4.0, 12, 1);
    glPopMatrix();

    // X (red)
    glPushMatrix();
    glRotatef(90, 0, 1, 0);
    glColor3f(1, 0, 0);
    gluCylinder(gCyl, 0.1, 0.1, 4.0, 12, 1);
    glPopMatrix();

    glPopMatrix();

    if (lit) glEnable(GL_LIGHTING);
}


void draw_car(void) {
    ensure_quadrics();

    set_material_car_shiny();

    /* 車身 */
    glColor3f(0.9f, 0.2f, 0.2f);
    glPushMatrix();
    glScalef(CAR_HALF_L * 2.0f, CAR_BODY_H, CAR_HALF_W * 2.0f);  // 長 * 高 * 寬
    draw_unit_cube();
    glPopMatrix();

    /* 四顆輪胎 (偏霧面) */
    {
        GLfloat spec0[4] = { 0,0,0,1 };
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec0);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);

        glColor3f(0.1f, 0.1f, 0.1f);
        float wy = -CAR_BODY_H * 0.5f + 0.8f;  // 輪胎中心高度
        float dx = CAR_HALF_L - 1.0f;
        float dz = CAR_HALF_W + 0.6f;

        glPushMatrix(); glTranslatef(-dx, wy, dz); glutSolidTorus(0.5, 1.0, 24, 24); glPopMatrix();
        glPushMatrix(); glTranslatef(-dx, wy, -dz); glutSolidTorus(0.5, 1.0, 24, 24); glPopMatrix();
        glPushMatrix(); glTranslatef(dx, wy, dz); glutSolidTorus(0.5, 1.0, 24, 24); glPopMatrix();
        glPushMatrix(); glTranslatef(dx, wy, -dz); glutSolidTorus(0.5, 1.0, 24, 24); glPopMatrix();

        // restore shiny for axes if needed later
        set_material_car_shiny();
    }

    /* 車身座標系 (for fun) */
    draw_world_axes_at(0.0f, CAR_BODY_H * 0.6f, 0.0f, 0.6f);
}


void draw_perspective_frustum_lines(void) {
    /* 如果在正投影模式才畫（呼叫端自己控制也可以） */
    Vec3 eye = make_vec3(cam_eye_x, cam_eye_y, cam_eye_z);
    Vec3 center = make_vec3(cam_ctr_x, cam_ctr_y, cam_ctr_z);
    Vec3 up = make_vec3(cam_up_x, cam_up_y, cam_up_z);

    Vec3 f = v_normalize(v_sub(center, eye));
    Vec3 s = v_normalize(v_cross(f, up));
    Vec3 u = v_normalize(v_cross(s, f));

    float fov = cam_fov_y / gZoom;
    if (fov < 15.0f)
        fov = 15.0f;
    if (fov > 120.0f)
        fov = 120.0f;
    float fovRad = DEG2RAD(fov);

    float nh = tanf(fovRad * 0.5f) * cam_near;
    float nw = nh * gLastAspect;
    float fh = tanf(fovRad * 0.5f) * cam_far;
    float fw = fh * gLastAspect;

    Vec3 nc = v_add(eye, v_scale(f, cam_near));
    Vec3 fc = v_add(eye, v_scale(f, cam_far));

    Vec3 ntl = v_add(nc, v_add(v_scale(u, nh), v_scale(s, -nw)));
    Vec3 ntr = v_add(nc, v_add(v_scale(u, nh), v_scale(s, nw)));
    Vec3 nbl = v_add(nc, v_add(v_scale(u, -nh), v_scale(s, -nw)));
    Vec3 nbr = v_add(nc, v_add(v_scale(u, -nh), v_scale(s, nw)));

    Vec3 ftl = v_add(fc, v_add(v_scale(u, fh), v_scale(s, -fw)));
    Vec3 ftr = v_add(fc, v_add(v_scale(u, fh), v_scale(s, fw)));
    Vec3 fbl = v_add(fc, v_add(v_scale(u, -fh), v_scale(s, -fw)));
    Vec3 fbr = v_add(fc, v_add(v_scale(u, -fh), v_scale(s, fw)));

    GLboolean lit = glIsEnabled(GL_LIGHTING);
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glColor3f(1.0f, 1.0f, 0.0f);

    glBegin(GL_LINES);
    /* near plane */
    glVertex3f(ntl.x, ntl.y, ntl.z);
    glVertex3f(ntr.x, ntr.y, ntr.z);
    glVertex3f(ntr.x, ntr.y, ntr.z);
    glVertex3f(nbr.x, nbr.y, nbr.z);
    glVertex3f(nbr.x, nbr.y, nbr.z);
    glVertex3f(nbl.x, nbl.y, nbl.z);
    glVertex3f(nbl.x, nbl.y, nbl.z);
    glVertex3f(ntl.x, ntl.y, ntl.z);

    /* far plane */
    glVertex3f(ftl.x, ftl.y, ftl.z);
    glVertex3f(ftr.x, ftr.y, ftr.z);
    glVertex3f(ftr.x, ftr.y, ftr.z);
    glVertex3f(fbr.x, fbr.y, fbr.z);
    glVertex3f(fbr.x, fbr.y, fbr.z);
    glVertex3f(fbl.x, fbl.y, fbl.z);
    glVertex3f(fbl.x, fbl.y, fbl.z);
    glVertex3f(ftl.x, ftl.y, ftl.z);

    /* connect near and far */
    glVertex3f(ntl.x, ntl.y, ntl.z);
    glVertex3f(ftl.x, ftl.y, ftl.z);
    glVertex3f(ntr.x, ntr.y, ntr.z);
    glVertex3f(ftr.x, ftr.y, ftr.z);
    glVertex3f(nbl.x, nbl.y, nbl.z);
    glVertex3f(fbl.x, fbl.y, fbl.z);
    glVertex3f(nbr.x, nbr.y, nbr.z);
    glVertex3f(fbr.x, fbr.y, fbr.z);
    glEnd();

    glLineWidth(1.0f);
    if (lit) glEnable(GL_LIGHTING);
}

/*==================== World Rendering (independent of view/projection) ====================*/
void render_world_scene(void) {
    // 地板＋斜坡＋建築
    draw_floor_tiles();
    draw_ramps();
    draw_buildings();

    // 世界原點座標軸 (RGB = XYZ)
    draw_world_axes_at(0.0f, 0.0f, 0.0f, 3.0f);

    // 車子的姿態: 位置 (car_x, car_y, car_z) + 朝向 car_dir_deg
    // 根據地形梯度給 pitch (前後傾)與 roll (左右傾)一點點角度
    float dydx = 0.0f, dydz = 0.0f;
    terrain_gradient(car_x, car_z, &dydx, &dydz);

    /* 先算車頭方向 (forward) 與右手方向 (right) (都在水平面)
       注意：try_move 使用 forward=(cos(yaw), 0, -sin(yaw)) */
    float yaw = car_dir_deg * (PI / 180.0f);
    float fx = cosf(yaw), fz = -sinf(yaw); // forward
    float rx = sinf(yaw), rz = cosf(yaw);  // right

    /* 用左右/前後高度差計算 pitch / roll */
    const float sample_half = 3.0f;  // 前後各取 3 單位
    const float xF = car_x + fx * sample_half, zF = car_z + fz * sample_half;
    const float xB = car_x - fx * sample_half, zB = car_z - fz * sample_half;

    float pitch_deg = atan2f(terrain_height(xF, zF) - terrain_height(xB, zB), 2.0f * sample_half) * 57.29578f;

    const float sample_half_w = 2.0f;  // 左右各取 2 單位
    const float xR = car_x + rx * sample_half_w, zR = car_z + rz * sample_half_w;
    const float xL = car_x - rx * sample_half_w, zL = car_z - rz * sample_half_w;

    float roll_deg = atan2f(terrain_height(xR, zR) - terrain_height(xL, zL), 2.0f * sample_half_w) * 57.29578f;

    /* 套用旋轉: Yaw => Pitch => Roll */
    glPushMatrix();
    glTranslatef(car_x, car_y, car_z);
    glRotatef(car_dir_deg, 0, 1, 0);
    glRotatef(pitch_deg, rx, 0.0f, rz);  // pitch around local Right = (rx,0,rz)
    glRotatef(-roll_deg, fx, 0.0f, fz);  // roll  around local Forward = (fx,0,fz)
    draw_car();
    glPopMatrix();

    // 光源位置提示 (emission spheres)
    draw_light_markers();
}


/*==================== Camera & Projection Setup ====================*/

/* 一些方便的世界中心位置（看整個場景用） */
void get_world_center(Vec3* center) {
    center->x = 0.5f * (WORLD_MIN_X + WORLD_MAX_X);
    center->z = 0.5f * (WORLD_MIN_Z + WORLD_MAX_Z);
    center->y = 5.0f; /* 大概高度 */
}

/* Orthographic projection with size & aspect (在 camera 座標系) */
void setup_ortho(float size, float aspect, float znear, float zfar) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if (aspect >= 1.0f) {
        glOrtho(-size * aspect, size * aspect, -size, size, znear, zfar);
    }
    else {
        glOrtho(-size, size, -size / aspect, size / aspect, znear, zfar);
    }
}

/* Perspective projection */
void setup_perspective(float fovY, float aspect, float znear, float zfar) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovY, aspect, znear, zfar);
}

/* 固定 X/Y/Z 正投影用 view + projection */
void apply_ortho_view_x(int vpw, int vph) {
    float aspect = (float)vpw / (float)vph;
    float baseSize = 80.0f;
    float size = baseSize / gZoom;
    setup_ortho(size, aspect, -300.0f, 300.0f);

    Vec3 center;
    get_world_center(&center);
    float dist = 200.0f;

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(center.x + dist, center.y, center.z, center.x, center.y, center.z, 0.0, 1.0, 0.0);
}

void apply_ortho_view_y(int vpw, int vph) {
    float aspect = (float)vpw / (float)vph;
    float baseSize = 80.0f;
    float size = baseSize / gZoom;
    setup_ortho(size, aspect, -300.0f, 300.0f);

    Vec3 center;
    get_world_center(&center);
    float dist = 200.0f;

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    /* 從 +Y 往下看 XZ 平面，up 指向 -Z，讓 X 向右、Z 向下或依你的習慣 */
    gluLookAt(center.x, center.y + dist, center.z, center.x, center.y, center.z, 0.0, 0.0, -1.0);
}

void apply_ortho_view_z(int vpw, int vph) {
    float aspect = (float)vpw / (float)vph;
    float baseSize = 80.0f;
    float size = baseSize / gZoom;
    setup_ortho(size, aspect, -300.0f, 300.0f);

    Vec3 center;
    get_world_center(&center);
    float dist = 200.0f;

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(center.x, center.y, center.z + dist, center.x, center.y, center.z, 0.0, 1.0, 0.0);
}

/* 透視投影：使用可編輯攝影機 */
void apply_perspective_view(int vpw, int vph) {
    float aspect = (float)vpw / (float)vph;
    gLastAspect = aspect;

    float fov = cam_fov_y / gZoom;
    if (fov < 15.0f)
        fov = 15.0f;
    if (fov > 120.0f)
        fov = 120.0f;

    setup_perspective(fov, aspect, cam_near, cam_far);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(cam_eye_x, cam_eye_y, cam_eye_z, cam_ctr_x, cam_ctr_y, cam_ctr_z, cam_up_x, cam_up_y, cam_up_z);
}

void camera_translate(const Vec3& dir, float amount) {
    Vec3 d = v_scale(v_normalize(dir), amount);
    cam_eye_x += d.x;
    cam_eye_y += d.y;
    cam_eye_z += d.z;
    cam_ctr_x += d.x;
    cam_ctr_y += d.y;
    cam_ctr_z += d.z;
}

/* yaw/pitch/roll on camera in world space */
void camera_yaw(float deg) {
    float rad = DEG2RAD(deg);
    Vec3 eye = make_vec3(cam_eye_x, cam_eye_y, cam_eye_z);
    Vec3 center = make_vec3(cam_ctr_x, cam_ctr_y, cam_ctr_z);
    Vec3 up = make_vec3(cam_up_x, cam_up_y, cam_up_z);

    Vec3 f = v_sub(center, eye);
    Vec3 u = v_normalize(up);

    Vec3 f2 = v_rotate_axis(f, u, rad);
    Vec3 c2 = v_add(eye, f2);

    cam_ctr_x = c2.x;
    cam_ctr_y = c2.y;
    cam_ctr_z = c2.z;
}

void camera_pitch(float deg) {
    float rad = DEG2RAD(deg);
    Vec3 eye = make_vec3(cam_eye_x, cam_eye_y, cam_eye_z);
    Vec3 center = make_vec3(cam_ctr_x, cam_ctr_y, cam_ctr_z);
    Vec3 up = make_vec3(cam_up_x, cam_up_y, cam_up_z);

    Vec3 f = v_normalize(v_sub(center, eye));
    Vec3 r = v_normalize(v_cross(f, up));  // right
    Vec3 u = v_normalize(up);

    Vec3 f2 = v_rotate_axis(f, r, rad);
    Vec3 u2 = v_rotate_axis(u, r, rad);
    Vec3 c2 = v_add(eye, v_scale(f2, (float)sqrt(v_dot(v_sub(center, eye), v_sub(center, eye)))));

    cam_ctr_x = c2.x;
    cam_ctr_y = c2.y;
    cam_ctr_z = c2.z;
    cam_up_x = u2.x;
    cam_up_y = u2.y;
    cam_up_z = u2.z;
}

void camera_roll(float deg) {
    float rad = DEG2RAD(deg);
    Vec3 eye = make_vec3(cam_eye_x, cam_eye_y, cam_eye_z);
    Vec3 center = make_vec3(cam_ctr_x, cam_ctr_y, cam_ctr_z);
    Vec3 up = make_vec3(cam_up_x, cam_up_y, cam_up_z);

    Vec3 f = v_normalize(v_sub(center, eye));
    Vec3 u2 = v_rotate_axis(up, f, rad);

    cam_up_x = u2.x;
    cam_up_y = u2.y;
    cam_up_z = u2.z;
}

int point_in_rect(float x, float z, float xmin, float xmax, float zmin, float zmax) {
    return (x >= xmin && x <= xmax && z >= zmin && z <= zmax);
}

int can_enter_ramp(const Ramp* r, float px, float pz, float nx, float nz) {
    const float TOL = 1.2f;

    float xmin = fminf(r->x0, r->x1), xmax = fmaxf(r->x0, r->x1);
    float zmin = fminf(r->z0, r->z1), zmax = fmaxf(r->z0, r->z1);

    auto in_rect = [&](float x, float z, float tol) {
        return (x >= xmin - tol && x <= xmax + tol && z >= zmin - tol && z <= zmax + tol);
        };

    int prev_in = in_rect(px, pz, 0.0f);
    int next_in = in_rect(nx, nz, 0.0f);

    if (!next_in)
        return 1;  // 沒打算進這個坡: 不干涉
    if (prev_in)
        return 1;  // 已在坡內: 允許移動

    if (r->alongX) {
        int from_low_side = (px <= xmin + TOL) && (nx >= xmin - TOL) && (nz >= zmin - TOL && nz <= zmax + TOL);
        return from_low_side ? 1 : 0;
    }
    else {
        int from_low_side = (pz <= zmin + TOL) && (nz >= zmin - TOL) && (nx >= xmin - TOL && nx <= xmax + TOL);
        return from_low_side ? 1 : 0;
    }
}

void try_move(float step_forward) {
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

void enter_screensaver_mode() {
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

void leave_screensaver_mode() {
    screensaver_mode = false;
    vp_x = 0;
    vp_y = 0;
    vp_w = gWinW;
    vp_h = gWinH;  // 回到全視窗
}

void my_init(void) {
    glClearColor(0, 0, 0, 1);
    glEnable(GL_DEPTH_TEST);

    // Smooth shading + correct normals after scaling
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);

    // Lighting
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    // A bit nicer specular
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 32.0f);

    ensure_quadrics();
}


/* display 一個「單一視圖」*/
void display_single_view(void) {
    int use_w = (vp_w > 0 ? vp_w : gWinW);
    int use_h = (vp_h > 0 ? vp_h : gWinH);
    int use_x = screensaver_mode ? vp_x : 0;
    int use_y = screensaver_mode ? vp_y : 0;

    glViewport(use_x, use_y, use_w, use_h);

    if (gProjMode == PROJ_ORTHO_X) {
        apply_ortho_view_x(use_w, use_h);
    }
    else if (gProjMode == PROJ_ORTHO_Y) {
        apply_ortho_view_y(use_w, use_h);
    }
    else if (gProjMode == PROJ_ORTHO_Z) {
        apply_ortho_view_z(use_w, use_h);
    }
    else {  // default: perspective
        apply_perspective_view(use_w, use_h);
    }

    // Update light parameters in this view (HW4)
    setup_lights_in_current_view();

    // Polygon mode
    if (gPolyModeFill)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    render_world_scene();

    // 若是正投影模式，畫出透視 view volume
    if (gProjMode == PROJ_ORTHO_X || gProjMode == PROJ_ORTHO_Y || gProjMode == PROJ_ORTHO_Z) {
        draw_perspective_frustum_lines();
    }
}

/* display 四分割視圖 */
void display_four_views(void) {
    int halfW = gWinW / 2;
    int halfH = gWinH / 2;

    // 左下：X-orthographic
    glViewport(0, 0, halfW, halfH);
    apply_ortho_view_x(halfW, halfH);
    setup_lights_in_current_view();
    if (gPolyModeFill)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    render_world_scene();
    draw_perspective_frustum_lines();

    // 右下：Y-orthographic
    glViewport(halfW, 0, halfW, halfH);
    apply_ortho_view_y(halfW, halfH);
    setup_lights_in_current_view();
    if (gPolyModeFill)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    render_world_scene();
    draw_perspective_frustum_lines();

    // 左上：Z-orthographic
    glViewport(0, halfH, halfW, halfH);
    apply_ortho_view_z(halfW, halfH);
    setup_lights_in_current_view();
    if (gPolyModeFill)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    render_world_scene();
    draw_perspective_frustum_lines();

    // 右上：Perspective
    glViewport(halfW, halfH, halfW, halfH);
    apply_perspective_view(halfW, halfH);
    setup_lights_in_current_view();
    if (gPolyModeFill)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    render_world_scene();
}

void display(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (gProjMode == PROJ_FOUR_VIEW) {
        // 四分割模式不使用 screensaver 的 bouncing 視窗
        display_four_views();
    }
    else {
        display_single_view();
    }

    glutSwapBuffers();
}

void idle() {
    static int last_ms = 0;
    int now = glutGet(GLUT_ELAPSED_TIME);
    if (last_ms == 0) last_ms = now;
    float dt = (now - last_ms) / 1000.0f;
    last_ms = now;

    if (screensaver_mode && gProjMode != PROJ_FOUR_VIEW) {
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

void on_reshape(int w, int h) {
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

/*Keyboard*/
/* W/S/A/D、R、1/2、Q、3~7、Z/X、C、V、T/G、I/K/J/L/U/O */
void on_keyboard(unsigned char key, int x, int y) {
    (void)x;
    (void)y;
    switch (key) {
    case 'Q':
    case 'q':
        exit(0);
        break;
    case '1':  // polygon fill
        gPolyModeFill = 1;
        glutPostRedisplay();
        break;
    case '2':  // polygon line
        gPolyModeFill = 0;
        glutPostRedisplay();
        break;

        /* Projection modes */
    case '3':
        gProjMode = PROJ_ORTHO_X;
        glutPostRedisplay();
        break;
    case '4':
        gProjMode = PROJ_ORTHO_Y;
        glutPostRedisplay();
        break;
    case '5':
        gProjMode = PROJ_ORTHO_Z;
        glutPostRedisplay();
        break;
    case '6':
        gProjMode = PROJ_PERSPECTIVE;
        glutPostRedisplay();
        break;
    case '7':
        gProjMode = PROJ_FOUR_VIEW;
        glutPostRedisplay();
        break;

        /* Zoom */
    case 'z':
    case 'Z':
        gZoom *= 1.1f;
        if (gZoom > ZOOM_MAX)
            gZoom = ZOOM_MAX;
        glutPostRedisplay();
        break;
    case 'x':
    case 'X':
        gZoom /= 1.1f;
        if (gZoom < ZOOM_MIN)
            gZoom = ZOOM_MIN;
        glutPostRedisplay();
        break;


        /* Lighting controls */
    case 'm':  // toggle directional light
        gDirLightOn = !gDirLightOn;
        glutPostRedisplay();
        break;
    case 'M':  // cycle directional presets: sun / moon / sunset
        gDirPreset = (gDirPreset + 1) % 3;
        if (gDirPreset == 0) { // sun
            gDirColor[0] = 1.0f; gDirColor[1] = 0.98f; gDirColor[2] = 0.90f;
            gDirIntensity = 0.85f;
        }
        else if (gDirPreset == 1) { // moon
            gDirColor[0] = 0.55f; gDirColor[1] = 0.65f; gDirColor[2] = 1.0f;
            gDirIntensity = 0.45f;
        }
        else { // sunset
            gDirColor[0] = 1.0f; gDirColor[1] = 0.55f; gDirColor[2] = 0.25f;
            gDirIntensity = 0.75f;
        }
        glutPostRedisplay();
        break;
    case ';':  // dir intensity -
        gDirIntensity -= LIGHT_INT_STEP;
        if (gDirIntensity < 0.0f) gDirIntensity = 0.0f;
        glutPostRedisplay();
        break;
    case '\'': // dir intensity +
        gDirIntensity += LIGHT_INT_STEP;
        if (gDirIntensity > 2.5f) gDirIntensity = 2.5f;
        glutPostRedisplay();
        break;

    case 'p':
    case 'P':  // toggle point light
        gPointLightOn = !gPointLightOn;
        glutPostRedisplay();
        break;
    case '[':  // point intensity -
        gPointIntensity -= LIGHT_INT_STEP;
        if (gPointIntensity < 0.0f) gPointIntensity = 0.0f;
        glutPostRedisplay();
        break;
    case ']':  // point intensity +
        gPointIntensity += LIGHT_INT_STEP;
        if (gPointIntensity > 3.0f) gPointIntensity = 3.0f;
        glutPostRedisplay();
        break;
    case 'e':  // point red -
        gPointColor[0] -= LIGHT_COLOR_STEP;
        if (gPointColor[0] < 0.0f) gPointColor[0] = 0.0f;
        glutPostRedisplay();
        break;
    case 'E':  // point red +
        gPointColor[0] += LIGHT_COLOR_STEP;
        if (gPointColor[0] > 1.0f) gPointColor[0] = 1.0f;
        glutPostRedisplay();
        break;
    case 'f':  // point green -
        gPointColor[1] -= LIGHT_COLOR_STEP;
        if (gPointColor[1] < 0.0f) gPointColor[1] = 0.0f;
        glutPostRedisplay();
        break;
    case 'F':  // point green +
        gPointColor[1] += LIGHT_COLOR_STEP;
        if (gPointColor[1] > 1.0f) gPointColor[1] = 1.0f;
        glutPostRedisplay();
        break;
    case 'n':  // point blue -
        gPointColor[2] -= LIGHT_COLOR_STEP;
        if (gPointColor[2] < 0.0f) gPointColor[2] = 0.0f;
        glutPostRedisplay();
        break;
    case 'N':  // point blue +
        gPointColor[2] += LIGHT_COLOR_STEP;
        if (gPointColor[2] > 1.0f) gPointColor[2] = 1.0f;
        glutPostRedisplay();
        break;

    case 'h':
    case 'H':  // toggle car spotlights
        gCarLightOn = !gCarLightOn;
        glutPostRedisplay();
        break;
    case 'b':
    case 'B':  // switch 1/2 headlights
        gCarLightCount = (gCarLightCount == 2) ? 1 : 2;
        glutPostRedisplay();
        break;
    case ',':  // cutoff -
        gCarLightCutoff -= LIGHT_ANGLE_STEP;
        if (gCarLightCutoff < 5.0f) gCarLightCutoff = 5.0f;
        glutPostRedisplay();
        break;
    case '.':  // cutoff +
        gCarLightCutoff += LIGHT_ANGLE_STEP;
        if (gCarLightCutoff > 60.0f) gCarLightCutoff = 60.0f;
        glutPostRedisplay();
        break;
    case '-':  // car light intensity -
        gCarLightIntensity -= LIGHT_INT_STEP;
        if (gCarLightIntensity < 0.0f) gCarLightIntensity = 0.0f;
        glutPostRedisplay();
        break;
    case '=':  // car light intensity +
        gCarLightIntensity += LIGHT_INT_STEP;
        if (gCarLightIntensity > 5.0f) gCarLightIntensity = 5.0f;
        glutPostRedisplay();
        break;
    case '/':  // exponent -
        gCarLightExponent -= 2.0f;
        if (gCarLightExponent < 0.0f) gCarLightExponent = 0.0f;
        glutPostRedisplay();
        break;
    case '?':  // exponent + (Shift + '/')
        gCarLightExponent += 2.0f;
        if (gCarLightExponent > 128.0f) gCarLightExponent = 128.0f;
        glutPostRedisplay();
        break;
    case 'y':  // yaw offset -
        gCarLightYawOffset -= LIGHT_ANGLE_STEP;
        if (gCarLightYawOffset < -45.0f) gCarLightYawOffset = -45.0f;
        glutPostRedisplay();
        break;
    case 'Y':  // yaw offset +
        gCarLightYawOffset += LIGHT_ANGLE_STEP;
        if (gCarLightYawOffset > 45.0f) gCarLightYawOffset = 45.0f;
        glutPostRedisplay();
        break;
    case '8':  // car light pitch down more
        gCarLightPitch -= LIGHT_ANGLE_STEP;
        if (gCarLightPitch < -45.0f) gCarLightPitch = -45.0f;
        glutPostRedisplay();
        break;
    case '9':  // car light pitch up
        gCarLightPitch += LIGHT_ANGLE_STEP;
        if (gCarLightPitch > 10.0f) gCarLightPitch = 10.0f;
        glutPostRedisplay();
        break;


        /* Car / Camera movement */
    case 'w':
    case 'W':
        if (screensaver_mode && gProjMode != PROJ_FOUR_VIEW)
            return;
        if (!cam_edit_mode) {
            try_move(+car_speed);
        }
        else {
            /* Camera surge forward */
            Vec3 eye = make_vec3(cam_eye_x, cam_eye_y, cam_eye_z);
            Vec3 center = make_vec3(cam_ctr_x, cam_ctr_y, cam_ctr_z);
            Vec3 f = v_sub(center, eye);
            camera_translate(f, cam_move_speed);
        }
        glutPostRedisplay();
        break;
    case 's':
    case 'S':
        if (screensaver_mode && gProjMode != PROJ_FOUR_VIEW)
            return;
        if (!cam_edit_mode) {
            try_move(-car_speed);
        }
        else {
            Vec3 eye = make_vec3(cam_eye_x, cam_eye_y, cam_eye_z);
            Vec3 center = make_vec3(cam_ctr_x, cam_ctr_y, cam_ctr_z);
            Vec3 f = v_sub(center, eye);
            camera_translate(f, -cam_move_speed);
        }
        glutPostRedisplay();
        break;
    case 'a':
    case 'A':
        if (screensaver_mode && gProjMode != PROJ_FOUR_VIEW)
            return;
        if (!cam_edit_mode) {
            car_dir_deg += car_turn;
            if (car_dir_deg >= 360.0f)
                car_dir_deg -= 360.0f;
        }
        else {
            /* Camera strafe left */
            Vec3 eye = make_vec3(cam_eye_x, cam_eye_y, cam_eye_z);
            Vec3 center = make_vec3(cam_ctr_x, cam_ctr_y, cam_ctr_z);
            Vec3 up = make_vec3(cam_up_x, cam_up_y, cam_up_z);
            Vec3 f = v_normalize(v_sub(center, eye));
            Vec3 r = v_normalize(v_cross(f, up));
            camera_translate(r, -cam_move_speed);
        }
        glutPostRedisplay();
        break;
    case 'd':
    case 'D':
        if (screensaver_mode && gProjMode != PROJ_FOUR_VIEW)
            return;
        if (!cam_edit_mode) {
            car_dir_deg -= car_turn;
            if (car_dir_deg < 0.0f)
                car_dir_deg += 360.0f;
        }
        else {
            Vec3 eye = make_vec3(cam_eye_x, cam_eye_y, cam_eye_z);
            Vec3 center = make_vec3(cam_ctr_x, cam_ctr_y, cam_ctr_z);
            Vec3 up = make_vec3(cam_up_x, cam_up_y, cam_up_z);
            Vec3 f = v_normalize(v_sub(center, eye));
            Vec3 r = v_normalize(v_cross(f, up));
            camera_translate(r, +cam_move_speed);
        }
        glutPostRedisplay();
        break;

        /* heave up/down (camera edit only) */
    case 't':
    case 'T':
        if (cam_edit_mode) {
            camera_translate(make_vec3(0.0f, 1.0f, 0.0f), cam_move_speed);
        }
        glutPostRedisplay();
        break;
    case 'g':
    case 'G':
        if (cam_edit_mode) {
            camera_translate(make_vec3(0.0f, -1.0f, 0.0f), cam_move_speed);
        }
        glutPostRedisplay();
        break;

    case 'r':
    case 'R':
        if (screensaver_mode && gProjMode != PROJ_FOUR_VIEW)
            return;
        if (!cam_edit_mode) {
            car_x = 10.0f;
            car_z = 10.0f;
            car_dir_deg = 0.0f;
            car_y = terrain_height(car_x, car_z) + CAR_BODY_H * 0.5f + 0.4f;
        }
        else {
            cam_eye_x = 95.0f;
            cam_eye_y = 70.0f;
            cam_eye_z = 95.0f;
            cam_ctr_x = 45.0f;
            cam_ctr_y = 0.0f;
            cam_ctr_z = 45.0f;
            cam_up_x = 0.0f;
            cam_up_y = 1.0f;
            cam_up_z = 0.0f;
            gZoom = 1.0f;
        }
        glutPostRedisplay();
        break;
    case 'c':
    case 'C':
        cam_edit_mode ^= 1;  // 取反
        glutPostRedisplay();
        break;
    case 'v':
    case 'V':
        if (gProjMode == PROJ_FOUR_VIEW) {
            // 四分割模式下就不要啟用 screensaver，比較清楚
            break;
        }
        if (screensaver_mode)
            leave_screensaver_mode();
        else
            enter_screensaver_mode();
        break;

        /* Camera rotations in edit mode (J/L yaw, I/K pitch, U/O roll) */
    case 'j':
    case 'J':
        if (cam_edit_mode) {
            camera_yaw(+cam_rotate_deg);
            glutPostRedisplay();
        }
        else {
            gDirAzimDeg += LIGHT_ANGLE_STEP; if (gDirAzimDeg >= 360.0f) gDirAzimDeg -= 360.0f;
            glutPostRedisplay();
        }
        break;
    case 'l':
    case 'L':
        if (cam_edit_mode) {
            camera_yaw(-cam_rotate_deg);
            glutPostRedisplay();
        }
        else {
            gDirAzimDeg -= LIGHT_ANGLE_STEP; if (gDirAzimDeg < 0.0f) gDirAzimDeg += 360.0f;
            glutPostRedisplay();
        }
        break;
    case 'i':
    case 'I':
        if (cam_edit_mode) {
            camera_pitch(+cam_rotate_deg);
            glutPostRedisplay();
        }
        else {
            gDirElevDeg += LIGHT_ANGLE_STEP; if (gDirElevDeg > 89.0f) gDirElevDeg = 89.0f;
            glutPostRedisplay();
        }
        break;
    case 'k':
    case 'K':
        if (cam_edit_mode) {
            camera_pitch(-cam_rotate_deg);
            glutPostRedisplay();
        }
        else {
            gDirElevDeg -= LIGHT_ANGLE_STEP; if (gDirElevDeg < 1.0f) gDirElevDeg = 1.0f;
            glutPostRedisplay();
        }
        break;
    case 'u':
    case 'U':
        if (cam_edit_mode) {
            camera_roll(+cam_rotate_deg);
            glutPostRedisplay();
        }
        break;
    case 'o':
    case 'O':
        if (cam_edit_mode) {
            camera_roll(-cam_rotate_deg);
            glutPostRedisplay();
        }
        break;

    default:
        break;
    }
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(gWinW, gWinH);
    glutCreateWindow("HW4 - Lighting and Shading");

    my_init();

    /* 初始車高 */
    car_y = terrain_height(car_x, car_z) + CAR_BODY_H * 0.5f + 0.4f;

    glutDisplayFunc(display);
    glutReshapeFunc(on_reshape);
    glutKeyboardFunc(on_keyboard);
    glutIdleFunc(idle);

    glutMainLoop();
    return 0;
}
