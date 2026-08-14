#include <GL/glut.h>
#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <random>
#include <vector>

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
int gDirPreset = 0;

bool gPointLightOn = true;
float gPointPos[3] = { 50.0f, 85.0f, 50.0f };
float gPointColor[3] = { 1.0f, 1.0f, 1.0f };
float gPointIntensity = 0.9f;

bool gCarLightOn = true;
int gCarLightCount = 2;
float gCarLightColor[3] = { 1.0f, 1.0f, 0.92f };
float gCarLightIntensity = 1.2f;
float gCarLightCutoff = 18.0f;
float gCarLightExponent = 20.0f;
float gCarLightYawOffset = 0.0f;
float gCarLightPitch = -8.0f;

const float LIGHT_COLOR_STEP = 0.05f;
const float LIGHT_INT_STEP = 0.10f;
const float LIGHT_ANGLE_STEP = 2.0f;

// ==================== HW5: Textures & Billboards ====================
GLuint gTexFloor = 0, gTexBrick = 0, gTexRamp = 0, gTexWater = 0, gTexSky = 0;
GLuint gTexTree = 0, gTexGrass = 0, gTexCloud = 0;
bool gTexturesReady = false;

struct Billboard {
    float x, y, z;
    float w, h;
    GLuint tex;
    bool grounded;  // y will be derived from terrain if true
};
std::vector<Billboard> gBillboards;

bool gAnimateWater = true;
float gWaterScroll = 0.0f;

bool gFogOn = true;
int gFogMode = 0;  // 0: linear, 1: exp, 2: exp2
float gFogDensity = 0.012f;
float gFogStart = 40.0f, gFogEnd = 220.0f;
float gFogColor[4] = { 0.55f, 0.65f, 0.75f, 1.0f };
int gFogPreset = 0;

GLUquadric* gSkyQuad = NULL;
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
    GLfloat spec[4] = { 0, 0, 0, 1 };
    GLfloat amb[4] = { 0.18f, 0.18f, 0.18f, 1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
}
void set_material_car_shiny() {
    GLfloat spec[4] = { 0.95f, 0.95f, 0.95f, 1 };
    GLfloat amb[4] = { 0.10f, 0.10f, 0.10f, 1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 90.0f);
}
void set_material_glossy() {
    GLfloat spec[4] = { 0.45f, 0.45f, 0.45f, 1 };
    GLfloat amb[4] = { 0.15f, 0.15f, 0.15f, 1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 35.0f);
}
void set_material_ramp() {
    GLfloat spec[4] = { 0.20f, 0.20f, 0.20f, 1 };
    GLfloat amb[4] = { 0.12f, 0.12f, 0.12f, 1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 18.0f);
}

void push_emission(float r, float g, float b) {
    GLfloat emi[4] = { r, g, b, 1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi);
}
void pop_emission() {
    GLfloat emi0[4] = { 0, 0, 0, 1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emi0);
}

void setup_lights_in_current_view() {
    glEnable(GL_LIGHTING);

    GLfloat globalAmb[4] = { 0.16f, 0.16f, 0.16f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmb);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    if (gDirLightOn)
        glEnable(GL_LIGHT0);
    else
        glDisable(GL_LIGHT0);
    {
        float az = DEG2RAD(gDirAzimDeg);
        float el = DEG2RAD(gDirElevDeg);
        GLfloat pos[4] = { cosf(el) * cosf(az), sinf(el), cosf(el) * sinf(az), 0.0f };

        GLfloat amb[4] = { 0.05f * gDirIntensity * gDirColor[0], 0.05f * gDirIntensity * gDirColor[1],
                          0.05f * gDirIntensity * gDirColor[2], 1.0f };
        GLfloat dif[4] = { gDirIntensity * gDirColor[0], gDirIntensity * gDirColor[1], gDirIntensity * gDirColor[2],
                          1.0f };
        GLfloat spe[4] = { 0.9f * gDirIntensity * gDirColor[0], 0.9f * gDirIntensity * gDirColor[1],
                          0.9f * gDirIntensity * gDirColor[2], 1.0f };

        glLightfv(GL_LIGHT0, GL_POSITION, pos);
        glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
        glLightfv(GL_LIGHT0, GL_SPECULAR, spe);
    }

    if (gPointLightOn)
        glEnable(GL_LIGHT1);
    else
        glDisable(GL_LIGHT1);
    {
        GLfloat pos[4] = { gPointPos[0], gPointPos[1], gPointPos[2], 1.0f };
        GLfloat dif[4] = { gPointIntensity * gPointColor[0], gPointIntensity * gPointColor[1],
                          gPointIntensity * gPointColor[2], 1.0f };
        GLfloat spe[4] = { dif[0], dif[1], dif[2], 1.0f };
        GLfloat amb[4] = { 0, 0, 0, 1 };

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
    if (fabsf(gCarLightYawOffset) > 1e-4f)
        dir = v_rotate_axis(dir, up, DEG2RAD(gCarLightYawOffset));
    Vec3 pitchAxis = v_normalize(v_cross(up, dir));
    dir = v_rotate_axis(dir, pitchAxis, DEG2RAD(gCarLightPitch));
    dir = v_normalize(dir);

    auto setup_spot = [&](GLenum lightId, const Vec3& pos, bool enable) {
        if (enable)
            glEnable(lightId);
        else
            glDisable(lightId);

        GLfloat p4[4] = { pos.x, pos.y, pos.z, 1.0f };
        GLfloat dif[4] = { gCarLightIntensity * gCarLightColor[0], gCarLightIntensity * gCarLightColor[1],
                          gCarLightIntensity * gCarLightColor[2], 1.0f };
        GLfloat spe[4] = { dif[0], dif[1], dif[2], 1.0f };
        GLfloat amb[4] = { 0, 0, 0, 1 };
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

// ---------- HW5 texture helpers (procedural, no external image files) ----------
static unsigned char u8_from_float(float v) {
    if (v < 0.0f)
        v = 0.0f;
    if (v > 1.0f)
        v = 1.0f;
    return (unsigned char)(v * 255.0f + 0.5f);
}

static void create_texture_2d(GLuint& texId, int w, int h, const unsigned char* data, bool rgba) {
    if (texId == 0)
        glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    GLenum fmt = rgba ? GL_RGBA : GL_RGB;
    gluBuild2DMipmaps(GL_TEXTURE_2D, fmt, w, h, fmt, GL_UNSIGNED_BYTE, data);
}

static void make_checker_texture(std::vector<unsigned char>& out, int w, int h, int cell) {
    out.assign(w * h * 3, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int cx = x / cell;
            int cy = y / cell;
            bool a = ((cx + cy) % 2) == 0;
            float base = a ? 0.90f : 0.20f;
            float tintG = a ? 0.85f : 0.25f;
            float tintB = a ? 0.75f : 0.30f;
            int i = (y * w + x) * 3;
            out[i + 0] = u8_from_float(base);
            out[i + 1] = u8_from_float(tintG);
            out[i + 2] = u8_from_float(tintB);
        }
    }
}

static void make_brick_texture(std::vector<unsigned char>& out, int w, int h) {
    out.assign(w * h * 3, 0);
    int brickW = 48;
    int brickH = 24;
    int mortar = 3;
    for (int y = 0; y < h; ++y) {
        int row = y / brickH;
        int yIn = y % brickH;
        int xOffset = (row % 2) ? (brickW / 2) : 0;
        for (int x = 0; x < w; ++x) {
            int x2 = (x + xOffset) % w;
            int xIn = x2 % brickW;
            bool isMortar = (xIn < mortar) || (yIn < mortar);
            float r, g, b;
            if (isMortar) {
                r = g = b = 0.72f;
            }
            else {
                float n = 0.04f * sinf(0.08f * x) + 0.04f * cosf(0.10f * y);
                r = 0.60f + n;
                g = 0.22f + 0.5f * n;
                b = 0.16f + 0.3f * n;
            }
            int i = (y * w + x) * 3;
            out[i + 0] = u8_from_float(r);
            out[i + 1] = u8_from_float(g);
            out[i + 2] = u8_from_float(b);
        }
    }
}

static void make_ramp_stripe_texture(std::vector<unsigned char>& out, int w, int h) {
    out.assign(w * h * 3, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float t = (float)x / (float)w;
            float stripe = (fmodf(t * 12.0f, 1.0f) < 0.5f) ? 1.0f : 0.0f;
            float r = 0.20f + 0.55f * stripe;
            float g = 0.20f + 0.55f * stripe;
            float b = 0.20f + 0.05f * stripe;
            int i = (y * w + x) * 3;
            out[i + 0] = u8_from_float(r);
            out[i + 1] = u8_from_float(g);
            out[i + 2] = u8_from_float(b);
        }
    }
}

static void make_water_texture(std::vector<unsigned char>& out, int w, int h) {
    out.assign(w * h * 3, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float fx = (float)x / (float)w;
            float fy = (float)y / (float)h;
            float wv = 0.5f + 0.5f * sinf(2.0f * PI * (fx * 6.0f + fy * 2.0f));
            float r = 0.05f + 0.05f * wv;
            float g = 0.25f + 0.20f * wv;
            float b = 0.35f + 0.35f * wv;
            int i = (y * w + x) * 3;
            out[i + 0] = u8_from_float(r);
            out[i + 1] = u8_from_float(g);
            out[i + 2] = u8_from_float(b);
        }
    }
}

static void make_sky_texture(std::vector<unsigned char>& out, int w, int h) {
    out.assign(w * h * 3, 0);
    for (int y = 0; y < h; ++y) {
        float t = (float)y / (float)(h - 1);
        // bottom -> top
        float r0 = 0.90f, g0 = 0.65f, b0 = 0.35f;  // horizon warm
        float r1 = 0.15f, g1 = 0.35f, b1 = 0.70f;  // zenith blue
        float r = r0 * (1 - t) + r1 * t;
        float g = g0 * (1 - t) + g1 * t;
        float b = b0 * (1 - t) + b1 * t;
        for (int x = 0; x < w; ++x) {
            float n = 0.02f * sinf(0.02f * x) * cosf(0.015f * y);
            int i = (y * w + x) * 3;
            out[i + 0] = u8_from_float(r + n);
            out[i + 1] = u8_from_float(g + n);
            out[i + 2] = u8_from_float(b + n);
        }
    }
}

static void make_tree_texture_rgba(std::vector<unsigned char>& out, int w, int h) {
    out.assign(w * h * 4, 0);
    float cx = 0.5f * w;
    float cy = 0.55f * h;
    float rad = 0.33f * fminf((float)w, (float)h);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float a = 0.0f;
            float r = 0.0f, g = 0.0f, b = 0.0f;

            // trunk
            float tx0 = 0.46f * w, tx1 = 0.54f * w;
            float ty0 = 0.05f * h, ty1 = 0.55f * h;
            if (x >= tx0 && x <= tx1 && y >= ty0 && y <= ty1) {
                a = 1.0f;
                r = 0.35f;
                g = 0.22f;
                b = 0.12f;
            }

            // canopy (three circles)
            float dx = x - cx;
            float dy = y - cy;
            float d = sqrtf(dx * dx + dy * dy);
            float d2 = sqrtf((x - (cx - 0.18f * w)) * (x - (cx - 0.18f * w)) +
                (y - (cy + 0.10f * h)) * (y - (cy + 0.10f * h)));
            float d3 = sqrtf((x - (cx + 0.18f * w)) * (x - (cx + 0.18f * w)) +
                (y - (cy + 0.10f * h)) * (y - (cy + 0.10f * h)));
            if (d < rad || d2 < 0.85f * rad || d3 < 0.85f * rad) {
                a = 1.0f;
                float n = 0.06f * sinf(0.12f * x) * cosf(0.10f * y);
                r = 0.10f + n;
                g = 0.45f + 0.5f * n;
                b = 0.12f + 0.3f * n;
            }

            int i = (y * w + x) * 4;
            out[i + 0] = u8_from_float(r);
            out[i + 1] = u8_from_float(g);
            out[i + 2] = u8_from_float(b);
            out[i + 3] = u8_from_float(a);
        }
    }
}

static void make_grass_texture_rgba(std::vector<unsigned char>& out, int w, int h) {
    out.assign(w * h * 4, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float a = 0.0f;
            // blade pattern: sparse diagonal lines
            float fx = (float)x / (float)w;
            float fy = (float)y / (float)h;
            float v = sinf((fx * 22.0f + fy * 5.0f) * 2.0f * PI);
            if (v > 0.70f && fy < 0.9f)
                a = 0.85f;
            if (fy < 0.2f)
                a *= fy / 0.2f;
            float r = 0.08f, g = 0.50f + 0.15f * fy, b = 0.10f;
            int i = (y * w + x) * 4;
            out[i + 0] = u8_from_float(r);
            out[i + 1] = u8_from_float(g);
            out[i + 2] = u8_from_float(b);
            out[i + 3] = u8_from_float(a);
        }
    }
}

static void make_cloud_texture_rgba(std::vector<unsigned char>& out, int w, int h) {
    out.assign(w * h * 4, 0);
    // few gaussian blobs
    struct Blob {
        float x, y, r;
    } blobs[4] = {
        {0.35f * w, 0.55f * h, 0.28f * h},
        {0.50f * w, 0.60f * h, 0.34f * h},
        {0.65f * w, 0.52f * h, 0.26f * h},
        {0.52f * w, 0.45f * h, 0.22f * h},
    };

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float alpha = 0.0f;
            for (auto& b : blobs) {
                float dx = x - b.x;
                float dy = y - b.y;
                float d2 = (dx * dx + dy * dy) / (b.r * b.r);
                alpha += expf(-2.2f * d2);
            }
            alpha = fminf(alpha, 1.0f);
            float edge = (float)y / (float)h;
            alpha *= (0.9f - 0.3f * fabsf(edge - 0.5f));
            float r = 1.0f, g = 1.0f, b = 1.0f;
            int i = (y * w + x) * 4;
            out[i + 0] = u8_from_float(r);
            out[i + 1] = u8_from_float(g);
            out[i + 2] = u8_from_float(b);
            out[i + 3] = u8_from_float(alpha);
        }
    }
}

static void init_textures_hw5() {
    if (gTexturesReady)
        return;
    std::vector<unsigned char> buf;

    make_checker_texture(buf, 256, 256, 16);
    create_texture_2d(gTexFloor, 256, 256, buf.data(), false);

    make_brick_texture(buf, 256, 256);
    create_texture_2d(gTexBrick, 256, 256, buf.data(), false);

    make_ramp_stripe_texture(buf, 256, 256);
    create_texture_2d(gTexRamp, 256, 256, buf.data(), false);

    make_water_texture(buf, 256, 256);
    create_texture_2d(gTexWater, 256, 256, buf.data(), false);

    make_sky_texture(buf, 512, 256);
    create_texture_2d(gTexSky, 512, 256, buf.data(), false);

    make_tree_texture_rgba(buf, 256, 256);
    create_texture_2d(gTexTree, 256, 256, buf.data(), true);

    make_grass_texture_rgba(buf, 128, 128);
    create_texture_2d(gTexGrass, 128, 128, buf.data(), true);

    make_cloud_texture_rgba(buf, 256, 128);
    create_texture_2d(gTexCloud, 256, 128, buf.data(), true);

    gTexturesReady = true;
}

void draw_light_markers() {
    ensure_quadrics();

    if (gPointLightOn) {
        glPushMatrix();
        glTranslatef(gPointPos[0], gPointPos[1], gPointPos[2]);
        glColor3f(gPointColor[0], gPointColor[1], gPointColor[2]);
        push_emission(gPointIntensity * gPointColor[0], gPointIntensity * gPointColor[1],
            gPointIntensity * gPointColor[2]);
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
            push_emission(gCarLightIntensity * gCarLightColor[0], gCarLightIntensity * gCarLightColor[1],
                gCarLightIntensity * gCarLightColor[2]);
            glutSolidSphere(0.8, 16, 16);
            pop_emission();
            glPopMatrix();
            };

        if (gCarLightCount >= 1)
            draw_one(Lpos);
        if (gCarLightCount >= 2)
            draw_one(Rpos);
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

// ---------- HW5: fog setup (call per viewport after setting view) ----------
static void setup_fog_in_current_view() {
    if (!gFogOn) {
        glDisable(GL_FOG);
        return;
    }
    glEnable(GL_FOG);
    if (gFogMode == 0)
        glFogi(GL_FOG_MODE, GL_LINEAR);
    else if (gFogMode == 1)
        glFogi(GL_FOG_MODE, GL_EXP);
    else
        glFogi(GL_FOG_MODE, GL_EXP2);

    glFogfv(GL_FOG_COLOR, gFogColor);
    glFogf(GL_FOG_DENSITY, gFogDensity);
    glFogf(GL_FOG_START, gFogStart);
    glFogf(GL_FOG_END, gFogEnd);

    glHint(GL_FOG_HINT, GL_NICEST);
}

// ---------- HW5: textured box with tiling (used for buildings, etc.) ----------
static void draw_box_tiled(float sx, float sy, float sz, float tileUnit) {
    const float hx = 0.5f * sx;
    const float hy = 0.5f * sy;
    const float hz = 0.5f * sz;

    const float ux = (tileUnit > 1e-6f) ? (sx / tileUnit) : 1.0f;
    const float uy = (tileUnit > 1e-6f) ? (sy / tileUnit) : 1.0f;
    const float uz = (tileUnit > 1e-6f) ? (sz / tileUnit) : 1.0f;

    // -Z
    glNormal3f(0, 0, -1);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(-hx, -hy, -hz);
    glTexCoord2f(ux, 0);
    glVertex3f(+hx, -hy, -hz);
    glTexCoord2f(ux, uy);
    glVertex3f(+hx, +hy, -hz);
    glTexCoord2f(0, uy);
    glVertex3f(-hx, +hy, -hz);
    glEnd();

    // +Z
    glNormal3f(0, 0, 1);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(-hx, -hy, +hz);
    glTexCoord2f(ux, 0);
    glVertex3f(+hx, -hy, +hz);
    glTexCoord2f(ux, uy);
    glVertex3f(+hx, +hy, +hz);
    glTexCoord2f(0, uy);
    glVertex3f(-hx, +hy, +hz);
    glEnd();

    // -Y
    glNormal3f(0, -1, 0);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(-hx, -hy, -hz);
    glTexCoord2f(ux, 0);
    glVertex3f(+hx, -hy, -hz);
    glTexCoord2f(ux, uz);
    glVertex3f(+hx, -hy, +hz);
    glTexCoord2f(0, uz);
    glVertex3f(-hx, -hy, +hz);
    glEnd();

    // +Y
    glNormal3f(0, 1, 0);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(-hx, +hy, -hz);
    glTexCoord2f(ux, 0);
    glVertex3f(+hx, +hy, -hz);
    glTexCoord2f(ux, uz);
    glVertex3f(+hx, +hy, +hz);
    glTexCoord2f(0, uz);
    glVertex3f(-hx, +hy, +hz);
    glEnd();

    // +X
    glNormal3f(1, 0, 0);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(+hx, -hy, -hz);
    glTexCoord2f(uz, 0);
    glVertex3f(+hx, -hy, +hz);
    glTexCoord2f(uz, uy);
    glVertex3f(+hx, +hy, +hz);
    glTexCoord2f(0, uy);
    glVertex3f(+hx, +hy, -hz);
    glEnd();

    // -X
    glNormal3f(-1, 0, 0);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(-hx, -hy, -hz);
    glTexCoord2f(uz, 0);
    glVertex3f(-hx, -hy, +hz);
    glTexCoord2f(uz, uy);
    glVertex3f(-hx, +hy, +hz);
    glTexCoord2f(0, uy);
    glVertex3f(-hx, +hy, -hz);
    glEnd();
}

// ---------- HW5: billboard population (many instances) ----------
static void init_billboards() {
    gBillboards.clear();

    auto push_one = [&](float x, float z, float w, float h, GLuint tex, bool grounded, float y) {
        Billboard b;
        b.x = x;
        b.z = z;
        b.w = w;
        b.h = h;
        b.tex = tex;
        b.grounded = grounded;
        b.y = y;
        gBillboards.push_back(b);
        };

    // random placement
    std::mt19937 rng((unsigned)std::random_device{}());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    auto rnd01 = [&]() { return dist(rng); };

    // deterministic placement
    /*unsigned int seed = 12345u;
    auto rnd01 = [&]() {
        seed = 1664525u * seed + 1013904223u;
        return (seed & 0xFFFFFF) / float(0x1000000);
        };*/

        // Trees: 20
    for (int i = 0; i < 20; ++i) {
        for (int tries = 0; tries < 100; ++tries) {
            float x = 5.0f + rnd01() * 90.0f;
            float z = 5.0f + rnd01() * 90.0f;
            if (hit_building_xy(x, z, 2.0f))
                continue;
            // avoid car start area
            if (fabsf(x - 10.0f) < 10.0f && fabsf(z - 10.0f) < 10.0f)
                continue;
            float y = terrain_height(x, z);
            push_one(x, z, 8.0f + rnd01() * 3.0f, 14.0f + rnd01() * 6.0f, gTexTree, true, y);
            break;
        }
    }

    // Grass: 60
    for (int i = 0; i < 60; ++i) {
        for (int tries = 0; tries < 60; ++tries) {
            float x = rnd01() * 100.0f;
            float z = rnd01() * 100.0f;
            if (hit_building_xy(x, z, 1.0f))
                continue;
            float y = terrain_height(x, z);
            push_one(x, z, 2.2f + rnd01() * 1.2f, 1.6f + rnd01() * 0.8f, gTexGrass, true, y);
            break;
        }
    }

    // Clouds: 8 (not grounded)
    for (int i = 0; i < 8; ++i) {
        float x = rnd01() * 100.0f;
        float z = rnd01() * 100.0f;
        float y = 85.0f + rnd01() * 35.0f;
        push_one(x, z, 30.0f + rnd01() * 18.0f, 14.0f + rnd01() * 8.0f, gTexCloud, false, y);
    }
}

// ---------- HW5: sky dome ----------
static void draw_sky_dome() {
    ensure_quadrics();
    if (!gSkyQuad) {
        gSkyQuad = gluNewQuadric();
        gluQuadricDrawStyle(gSkyQuad, GLU_FILL);
        gluQuadricNormals(gSkyQuad, GLU_SMOOTH);
        gluQuadricTexture(gSkyQuad, GL_TRUE);
        gluQuadricOrientation(gSkyQuad, GLU_INSIDE);
    }

    GLboolean lit = glIsEnabled(GL_LIGHTING);
    GLboolean tex = glIsEnabled(GL_TEXTURE_2D);

    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, gTexSky);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    glDepthMask(GL_FALSE);
    glPushMatrix();
    // Centered on camera to avoid parallax
    glTranslatef(cam_eye_x, cam_eye_y, cam_eye_z);
    glScalef(1.0f, 1.0f, 1.0f);
    gluSphere(gSkyQuad, 260.0, 32, 24);
    glPopMatrix();
    glDepthMask(GL_TRUE);

    if (!tex)
        glDisable(GL_TEXTURE_2D);
    if (lit)
        glEnable(GL_LIGHTING);
}

// ---------- HW5: water plane with animated texture matrix ----------
static void draw_water_plane(float x0, float z0, float x1, float z1, float y) {
    if (gTexWater == 0)
        return;

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, gTexWater);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    // animate texture coordinates via texture matrix
    glMatrixMode(GL_TEXTURE);
    glPushMatrix();
    glLoadIdentity();
    if (gAnimateWater) {
        glTranslatef(gWaterScroll, gWaterScroll * 0.5f, 0.0f);
    }
    glMatrixMode(GL_MODELVIEW);

    // slightly shiny
    GLfloat spec[4] = { 0.55f, 0.55f, 0.55f, 1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64.0f);

    glColor3f(0.75f, 0.85f, 0.95f);
    glNormal3f(0, 1, 0);

    float til = 6.0f;
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(x0, y, z0);
    glTexCoord2f(til, 0);
    glVertex3f(x1, y, z0);
    glTexCoord2f(til, til);
    glVertex3f(x1, y, z1);
    glTexCoord2f(0, til);
    glVertex3f(x0, y, z1);
    glEnd();

    // restore texture matrix
    glMatrixMode(GL_TEXTURE);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glDisable(GL_TEXTURE_2D);
}

// ---------- HW5: draw many billboards (sorted back-to-front) ----------
static void draw_billboards() {
    if (gBillboards.empty())
        return;

    struct Item {
        float d2;
        Billboard b;
    };
    std::vector<Item> items;
    items.reserve(gBillboards.size());

    for (const auto& b : gBillboards) {
        Billboard bb = b;
        if (bb.grounded)
            bb.y = terrain_height(bb.x, bb.z);
        float dx = cam_eye_x - bb.x;
        float dy = cam_eye_y - bb.y;
        float dz = cam_eye_z - bb.z;
        items.push_back({ dx * dx + dy * dy + dz * dz, bb });
    }

    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.d2 > b.d2; });

    GLboolean lit = glIsEnabled(GL_LIGHTING);
    glDisable(GL_LIGHTING);

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GREATER, 0.08f);

    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    for (const auto& it : items) {
        const Billboard& b = it.b;

        // yaw so that quad normal (+Z) points to camera in XZ plane
        float dx = cam_eye_x - b.x;
        float dz = cam_eye_z - b.z;
        float yawDeg = atan2f(dx, dz) * 57.29578f;

        glBindTexture(GL_TEXTURE_2D, b.tex);

        glPushMatrix();
        glTranslatef(b.x, b.y, b.z);
        glRotatef(yawDeg, 0, 1, 0);

        float hw = 0.5f * b.w;
        float hh = b.h;

        glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex3f(-hw, 0.0f, 0.0f);
        glTexCoord2f(1, 0);
        glVertex3f(+hw, 0.0f, 0.0f);
        glTexCoord2f(1, 1);
        glVertex3f(+hw, hh, 0.0f);
        glTexCoord2f(0, 1);
        glVertex3f(-hw, hh, 0.0f);
        glEnd();

        glPopMatrix();
    }

    glDisable(GL_ALPHA_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_TEXTURE_2D);
    if (lit)
        glEnable(GL_LIGHTING);
}

void draw_unit_cube(void) {
    /* 一個 1x1x1 立方體，中心在原點 (含法向量) */
    const GLfloat v[8][3] = {
        {-0.5f, -0.5f, -0.5f},
        {0.5f,  -0.5f, -0.5f},
        {0.5f,  0.5f,  -0.5f},
        {-0.5f, 0.5f,  -0.5f},
        {-0.5f, -0.5f, 0.5f },
        {0.5f,  -0.5f, 0.5f },
        {0.5f,  0.5f,  0.5f },
        {-0.5f, 0.5f,  0.5f }
    };

    struct Face {
        int a, b, c, d;
        float nx, ny, nz;
    };
    const Face faces[6] = {
        {0, 1, 2, 3, 0,  0,  -1}, // -Z
        {4, 5, 6, 7, 0,  0,  1 }, // +Z
        {0, 4, 5, 1, 0,  -1, 0 }, // -Y
        {3, 2, 6, 7, 0,  1,  0 }, // +Y
        {1, 5, 6, 2, 1,  0,  0 }, // +X
        {0, 3, 7, 4, -1, 0,  0 }, // -X
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
    // Textured ground plane (repeat texture) - uses GL_MODULATE for shading
    set_material_ground();
    glNormal3f(0.0f, 1.0f, 0.0f);

    const float y = BASE_Y - EPS;
    if (gTexturesReady && gTexFloor != 0) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, gTexFloor);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glColor3f(1.0f, 1.0f, 1.0f);

        const float repeat = 10.0f;  // 0..100 => 10 repeats
        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f);
        glVertex3f(0.0f, y, 0.0f);
        glTexCoord2f(repeat, 0.0f);
        glVertex3f(100.0f, y, 0.0f);
        glTexCoord2f(repeat, repeat);
        glVertex3f(100.0f, y, 100.0f);
        glTexCoord2f(0.0f, repeat);
        glVertex3f(0.0f, y, 100.0f);
        glEnd();

        glDisable(GL_TEXTURE_2D);
    }
    else {
        // Fallback: plain color
        glColor3f(0.35f, 0.35f, 0.35f);
        glBegin(GL_QUADS);
        glVertex3f(0.0f, y, 0.0f);
        glVertex3f(100.0f, y, 0.0f);
        glVertex3f(100.0f, y, 100.0f);
        glVertex3f(0.0f, y, 100.0f);
        glEnd();
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
}

void draw_buildings(void) {
    set_material_glossy();

    GLboolean texWas = glIsEnabled(GL_TEXTURE_2D);
    if (gTexturesReady && gTexBrick != 0) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, gTexBrick);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    }

    for (int i = 0; i < gNumBlds; ++i) {
        float xmin = fminf(gBlds[i].x0, gBlds[i].x1);
        float xmax = fmaxf(gBlds[i].x0, gBlds[i].x1);
        float zmin = fminf(gBlds[i].z0, gBlds[i].z1);
        float zmax = fmaxf(gBlds[i].z0, gBlds[i].z1);
        float cx = 0.5f * (xmin + xmax);
        float cz = 0.5f * (zmin + zmax);
        float w = (xmax - xmin);
        float d = (zmax - zmin);
        float h = gBlds[i].h;
        float y0 = terrain_height(cx, cz);

        glPushMatrix();
        glTranslatef(cx, y0 + h * 0.5f, cz);
        glColor3f(1.0f, 1.0f, 1.0f);
        // Tile the brick texture roughly every 4 world units
        draw_box_tiled(w, h, d, 4.0f);
        glPopMatrix();
    }

    if (!texWas)
        glDisable(GL_TEXTURE_2D);
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

    if (lit)
        glEnable(GL_LIGHTING);
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
        GLfloat spec0[4] = { 0, 0, 0, 1 };
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec0);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);

        glColor3f(0.1f, 0.1f, 0.1f);
        float wy = -CAR_BODY_H * 0.5f + 0.8f;  // 輪胎中心高度
        float dx = CAR_HALF_L - 1.0f;
        float dz = CAR_HALF_W + 0.6f;

        glPushMatrix();
        glTranslatef(-dx, wy, dz);
        glutSolidTorus(0.5, 1.0, 24, 24);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(-dx, wy, -dz);
        glutSolidTorus(0.5, 1.0, 24, 24);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(dx, wy, dz);
        glutSolidTorus(0.5, 1.0, 24, 24);
        glPopMatrix();
        glPushMatrix();
        glTranslatef(dx, wy, -dz);
        glutSolidTorus(0.5, 1.0, 24, 24);
        glPopMatrix();

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
    if (lit)
        glEnable(GL_LIGHTING);
}

/*==================== World Rendering (independent of view/projection) ====================*/
void render_world_scene(void) {
    // Sky goes first (depth write off inside)
    draw_sky_dome();

    // Opaque world
    draw_floor_tiles();
    draw_ramps();
    draw_buildings();

    // Animated water patch (texture matrix scrolling)
    draw_water_plane(40.0f, 15.0f, 65.0f, 40.0f, BASE_Y + 0.25f);

    // World axes
    draw_world_axes_at(0.0f, 0.0f, 0.0f, 3.0f);

    // Car pose: position (car_x, car_y, car_z) + yaw car_dir_deg
    // Use terrain sampling to add a bit of pitch/roll
    float dydx = 0.0f, dydz = 0.0f;
    terrain_gradient(car_x, car_z, &dydx, &dydz);

    float yaw = car_dir_deg * (PI / 180.0f);
    float fx = cosf(yaw), fz = -sinf(yaw);  // forward
    float rx = sinf(yaw), rz = cosf(yaw);   // right

    const float sample_half = 3.0f;
    const float xF = car_x + fx * sample_half, zF = car_z + fz * sample_half;
    const float xB = car_x - fx * sample_half, zB = car_z - fz * sample_half;
    float pitch_deg = atan2f(terrain_height(xF, zF) - terrain_height(xB, zB), 2.0f * sample_half) * 57.29578f;

    const float sample_half_w = 2.0f;
    const float xR = car_x + rx * sample_half_w, zR = car_z + rz * sample_half_w;
    const float xL = car_x - rx * sample_half_w, zL = car_z - rz * sample_half_w;
    float roll_deg = atan2f(terrain_height(xR, zR) - terrain_height(xL, zL), 2.0f * sample_half_w) * 57.29578f;

    glPushMatrix();
    glTranslatef(car_x, car_y, car_z);
    glRotatef(car_dir_deg, 0, 1, 0);
    glRotatef(pitch_deg, rx, 0.0f, rz);
    glRotatef(-roll_deg, fx, 0.0f, fz);
    draw_car();
    glPopMatrix();

    // Light markers (opaque)
    draw_light_markers();

    // Transparent things at the end
    draw_billboards();
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

    // Texture perspective correctness
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    ensure_quadrics();
    init_textures_hw5();
    init_billboards();
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
    setup_fog_in_current_view();

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
    setup_fog_in_current_view();
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
    setup_fog_in_current_view();
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
    setup_fog_in_current_view();
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
    setup_fog_in_current_view();
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
    if (last_ms == 0)
        last_ms = now;
    float dt = (now - last_ms) / 1000.0f;
    last_ms = now;

    // HW5: animate water texture scroll
    if (gAnimateWater) {
        gWaterScroll += dt * 0.20f;
        if (gWaterScroll > 1.0f)
            gWaterScroll -= floorf(gWaterScroll);
    }

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
        if (gDirPreset == 0) {  // sun
            gDirColor[0] = 1.0f;
            gDirColor[1] = 0.98f;
            gDirColor[2] = 0.90f;
            gDirIntensity = 0.85f;
        }
        else if (gDirPreset == 1) {  // moon
            gDirColor[0] = 0.55f;
            gDirColor[1] = 0.65f;
            gDirColor[2] = 1.0f;
            gDirIntensity = 0.45f;
        }
        else {  // sunset
            gDirColor[0] = 1.0f;
            gDirColor[1] = 0.55f;
            gDirColor[2] = 0.25f;
            gDirIntensity = 0.75f;
        }
        glutPostRedisplay();
        break;
    case ';':  // dir intensity -
        gDirIntensity -= LIGHT_INT_STEP;
        if (gDirIntensity < 0.0f)
            gDirIntensity = 0.0f;
        glutPostRedisplay();
        break;
    case '\'':  // dir intensity +
        gDirIntensity += LIGHT_INT_STEP;
        if (gDirIntensity > 2.5f)
            gDirIntensity = 2.5f;
        glutPostRedisplay();
        break;

    case 'p':
    case 'P':  // toggle point light
        gPointLightOn = !gPointLightOn;
        glutPostRedisplay();
        break;
    case '[':  // point intensity -
        gPointIntensity -= LIGHT_INT_STEP;
        if (gPointIntensity < 0.0f)
            gPointIntensity = 0.0f;
        glutPostRedisplay();
        break;
    case ']':  // point intensity +
        gPointIntensity += LIGHT_INT_STEP;
        if (gPointIntensity > 3.0f)
            gPointIntensity = 3.0f;
        glutPostRedisplay();
        break;
    case 'e':  // point red -
        gPointColor[0] -= LIGHT_COLOR_STEP;
        if (gPointColor[0] < 0.0f)
            gPointColor[0] = 0.0f;
        glutPostRedisplay();
        break;
    case 'E':  // point red +
        gPointColor[0] += LIGHT_COLOR_STEP;
        if (gPointColor[0] > 1.0f)
            gPointColor[0] = 1.0f;
        glutPostRedisplay();
        break;
    case 'f':  // point green -
        gPointColor[1] -= LIGHT_COLOR_STEP;
        if (gPointColor[1] < 0.0f)
            gPointColor[1] = 0.0f;
        glutPostRedisplay();
        break;
    case 'F':  // point green +
        gPointColor[1] += LIGHT_COLOR_STEP;
        if (gPointColor[1] > 1.0f)
            gPointColor[1] = 1.0f;
        glutPostRedisplay();
        break;
    case 'n':  // point blue -
        gPointColor[2] -= LIGHT_COLOR_STEP;
        if (gPointColor[2] < 0.0f)
            gPointColor[2] = 0.0f;
        glutPostRedisplay();
        break;
    case 'N':  // point blue +
        gPointColor[2] += LIGHT_COLOR_STEP;
        if (gPointColor[2] > 1.0f)
            gPointColor[2] = 1.0f;
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
        if (gCarLightCutoff < 5.0f)
            gCarLightCutoff = 5.0f;
        glutPostRedisplay();
        break;
    case '.':  // cutoff +
        gCarLightCutoff += LIGHT_ANGLE_STEP;
        if (gCarLightCutoff > 60.0f)
            gCarLightCutoff = 60.0f;
        glutPostRedisplay();
        break;
    case '-':  // car light intensity -
        gCarLightIntensity -= LIGHT_INT_STEP;
        if (gCarLightIntensity < 0.0f)
            gCarLightIntensity = 0.0f;
        glutPostRedisplay();
        break;
    case '=':  // car light intensity +
        gCarLightIntensity += LIGHT_INT_STEP;
        if (gCarLightIntensity > 5.0f)
            gCarLightIntensity = 5.0f;
        glutPostRedisplay();
        break;
    case '/':  // exponent -
        gCarLightExponent -= 2.0f;
        if (gCarLightExponent < 0.0f)
            gCarLightExponent = 0.0f;
        glutPostRedisplay();
        break;
    case '?':  // exponent + (Shift + '/')
        gCarLightExponent += 2.0f;
        if (gCarLightExponent > 128.0f)
            gCarLightExponent = 128.0f;
        glutPostRedisplay();
        break;
    case 'y':  // yaw offset -
        gCarLightYawOffset -= LIGHT_ANGLE_STEP;
        if (gCarLightYawOffset < -45.0f)
            gCarLightYawOffset = -45.0f;
        glutPostRedisplay();
        break;
    case 'Y':  // yaw offset +
        gCarLightYawOffset += LIGHT_ANGLE_STEP;
        if (gCarLightYawOffset > 45.0f)
            gCarLightYawOffset = 45.0f;
        glutPostRedisplay();
        break;
    case '8':  // car light pitch down more
        gCarLightPitch -= LIGHT_ANGLE_STEP;
        if (gCarLightPitch < -45.0f)
            gCarLightPitch = -45.0f;
        glutPostRedisplay();
        break;
    case '9':  // car light pitch up
        gCarLightPitch += LIGHT_ANGLE_STEP;
        if (gCarLightPitch > 10.0f)
            gCarLightPitch = 10.0f;
        glutPostRedisplay();
        break;

        // ==================== HW5 controls ====================
    case '0':  // toggle fog
        gFogOn = !gFogOn;
        glutPostRedisplay();
        break;
    case '!':  // cycle fog mode (Linear -> Exp -> Exp2)
        gFogMode = (gFogMode + 1) % 3;
        glutPostRedisplay();
        break;
    case '(':  // fog density -
        gFogDensity -= 0.002f;
        if (gFogDensity < 0.0f)
            gFogDensity = 0.0f;
        glutPostRedisplay();
        break;
    case ')':  // fog density +
        gFogDensity += 0.002f;
        if (gFogDensity > 0.10f)
            gFogDensity = 0.10f;
        glutPostRedisplay();
        break;
    case '$':  // cycle fog color preset
        gFogPreset = (gFogPreset + 1) % 3;
        if (gFogPreset == 0) {
            gFogColor[0] = 0.55f;
            gFogColor[1] = 0.65f;
            gFogColor[2] = 0.75f;
        }
        else if (gFogPreset == 1) {
            gFogColor[0] = 0.75f;
            gFogColor[1] = 0.75f;
            gFogColor[2] = 0.75f;
        }
        else {
            gFogColor[0] = 0.80f;
            gFogColor[1] = 0.70f;
            gFogColor[2] = 0.55f;
        }
        glutPostRedisplay();
        break;
    case '~':  // toggle water scrolling (texture matrix animation)
        gAnimateWater = !gAnimateWater;
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
            gDirAzimDeg += LIGHT_ANGLE_STEP;
            if (gDirAzimDeg >= 360.0f)
                gDirAzimDeg -= 360.0f;
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
            gDirAzimDeg -= LIGHT_ANGLE_STEP;
            if (gDirAzimDeg < 0.0f)
                gDirAzimDeg += 360.0f;
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
            gDirElevDeg += LIGHT_ANGLE_STEP;
            if (gDirElevDeg > 89.0f)
                gDirElevDeg = 89.0f;
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
            gDirElevDeg -= LIGHT_ANGLE_STEP;
            if (gDirElevDeg < 1.0f)
                gDirElevDeg = 1.0f;
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
    glutCreateWindow("HW5 - Texture Mapping & Billboards");

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
