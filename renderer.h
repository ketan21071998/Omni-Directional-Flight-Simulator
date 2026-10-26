#pragma once
// =========================================================
//  Renderer – SFML window + raw OpenGL 2.1 / fixed-function
//  Matches the Panda3D Python reference visually.
//
//  Coordinate conversion:
//    EUS:   X right, Y up,   Z south/back
//    OpenGL:X right, Y up,   Z toward viewer (right-hand)
//
//    eus_to_gl(x,y,z) → (-x, y, z)   (flip X to match orientation)
//    Actually we keep X=X, Y=Y and NEGATE Z so +Z_eus = -Z_gl (into screen)
//    i.e. gl_pos = (eus.x, eus.y, -eus.z)
// =========================================================
#include "stl_mesh.h"
#include "physics.h"
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <GL/glu.h>
#include <cmath>
#include <optional>
#include <iostream>
#include <chrono>
#include <string>
#include <array>
#include <vector>
#include <algorithm>

// Small helper to negate Z for EUS→OpenGL
static inline void glVertex_EUS(double x, double y, double z)
{ glVertex3d(x, y, -z); }

// =========================================================
// Draw helpers
// =========================================================

static void draw_box(double cx, double cy, double cz,
                     double sx, double sy, double sz)
{
    double x0=cx, x1=cx+sx;
    double y0=cy, y1=cy+sy;
    double z0=cz, z1=cz+sz;

    // Negate Z component for OpenGL
    auto v = [&](double x,double y,double z){ glVertex3d(x,y,-z); };

    glBegin(GL_QUADS);
    // bottom
    glNormal3d(0,-1,0); v(x0,y0,z0); v(x1,y0,z0); v(x1,y0,z1); v(x0,y0,z1);
    // top
    glNormal3d(0,1,0);  v(x0,y1,z0); v(x0,y1,z1); v(x1,y1,z1); v(x1,y1,z0);
    // front (−Z in EUS = +Z in GL → pointing toward viewer)
    glNormal3d(0,0,-1); v(x0,y0,z0); v(x0,y1,z0); v(x1,y1,z0); v(x1,y0,z0);
    // back
    glNormal3d(0,0,1);  v(x0,y0,z1); v(x1,y0,z1); v(x1,y1,z1); v(x0,y1,z1);
    // left
    glNormal3d(-1,0,0); v(x0,y0,z0); v(x0,y0,z1); v(x0,y1,z1); v(x0,y1,z0);
    // right
    glNormal3d(1,0,0);  v(x1,y0,z0); v(x1,y1,z0); v(x1,y1,z1); v(x1,y0,z1);
    glEnd();
}

static void draw_cylinder_y(double radius, double height, int segs=16)
{
    // Vertical cylinder centred at origin (along Y axis)
    glBegin(GL_QUAD_STRIP);
    for(int i=0;i<=segs;i++){
        double a = 2*PI*i/segs;
        double nx=std::cos(a), nz=std::sin(a);
        glNormal3d(nx,0,nz);
        glVertex3d(nx*radius, 0,       nz*radius);
        glVertex3d(nx*radius, height,  nz*radius);
    }
    glEnd();
    // Caps
    glNormal3d(0,-1,0);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3d(0,0,0);
    for(int i=segs;i>=0;i--){
        double a=2*PI*i/segs;
        glVertex3d(std::cos(a)*radius,0,std::sin(a)*radius);
    }
    glEnd();
    glNormal3d(0,1,0);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3d(0,height,0);
    for(int i=0;i<=segs;i++){
        double a=2*PI*i/segs;
        glVertex3d(std::cos(a)*radius,height,std::sin(a)*radius);
    }
    glEnd();
}

// =========================================================
// 4×4 matrix utils (OpenGL column-major)
// =========================================================
static void mat4_identity(float M[16]){
    for(int i=0;i<16;i++) M[i]=0;
    M[0]=M[5]=M[10]=M[15]=1;
}

static void mat4_multiply(float out[16],const float A[16],const float B[16]){
    float tmp[16]={};
    for(int r=0;r<4;r++) for(int c=0;c<4;c++)
        for(int k=0;k<4;k++) tmp[c*4+r]+=A[k*4+r]*B[c*4+k];
    for(int i=0;i<16;i++) out[i]=tmp[i];
}

// Build rotation matrix from euler angles (degrees) pitch,yaw,roll → EUS→GL
static void apply_drone_rotation(double pitch_d, double yaw_d, double roll_d)
{
    // Panda3D uses HPR = H=yaw, P=pitch, R=roll
    // We replicate: glRotate yaw around Y, pitch around X, roll around Z
    // but negate yaw sign (same as Python: setHpr(-yaw, pitch, roll))
    glRotated( yaw_d,  0,1,0);
    glRotated(-pitch_d,1,0,0);
    glRotated( roll_d, 0,0,1);
}

// =========================================================
// Renderer
// =========================================================
class Renderer {
public:
    explicit Renderer(SimState& state) : state_(state) {}

    void run()
    {
        sf::ContextSettings settings;
        settings.depthBits   = 24;
        settings.stencilBits = 8;
        settings.antiAliasingLevel = 4;
        settings.majorVersion = 2;
        settings.minorVersion = 1;

        sf::RenderWindow window(
            sf::VideoMode({1280, 720}),
            "Drone Simulation",
            sf::Style::Default,
            sf::State::Windowed,
            settings
        );
        window.setVerticalSyncEnabled(false);
        window.setFramerateLimit(0);   // unlocked – physics accumulator handles it

        init_gl(window);

        body_mesh_.load("models/BODY.stl");
        // motor.stl intentionally excluded — motor mesh not drawn.
        propeller_mesh_.load("models/propeller.stl");
        arm1_mesh_.load("models/arm_1.stl");
        arm2_mesh_.load("models/arm_2.stl");

        // If any mesh failed to load, it draws nothing at all and the
        // console error above is your only clue. This is by far the
        // most common reason the drone appears "invisible" -- flag it
        // loudly here too so it isn't missed.
        // (motor_mesh_ intentionally excluded from this check — it's
        // never loaded on purpose, so it staying "empty" isn't an error.)
        if(body_mesh_.empty() || propeller_mesh_.empty()
           || arm1_mesh_.empty() || arm2_mesh_.empty())
        {
            std::cerr << "\n"
                "[SIM] *** WARNING: one or more drone meshes failed to "
                "load. The drone will be INVISIBLE (a failed mesh draws "
                "nothing, it is not an error at render time). Check the "
                "[STL] messages above, and verify models/*.stl exist "
                "relative to the directory you launched the .exe from "
                "(not the project/source folder). ***\n\n";
        }
        // Font for HUD
        sf::Font font;

        // Try a font shipped alongside the exe first (portable), then
        // fall back to the hardcoded Windows system path. Previously,
        // if the system path didn't exist, has_font silently became
        // false and the ENTIRE HUD (PWM bars, mission info, distance)
        // disappeared with no indication why.
        bool has_font =
            font.openFromFile("assets/arial.ttf") ||
            font.openFromFile("C:/Windows/Fonts/arial.ttf");

        if(!has_font)
        {
            std::cerr << "[SIM] WARN: no font found (tried "
                         "assets/arial.ttf and "
                         "C:/Windows/Fonts/arial.ttf) — HUD text will "
                         "not be drawn.\n";
        }

        // Camera orbit state
        float cam_dist  = 6.0f;
        float cam_yaw   = 45.0f;
        float cam_pitch = -18.0f;
        bool  mouse_held= false;
        sf::Vector2i last_mouse{};

        // Motor spin angles
        float motor_angle[4]={0,0,0,0};

        // Smooth visual state (lerp)
        Vec3 vis_pos  = state_.position;
        Vec3 vis_rot  = state_.rotation;

        // Physics accumulator
        double accumulator = 0.0;
        auto   last_time   = std::chrono::steady_clock::now();

        // FPS counter
        int    frame_count=0;
        double fps=0;
        auto   fps_timer = std::chrono::steady_clock::now();

        while(window.isOpen())
        {
            // ---- timing ----
            auto now   = std::chrono::steady_clock::now();
            double frame_dt = std::chrono::duration<double>(now - last_time).count();
            last_time  = now;
            frame_dt   = std::min(frame_dt, 0.05);
            accumulator += frame_dt;

            // ---- physics ----
            while(accumulator >= DT){
                physics_step(state_);
                accumulator -= DT;
            }

            // ---- snapshot ----
            Vec3 pos, rot;
            std::array<double,4> pwm_local;
            std::array<double,4> joint1_local;
            std::array<double,4> joint2_local;
            Vec3 target;
            double sim_t;
            {
                std::lock_guard<std::mutex> lk(state_.mtx);
                pos          = state_.position;
                rot          = state_.rotation;
                pwm_local    = state_.pwm;
                joint1_local = state_.joint1_deg;
                joint2_local = state_.joint2_deg;
                target       = state_.target;
                sim_t        = state_.sim_time;
            }

            // Smooth interpolation (matches Python 0.85/0.15 lerp)
            vis_pos = vis_pos*0.85 + pos*0.15;
            vis_rot = vis_rot*0.85 + rot*0.15;

            // Motor spin  (direction: FR=+, FL=-, BR=-, BL=+)
            int dir[4]={1,-1,-1,1};
            

            std::array<double,4> rpm_local;

            {
                std::lock_guard<std::mutex> lk(state_.mtx);
                rpm_local = state_.rpm;
            }

            for(int i=0;i<4;i++)
            {
                double rpm = rpm_local[i];

                motor_angle[i] += dir[i] *
                    static_cast<float>((rpm * 360.0 / 60.0) * DT);
            }

            // FPS
            frame_count++;
            double fps_elapsed = std::chrono::duration<double>(now-fps_timer).count();
            if(fps_elapsed >= 0.5){
                fps = frame_count / fps_elapsed;
                frame_count=0; fps_timer=now;
            }

            // ---- events ----
            while (const auto event = window.pollEvent())
            {
                if (event->is<sf::Event::Closed>())
                    window.close();

                if (const auto* resized = event->getIf<sf::Event::Resized>())
                {
                    glViewport(
                        0,
                        0,
                        static_cast<GLsizei>(resized->size.x),
                        static_cast<GLsizei>(resized->size.y)
                    );
                }

                if (const auto* key = event->getIf<sf::Event::KeyPressed>())
                {
                    if (key->code == sf::Keyboard::Key::Escape)
                        window.close();

                    if (key->code == sf::Keyboard::Key::R)
                        state_.reset();
                }

                if (const auto* mousePressed =
                    event->getIf<sf::Event::MouseButtonPressed>())
                {
                    if (mousePressed->button == sf::Mouse::Button::Left)
                    {
                        mouse_held = true;
                        last_mouse = sf::Mouse::getPosition(window);
                    }
                }

                if (const auto* mouseReleased =
                    event->getIf<sf::Event::MouseButtonReleased>())
                {
                    if (mouseReleased->button == sf::Mouse::Button::Left)
                        mouse_held = false;
                }

                if (const auto* wheel =
                    event->getIf<sf::Event::MouseWheelScrolled>())
                {
                    cam_dist = std::clamp(
                        cam_dist - static_cast<float>(wheel->delta),
                        3.f,
                        40.f
                    );
                }
            }

            if(mouse_held){
                auto cur=sf::Mouse::getPosition(window);
                int dx=cur.x-last_mouse.x, dy=cur.y-last_mouse.y;
                cam_yaw  -= dx*0.5f;
                cam_pitch += dy*0.5f;
                cam_pitch  = std::clamp(cam_pitch,-80.f,80.f);
                last_mouse=cur;
            }

            // ---- 3-D render ----
            auto sz = window.getSize();
            glViewport(0,0,sz.x,sz.y);
            glClearColor(0.75f,0.85f,1.0f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

            // Projection
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            double aspect = (double)sz.x/sz.y;
            perspective_gl(60.0, aspect, 0.1, 500.0);

            // Camera
            double yr = cam_yaw  *PI/180.0;
            double pr = cam_pitch*PI/180.0;
            // Camera looks at drone; EUS→GL: negate Z
            double cx = vis_pos.x + cam_dist*std::cos(pr)*std::sin(yr);
            double cy = vis_pos.y + cam_dist*std::sin(pr) + 3.0;
            double cz =-(vis_pos.z- cam_dist*std::cos(pr)*std::cos(yr));

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            gluLookAt(cx,cy,cz,
                      vis_pos.x, vis_pos.y, -vis_pos.z,
                      0,1,0);

            setup_lighting();

            draw_ground();
            draw_target(target, pos);
            draw_drone(vis_pos, vis_rot, motor_angle, pwm_local, joint1_local, joint2_local);

            // ---- HUD ----
            window.pushGLStates();
            draw_hud(window,
                font,
                has_font,
                pwm_local,
                joint1_local,
                joint2_local,
                sim_t,
                fps,
                pos,
                target);
            window.popGLStates();

            window.display();
        }
    }

private:

    SimState& state_;

    STLMesh body_mesh_;
    STLMesh propeller_mesh_;
    STLMesh arm1_mesh_;
    STLMesh arm2_mesh_;

    void init_gl(sf::RenderWindow& w)
    {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHT1);
        glEnable(GL_COLOR_MATERIAL);
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glShadeModel(GL_SMOOTH);
        glEnable(GL_NORMALIZE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // Safety net: if any STL triangle has inverted winding order
        // (a common export mistake), two-sided lighting still lights it
        // correctly from whichever side the camera sees, instead of
        // that face rendering dark/black.
        glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    }

    void setup_lighting()
    {
        // Ambient
        float amb[4]={0.5f,0.5f,0.5f,1};
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);

        // Sun (directional)
        float sun_dir[4]={0.6f,1.0f,-0.3f,0};
        float sun_col[4]={1,1,1,1};
        glLightfv(GL_LIGHT0, GL_POSITION, sun_dir);
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  sun_col);
        glLightfv(GL_LIGHT0, GL_SPECULAR, sun_col);

        float off[4]={0,0,0,1};
        glLightfv(GL_LIGHT1, GL_DIFFUSE,  off);
    }

    // glFrustum-based perspective (no GLU available sometimes)
    void perspective_gl(double fovy_deg, double aspect, double near_, double far_)
    {
        double f = 1.0/std::tan(fovy_deg*PI/360.0);
        double M[16]={};
        M[0] =(float)(f/aspect);
        M[5] =(float)f;
        M[10]=(float)((far_+near_)/(near_-far_));
        M[11]=-1;
        M[14]=(float)(2*far_*near_/(near_-far_));
        glLoadMatrixd(M);
    }

    void draw_ground()
    {
        int GRID = 40;
        float TILE = 4.0f;

        glDisable(GL_LIGHTING);
        glBegin(GL_QUADS);
        for(int ix=-GRID;ix<GRID;ix++){
            for(int iz=-GRID;iz<GRID;iz++){
                if((ix+iz)%2==0)
                    glColor3f(0.88f,0.88f,0.88f);
                else
                    glColor3f(0.55f,0.55f,0.55f);
                float x0=ix*TILE, x1=x0+TILE;
                float z0=iz*TILE, z1=z0+TILE;
                glVertex3f(x0,0,z0); glVertex3f(x1,0,z0);
                glVertex3f(x1,0,z1); glVertex3f(x0,0,z1);
            }
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }

    void draw_target(const Vec3& target,
                    const Vec3& drone_pos)
    {
        const float size = 0.8f;
        const float half = size * 0.5f;

        bool inside =
            std::abs(drone_pos.x - target.x) <= half &&
            std::abs(drone_pos.y - target.y) <= half &&
            std::abs(drone_pos.z - target.z) <= half;

        glPushMatrix();

        glTranslated(
            target.x,
            target.y,
            -target.z);

        // Enable transparency
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        if (inside)
            glColor4f(0.0f, 1.0f, 0.0f, 0.25f);   // green transparent
        else
            glColor4f(1.0f, 0.0f, 0.0f, 0.25f);   // red transparent

        draw_box(
            -half,
            -half,
            -half,
            size,
            size,
            size);

        // Draw border for visibility
        if (inside)
            glColor3f(0.0f, 1.0f, 0.0f);
        else
            glColor3f(1.0f, 0.0f, 0.0f);

        glLineWidth(2.0f);

        glBegin(GL_LINES);

        // bottom square
        glVertex3f(-half,-half,-half); glVertex3f( half,-half,-half);
        glVertex3f( half,-half,-half); glVertex3f( half,-half, half);
        glVertex3f( half,-half, half); glVertex3f(-half,-half, half);
        glVertex3f(-half,-half, half); glVertex3f(-half,-half,-half);

        // top square
        glVertex3f(-half, half,-half); glVertex3f( half, half,-half);
        glVertex3f( half, half,-half); glVertex3f( half, half, half);
        glVertex3f( half, half, half); glVertex3f(-half, half, half);
        glVertex3f(-half, half, half); glVertex3f(-half, half,-half);

        // vertical edges
        glVertex3f(-half,-half,-half); glVertex3f(-half, half,-half);
        glVertex3f( half,-half,-half); glVertex3f( half, half,-half);
        glVertex3f( half,-half, half); glVertex3f( half, half, half);
        glVertex3f(-half,-half, half); glVertex3f(-half, half, half);

        glEnd();

        glDisable(GL_BLEND);

        glPopMatrix();
    }

    void draw_drone(const Vec3& pos, const Vec3& rot,
                    const float motor_ang[4],
                    const std::array<double,4>& pwm_vis,
                    const std::array<double,4>& joint1_vis,
                    const std::array<double,4>& joint2_vis)
    {
        glPushMatrix();

        glTranslated(pos.x, pos.y + 0.05, -pos.z);

        apply_drone_rotation(rot.x, rot.y, rot.z);

        //---------------------------------------------------
        // Scale
        //---------------------------------------------------

        // Overall drone scale, applied AFTER each individual mesh has
        // been normalized to a consistent internal size (see
        // scale_to_fit() calls below) — so this one number controls
        // the whole assembled drone's final world-space size,
        // independent of what units any individual STL was exported in.
        // Start here; go smaller (e.g. 0.2-0.3) if still too big,
        // larger (e.g. 0.8-1.2) if too small.
        const double SCALE = 0.5;
        glScaled(SCALE, SCALE, SCALE);

        //---------------------------------------------------
        // Colors — each part gets its own explicit color so nothing
        // silently inherits a leftover color from a previous draw call
        // (this was the cause of the whole drone appearing black:
        // the body's near-black color was never reset before drawing
        // the motors/propellers).
        //---------------------------------------------------

        const float BODY_COLOR[3]  = {1.00f, 0.45f, 0.00f};  // light gray
        const float ARM1_COLOR[3]   = {0.30f, 0.55f, 0.18f};  // mid gray
        const float ARM2_COLOR[3]   = {0.25f, 0.25f, 0.28f};  // mid gray
        const float PROP_COLOR[3]  = {0.10f, 0.10f, 0.10f};  // near-black (props are usually dark)

        glColor3f(BODY_COLOR[0], BODY_COLOR[1], BODY_COLOR[2]);

        // Fallback: if meshes didn't load, draw an unmistakable bright
        // magenta box at the drone's position/orientation instead of
        // nothing. This confirms position/rotation/camera math is fine
        // and isolates the problem to mesh loading specifically.
        bool any_missing =
            body_mesh_.empty() ||
            propeller_mesh_.empty() || arm1_mesh_.empty() ||
            arm2_mesh_.empty();

        if(any_missing)
        {
            glDisable(GL_LIGHTING);
            glColor3f(1.0f, 0.0f, 1.0f);   // magenta = unmissable
            draw_box(-1.0, -0.2, -1.0, 2.0, 0.4, 2.0);
            glEnable(GL_LIGHTING);
        }

        //---------------------------------------------------
        // Per-part manual adjustments
        //---------------------------------------------------
        // Each part is auto-normalized to size 1.0 by scale_to_fit(),
        // then these knobs let you fine-tune each part independently:
        //   extra_scale : multiplier on top of the auto-normalized size
        //                 (1.0 = no change, 0.5 = half size, 2.0 = double)
        //   rot_deg     : {x,y,z} rotation in degrees, applied about the
        //                 part's own origin, BEFORE it's positioned.
        //                 Use this to fix a part that loaded sideways/
        //                 upside-down/backwards.
        //   offset      : {x,y,z} position offset in the same local
        //                 space the part is drawn in (i.e. before the
        //                 outer SCALE and drone position/rotation are
        //                 applied) — use this to slide a part into place.
        struct PartAdjust {
            double extra_scale;
            Vec3   rot_deg;
            Vec3   offset;
        };

        PartAdjust BODY_ADJUST { 3.0, {90, 0, 0}, {0, 0.12, 0} };
        // Both arm_1 and arm_2 are drawn together at EVERY leg (they're
        // two parts of one complete leg assembly, not two alternate
        // leg styles) — each gets its own small local adjust since
        // they're different physical shapes and likely need different
        // fine-tuning even though they share the same leg position.
        PartAdjust ARM1_ADJUST { 0.5, {90, 0, 0}, {0, 0, 0} };  // arm_1: rotates on X
        PartAdjust ARM2_ADJUST { 0.5, {90, 0, 0}, {0, 0, 0} };  // arm_2: rotates on Z
        PartAdjust PROP_ADJUST { 0.35, {0, 0, 0}, {0, -0.035, 0} };

        // Applies offset -> rotation -> auto-scale -> extra_scale, in that
        // order, for one mesh. Call this right after glPushMatrix().
        auto apply_part_transform = [](const STLMesh& mesh, const PartAdjust& adj)
        {
            glTranslated(adj.offset.x, adj.offset.y, adj.offset.z);
            glRotated(adj.rot_deg.x, 1, 0, 0);
            glRotated(adj.rot_deg.y, 0, 1, 0);
            glRotated(adj.rot_deg.z, 0, 0, 1);
            double s = mesh.scale_to_fit(1.0) * adj.extra_scale;
            glScaled(s, s, s);
        };

        //---------------------------------------------------
        // Motor positions (X configuration)
        //---------------------------------------------------

        struct Motor
        {
            double x;
            double z;
            bool front;
        };

        Motor motors[4] =
        {
            { 1.45,-1.1,true  },   // FR
            {-1.45,-1.1,true  },   // FL
            { 1.45, 1.015,false },   // BR
            {-1.45, 1.015,false }    // BL
        };

        //---------------------------------------------------
        // Arms — BOTH arm_1 and arm_2 drawn together at every one of
        // the 4 leg positions. Left-side legs (motors[i].x < 0) get an
        // extra 180 degree flip around the Z axis, mirroring the
        // orientation that already works correctly on the right side.
        //---------------------------------------------------
        glColor3f(ARM1_COLOR[0], ARM1_COLOR[1], ARM1_COLOR[2]);

        double arm_yaw_deg[4] = { 0, 0, 0, 0 };   // FR, FL, BR, BL

        // Height applied at the MOUNT POINT, before the left-side flip
        // — this is what used to be ARM1_ADJUST/ARM2_ADJUST's offset.y.
        // Doing it here (outside the flip) means "up" stays up in world
        // space on both sides; doing it inside the flip (as before)
        // made it mean "down" on the flipped left-side legs.
        const double ARM_MOUNT_HEIGHT = 0.12;

        for(int i=0;i<4;i++)
        {
            glPushMatrix();

            glTranslated(
                motors[i].x,
                ARM_MOUNT_HEIGHT,
                -motors[i].z
            );

            glRotated(arm_yaw_deg[i], 0,1,0);

            // Flip left-side legs 180 degrees around Z to mirror the
            // right side's orientation.
            bool is_left_side = motors[i].x < 0;
            if(is_left_side)
                glRotated(180, 0,0,1);
                

            glPushMatrix();

            double joint1 = -joint2_vis[i];

            if (i == 1 || i == 3)   // Left side
                joint1 = -joint1;
                glTranslated(0, 0.013, 0);

            glRotated(joint1, 1, 0, 0);
            glColor3f(ARM1_COLOR[0], ARM1_COLOR[1], ARM1_COLOR[2]);
            
            apply_part_transform(arm1_mesh_, ARM1_ADJUST);
            arm1_mesh_.draw();
            glPopMatrix();

            glPushMatrix();
            // joint1
            if (i == 1 || i == 3)   // Left side
                glTranslated(0, 0.013, 0);

            glRotated(joint1, 1,0,0);
            
            // joint2
            glRotated(joint1_vis[i], 0,0,1);
            
            glColor3f(ARM2_COLOR[0], ARM2_COLOR[1], ARM2_COLOR[2]);
            apply_part_transform(arm2_mesh_, ARM2_ADJUST);
            arm2_mesh_.draw();
            glPopMatrix();

            glPopMatrix();
        }

        glLineWidth(5.0f);

        glPushMatrix();
        glColor3f(BODY_COLOR[0], BODY_COLOR[1], BODY_COLOR[2]);
        apply_part_transform(body_mesh_, BODY_ADJUST);
        body_mesh_.draw();
        glPopMatrix();

        glLineWidth(1.0f);

        //---------------------------------------------------
        // Motors + props
        //---------------------------------------------------

        for(int i=0;i<4;i++)
        {
            glPushMatrix();

            glTranslated(
                motors[i].x,
                0.05,
                -motors[i].z
            );

            // Motor mesh intentionally excluded (motor.stl not loaded).

            // Propeller — mount point sits 0.10 above the motor origin.
            glTranslated(0,0.10,0);

            // Tilt the propeller's local Y-axis (its own spin axis) to
            // match the ACTUAL gimbal direction from joint1/joint2 —
            // same motor_direction() + Z-negation used for the force
            // arrows, so the visible propeller tilt matches both the
            // physics and the arrows. Without this, the mesh always
            // spun around a fixed vertical axis regardless of joint
            // angles — the gimbal existed in physics/arrows but was
            // never actually visible on the propeller itself.
            {
                Vec3 j_dir = motor_direction(joint1_vis[i], joint2_vis[i]);
                Vec3 j_dir_gl { j_dir.x, j_dir.y, -j_dir.z };

                Vec3 up{0,1,0};
                Vec3 axis = up.cross(j_dir_gl);
                double axis_len = axis.length();
                double dp = std::clamp(up.dot(j_dir_gl), -1.0, 1.0);
                double angle_deg = std::acos(dp) * 180.0 / PI;

                if(axis_len > 1e-6)
                    glRotated(angle_deg, axis.x, axis.y, axis.z);
                else if(dp < 0)
                    glRotated(180.0, 1,0,0);   // pointing straight down
            }

            // Spin animation — applied around the NOW-TILTED local Y
            // axis (the propeller's actual shaft, wherever the gimbal
            // is pointing it), not the original vertical.
            glRotated(
                motor_ang[i],
                0,1,0
            );

            glPushMatrix();

            glColor3f(PROP_COLOR[0], PROP_COLOR[1], PROP_COLOR[2]);
            apply_part_transform(propeller_mesh_, PROP_ADJUST);
            propeller_mesh_.draw();

            glPopMatrix();

            glPopMatrix();   // <-- ADD THIS
        }

        //---------------------------------------------------
        // Thrust vector visualization — one arrow per motor, drawn from
        // the SAME local frame draw_drone is already in (translated +
        // rotated to the drone's actual pose above), using the exact
        // same motor_direction() function physics.h uses for the real
        // force calculation. This guarantees what you see matches what
        // the physics engine is actually computing — not a separate
        // guess at orientation.
        //---------------------------------------------------
        {
            // Visual length scale: MAX_THRUST (20 N by default) maps to
            // this many world units of arrow length. Tune if arrows
            // look too long/short relative to the drone's size.
            const double THRUST_ARROW_SCALE = 0.06;

            for(int i=0;i<4;i++)
            {
                Vec3 dir = motor_direction(joint1_vis[i], joint2_vis[i]);
                // Same EUS -> OpenGL conversion applied everywhere else
                // in this renderer (negate Z) — the origin position
                // below already gets this via -motors[i].z, but the
                // DIRECTION vector needs it too, or its Z-component
                // ends up mismatched with the rest of the scene
                // (this was the forward/back flip).
                Vec3 dir_gl { dir.x, dir.y, -dir.z };
                double thrust_n = pwm_vis[i] * MAX_THRUST;
                double arrow_len = thrust_n * THRUST_ARROW_SCALE;

                Vec3 origin { motors[i].x, ARM_MOUNT_HEIGHT, -motors[i].z };
                Vec3 tip = origin + dir_gl * arrow_len;

                // Color: blue (low thrust) -> red (high thrust), so
                // magnitude is visible at a glance alongside direction.
                float t = (float)std::clamp(pwm_vis[i], 0.0, 1.0);
                glDisable(GL_LIGHTING);
                glLineWidth(3.0f);
                glColor3f(t, 0.2f, 1.0f - t);

                glBegin(GL_LINES);
                glVertex3d(origin.x, origin.y, origin.z);
                glVertex3d(tip.x, tip.y, tip.z);
                glEnd();

                // Simple arrowhead: two short lines splayed back from
                // the tip, roughly perpendicular-ish to the shaft —
                // good enough to read direction without needing a full
                // cone mesh.
                Vec3 back = dir_gl * (-arrow_len * 0.15 - 0.02);
                Vec3 side1 { back.x + 0.02, back.y, back.z };
                Vec3 side2 { back.x - 0.02, back.y, back.z + 0.02 };
                glBegin(GL_LINES);
                glVertex3d(tip.x, tip.y, tip.z);
                glVertex3d(tip.x+side1.x, tip.y+side1.y, tip.z+side1.z);
                glVertex3d(tip.x, tip.y, tip.z);
                glVertex3d(tip.x+side2.x, tip.y+side2.y, tip.z+side2.z);
                glEnd();

                glLineWidth(1.0f);
                glEnable(GL_LIGHTING);
            }
        }

        glPopMatrix();
    }

void draw_hud(sf::RenderWindow& w,
              const std::optional<sf::Font>& font,
              bool has_font,
              const std::array<double,4>& pwm_local,
              const std::array<double,4>& joint1_local,
              const std::array<double,4>& joint2_local,
              double sim_t,
              double fps,
              const Vec3& pos,
              const Vec3& target)
    {
        if(!has_font) return;

        auto sz = w.getSize();
        float FS = 18.f;

        // Left panel background
        sf::RectangleShape bg({350,240});
        bg.setFillColor(sf::Color(0,0,0,170));
        bg.setPosition({5.f, 5.f});
        w.draw(bg);

        // Right panel background
        sf::RectangleShape bg2({350,140});
        bg2.setFillColor(sf::Color(0,0,0,170));
        bg2.setPosition(
        {
            static_cast<float>(sz.x) - 225.f,
            5.f
        });
        w.draw(bg2);

        char buf[128];

        auto make_text = [&](const std::string& str, float x, float y,
                              sf::Color col=sf::Color::White) {
            sf::Text t(*font, str);
            t.setCharacterSize((unsigned)FS);
            t.setPosition({x,y});
            t.setFillColor(col);
            w.draw(t);
        };

        // Left: FPS + PWM
        std::snprintf(buf,sizeof(buf),"UI: %.1f fps", fps);
        make_text(buf, 10, 10, sf::Color::Cyan);
        std::snprintf(buf,sizeof(buf),"Physics: 120.0 fps");
        make_text(buf, 10, 28, sf::Color::Cyan);
        make_text("PWM Signals:", 10, 46, sf::Color::White);

        const char* lbl[4]={"0 (FR)","1 (FL)","2 (BR)","3 (BL)"};
        for(int i=0;i<4;i++){
            std::snprintf(buf,sizeof(buf),"%s: %.3f",
              lbl[i], pwm_local[i]);
              
            make_text(buf, 10, 64+i*18, sf::Color::White);

            // Green bar
            sf::RectangleShape bar({(float)(pwm_local[i]*60),8});
            bar.setFillColor(sf::Color::Green);
            bar.setPosition(
            {
                115.f,
                66.f + i * 18.f
            });
            w.draw(bar);
        }

        make_text("Joint Angles:", 10, 145, sf::Color::Yellow);

        const char* motor_name[4] = {"FR: ","FL: ","BR: ","BL: "};

        for(int i=0;i<4;i++)
        {
            std::snprintf(buf, sizeof(buf),
                        "%s  pitch_servo:%6.1f  roll_servo:%6.1f",
                        motor_name[i],
                        joint1_local[i],
                        joint2_local[i]);

            make_text(buf, 10, 165 + i*18, sf::Color::White);
        }

        // Right: distance to target
        Vec3 diff = pos - target;
        double dist = diff.length();
        const char* mission_names[] =
        {
            "Static Target",
            "Static Target",
            "Static Target",
            "Static Target",
            "Linear Motion",
            "Reciprocating Motion Target",
            "Circular Motion Target"
        };

        int idx = std::clamp(
            state_.mission_index,
            0,
            (int)(std::size(mission_names)-1)
        );

        std::snprintf(
            buf,
            sizeof(buf),
            "Level %d: %s",
            idx + 1,
            mission_names[idx]
        );
        make_text(buf, (float)sz.x-220, 10, sf::Color::White);
        std::snprintf(buf,sizeof(buf),"Socket: 50 Hz");
        make_text(buf, (float)sz.x-220, 28, sf::Color::White);
        std::snprintf(buf,sizeof(buf),"Time: %.1f sec", sim_t);
        make_text(buf, (float)sz.x-220, 46, sf::Color::White);
        std::snprintf(buf,sizeof(buf),"Distance: %.2f", dist);
        make_text(buf, (float)sz.x-220, 64, sf::Color::Yellow);
        if(dist < TARGET_RADIUS){
            make_text("TARGET REACHED!", (float)sz.x-220, 82, sf::Color::Green);
        }
    }
};
