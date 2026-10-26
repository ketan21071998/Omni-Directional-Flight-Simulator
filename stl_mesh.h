#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <cstdint>
#include <iostream>
#include <algorithm>
#include <cmath>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <GL/gl.h>

class STLMesh
{
public:

    STLMesh() = default;
    STLMesh(const STLMesh&) = delete;
    STLMesh& operator=(const STLMesh&) = delete;

    struct Triangle
    {
        float nx, ny, nz;

        float v1x, v1y, v1z;
        float v2x, v2y, v2z;
        float v3x, v3y, v3z;
    };

    bool load(const std::string& filename)
    {
        triangles_.clear();

        std::ifstream file(filename, std::ios::binary);

        if (!file)
        {
            std::cerr << "[STL] Failed to open: "
                      << filename << std::endl;
            return false;
        }

        // Get actual file size so we can sanity-check the triangle count
        // we're about to read. This is the key defense against ASCII STL
        // files: if this .stl was exported as ASCII text (starts with
        // "solid ...", human-readable), the binary parser below would
        // otherwise misread text bytes as a triangle count -- often a
        // huge garbage number -- and either crash or silently produce
        // nonsense geometry that renders somewhere off in space (which
        // looks exactly like "the model is invisible").
        file.seekg(0, std::ios::end);
        std::streamoff file_size = file.tellg();
        file.seekg(0, std::ios::beg);

        char header[80];
        file.read(header, 80);

        uint32_t triangle_count = 0;
        file.read(
            reinterpret_cast<char*>(&triangle_count),
            sizeof(uint32_t));

        std::streamoff expected_size =
            84 + static_cast<std::streamoff>(triangle_count) * 50;

        if (file_size < 84 || expected_size != file_size)
        {
            std::cerr << "[STL] " << filename
                      << " does NOT look like a valid binary STL "
                         "(file_size=" << file_size
                      << ", expected=" << expected_size
                      << " for triangle_count=" << triangle_count << "). "
                         "This usually means the file is an ASCII STL "
                         "(re-export as BINARY STL) or is corrupted. "
                         "Refusing to load garbage.\n";
            return false;
        }

        triangles_.reserve(triangle_count);

        for (uint32_t i = 0; i < triangle_count; i++)
        {
            Triangle t;

            file.read(reinterpret_cast<char*>(&t.nx), sizeof(float));
            file.read(reinterpret_cast<char*>(&t.ny), sizeof(float));
            file.read(reinterpret_cast<char*>(&t.nz), sizeof(float));

            file.read(reinterpret_cast<char*>(&t.v1x), sizeof(float));
            file.read(reinterpret_cast<char*>(&t.v1y), sizeof(float));
            file.read(reinterpret_cast<char*>(&t.v1z), sizeof(float));

            file.read(reinterpret_cast<char*>(&t.v2x), sizeof(float));
            file.read(reinterpret_cast<char*>(&t.v2y), sizeof(float));
            file.read(reinterpret_cast<char*>(&t.v2z), sizeof(float));

            file.read(reinterpret_cast<char*>(&t.v3x), sizeof(float));
            file.read(reinterpret_cast<char*>(&t.v3y), sizeof(float));
            file.read(reinterpret_cast<char*>(&t.v3z), sizeof(float));

            uint16_t attribute_byte_count;
            file.read(
                reinterpret_cast<char*>(&attribute_byte_count),
                sizeof(uint16_t));

            triangles_.push_back(t);
        }

        // Recompute every normal from the triangle's own vertices
        // instead of trusting the value stored in the file. Many STL
        // exporters write degenerate normals (0,0,0) or garbage —
        // with a zero normal, OpenGL's lighting math produces zero
        // diffuse contribution, so the whole mesh renders as a flat
        // black silhouette regardless of what color you set. Computing
        // the normal ourselves (cross product of two edges) guarantees
        // correct shading no matter what the file actually contains.
        for(auto& t : triangles_)
        {
            float e1x = t.v2x - t.v1x, e1y = t.v2y - t.v1y, e1z = t.v2z - t.v1z;
            float e2x = t.v3x - t.v1x, e2y = t.v3y - t.v1y, e2z = t.v3z - t.v1z;

            float nx = e1y*e2z - e1z*e2y;
            float ny = e1z*e2x - e1x*e2z;
            float nz = e1x*e2y - e1y*e2x;

            float len = std::sqrt(nx*nx + ny*ny + nz*nz);
            if(len > 1e-9f){
                t.nx = nx/len; t.ny = ny/len; t.nz = nz/len;
            }
            // else: degenerate triangle (zero area) — leave as-is, harmless
        }

        // Bounding box diagnostics -- tells you the RAW model size in
        // whatever units the STL was exported in, before any glScaled()
        // is applied in renderer.h. If this looks like it's in mm
        // (hundreds of units) vs meters (single digits), that tells you
        // exactly what scale factor you actually need.
        float minx=1e9f,miny=1e9f,minz=1e9f;
        float maxx=-1e9f,maxy=-1e9f,maxz=-1e9f;
        for(const auto& t : triangles_)
        {
            for(float x : {t.v1x, t.v2x, t.v3x}){ minx=std::min(minx,x); maxx=std::max(maxx,x); }
            for(float y : {t.v1y, t.v2y, t.v3y}){ miny=std::min(miny,y); maxy=std::max(maxy,y); }
            for(float z : {t.v1z, t.v2z, t.v3z}){ minz=std::min(minz,z); maxz=std::max(maxz,z); }
        }

        std::cout
            << "[STL] Loaded: " << filename
            << "  triangles=" << triangles_.size()
            << "  raw_bbox_size=("
            << (maxx-minx) << ", " << (maxy-miny) << ", " << (maxz-minz)
            << ")" << std::endl;

        bbox_max_dim_ = std::max({maxx-minx, maxy-miny, maxz-minz});

        return true;
    }

    void draw() const
    {
        // Compile the geometry into a display list once, on first draw.
        // Immediate-mode (glBegin/glVertex per vertex, every frame) is
        // extremely slow for meshes with more than a few hundred
        // triangles — re-issuing potentially tens of thousands of GL
        // calls per frame, per mesh, every frame is almost certainly
        // why FPS was ~2. A display list uploads the geometry to the
        // driver once and replays it with a single glCallList().
        if(display_list_ == 0 && !triangles_.empty())
        {
            display_list_ = glGenLists(1);
            glNewList(display_list_, GL_COMPILE);
            draw_immediate();
            glEndList();
        }

        if(display_list_ != 0)
            glCallList(display_list_);
    }

    ~STLMesh()
    {
        if(display_list_ != 0)
            glDeleteLists(display_list_, 1);
    }

    bool empty() const
    {
        return triangles_.empty();
    }

    // Returns the scale factor to apply so this mesh's LARGEST bounding-box
    // dimension becomes exactly target_size world units. This makes mesh
    // sizing independent of whatever units the STL was authored in (mm,
    // cm, inches, arbitrary CAD units...) — just ask for the size you
    // actually want instead of guessing a raw multiplier.
    double scale_to_fit(double target_size) const
    {
        if(triangles_.empty() || bbox_max_dim_ <= 1e-9f)
            return 1.0;
        return target_size / bbox_max_dim_;
    }

    float bbox_max_dim() const { return bbox_max_dim_; }

private:

    void draw_immediate() const
    {
        glBegin(GL_TRIANGLES);

        for (const auto& t : triangles_)
        {
            glNormal3f(t.nx, t.ny, t.nz);
            glVertex3f(t.v1x, t.v1y, t.v1z);
            glVertex3f(t.v2x, t.v2y, t.v2z);
            glVertex3f(t.v3x, t.v3y, t.v3z);
        }

        glEnd();
    }

    std::vector<Triangle> triangles_;
    float bbox_max_dim_ = 0.0f;
    mutable GLuint display_list_ = 0;   // 0 = not yet compiled
};