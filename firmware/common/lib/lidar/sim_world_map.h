// Copyright (c) 2026 Linorobot contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef SIM_WORLD_MAP_H
#define SIM_WORLD_MAP_H
// A saved occupancy map as the simulated world, for the Sim MCU only (LINO_HOST).
//
// No board can hold a map, but the Sim MCU is this firmware running on the robot
// computer, where the map file is. So on world "map" its LD19 emulator raycasts the
// map itself and the scan still goes through the LD driver, like any board's (user,
// 2026-10-06: "sim mcu should do the raycasts from imported map and let stl driver
// publish /scan"). The sonar cone is raycast from the same map.
//
// The same reading of the map as scripts/depth_camera.py GridWorld, which the host's
// simulated camera (and the sim_base path's laser) use: a map_server .yaml beside a
// binary PGM (P5), `negate`, `occupied_thresh`, `origin`; cells past the edge are
// empty; the robot's odometry origin is placed in the map at `start` (x, y, yaw).
// Env keys sim_world_map (the .yaml path) and sim_world_start ("x,y,yaw").
#ifdef LINO_HOST
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

class SimWorldMap
{
public:
    bool loaded() const { return loaded_; }
    int width() const { return w_; }
    int height() const { return h_; }
    float resolution() const { return res_; }
    const char *error() const { return why_; }

    // Load `yaml_path` and place the odometry origin at `start` ("x,y,yaw", may be
    // empty). False, with error() saying why, on anything it cannot read.
    bool load(const char *yaml_path, const char *start)
    {
        loaded_ = false;
        why_[0] = 0;
        FILE *fh = fopen(yaml_path, "r");
        if (!fh) return fail("cannot open %s", yaml_path);
        std::string image;
        float origin[3] = {0, 0, 0};
        bool have_res = false, have_origin = false;
        int negate = 0;
        float occ_thresh = 0.65f;
        char line[512];
        int origin_item = -1;          // origin written as a block list ("origin:" then "- x" lines)
        while (fgets(line, sizeof(line), fh))
        {
            if (origin_item >= 0)
            {
                const std::string item = trim(line);
                if (item.size() > 1 && item[0] == '-' && origin_item < 3)
                {
                    origin[origin_item++] = strtof(item.c_str() + 1, NULL);
                    have_origin = (origin_item >= 2);
                    continue;
                }
                origin_item = -1;
            }
            char *colon = strchr(line, ':');
            if (!colon) continue;
            *colon = 0;
            const std::string key = trim(line);
            std::string val = trim(colon + 1);
            if (key == "image")
            {
                if (val.size() >= 2 && (val[0] == '"' || val[0] == '\'') && val.back() == val[0])
                    val = val.substr(1, val.size() - 2);
                image = val;
            }
            else if (key == "resolution") { res_ = strtof(val.c_str(), NULL); have_res = true; }
            else if (key == "negate") negate = atoi(val.c_str());
            else if (key == "occupied_thresh") occ_thresh = strtof(val.c_str(), NULL);
            else if (key == "origin" && val.empty())
                origin_item = 0;
            else if (key == "origin")
            {
                const char *p = val.c_str();
                while (*p == '[' || *p == ' ') p++;
                int k = 0;
                for (; k < 3; k++)
                {
                    char *end = NULL;
                    origin[k] = strtof(p, &end);
                    if (end == p) break;
                    p = end;
                    while (*p == ',' || *p == ' ') p++;
                }
                have_origin = (k >= 2);
            }
        }
        fclose(fh);
        if (image.empty() || !have_res || !have_origin || res_ <= 0.0f)
            return fail("%s: needs image, resolution and origin", yaml_path);
        if (image[0] != '/')
        {
            const std::string dir(yaml_path);
            const size_t slash = dir.rfind('/');
            image = (slash == std::string::npos ? std::string(".") : dir.substr(0, slash)) + "/" + image;
        }
        if (!readPgm(image.c_str(), negate != 0, occ_thresh)) return false;
        ox_ = origin[0];
        oy_ = origin[1];
        sx_ = sy_ = syaw_ = 0.0f;
        if (start && *start)
        {
            float v[3] = {0, 0, 0};
            const char *p = start;
            for (int k = 0; k < 3; k++)
            {
                char *end = NULL;
                v[k] = strtof(p, &end);
                if (end == p) break;
                p = end;
                while (*p == ',' || *p == ' ') p++;
            }
            sx_ = v[0]; sy_ = v[1]; syaw_ = v[2];
        }
        loaded_ = true;
        return true;
    }

    // Range along a ray from (x, y) at `angle`, all in the ODOMETRY frame, to the first
    // occupied cell; max_range when nothing is hit. Marched at half a cell from 3 cm
    // out, as GridWorld.ranges() is.
    float range(float x, float y, float angle, float max_range) const
    {
        const float c = cosf(syaw_), s = sinf(syaw_);
        const float wx = sx_ + c * x - s * y;
        const float wy = sy_ + s * x + c * y;
        const float a = syaw_ + angle;
        const float dx = cosf(a), dy = sinf(a);
        const float step = res_ * 0.5f;
        for (float t = 0.03f; t < max_range; t += step)
        {
            if (occupiedAt(wx + dx * t, wy + dy * t)) return t;
        }
        return max_range;
    }

private:
    bool loaded_ = false;
    int w_ = 0, h_ = 0;
    float res_ = 0.05f, ox_ = 0.0f, oy_ = 0.0f;
    float sx_ = 0.0f, sy_ = 0.0f, syaw_ = 0.0f;
    std::vector<uint8_t> occ_;     // row 0 is the BOTTOM of the map (the image's last row)
    char why_[200] = {0};

    bool occupiedAt(float x, float y) const
    {
        const int cx = (int)floorf((x - ox_) / res_);
        const int cy = (int)floorf((y - oy_) / res_);
        if (cx < 0 || cy < 0 || cx >= w_ || cy >= h_) return false;   // past the edge: empty
        return occ_[(size_t)cy * w_ + cx] != 0;
    }

    bool fail(const char *fmt, const char *arg)
    {
        snprintf(why_, sizeof(why_), fmt, arg);
        return false;
    }

    static std::string trim(const char *s)
    {
        while (*s == ' ' || *s == '\t') s++;
        std::string out(s);
        const size_t hash = out.find(" #");
        if (hash != std::string::npos) out.erase(hash);
        while (!out.empty() && (out.back() == '\n' || out.back() == '\r' || out.back() == ' ' || out.back() == '\t'))
            out.pop_back();
        return out;
    }

    bool readPgm(const char *path, bool negate, float thresh)
    {
        FILE *fh = fopen(path, "rb");
        if (!fh) return fail("cannot open %s", path);
        std::vector<uint8_t> data;
        uint8_t buf[65536];
        size_t n;
        while ((n = fread(buf, 1, sizeof(buf), fh)) > 0) data.insert(data.end(), buf, buf + n);
        fclose(fh);
        // magic, then width, height, maxval -- with # comments allowed between them
        size_t pos = 0;
        std::string tok[4];
        for (int k = 0; k < 4; k++)
        {
            for (;;)
            {
                while (pos < data.size() && isspace(data[pos])) pos++;
                if (pos < data.size() && data[pos] == '#')
                {
                    while (pos < data.size() && data[pos] != '\n') pos++;
                    continue;
                }
                break;
            }
            while (pos < data.size() && !isspace(data[pos])) tok[k] += (char)data[pos++];
        }
        if (tok[0] != "P5") return fail("%s: only binary PGM (P5) maps are read", path);
        w_ = atoi(tok[1].c_str());
        h_ = atoi(tok[2].c_str());
        const int maxval = atoi(tok[3].c_str());
        pos++;                                   // the single whitespace before the pixels
        if (w_ <= 0 || h_ <= 0 || maxval <= 0 || maxval > 255 || data.size() < pos + (size_t)w_ * h_)
            return fail("%s: not a complete 8-bit PGM", path);
        occ_.assign((size_t)w_ * h_, 0);
        for (int r = 0; r < h_; r++)
        {
            for (int x = 0; x < w_; x++)
            {
                const float px = (float)data[pos + (size_t)r * w_ + x];
                const float occ = negate ? px / maxval : (maxval - px) / maxval;
                occ_[(size_t)(h_ - 1 - r) * w_ + x] = (occ > thresh) ? 1 : 0;
            }
        }
        return true;
    }
};
#endif  // LINO_HOST
#endif  // SIM_WORLD_MAP_H
