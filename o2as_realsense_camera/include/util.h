#pragma once

#include <string>
#include <sstream>
#include <iostream>
#include <librealsense2/rs.hpp>

struct float3 { float x, y, z; };
struct float2 { float x, y; };

struct rect
{
    float x, y;
    float w, h;

    // Create new rect within original boundaries with give aspect ration
    rect adjust_ratio(float2 size) const
    {
        auto H = static_cast<float>(h), W = static_cast<float>(h) * size.x / size.y;
        if (W > w)
        {
            auto scale = w / W;
            W *= scale;
            H *= scale;
        }

        return{ x + (w - W) / 2, y + (h - H) / 2, W, H };
    }
};

class texture
{
public:
    void render(const rs2::video_frame& frame, const rect& r)
    {
    }
    void upload(const rs2::video_frame& frame)
    {
    }
    void show(const rect& r) const
    {
    }
};
