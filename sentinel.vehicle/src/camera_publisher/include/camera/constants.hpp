#pragma once

namespace Constants
{

    inline constexpr const char *vehicle_host = "0.0.0.0";
    inline constexpr int vehicle_port = 8000;
    inline constexpr int computer_port = 9000;
    inline constexpr int chunk_size = 60000;

    inline constexpr int camera_width = 640;
    inline constexpr int camera_heigh = 480;
    inline constexpr int camera_fps = 30;
    inline constexpr const char *camera_encoding = ".jpeg";
#ifdef LIBCAMERA
    inline constexpr libcamera::PixelFormat camera_pixel_format =
        libcamera::formats::RGB888;
#endif

} // namespace Constants
