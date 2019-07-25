// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>

static std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

static std::string to_gauge(float value) {
    constexpr float min_v = -12;  // range of values
    constexpr float max_v = +12;
    constexpr int L = 45;  // size of gauge
    constexpr int N = L * 2;
   
    float v = std::min(std::max(value, min_v), max_v);
    float p = (v - min_v) / (max_v - min_v);
    int n1 = N * p;
    int n2 = L;
    if (n1 > n2) std::swap(n1, n2);

    return std::to_string((int)min_v) + 
        std::string(" [") + std::string(n1,' ') + std::string(n2-n1+1,'X') + std::string(N-n2,' ') + std::string("] ") +
        std::to_string((int)max_v) +
        "  |  value: " + std::to_string(value);
}

int main(int argc, char * argv[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start pipeline with chosen configuration
    pipe.start();

    auto last_time = now();
    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        auto cur_time = now();
        float frequency = 1000.f / std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - last_time).count();
        last_time = cur_time;

        // Get a frame from the pose stream
        auto fa = frames.first_or_default(RS2_STREAM_ACCEL);
        auto fg = frames.first_or_default(RS2_STREAM_GYRO);

        rs2_vector accel_data = fa.as<rs2::motion_frame>().get_motion_data();
        rs2_vector gyro_data = fg.as<rs2::motion_frame>().get_motion_data();

        std::cout << "\033[2J\033[1;1H"  // ANSI code to clear screen
                  << "frequency: " << frequency << "Hz\n\n" 
                  << "accelerometer:\n"
                  << "x: \t" << to_gauge(accel_data.x) << "\n"
                  << "y: \t" << to_gauge(accel_data.y) << "\n"
                  << "z: \t" << to_gauge(accel_data.z) << "\n\n"
                  << "gyro:\n"
                  << "x: \t" << to_gauge(gyro_data.x) << "\n"
                  << "y: \t" << to_gauge(gyro_data.y) << "\n"
                  << "z: \t" << to_gauge(gyro_data.z) << std::endl;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
