// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include "example.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <thread>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "third_party/stb_image_write.h"

texture_buffer buffers[3];

#pragma pack(push, 1)
struct rgb_pixel
{
    uint8_t r,g,b; 
};
#pragma pack(pop)

int capture_frame, frame_number;

static void on_mouse_button(GLFWwindow * win, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT) capture_frame = action == GLFW_PRESS;
}

static void write_depth_data(std::ofstream * file, rs::intrinsics * intrin, float scale, const uint16_t * depth_image)
{
	// write one line for each point. bad points get 0 0 0
	*file << intrin->height << " " << intrin->width << std::endl;
	for (int dy = 0; dy < intrin->height; ++dy)
	{
		for (int dx = 0; dx < intrin->width; ++dx)
		{
			// Retrieve the 16-bit depth value and map it into a depth in meters
			uint16_t depth_value = depth_image[dy * intrin->width + dx];
			float depth_in_meters = depth_value * scale;

			// Skip over pixels with a depth value of zero, which is used to indicate no data
			// can't do this for our purposes
			if (depth_value == 0)
			{
				*file << "0 0 0" << std::endl;
				continue;
			}

			// Map from pixel coordinates in the depth image to pixel coordinates in the color image
			rs::float2 depth_pixel = { (float)dx, (float)dy };
			rs::float3 depth_point = intrin->deproject(depth_pixel, depth_in_meters);
			*file << depth_point.x << " " << depth_point.y << " " << depth_point.z << std::endl;
		}
	}
}

int main(int argc, char * argv[]) try
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    dev.start();

    // Open a GLFW window
    glfwInit();
    std::ostringstream ss; ss << "Cheap capture tool (" << dev.get_name() << ")";
    GLFWwindow * win = glfwCreateWindow(1280, 960, ss.str().c_str(), 0, 0);
	glfwSetMouseButtonCallback(win, on_mouse_button);
    glfwMakeContextCurrent(win);

    while (!glfwWindowShouldClose(win))
    {
        // Wait for new images
        glfwPollEvents();
        dev.wait_for_frames();

		// Retrieve our images
		const uint16_t * depth_image = (const uint16_t *)dev.get_frame_data(rs::stream::depth);
		const uint8_t * color_image = (const uint8_t *)dev.get_frame_data(rs::stream::color);
		const uint8_t * color_aligned_to_depth_image = (const uint8_t *)dev.get_frame_data(rs::stream::color_aligned_to_depth);

		// Retrieve camera parameters for mapping between depth and color
		rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
		rs::extrinsics depth_to_color = dev.get_extrinsics(rs::stream::depth, rs::stream::color);
		rs::intrinsics color_intrin = dev.get_stream_intrinsics(rs::stream::color);
		rs::intrinsics color_aligned_intrin = dev.get_stream_intrinsics(rs::stream::color_aligned_to_depth);
		float scale = dev.get_depth_scale();


        // Clear the framebuffer
        int w,h;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw the images        
        glPushMatrix();
        glfwGetWindowSize(win, &w, &h);
        glOrtho(0, w, h, 0, -1, +1);
        int s = w / 2;
        buffers[0].show(dev, rs::stream::color, 0, 0, s, h-h/2);
        buffers[1].show(dev, rs::stream::color_aligned_to_depth, s, 0, s, h-h/2);
        buffers[2].show(dev, rs::stream::depth, s, h/2, s, h-h/2);
        glPopMatrix();
        glfwSwapBuffers(win);

		if (capture_frame) 
		{
			std::stringstream ds;
			ds << "depth_data_" << frame_number << ".dat";
			std::cout << "Writing " << ds.str().data() << ", " << depth_intrin.width * depth_intrin.height << " points" << std::endl;
			std::ofstream depth_file(ds.str().data());
			if (depth_file.is_open())
			{
				write_depth_data(&depth_file, &depth_intrin, scale, depth_image);
			}
			depth_file.close();

			std::stringstream cs;
			cs << "color_image_" << frame_number << ".png";
			std::cout << "Writing " << cs.str().data() << ", " << color_intrin.width << " x " << color_intrin.height << " pixels" << std::endl;
			stbi_write_png(cs.str().data(),
				color_intrin.width, color_intrin.height,
				3,
				color_image,
				color_intrin.width * 3);

			std::stringstream cas;
			cas << "color_aligned_to_depth_image_" << frame_number << ".png";
			std::cout << "Writing " << cas.str().data() << ", " << color_intrin.width << " x " << color_intrin.height << " pixels" << std::endl;
			stbi_write_png(cas.str().data(),
				color_aligned_intrin.width, color_aligned_intrin.height,
				3,
				color_aligned_to_depth_image,
				color_aligned_intrin.width * 3);
			
			capture_frame = 0;
			frame_number++;
		}
    }

    glfwDestroyWindow(win);
    glfwTerminate();
	system("pause");
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}