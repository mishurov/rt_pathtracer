/* ************************************************************************
 * Copyright 2017 Alexander Mishurov
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ************************************************************************/

#ifndef RTPT_RENDERER_H_
#define RTPT_RENDERER_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cuda_gl_interop.h>

#include "nvidia_bvh/CudaBVH.hpp"
#include <curand_kernel.h>
#include "integrator/integrator.h"
#include "materials/materials.h"
#include "cameras/camera.h"
#include "network/server.h"


#define RT_GUI        1 << 0
#define RT_HEADLESS   1 << 1


namespace rt_pathtracer {

struct BvhResources {
	FW::Buffer nodeBuffer;
	FW::Buffer triWoopBuffer;
	FW::Buffer triIndexBuffer;
};


class Renderer {
public:
	Renderer(int im_width, int im_height, unsigned char flags_);
	// get socket if remote
	void loadScene(const char* filename);
	void startRendering();
	void clean();
private:
	void render();
	void allocateRenderingData();
	void allocateBuffers();
	void createBvhTextures();
	void CreateVBO();
	void InitialiseCudaGL();
	bool readBvhCache(const std::string& path);
	bool readBuffer(FILE *cache_file, FW::Buffer* buf);
	bool writeBvhCache(const std::string& path);
	bool writeBuffer(FILE *cache_file, FW::Buffer* buf);
private:
	unsigned char flags_;
	unsigned int frame_number_ = 0;
	FW::Vec2i im_size_;

	BaseCamera* camera_;
	CameraInfo* gpu_camera_;
	BoundingBox* scene_bb_;
	SceneData* scene_;
	AreaLights* area_lights_;
	EnvironmentLight* env_light_;
	BvhData* bvh_;
	FW::Ray* rays_;
	curandState* rand_states_;

	GLuint image_vbo_;
	FW::Vec3f* accum_buf_;
	FW::Vec3f* output_buf_;
	// glfw
	GLFWwindow* window_;
	// headless
	RenderServer server_;
	std::vector<std::uint8_t> image_data_;

	FW::Vec3f scene_bounds_[2];
	int light_depth_;
	FW::Vec3f* image_;
	KernelConfig kernel_config_;

	TexResources tex_resources_;
	BvhResources bvh_resources_;
};


} // namespace rt_pathtracer

#endif // RTPT_RENDERER_H_

