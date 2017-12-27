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

#include "renderer.h"
#include <stdlib.h>
#include <stdio.h>
#include "geometry/rib_loader.h"

namespace rt_pathtracer {


Renderer::Renderer(int im_width, int im_height, unsigned char flags)
{
	flags_ = flags;
	light_depth_ = 5;
	
	int w = im_width;
	int h = im_height;

	if ((flags_ & RT_GUI) != RT_GUI) {
		printf("Waiting for client...\n");
		server_.startListening();
		while(server_.width == 0 && server_.height == 0);
		printf("Recieved camera parameters\n");
		w = server_.width;
		h = server_.height;
		camera_ = new CardboardCamera(w, h);
	} else {
		camera_ = new MayaCamera(w, h);
	}

	im_size_ = FW::Vec2i(w, h);
	kernel_config_ = KernelConfig(w, h);
}


void Renderer::startRendering()
{
	if ((flags_ & RT_GUI) == RT_GUI) {
		MayaCamera::instance = (MayaCamera*)this->camera_;
		glfwSetMouseButtonCallback(window_, MayaCamera::onMouseButton);
		glfwSetCursorPosCallback(window_, MayaCamera::onMouseMove);
		glfwSetScrollCallback(window_, MayaCamera::onMouseScroll);
	}

	glfwSwapInterval(1);
	while(!glfwWindowShouldClose(window_))
	{
		render();
	}
}

void Renderer::allocateBuffers()
{
	cudaError_t error;
	cudaMalloc(&accum_buf_, im_size_[0] * im_size_[1] * sizeof(FW::Vec3f));
	cudaMalloc((void**)&gpu_camera_, sizeof(CameraInfo));
}

void Renderer::createBvhTextures()
{
	// Vertices
	cudaTextureDesc nodeOfsA_td;
	memset(&nodeOfsA_td, 0, sizeof(nodeOfsA_td));

	nodeOfsA_td.normalizedCoords = 0;
	nodeOfsA_td.addressMode[0] = cudaAddressModeClamp;
	nodeOfsA_td.readMode = cudaReadModeElementType;
	nodeOfsA_td.filterMode = cudaFilterModePoint;

	cudaResourceDesc nodeOfsA_rd;
	memset(&nodeOfsA_rd, 0, sizeof(nodeOfsA_rd));
	nodeOfsA_rd.resType = cudaResourceTypeLinear;
	nodeOfsA_rd.res.linear.devPtr =
				(void*)bvh_resources_.nodeBuffer.getCudaPtr();
	nodeOfsA_rd.res.linear.sizeInBytes =
				bvh_resources_.nodeBuffer.getSize();
	nodeOfsA_rd.res.linear.desc.f = cudaChannelFormatKindFloat;
	nodeOfsA_rd.res.linear.desc.x = 32;
	nodeOfsA_rd.res.linear.desc.y = 32;
	nodeOfsA_rd.res.linear.desc.z = 32;
	nodeOfsA_rd.res.linear.desc.w = 32;
	
	cudaTextureObject_t t_nodesA;
	cudaCreateTextureObject(&t_nodesA, &nodeOfsA_rd, &nodeOfsA_td, 0);

	// Triangles
	cudaTextureDesc triOfsA_td;
	memset(&triOfsA_td, 0, sizeof(triOfsA_td));

	triOfsA_td.normalizedCoords = 0;
	triOfsA_td.addressMode[0] = cudaAddressModeClamp;
	triOfsA_td.readMode = cudaReadModeElementType;
	triOfsA_td.filterMode = cudaFilterModePoint;

	cudaResourceDesc triOfsA_rd;
	memset(&triOfsA_rd, 0, sizeof(triOfsA_rd));
	triOfsA_rd.resType = cudaResourceTypeLinear;
	triOfsA_rd.res.linear.devPtr =
			(void*)bvh_resources_.triWoopBuffer.getCudaPtr();
	triOfsA_rd.res.linear.sizeInBytes =
			bvh_resources_.triWoopBuffer.getSize();
	triOfsA_rd.res.linear.desc.f = cudaChannelFormatKindFloat;
	triOfsA_rd.res.linear.desc.x = 32;
	triOfsA_rd.res.linear.desc.y = 32;
	triOfsA_rd.res.linear.desc.z = 32;
	triOfsA_rd.res.linear.desc.w = 32;

	cudaTextureObject_t t_trisA;
	cudaCreateTextureObject(&t_trisA, &triOfsA_rd, &triOfsA_td, 0);

	// Indices
	cudaTextureDesc triIndices_td;
	memset(&triIndices_td, 0, sizeof(triIndices_td));

	triIndices_td.normalizedCoords = 0;
	triIndices_td.addressMode[0] = cudaAddressModeClamp;
	triIndices_td.readMode = cudaReadModeElementType;
	triIndices_td.filterMode = cudaFilterModePoint;

	cudaResourceDesc triIndices_rd;
	memset(&triIndices_rd, 0, sizeof(triIndices_rd));
	triIndices_rd.resType = cudaResourceTypeLinear;
	triIndices_rd.res.linear.devPtr =
			(void*)bvh_resources_.triIndexBuffer.getCudaPtr();
	triIndices_rd.res.linear.sizeInBytes =
			bvh_resources_.triIndexBuffer.getSize();
	triIndices_rd.res.linear.desc.f = cudaChannelFormatKindSigned;
	triIndices_rd.res.linear.desc.x = 32;

	cudaTextureObject_t t_triIndices;
	cudaCreateTextureObject(&t_triIndices, &triIndices_rd, &triIndices_td, 0);


	BvhData bvh;
	bvh.t_nodesA = t_nodesA;
	bvh.t_trisA = t_trisA;
	bvh.t_triIndices = t_triIndices;

	cudaMalloc((void**)&bvh_, sizeof(BvhData));
	cudaMemcpy(bvh_, &bvh, sizeof(BvhData), cudaMemcpyHostToDevice);
}

void Renderer::allocateRenderingData()
{
	cudaError_t error;
	int img_size = im_size_.x * im_size_.y;

	cudaMalloc(&rays_, img_size * sizeof(FW::Ray));
	cudaMalloc(&image_, img_size * sizeof(FW::Vec3f));
	cudaMalloc(&rand_states_, img_size * sizeof(curandState));
}


bool Renderer::readBuffer(FILE *cache_file, FW::Buffer* buf)
{
	FW::S32 size;
	if (fread(&size, sizeof(FW::S32), 1, cache_file) != 1) {
		printf("Error reading buffer size\n");
		fclose(cache_file);
		return false;
	}
	buf->resizeDiscard(size);
	if (fread(buf->getMutablePtr(),
	    sizeof(FW::U8), size, cache_file) != size) {
		printf("Error reading buffer data\n");
		fclose(cache_file);
		return false;
	}
	return true;
}


bool Renderer::writeBuffer(FILE *cache_file, FW::Buffer* buf)
{
	FW::S32 size = buf->getSize();
	if (fwrite(&size, sizeof(FW::S32), 1, cache_file) != 1) {
		printf("Couldn't write buffer size\n");
		fclose(cache_file);
		return false;
	}
	if (fwrite(buf->getMutablePtr(),
	           sizeof(FW::U8), size, cache_file) != size) {
		printf("Couldn't write buffer data\n");
		fclose(cache_file);
		return false;
	}
	return true;
}


bool Renderer::readBvhCache(const std::string& path)
{
	FILE *cache_file = fopen(path.c_str(), "rb");
	if (!cache_file) {
		printf("Couldn't open BVH cache file\n");
		return false;
	}
	if (!readBuffer(cache_file, &bvh_resources_.nodeBuffer) ||
	    !readBuffer(cache_file, &bvh_resources_.triWoopBuffer) ||
	    !readBuffer(cache_file, &bvh_resources_.triIndexBuffer))
		return false;
	fclose(cache_file);
	return true;
}


bool Renderer::writeBvhCache(const std::string& path)
{
	FILE *cache_file = fopen(path.c_str(), "wb");
	if (!cache_file) {
		printf("Couldn't open BVH cache file\n");
		return false;
	}
	if (!writeBuffer(cache_file, &bvh_resources_.nodeBuffer) ||
	    !writeBuffer(cache_file, &bvh_resources_.triWoopBuffer) ||
	    !writeBuffer(cache_file, &bvh_resources_.triIndexBuffer))
		return false;
	fclose(cache_file);
	return true;
}

void Renderer::loadScene(const char* filename)
{
	RibScene rib_scene;
	rib_scene.loadRib(filename);
	FW::Scene fw_scene(&rib_scene);
	
	// Quadrics
	Quadric* gpu_quadrics;
	Quadric* cpu_quadrics = rib_scene.quadrics_ptr()->getPtr();
	int quadrics_count = rib_scene.quadrics_ptr()->getSize();
	cudaMalloc(&gpu_quadrics, quadrics_count * sizeof(Quadric));
	cudaMemcpy(gpu_quadrics, cpu_quadrics,
		quadrics_count * sizeof(Quadric),
		cudaMemcpyHostToDevice);

	// Triangle data
	TriangleData* gpu_tri_data;
	TriangleData* cpu_tri_data = rib_scene.triangle_data_ptr()->getPtr();
	int cpu_tri_data_size = rib_scene.triangle_data_ptr()->getSize();
	cudaMalloc(&gpu_tri_data, cpu_tri_data_size * sizeof(TriangleData));
	cudaMemcpy(gpu_tri_data, cpu_tri_data,
		cpu_tri_data_size * sizeof(TriangleData),
		cudaMemcpyHostToDevice);

	// Lights
	unsigned int* gpu_light_indices;
	unsigned int light_indices_size;
	unsigned int* light_indices = rib_scene.light_indices_ptr()->getPtr();
	light_indices_size = rib_scene.light_indices_ptr()->getSize();
	cudaMalloc(&gpu_light_indices, light_indices_size * sizeof(unsigned int));
	cudaMemcpy(gpu_light_indices, light_indices,
		light_indices_size * sizeof(unsigned int),
		cudaMemcpyHostToDevice);
	
	FW::Vec3f* gpu_light_vertices;
	FW::Vec3f* light_vertices = rib_scene.light_vertices_ptr()->getPtr();
	int light_vertices_size = rib_scene.light_vertices_ptr()->getSize();
	cudaMalloc(&gpu_light_vertices, light_vertices_size * sizeof(FW::Vec3f));
	cudaMemcpy(gpu_light_vertices, light_vertices,
		light_vertices_size * sizeof(FW::Vec3f),
		cudaMemcpyHostToDevice);

	// Materials
	Material* gpu_materials;
	Material* cpu_materials = rib_scene.materials_ptr()->getPtr();
	int cpu_materials_size = rib_scene.materials_ptr()->getSize();
	cudaMalloc(&gpu_materials, cpu_materials_size * sizeof(Material));
	cudaMemcpy(gpu_materials, cpu_materials,
		cpu_materials_size * sizeof(Material),
		cudaMemcpyHostToDevice);

	// BVH
	std::string path = rib_scene.dir_path() + "/" +
				rib_scene.rib_name() + ".bvh";
	if (!readBvhCache(path)) {
		FW::Platform platform;
		FW::BVH::BuildParams params;
		FW::BVH::Stats bvh_stats;
		params.stats = &bvh_stats;

		FW::BVH bvh(&fw_scene, platform, params);
		bvh_stats.print();

		BVHLayout layout = BVHLayout_Compact2;
		FW::CudaBVH cuda_bvh(bvh, layout);

		bvh_resources_.nodeBuffer = cuda_bvh.getNodeBuffer();
		bvh_resources_.triWoopBuffer = cuda_bvh.getTriWoopBuffer();
		bvh_resources_.triIndexBuffer = cuda_bvh.getTriIndexBuffer();
		writeBvhCache(path);
	} else {
		printf("Using BVH cache\n");
	}

	createBvhTextures();

	// Rendering buffers
	allocateBuffers();

	glfwInit();

	if ((flags_ & RT_GUI) != RT_GUI) {
		glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	}

	std::string title = "RT pathtracer - " + rib_scene.rib_name() + ".rib";

	window_ = glfwCreateWindow(
		im_size_[0],
		im_size_[1],
		title.c_str(),
		NULL, NULL
	);

	glfwMakeContextCurrent(window_);
	InitialiseCudaGL();

	CreateVBO();
	rib_scene.loadTexturesGL();

	// reset buffer
	cudaMemset(accum_buf_,
		1,
		im_size_[0] * im_size_[1] * sizeof(FW::Vec3f)
	);

	allocateRenderingData();

	// Textures
	rib_scene.loadTexturesGL();
	cudaTextureObject_t* gpu_textures = rib_scene.cuda_textures();

	// Env
	rib_scene.loadEnvTexture();
	FW::Vec3f env_color = rib_scene.env_color();
	float env_intensity = rib_scene.env_intensity();
	cudaTextureObject_t gpu_env_texture = rib_scene.env_texture();
	FW::Vec2i env_texture_size = rib_scene.env_texture_size();


	// Structs
	SceneData scene;
	scene.tri_data = gpu_tri_data;
	scene.materials = gpu_materials;
	scene.textures = gpu_textures;
	scene.quadrics = gpu_quadrics;
	scene.quadrics_count = quadrics_count;
	
	

	cudaMalloc((void**)&scene_, sizeof(SceneData));
	cudaMemcpy(scene_, &scene, sizeof(SceneData), cudaMemcpyHostToDevice);

	EnvironmentLight env_light;
	env_light.color = env_color;
	env_light.intensity = env_intensity;
	env_light.tex = gpu_env_texture;
	env_light.tex_size = env_texture_size;

	cudaMalloc((void**)&env_light_, sizeof(EnvironmentLight));
	cudaMemcpy(env_light_, &env_light,
		sizeof(EnvironmentLight),
		cudaMemcpyHostToDevice);

	AreaLights area_lights;
	area_lights.vertices = gpu_light_vertices;
	area_lights.indices = gpu_light_indices;
	area_lights.indices_count = light_indices_size;

	cudaMalloc((void**)&area_lights_, sizeof(AreaLights));
	cudaMemcpy(area_lights_, &area_lights,
		sizeof(AreaLights),
		cudaMemcpyHostToDevice);

	BoundingBox scene_bb;
	scene_bb.max = rib_scene.bounds()[0];
	scene_bb.min = rib_scene.bounds()[1];

	cudaMalloc((void**)&scene_bb_, sizeof(BoundingBox));
	cudaMemcpy(scene_bb_, &scene_bb,
		sizeof(BoundingBox),
		cudaMemcpyHostToDevice);

	tex_resources_ = rib_scene.tex_resources();
	
	server_.is_scene_ready = true;
	printf("The scene is ready\n");
}

void Renderer::render()
{
	if ((flags_ & RT_GUI) != RT_GUI) {
		static_cast<CardboardCamera*>(this->camera_)->
				updateCam(server_.cam_params_ptr());
	}

	if (camera_->is_dirty) {
		camera_->is_dirty = false;
		cudaMemset(accum_buf_,
			1,
			im_size_.x * im_size_.y * sizeof(FW::Vec3f)
		);
		frame_number_ = 0;
	}

	frame_number_++;

	cudaMemcpy(gpu_camera_, camera_->camera_info_ptr(),
			sizeof(CameraInfo),
			cudaMemcpyHostToDevice);

	cudaThreadSynchronize();
	cudaGLMapBufferObject((void**)&output_buf_, image_vbo_);

	glClear(GL_COLOR_BUFFER_BIT);

	launchRender(
		&kernel_config_,
		frame_number_,
		light_depth_,
		im_size_.x,
		im_size_.y,
		scene_bb_,
		scene_,
		bvh_,
		env_light_,
		image_,
		area_lights_,
		gpu_camera_,
		rays_,
		rand_states_,
		accum_buf_,
		output_buf_
	);

	cudaThreadSynchronize();
	cudaGLUnmapBufferObject(image_vbo_);

	glFlush();
	glFinish();

	glBindBuffer(GL_ARRAY_BUFFER, image_vbo_);
	glVertexPointer(2, GL_FLOAT, 12, 0);
	glColorPointer(4, GL_UNSIGNED_BYTE, 12, (GLvoid*)8);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	
	glDrawArrays(GL_POINTS, 0, im_size_[0] * im_size_[1]);
	glDisableClientState(GL_VERTEX_ARRAY);
	glfwSwapBuffers(window_);
	glfwPollEvents();

	//if ((flags_ & RT_GUI) != RT_GUI && frame_number_ < 10) {
	if ((flags_ & RT_GUI) != RT_GUI) {
		char* buffer = (char*)calloc(4, im_size_[0] * im_size_[1]);
		glReadPixels(
			0, 0, im_size_[0], im_size_[1],
			GL_RGBA, GL_UNSIGNED_BYTE, buffer
		);
		server_.send(buffer, im_size_[0], im_size_[1]);
		free(buffer);
	}
}


void Renderer::CreateVBO()
{
	glGenBuffers(1, &image_vbo_);
	glBindBuffer(GL_ARRAY_BUFFER, image_vbo_);
	unsigned int size = im_size_[0] * im_size_[1] * sizeof(FW::Vec3f);
	glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	cudaGLRegisterBufferObject(image_vbo_);
}


void Renderer::InitialiseCudaGL()
{
	glewInit();
	/*
	if (!glewIsSupported("GL_VERSION_2_0 ")) {
		std::cerr << "ERROR: OpenGL 2.0 is not supported\n";
		exit(0);
	}
	*/
	//cudaGLSetGLDevice(0);
	//cudaSetDevice(0);

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glMatrixMode(GL_PROJECTION);
	gluOrtho2D(0.0, im_size_[0], 0.0, im_size_[1]);
}

void Renderer::clean() {
	// env light
	EnvironmentLight env_light;
	cudaMemcpy(&env_light, env_light_, sizeof(EnvironmentLight),
			cudaMemcpyDeviceToHost);
	if (env_light.tex_size.x > 0)
		cudaDestroyTextureObject(env_light.tex);
	cudaFree(env_light_);

	// area lights
	AreaLights area_lights;
	cudaMemcpy(&area_lights, area_lights_, sizeof(AreaLights),
			cudaMemcpyDeviceToHost);

	cudaFree(area_lights.vertices);
	cudaFree(area_lights.indices);
	cudaFree(area_lights_);
	
	// scene
	SceneData scene;
	cudaMemcpy(&scene, scene_, sizeof(SceneData),
			cudaMemcpyDeviceToHost);
	cudaFree(scene.tri_data);
	cudaFree(scene.quadrics);
	cudaFree(scene.materials);
	
	int tex_count = tex_resources_.tex_arrays.getSize();
	cudaTextureObject_t* textures;
	cudaMemcpy(&textures, scene.textures,
			sizeof(cudaTextureObject_t) * tex_count,
			cudaMemcpyDeviceToHost);
	for (int i = 0;  i < tex_count; i++)
		cudaDestroyTextureObject(textures[i]);
	cudaFree(scene_);

	// textures
	cudaFree(tex_resources_.env_texture);
	for (int i = 0;  i < tex_resources_.mapped_res.getSize(); i++)
		cudaGraphicsUnmapResources(1, &tex_resources_.mapped_res[i], 0);
	for (int i = 0;  i < tex_count; i++)
		cudaFreeArray(tex_resources_.tex_arrays[i]);

	BvhData bvh;
	cudaMemcpy(&bvh, bvh_, sizeof(BvhData),
			cudaMemcpyDeviceToHost);
	cudaDestroyTextureObject(bvh.t_nodesA);
	cudaDestroyTextureObject(bvh.t_trisA);
	cudaDestroyTextureObject(bvh.t_triIndices);
	cudaFree(bvh_);

	cudaFree((void *)bvh_resources_.nodeBuffer.getCudaPtr());
	cudaFree((void *)bvh_resources_.triWoopBuffer.getCudaPtr());
	cudaFree((void *)bvh_resources_.triIndexBuffer.getCudaPtr());

	cudaFree(gpu_camera_);
	cudaFree(scene_bb_);

	cudaFree(rays_);
	cudaFree(image_);
	cudaFree(rand_states_);
}


} // namespace rt_pathtracer
