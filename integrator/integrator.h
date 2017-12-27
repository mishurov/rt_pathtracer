#ifndef RTPT_INTEGRATOR_H_
#define RTPT_INTEGRATOR_H_

#include "materials/materials.h"
#include "cameras/camera.h"
#include "geometry/quadrics.h"


#if CUDA_VERSION >= 9000
#define __ballot(x) __ballot_sync(__activemask(), (x))
#endif

namespace rt_pathtracer {


struct EnvironmentLight
{
	FW::Vec3f color;
	float intensity;
	cudaTextureObject_t tex;
	FW::Vec2i tex_size;
};


struct AreaLights
{
	FW::Vec3f* vertices;
	unsigned int* indices;
	int indices_count;
};


struct LightEmission
{
	FW::Vec3f o;
	FW::Vec3f d;
	FW::Vec3f n;
	FW::Vec3f f;
	float pdf;
	float pdf_dir;
	float cos_theta;
};


struct BvhData
{
	cudaTextureObject_t t_nodesA;
	cudaTextureObject_t t_trisA;
	cudaTextureObject_t t_triIndices;
};


struct SceneData
{
	TriangleData* tri_data;
	Material* materials;
	cudaTextureObject_t* textures;
	Quadric* quadrics;
	int quadrics_count;
};


struct BoundingBox {
	FW::Vec3f max;
	FW::Vec3f min;
	FW_CUDA_FUNC FW::Vec3f diag() const { return max - min; }
	FW_CUDA_FUNC FW::Vec3f center() { return min + diag() * 0.5; }
};


struct KernelConfig
{
	dim3 reg_block;
	dim3 reg_grid;
	dim3 pers_block;
	dim3 pers_grid;
	KernelConfig() {}
	KernelConfig(int width, int height) {
		reg_block = dim3(16, 16, 1);
		reg_grid = dim3(width / reg_block.x + 1,
				 height / reg_block.y + 1, 1);
	
	
		// optimal for kepler;
		int blockWidth = 32;
		int blockHeight = 4;
		int blockWarps = (blockWidth * blockHeight + 31) / 32;
		int numBlocks = (720 + blockWarps - 1) / blockWarps;
		int numThreads = numBlocks * blockWidth * blockHeight;
		int threadsPerBlock = blockWidth * blockHeight;
		int gridWidth = (numThreads + threadsPerBlock - 1) /
							threadsPerBlock;
		int gridHeight = 1;
		int maxGridWidth = 65536;

		while (gridWidth > maxGridWidth)
		{
			gridWidth = (gridWidth + 1) >> 1;
			gridHeight <<= 1;
		}

		pers_block = dim3(blockWidth, blockHeight, 1);
		pers_grid = dim3(gridWidth, gridHeight, 1);
	}
};


void launchRender(
		KernelConfig* kernel_conf,
		int frame_number,
		int depth,
		int width,
		int height,
		BoundingBox* scene_bb,
		SceneData* scene,
		BvhData* bvh,
		EnvironmentLight* env_light,
		FW::Vec3f* image,
		AreaLights* area_lights,
		CameraInfo* gpu_camera,
		FW::Ray* rays,
		curandState* rand_states,
		FW::Vec3f* accum_buf,
		FW::Vec3f* output_buf
	);


} // namespace rt_pathtracer

#endif // RTPT_INTEGRATOR_H_
