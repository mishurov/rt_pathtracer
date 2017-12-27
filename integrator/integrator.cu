#include <curand_kernel.h>
#include "integrator/integrator.h"

#include <ctime>
#include <cstdio>
#include "nvidia_bvh/Util.hpp"
#include "integrator/lights.h"
#include "integrator/cameras.h"
#include "materials/bxdf.h"


namespace rt_pathtracer {

__device__ int g_warp_counter;


struct LightVertex {
	FW::Vec3f d;
	FW::Vec3f L;
	bool specular;
	float rc;
	Intersection isect;
};


__global__ void InitRays(
		float width,
		float height,
		CameraInfo* cam,
		curandState* rand_states,
		FW::Ray* rays
	)
{
	unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height)
		return;

	int i = (height - y - 1) * width + x;
	int pixel_x = x;
	int pixel_y = height - y - 1;
	
	float jitter_x = curand_uniform(&rand_states[i]) - 0.5;
	float jitter_y = curand_uniform(&rand_states[i]) - 0.5;

	float sx = (jitter_x + pixel_x) / (cam->resolution.x - 1);
	float sy = (jitter_y + pixel_y) / (cam->resolution.y - 1);

	FW::Vec3f o, d;
	float pdf_pos, pdf_dir;
	
	if (cam->is_stereo) {
		bool is_left = x < width / 2;
		SampleStereoCameraPoint(
			sx, sy, cam, is_left, o, d, pdf_pos, pdf_dir
		);
	} else {
		SampleCameraPoint(sx, sy, cam, o, d, pdf_pos, pdf_dir);
	}

	FW::Ray r;
	r.origin = o;
	r.direction = d;
	rays[i] = r;
}

__global__ void RandomWalk(
			FW::Ray* rays,
			curandState* rand_states,
			int num_paths,
			int depth,
			BoundingBox* scene_bb,
			BvhData* bvh,
			SceneData* scene,
			AreaLights* area_lights,
			EnvironmentLight* env_light,
			FW::Vec3f* image
	)
{
	int i;
	__shared__ volatile int batch[MaxBlockHeight];

do
{
	const int tidx = threadIdx.x;
	volatile int& idx_base = batch[threadIdx.y];
	const unsigned int mask_terminated = __ballot(true);
	const int num_terminated = __popc(mask_terminated);
	const int idx_terminated = __popc(mask_terminated & ((1u << tidx) - 1));

	if (idx_terminated == 0)
		idx_base = atomicAdd(&g_warp_counter, num_terminated);

	i = idx_base + idx_terminated;

	if (i >= num_paths)
		break;

	FW::Ray r = rays[i];
	FW::Vec3f L(0, 0, 0);
	FW::Vec3f beta(1, 1, 1);
	bool specular = false;

	int bounces = 0;
	while (bounces < depth) {
		Intersection isect;
		isect.material_id = 0;
		bool intersected = Intersect(r.origin, r.direction, scene, bvh, isect);
		Material m = scene->materials[isect.material_id];

		if (bounces == 0 || specular) {
			FW::Vec3f Le;

			if (dot(isect.n, -r.direction) > 0 &&
			    intersected && m.bxdf == kLight) {
				Le = m.color * m.value;
			}

			if (!intersected && env_light->intensity > 0) {
				LightEmission env_emi;
				SampleEnvLight(r.direction, rand_states[i],
					scene_bb, env_light, env_emi
				);
				Le = env_emi.f * env_emi.pdf;
			}
			image[i] += Le * beta;
			specular = false;
		}
		bounces++;

		if (!intersected)
			break;

		FW::Vec3f wo = WorldToShading(-r.direction, isect.du, isect.dv, isect.n);

		float rc = 0;
		FW::Vec3f wi = bxdfSample(wo, isect, scene,
					rand_states[i], specular,
					rc);
		
		r.direction = ShadingToWorld(wi, isect.du, isect.dv, isect.n);

		FW::Vec3f offset = specular || m.bxdf != kGlass ?
							isect.n : r.direction * TMIN;
		r.origin = isect.p + offset * TMIN;

		if (m.bxdf == kLight)
			continue;

		float bxdf_pdf = bxdfPdf(wo, wi, isect, scene, specular, rc);
		if (bxdf_pdf <= 0)
			break;

		FW::Vec3f f = bxdfF(wo, wi, isect, scene, specular, rc);
		if (f == FW::Vec3f(0))
			break;
		
		if (m.bxdf == kGlass)
			specular = true;

		beta *= f * AbsDot(wi, FW::Vec3f(0, 0, 1)) / bxdf_pdf;

		if (m.bxdf != kLight && !specular) {
			LightEmission emi;
			SampleLight(
				r.origin, isect.n, rand_states[i],
				scene_bb, env_light,
				area_lights, scene, emi
			);

			if (emi.pdf > 0 && emi.pdf_dir > 0 && emi.f != FW::Vec3f(0) &&
			    FW::dot(isect.n, emi.d) < 0) {
				FW::Vec3f wi_light = WorldToShading(
					-emi.d, isect.du, isect.dv, isect.n
				);
				float surf_pdf = bxdfPdf(
					wo, wi_light, isect,
					scene, specular, rc
				);
				if (surf_pdf > 0) {
					FW::Vec3f f = bxdfF(
						wo, wi_light, isect, scene,
						specular, rc
					);
					float weight = PowerHeuristic(
						1, emi.pdf_dir / emi.pdf_dir,
						1, surf_pdf
					);
					if (f != FW::Vec3f(0) && weight > 0 &&
					    IsVisibile(r.origin, emi.o, scene, bvh)) {
						FW::Vec3f Ld = emi.f * f *
								weight / (emi.pdf / emi.pdf_dir);
						image[i] += Ld * beta;
					}
				}
			}
		}

	}

} while (true);
}


union OutColor
{
	float c;
	uchar4 components;
};

__global__ void DrawColor(
		int frame_number,
		int width,
		int height,
		FW::Vec3f* res,
		FW::Vec3f* accum_buf,
		FW::Vec3f* output_buf
	)
{
	unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height)
		return;
	int i = (height - y - 1) * width + x;

	FW::Vec3f final = res[i];
	accum_buf[i] += final;
	FW::Vec3f temp = accum_buf[i] / frame_number;

	OutColor output;

	FW::Vec3f clamped = FW::Vec3f(clampf(temp.x, 0.0f, 1.0f),
				clampf(temp.y, 0.0f, 1.0f),
				clampf(temp.z, 0.0f, 1.0f));

	output.components = make_uchar4(
		(unsigned char)(powf(clamped.x, 1 / 2.2f) * 255),
		(unsigned char)(powf(clamped.y, 1 / 2.2f) * 255),
		(unsigned char)(powf(clamped.z, 1 / 2.2f) * 255),
		1
	);
	output_buf[i] = FW::Vec3f(x, y, output.c);
}


__global__ void InitRandoms(int width, int height,
				int frame_number, curandState* rand_states)
{
	unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height)
		return;
	int i = (height - y - 1) * width + x;

	unsigned int seed = RandHash(frame_number + i + (unsigned int)clock());
	curand_init(seed, 0, 0, &rand_states[i]);
}

__global__ void TestStereo(
		float width,
		float height,
		CameraInfo* cam,
		FW::Vec3f* res
	)
{
	unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= width || y >= height)
		return;
	FW::Vec3f c;
	if (x < width / 2) {
		c = FW::Vec3f(1, 0, 0);
	} else {
		c = FW::Vec3f(0, 0, 1);
	}

	if (!cam->is_stereo)
		c = FW::Vec3f(0, 1, 0);

	int i = (height - y - 1) * width + x;
	res[i] = c;
}


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
		CameraInfo* cam,
		FW::Ray* rays,
		curandState* rand_states,
		FW::Vec3f* accum_buf,
		FW::Vec3f* output_buf
	)
{
	cudaError_t error;
	error = cudaGetLastError();
	if (error != cudaSuccess)
		printf("start %i: %s\n", frame_number, cudaGetErrorString(error));
	
	int num_paths = width * height;

	cudaMemset(rand_states, 0, num_paths * sizeof(FW::Vec3f));
	InitRandoms<<<kernel_conf->reg_grid, kernel_conf->reg_block>>>(
		width, height, frame_number, rand_states
	);

	cudaMemset(rays, 0, num_paths * sizeof(FW::Ray));
	cudaMemset(image, 0, num_paths * sizeof(FW::Vec3f));

/*
	TestStereo<<<kernel_conf->reg_grid, kernel_conf->reg_block>>>(
		width, height, cam, image
	);
*/
	
	InitRays<<<kernel_conf->reg_grid, kernel_conf->reg_block>>>(
		width, height, cam, rand_states, rays
	);

	int zilch = 0;

	cudaMemcpyToSymbol(g_warp_counter, &zilch, sizeof(int));
	RandomWalk<<<kernel_conf->pers_grid, kernel_conf->pers_block>>>(
			rays,
			rand_states,
			num_paths,
			depth,
			scene_bb,
			bvh,
			scene,
			area_lights,
			env_light,
			image
	);

	error = cudaGetLastError();
	if (error != cudaSuccess)
		printf("walk %i: %s\n", frame_number, cudaGetErrorString(error));


	DrawColor<<<kernel_conf->reg_grid, kernel_conf->reg_block>>>(
		frame_number,
		width,
		height,
		image,
		accum_buf,
		output_buf
	);
	if (error != cudaSuccess)
		printf("walk %i: %s\n", frame_number, cudaGetErrorString(error));
}

} // namespace rt_pathtracer
