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

#ifndef RTPT_INTEGRATOR_UTILS_H_
#define RTPT_INTEGRATOR_UTILS_H_

#include "materials/materials.h"
#include "integrator/integrator.h"
#include "geometry/quadrics.h"
#include "nvidia_kernels/CudaTracerKernels.hpp"

#define PB_PI 3.14159265358979323846
#define PB_INV_PI 0.31830988618379067154

#define PB_PI_OVER_2 1.57079632679489661923
#define PB_PI_OVER_4 0.78539816339744830961

namespace rt_pathtracer {


FW_CUDA_FUNC unsigned int RandHash(unsigned int a) {
    a = (a+0x7ed55d16) + (a<<12);
    a = (a^0xc761c23c) ^ (a>>19);
    a = (a+0x165667b1) + (a<<5);
    a = (a+0xd3a2646c) ^ (a<<9);
    a = (a+0xfd7046c5) + (a<<3);
    a = (a^0xb55a4f09) ^ (a>>16);
    return a;
}

FW_CUDA_FUNC void Shuffle(curandState& rand_state)
{
	unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;
	curand_init(RandHash((unsigned int)clock() + x + y), 0, 0, &rand_state);
}

FW_CUDA_FUNC float clampf(float a, float lo, float hi)
{
	return a < lo ? lo : a > hi ? hi : a;
}

FW_CUDA_FUNC float AbsCosTheta(const FW::Vec3f &w) { return abs(w.z); }

FW_CUDA_FUNC float AbsDot(const FW::Vec3f &v1, const FW::Vec3f &v2) {
	return abs(FW::dot(v1, v2));
}

FW_CUDA_FUNC bool SameHemisphere(const FW::Vec3f &w, const FW::Vec3f &wp) {
	return w.z * wp.z > 0;
}

FW_CUDA_FUNC float PowerHeuristic(int num_f, float f_pdf,
					int num_g, float g_pdf)
{
	float f = num_f * f_pdf;
	float g = num_g * g_pdf;

	return (f * f) / (f * f + g * g);
}

FW_CUDA_FUNC FW::Vec3f Orthogonal(FW::Vec3f v)
{
	float x = abs(v.x);
	float y = abs(v.y);
	float z = abs(v.z);

	FW::Vec3f x_axis(1, 0, 0);
	FW::Vec3f y_axis(0, 1, 0);
	FW::Vec3f z_axis(0, 0, 1);

	FW::Vec3f other = x < y ?
			(x < z ? x_axis : z_axis) : (y < z ? y_axis : z_axis);

	return FW::cross(v, other);
}

FW_CUDA_FUNC void ComputeTangents(
			const FW::Vec3f& n, FW::Vec3f& s, FW::Vec3f& t)
{
	s = FW::cross((fabs(n.x) > .1 ?
				FW::Vec3f(0, 1, 0) : FW::Vec3f(1, 0, 0)), n);
	//s = Orthogonal(n);
	s.normalize();
	t = FW::cross(n, s);
}

FW_CUDA_FUNC FW::Vec3f WorldToShading(
				const FW::Vec3f& v,
				const FW::Vec3f& s,
				const FW::Vec3f& t,
				const FW::Vec3f& n
			)
{
        return FW::Vec3f(FW::dot(v, s), FW::dot(v, t), FW::dot(v, n));
}

FW_CUDA_FUNC FW::Vec3f ShadingToWorld(
				const FW::Vec3f& v,
				const FW::Vec3f& s,
				const FW::Vec3f& t,
				const FW::Vec3f& n
			)
{
	return FW::Vec3f(
		s.x * v.x + t.x * v.y + n.x * v.z,
		s.y * v.x + t.y * v.y + n.y * v.z,
		s.z * v.x + t.z * v.y + n.z * v.z
	);
}


FW_CUDA_FUNC bool IsVisibile(const FW::Vec3f& p1, const FW::Vec3f& p2,
				SceneData* scene, BvhData* bvh)
{
	FW::Vec3f delta = p2 - p1;
	FW::Vec3f dir = delta.normalized();

	float dist = delta.length();

	if (dist == 0)
		return false;
	
	float4 o = make_float4(p1.x, p1.y, p1.z, TMIN);
	float4 d = make_float4(dir.x, dir.y, dir.z, dist);

	FW::Ray r;
	r.origin = p1;
	r.direction = dir;
	r.tmin = TMIN;
	r.tmax = dist;

	Intersection isect;
	isect.t = TMAX;

	if (scene->quadrics_count > 0
		&& IntersectQuadrics(r, scene->quadrics,
	                 scene->quadrics_count, isect)) {
		return false;
	}

	int hit_index;
	float hit_t;
	FW::Vec2f bari;
	FW::Vec3f v[3];

	IntersectKepler(o, d, bvh->t_nodesA, bvh->t_trisA, bvh->t_triIndices,
			false, hit_index, hit_t, bari, v);

	if (hit_index > -1)
		return false;

	return true;
}


FW_CUDA_FUNC bool Intersect(
			const FW::Vec3f& o,
			const FW::Vec3f& d,
			SceneData* scene,
			BvhData* bvh,
			Intersection& isect
		)
{
	float4 orig = make_float4(o.x, o.y, o.z, TMIN);
	float4 dir = make_float4(d.x, d.y, d.z, TMAX);
	FW::Ray r;
	r.origin = o;
	r.direction = d;
	r.tmin = TMIN;
	r.tmax = TMAX;
	isect.t = TMAX;

	if (scene->quadrics_count > 0) {
		IntersectQuadrics(r, scene->quadrics,
				scene->quadrics_count, isect);
	}

	float hit_t;
	int hit_index;
	FW::Vec2f bari2;
	FW::Vec3f v[3];

	IntersectKepler(orig, dir,
			bvh->t_nodesA, bvh->t_trisA, bvh->t_triIndices,
			false, hit_index, hit_t, bari2, v);

	if (hit_index > -1 && hit_t < isect.t) {
		isect.t = hit_t;
		TriangleData td = scene->tri_data[hit_index];

		isect.material_id = td.material_id;

		FW::Vec3f bari(bari2.x, bari2.y, 1 - bari2.x - bari2.y);
		FW::Vec3f n(0);
		for (int i = 0; i < 3; i++) {
			n += td.normals[i] * bari[i];
		}
		if (n == FW::Vec3f(0)) {
			n = FW::cross(v[1], v[2]);
		}
		n.normalize();
		
		isect.n = n;

		FW::Vec2f uv(0);
		for (int i = 0; i < 3; i++)
			uv += td.sts[i] * bari[i];
	
		isect.uv = uv;
		isect.p = o + d.normalized() * hit_t;
		isect.du = v[1].normalized();
	}

	if (FW::dot(isect.n, d) > 0 &&
	    scene->materials[isect.material_id].bxdf != kGlass &&
	    scene->materials[isect.material_id].bxdf != kLight) {
		isect.n *= -1;
		isect.du *= -1;
	}

	if (isect.du == FW::Vec3f(0)) {
		ComputeTangents(isect.n, isect.du, isect.dv);
	} else {
		isect.dv = FW::cross(isect.n, isect.du);
	}

	return isect.t < TMAX;
}


FW_CUDA_FUNC FW::Vec2f ConcentricSampleDisk(const FW::Vec2f &u) {
	FW::Vec2f u_offset = 2.f * u - FW::Vec2f(1, 1);

	if (u_offset.x == 0 && u_offset.y == 0)
		return FW::Vec2f(0, 0);

	float theta, r;
	if (fabs(u_offset.x) > fabs(u_offset.y)) {
		r = u_offset.x;
		theta = PB_PI_OVER_4 * (u_offset.y / u_offset.x);
	} else {
		r = u_offset.y;
		theta = PB_PI_OVER_2 - PB_PI_OVER_4 * (u_offset.x / u_offset.y);
	}
	return r * FW::Vec2f(cos(theta), sin(theta));
}


FW_CUDA_FUNC FW::Vec3f CosineSampleHemisphere(curandState& rand_state)
{
	FW::Vec2f u(
		curand_uniform(&rand_state),
		curand_uniform(&rand_state)
	);
	FW::Vec2f d = ConcentricSampleDisk(u);
	float z = sqrt(fmax((float)0, 1 - d.x * d.x - d.y * d.y));
	return FW::Vec3f(d.x, d.y, z);
}


FW_CUDA_FUNC float CosineHemispherePdf(float cos_theta) {
	return cos_theta * PB_INV_PI;
}

} // namespace rt_pathtracer

#endif // RTPT_INTEGRATOR_UTILS_H_
