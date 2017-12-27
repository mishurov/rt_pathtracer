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

#ifndef RTPT_QUADRICS_H_
#define RTPT_QUADRICS_H_

#include "geometry/transforms.h"
#include "materials/materials.h"
#include "nvidia_bvh/Util.hpp"

#define TMIN 0.00001f
#define TMAX 1e20

#define MAX_VISIBLE_QUADRICS 25


namespace rt_pathtracer {


FW_CUDA_FUNC float Radians(float d) {
	return d * M_PI / 180;
}

// Attribute "tesselation" "int quadrics" [0]

enum QuadricType
{
	kqSphere,
	kqCone,
	kqCylinder,
	kqHyperboloid,
	kqParaboloid,
	kqDisk,
	kqTorus
};


struct Quadric
{
	QuadricType type;
	int material_id;
	TransformMatrix4f basis;
	TransformMatrix4f inv_basis;
	float params[7];

	FW::Vec3f b_min;
	FW::Vec3f b_max;

	FW_CUDA_FUNC void computeBounds();
	FW_CUDA_FUNC void computeTransformers()
	{
		inv_basis = basis.getInverted();
	}
	FW_CUDA_FUNC void toRadians()
	{
		if (type == kqSphere) {
			float radius = params[0];
			float zmin = params[1];
			float zmax = params[2];
			params[3] = Radians(params[3]);
			params[4] = acosf(FW::clamp(zmin/radius, -1.f, 1.f));
			params[5] = acosf(FW::clamp(zmax/radius, -1.f, 1.f));
		}
		if (type == kqCone || type == kqDisk) {
			params[2] = Radians(params[2]);
		}
		if (type == kqCylinder || type == kqParaboloid) {
			params[3] = Radians(params[3]);
		}
		if (type == kqTorus) {
			params[2] = Radians(params[2]);
			params[3] = Radians(params[3]);
			params[4] = Radians(params[4]);
		}
		if (type == kqHyperboloid) {
			params[6] = Radians(params[6]);
		}
	}
	
};

__device__ bool IntersectQuadrics(FW::Ray& r,
				Quadric* quadrics,
				int quadrics_count,
				Intersection& isect);


FW_CUDA_FUNC FW::Ray RayToObject(const FW::Ray& r, const Quadric& q) {
	FW::Vec3f o = q.inv_basis.transformP(r.origin);
	FW::Vec3f d = q.inv_basis.transformV(r.direction);
	float tmin = r.tmin;
	if (tmin > TMIN) {
		FW::Vec3f pmin = r.origin + r.direction * r.tmin;
		pmin = q.inv_basis.transformP(pmin);
		tmin = (o - pmin).length();
	}
	float tmax = r.tmax;
	if (tmax < TMAX) {
		FW::Vec3f pmax = r.origin + r.direction * r.tmax;
		pmax = q.inv_basis.transformP(pmax);
		tmax = (o - pmax).length();
	}
	FW::Ray tr;
	tr.origin = o;
	tr.tmin = tmin;
	tr.direction = d;
	tr.tmax = tmax;
	return tr;
}


FW_CUDA_FUNC void TransformBounds(const Quadric& q,
					FW::Vec3f& mini, FW::Vec3f& maxi)
{
	FW::Vec3f p[8];

	p[0] = FW::Vec3f(mini.x, mini.y, mini.z);
	p[1] = FW::Vec3f(mini.x, maxi.y, mini.z);
	p[2] = FW::Vec3f(maxi.x, mini.y, mini.z);
	p[3] = FW::Vec3f(maxi.x, maxi.y, mini.z);

	p[4] = FW::Vec3f(mini.x, mini.y, maxi.z);
	p[5] = FW::Vec3f(mini.x, maxi.y, maxi.z);
	p[6] = FW::Vec3f(maxi.x, mini.y, maxi.z);
	p[7] = FW::Vec3f(maxi.x, maxi.y, maxi.z);

	mini = FW::Vec3f(TMAX);
	maxi = FW::Vec3f(-TMAX);

	for (int i = 0; i < 8; i++) {
		FW::Vec3f t = q.basis.transformP(p[i]);
		mini = FW::min(t, mini);
		maxi = FW::max(t, maxi);
	}
}


FW_CUDA_FUNC void SphereBounds(const Quadric& q, FW::Vec3f& mn, FW::Vec3f& mx)
{
	float radius = q.params[0];
	float zmin = q.params[1];
	float zmax = q.params[2];

	mn = FW::Vec3f(-radius, -radius, zmin);
	mx = FW::Vec3f(radius, radius, zmax);

	TransformBounds(q, mn, mx);
}

__device__ bool IntersectSphere(const FW::Ray& r, const Quadric& q,
						Intersection& isect);


FW_CUDA_FUNC void ConeBounds(const Quadric& q, FW::Vec3f& mn, FW::Vec3f& mx)
{
	float height = q.params[0];
	float radius = q.params[1];

	mn = FW::Vec3f(-radius, -radius, 0);
	mx = FW::Vec3f(radius, radius, height);

	TransformBounds(q, mn, mx);
}

__device__ bool IntersectCone(const FW::Ray& r, const Quadric& q,
						Intersection& isect);


FW_CUDA_FUNC void CylinderBounds(const Quadric& q,
					FW::Vec3f& min, FW::Vec3f& max)
{
	SphereBounds(q, min, max);
}

__device__ bool IntersectCylinder(const FW::Ray& r, const Quadric& q,
						Intersection& isect);


FW_CUDA_FUNC void DiskBounds(const Quadric& q, FW::Vec3f& mn, FW::Vec3f& mx)
{
	float height = q.params[0];
	float radius = q.params[1];

	mn = FW::Vec3f(-radius, -radius, height);
	mx = FW::Vec3f(radius, radius, height);
	TransformBounds(q, mn, mx);
}

__device__ bool IntersectDisk(const FW::Ray& r, const Quadric& q,
						Intersection& isect);

FW_CUDA_FUNC void HyperboloidBounds(const Quadric& q,
					FW::Vec3f& mn, FW::Vec3f& mx)
{
	float x1 = q.params[0];
	float y1 = q.params[1];
	float z1 = q.params[2];
	float x2 = q.params[3];
	float y2 = q.params[4];
	float z2 = q.params[5];

	float d = x1 * x1 + y1 * y1;
	float d2 = x2 * x2 + y2 * y2;

	if (d2 > d) d = d2;
		d = sqrtf(d);

	mn = FW::Vec3f(-d, -d, fmin(z1, z2));
	mx = FW::Vec3f(d, d, fmax(z1, z2));
	TransformBounds(q, mn, mx);
}

__device__ bool IntersectHyperboloid(const FW::Ray& r, const Quadric& q,
						Intersection& isect);


FW_CUDA_FUNC void ParaboloidBounds(const Quadric& q,
					FW::Vec3f& min, FW::Vec3f& max)
{
	SphereBounds(q, min, max);
}

__device__ bool IntersectParaboloid(const FW::Ray& r, const Quadric& q,
						Intersection& isect);


FW_CUDA_FUNC void TorusBounds(const Quadric& q,
					FW::Vec3f& mn, FW::Vec3f& mx)
{
	float rmajor = q.params[0];
	float rminor = q.params[1];

	mn = FW::Vec3f(-rmajor - rminor, -rmajor - rminor, -rminor);
	mx = FW::Vec3f(rmajor + rminor, rmajor + rminor, rminor);
	TransformBounds(q, mn, mx);
}

__device__ bool IntersectTorus(const FW::Ray& r, const Quadric& q,
						Intersection& isect);

FW_CUDA_FUNC void Quadric::computeBounds() {
	computeTransformers();
	switch (type) {
	case kqSphere:
		SphereBounds(*this, b_min, b_max);
		break;
	case kqCone:
		ConeBounds(*this, b_min, b_max);
		break;
	case kqCylinder:
		CylinderBounds(*this, b_min, b_max);
		break;
	case kqHyperboloid:
		HyperboloidBounds(*this, b_min, b_max);
		break;
	case kqParaboloid:
		ParaboloidBounds(*this, b_min, b_max);
		break;
	case kqDisk:
		DiskBounds(*this, b_min, b_max);
		break;
	case kqTorus:
		TorusBounds(*this, b_min, b_max);
		break;
	}
}


} // namespace rt_pathtracer

#endif // RTPT_QUADRICS_
