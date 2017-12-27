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

#ifndef RTPT_INTEGRATORS_LIGHTS_H_
#define RTPT_INTEGRATORS_LIGHTS_H_

#include "integrator/utils.h"

namespace rt_pathtracer {

FW_CUDA_FUNC void SampleEnvLight(
		const FW::Vec3f& d,
		curandState rand_state,
		BoundingBox* scene_bb,
		EnvironmentLight* light,
		LightEmission& emi
	)
{
	//Shuffle(rand_state);

	float longlat_x = atan2f(d.x, d.z);
	longlat_x = longlat_x < 0.f ? longlat_x + 2 * M_PI : longlat_x;
	float longlat_y = acosf(d.y);
	float u = longlat_x /  (2 * M_PI);
	float v = longlat_y / M_PI;
	emi.d = -d;

	float radius = fmax(scene_bb->max.x,
				fmax(scene_bb->max.y, scene_bb->max.z));

	float phimax = M_PI * 0.5;
	float phimin = -M_PI * 0.5;
	//float height_coeff = -0.2;
	//float phimin = asin(height_coeff);

	float phi = phimin + v * (phimax - phimin);
	float theta = u * 2 * M_PI;
	
	emi.o.z = cos(theta) * cos(phi);
	emi.o.x = sin(theta) * cos(phi);
	emi.o.y = -sin(phi);
	emi.o *= radius;

	emi.f = light->color;

	if (light->tex_size[0] > 0 && light->tex_size[1] > 0) {
		int tu = (int)(u * (float)light->tex_size.x);
		int tv = (int)(v * (float)light->tex_size.y);
		int hdr_texel_idx = tu + tv * light->tex_size.x;
		float4 hdr_color = tex1Dfetch<float4>(light->tex, hdr_texel_idx);
		emi.f.x = hdr_color.x;
		emi.f.y = hdr_color.y;
		emi.f.z = hdr_color.z;
	}
	emi.f *= light->intensity;

	emi.n = (-emi.o).normalized();

	//float uv_pdf = 0.01;
	//emi.pdf_dir = uv_pdf / (2 * PB_PI * PB_PI * sin(theta));
	emi.pdf_dir = 1;

	radius *= 0.5;
	emi.pdf = 1 / (PB_PI * radius * radius);
}


FW_CUDA_FUNC void SampleAreaPoint(
		curandState rand_state,
		AreaLights* light,
		SceneData* scene,
		LightEmission& emi
	)
{
	//Shuffle(rand_state);

	float r_tri = curand_uniform(&rand_state);
	int index = (int) ((light->indices_count - 1) * r_tri);

	FW::Vec3f p0 = light->vertices[index * 3 + 0];
	FW::Vec3f p1 = light->vertices[index * 3 + 1];
	FW::Vec3f p2 = light->vertices[index * 3 + 2];

	float r_u = curand_uniform(&rand_state);
	float r_v = curand_uniform(&rand_state);
	float r_w = curand_uniform(&rand_state);
	float u = r_u;
	float v = (1 - r_v) * r_w;
	float w = 1 - u - v;

	float A = FW::cross(p1 - p0, p2 - p0).length();
	emi.pdf = 1 / A;

	emi.o = (u * p0) + (v * p1) + (w * p2);
	index = light->indices[index];
	TriangleData td = scene->tri_data[index];

	FW::Vec3f n0 = td.normals[0];
	FW::Vec3f n1 = td.normals[1];
	FW::Vec3f n2 = td.normals[2];
	emi.n = (u * n0) + (v * n1) + (w * n2);
	emi.n.normalize();

	Material m = scene->materials[td.material_id];
	emi.f = m.color * m.value;
}


FW_CUDA_FUNC void SampleLight(
		const FW::Vec3f& p,
		const FW::Vec3f& n,
		curandState rand_state,
		BoundingBox* scene_bb,
		EnvironmentLight* env_light,
		AreaLights* area_lights,
		SceneData* scene,
		LightEmission& emi
	)
{
	bool env_active = env_light->intensity > 0;
	bool area_active = area_lights->indices_count > 0;

	if (!env_active && !area_active)
		return;

	float fair_coin = curand_uniform(&rand_state);

	// randomly choose a light if both are active
	int lights = 1;
	if (env_active && area_active) {
		lights = 2;
		if (fair_coin > 0.5)
			env_active = false;
		else
			area_active = false;
	}

	if (env_active) {
		FW::Vec3f wi = CosineSampleHemisphere(rand_state);
		FW::Vec3f s, t, d;
		ComputeTangents(n, s, t);
		d = ShadingToWorld(wi, s, t, n);
		if (FW::dot(d, n) < 0) d * -1;
		d.normalize();
		SampleEnvLight(
			d, rand_state,
			scene_bb, env_light,
			emi
		);
	}

	if (area_active) {
		SampleAreaPoint(rand_state, area_lights, scene, emi);
		emi.d = p - emi.o;
		float sqr_dist = emi.d.lenSqr();
		if (sqr_dist == 0) {
			emi.pdf = 0;
			return;
		} else {
			emi.d.normalize();
		}

		float cos_theta = FW::dot(emi.d, emi.n);
		emi.pdf *= sqr_dist * 0.01;
		emi.pdf_dir = CosineHemispherePdf(cos_theta);
	}
	emi.pdf *= lights;
}

} // namespace rt_pathtracer

#endif // RTPT_INTEGRATORS_LIGHTS_H_
