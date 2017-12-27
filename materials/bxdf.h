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

#ifndef RTPT_INTEGRATORS_BXDF_H_
#define RTPT_INTEGRATORS_BXDF_H_

#include "materials/microfacet.h"


namespace rt_pathtracer {


#define GLASS_ETA_A 1
#define GLASS_ETA_B 1.5


FW_CUDA_FUNC FW::Vec3f bxdfSample(
			const FW::Vec3f& wo,
			const Intersection& isect,
			SceneData* scene,
			curandState& rand_state,
			bool& specular,
			float& C
	)
{
	//Shuffle(rand_state);

	switch (scene->materials[isect.material_id].bxdf)
	{
	case kDiffuse:
		{
			FW::Vec3f wi = CosineSampleHemisphere(rand_state);
			if (wo.z < 0) wi.z *= -1;
			return wi;
		}
	case kPlastic:
		{
			float rC = curand_uniform(&rand_state);
			C = 1 - wo.z;

			float ru = curand_uniform(&rand_state);
			float rv = curand_uniform(&rand_state);
			float alpha_x = scene->materials[
						isect.material_id].value;
			float alpha_y = alpha_x;
			FW::Vec3f wh = BeckmannSample_wh(
				wo, FW::Vec2f(ru, rv), alpha_x, alpha_y
			);

			if (rC < C) {
				specular = true;
        			return Reflect(wo, wh);
			}

			FW::Vec3f wi = CosineSampleHemisphere(rand_state);
			return wi;
		}
	case kMetal:
		{
			specular = true;
			float ru = curand_uniform(&rand_state);
			float rv = curand_uniform(&rand_state);
			float alpha_x = scene->materials[
						isect.material_id].value;
			float alpha_y = alpha_x;
			FW::Vec3f wh = BeckmannSample_wh(
				wo, FW::Vec2f(ru, rv), alpha_x, alpha_y
			);
			return Reflect(wo, wh);
		
		}
	case kGlass:
		{
			float etaA = GLASS_ETA_A;
			float etaB = GLASS_ETA_B;
			float rC = curand_uniform(&rand_state);
			C = FresnelDielectric(CosTheta(wo), etaA, etaB);

			float ru = curand_uniform(&rand_state);
			float rv = curand_uniform(&rand_state);
			float alpha_x = scene->materials[
						isect.material_id].value;
			float alpha_y = alpha_x;
			FW::Vec3f wh = BeckmannSample_wh(
				wo, FW::Vec2f(ru, rv), alpha_x, alpha_y
			);

			if (rC < C) {
				specular = true;
        			return Reflect(wo, wh);
			}

			float eta = CosTheta(wo) > 0 ?
						(etaA / etaB) : (etaB / etaA);
			FW::Vec3f wi;
			if (Refract(wo, wh, eta, &wi))
				return wi;
			else
				return FW::Vec3f(0);
		}
	case kLight:
		{
			FW::Vec3f wi = CosineSampleHemisphere(rand_state);
			if (wo.z < 0) wi.z *= -1;
			return wi;
		}
	}
	return FW::Vec3f(0);
}

FW_CUDA_FUNC float bxdfPdf(
			const FW::Vec3f& wo,
			const FW::Vec3f& wi,
			const Intersection& isect,
			SceneData* scene,
			const bool& specular,
			const float& F
	)
{
	switch (scene->materials[isect.material_id].bxdf)
	{
	case kDiffuse:
		return SameHemisphere(wo, wi) ? AbsCosTheta(wi) * PB_INV_PI : 0;
	case kPlastic:
		{
			if (specular) {
				float alpha_x = scene->materials[
							isect.material_id].value;
				float alpha_y = alpha_x;
				return MicrofacetReflectionPdf(
						wo, wi, alpha_x, alpha_y) * F;
			}
			return SameHemisphere(wo, wi) ?
					AbsCosTheta(wi) * PB_INV_PI * (1 - F) : 0;
		}
	case kMetal:
		{
			float alpha_x = scene->materials[
						isect.material_id].value;
			float alpha_y = alpha_x;
        		return MicrofacetReflectionPdf(
						wo, wi, alpha_x, alpha_y);
		}
	case kGlass:
		{
			float alpha_x = scene->materials[
						isect.material_id].value;
			float alpha_y = alpha_x;
			if (specular) {
				return MicrofacetReflectionPdf(
						wo, wi, alpha_x, alpha_y) * F;
			}
			float etaA = GLASS_ETA_A;
			float etaB = GLASS_ETA_B;
			return MicrofacetTransmissionPdf(wo, wi, etaA, etaB,
						alpha_x, alpha_y) * (1 - F);
		}
	case kLight:
		return SameHemisphere(wo, wi) ? AbsCosTheta(wi) * PB_INV_PI : 0;
	}
	return 0;
}


FW_CUDA_FUNC FW::Vec3f bxdfF(
			const FW::Vec3f& wo,
			const FW::Vec3f& wi,
			const Intersection& isect,
			SceneData* scene,
			const bool& specular,
			const float& F
		)
{
	Material m = scene->materials[isect.material_id];
	FW::Vec3f c = m.color;
	if (m.texture_id > -1) {
		float4 tc = tex2D<float4>(
			scene->textures[m.texture_id],
			isect.uv[0],
			isect.uv[1]
		);
		c = FW::Vec3f(tc.x, tc.y, tc.z);
	}
	switch (m.bxdf)
	{
	case kDiffuse:
		return c * PB_INV_PI;
	case kPlastic:
		{
			if (specular) {
				float alpha_x = m.value;
				float alpha_y = alpha_x;
				FW::Vec3f etaB(1);
				FW::Vec3f etaA = FW::Vec3f(1);
				FW::Vec3f k = FW::Vec3f(1);
				FW::Vec3f R(1);

				return MicrofacetReflectionF(
						wo, wi, etaA, etaB,
						R, k, alpha_x, alpha_y) * F;
			} else {
				return c * PB_INV_PI * (1 - F);
			}
		}
	case kMetal:
		{
			float alpha_x = m.value;
			float alpha_y = alpha_x;

			FW::Vec3f etaB(1);
			FW::Vec3f etaA = c;
			FW::Vec3f k = m.kappa;
			FW::Vec3f R(1);

			return MicrofacetReflectionF(wo, wi, etaA, etaB,
							R, k, alpha_x, alpha_y);
		}
	case kGlass:
		{
			float alpha_x = m.value;
			float alpha_y = alpha_x;

			float etaA = GLASS_ETA_A;
			float etaB = GLASS_ETA_B;

			FW::Vec3f T = c;

			if (specular) {
				FW::Vec3f etaB(1);
				FW::Vec3f etaA = FW::Vec3f(1);
				FW::Vec3f k = FW::Vec3f(1);
				FW::Vec3f R(1);

				return MicrofacetReflectionF(wo, wi, etaA, etaB,
						R, k, alpha_x, alpha_y
					) * F;
			} else {
				return MicrofacetTransmissionF(
						wo, wi, etaA, etaB, T,
						alpha_x, alpha_y
					) * (1 - F);
			}
		}
	case kLight:
		return c * m.value;
	}
	return FW::Vec3f(0);
}




} // namespace rt_pathtracer

#endif // RTPT_INTEGRATORS_BXDF_H_
