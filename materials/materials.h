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


#ifndef RTPT_MATERIALS_H_
#define RTPT_MATERIALS_H_

#include <nvidia_base/Math.hpp>

namespace rt_pathtracer {

enum Bxdf
{
	kDiffuse = 0,
	/*
	Bxdf "PxrLMDiffuse" "matte"
		"color frontColor" [0.178867 0.178867 0.178867]
	*/
	kPlastic,
	/*
	Bxdf "PxrLMPlastic" "smooth"
		"color diffuseColor" [0.178867 0.178867 0.178867]
		"float diffuseRoughness" [0.1]
	*/
	kMetal,
	/*
	Bxdf "PxrLMMetal" "smooth"
		"color eta" [4.369684 2.916713 1.654698]
		"color kappa" [5.20643 4.231366 3.754969]
		"float roughness" [0.1]
	*/
	kGlass,
	/*
	Bxdf "PxrLMGlass" "glass"
		"color refractionColor" [0.178867 0.178867 0.178867]
		"float roughness" [0.1]
	*/
	kLight,
	/*
	Light "PxrMeshLight" "glow"
		"color lightColor" [2 0.1 0.1]
		"float intensity" [22.5]
	*/
};

/*
	Pattern "PxrTexture" "wood"
		"string filename" ["wood.tga"] (tga, jpg)
	Bxdf PxrLMDiffuse" "matte"
		"reference color baseColor" ["wood:resultRGB"]
*/

/*
	Light "PxrDomeLight" "Light0"
		"string lightColorMap" ["Luxo-Jr_4000x2000.hdr"] (hdr, jpg, tga)
		"float intensity" [1]
		"color lightColor" [1 1 1]
*/

struct Material
{
	Bxdf bxdf;
	FW::Vec3f color;
	FW::Vec3f kappa;
	float value; // roughness for a shader or intensity for a light
	int texture_id;
};


struct TriangleData
{
	int material_id;
	FW::Vec2f sts[3];
	FW::Vec3f normals[3];
};


struct Intersection {
	FW::Vec3f p;
	FW::Vec3f n;
	FW::Vec3f du;
	FW::Vec3f dv;
	FW::Vec2f uv;
	int material_id;
	float t;
};


} // namespace rt_pathtracer

#endif // RTPT_MATERIALS_H_
