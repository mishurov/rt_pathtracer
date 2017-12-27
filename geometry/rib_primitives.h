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


#ifndef RTPT_RIB_PRIMITIVES_H_
#define RTPT_RIB_PRIMITIVES_H_

#include <cstdlib>
#include "nvidia_base/Array.hpp"
#include "geometry/rib_loader.h"

namespace rt_pathtracer {

void AddPolygons(RibScene* scene,
	const std::vector<int>& nloops,
	const std::vector<int>& nvertices,
	const std::vector<int>& vertices,
	const std::map<std::string,std::vector<float>>& params);

void SpherePoints(RibScene* scene,
	float radius, float zmin, float zmax, float thetamax
);

void ConePoints(
	RibScene* scene,
	float height, float radius, float thetamax
);

void CylinderPoints(
	RibScene* scene,
	float radius, float zmin, float zmax, float thetamax
);

void HyperboloidPoints(
	RibScene* scene,
	float x1, float y1, float z1,
	float x2, float y2, float z2,
	float thetamax
);

void ParaboloidPoints(
	RibScene* scene,
	float rmax, float zmin, float zmax, float thetamax
);

void DiskPoints(
	RibScene* scene,
	float height, float radius, float thetamax
);

void TorusPoints(
	RibScene* scene,
	float rmajor, float rminor,
	float phimin, float phimax, float thetamax
);

} // namespace rt_pathtracer

#endif // RTPT_RIB_PRIMITIVES_H_
