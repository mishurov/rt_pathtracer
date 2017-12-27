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

#include "geometry/rib_primitives.h"
#include "geometry/triangulation.h"
#include "geometry/quadrics_tesselator.h"
#include "rib_lexer_parser/utils/primitives.h"

namespace rt_pathtracer {

TriangleData GetAttribs(FW::Vec3i attr_indices, FW::Vec3i tri_indices,
		FW::Array<FW::Vec3f>* scene_vtx_positions,
		const std::map<std::string,std::vector<float>>& params,
		TransformMatrix4f* basis)
{
	TriangleData triangle_data;
	if (params.find("facevarying normal N") != params.end()) {
		const std::vector<float>* norms = &(
			params.at("facevarying normal N")
		);
		for (int i = 0; i < 3; i++) {
			FW::Vec3f normal;
			for (int j = 0; j < 3; j++) {
				normal[j] = norms->at(attr_indices[i] * 3 + j);
			}
			triangle_data.normals[i] = basis->transformV(normal);
			//triangle_data.normals[i] = normal;
		}
	} else {
		FW::Vec3f tri_points[3];
		for (int i = 0; i < 3; i++) {
			tri_points[i] = scene_vtx_positions->get(tri_indices[i]);
		}
		FW::Vec3f normal = FW::cross(
			tri_points[1] - tri_points[0],
			tri_points[1] - tri_points[2]
		);
		for (int i = 0; i < 3; i++) {
			triangle_data.normals[i] = normal;
		}
	}

	if (params.find("facevarying float s") != params.end() &&
	    params.find("facevarying float t") != params.end()) {
		const std::vector<float>* s_coords =
					&(params.at("facevarying float s"));
		const std::vector<float>* t_coords =
					&(params.at("facevarying float t"));
		for (int i = 0; i < 3; i++) {
			FW::Vec2f st;
			st[0] = s_coords->at(attr_indices[i]);
			st[1] = t_coords->at(attr_indices[i]);
			triangle_data.sts[i] = st;
		}
	} else {
		for (int i = 0; i < 3; i++) {
			triangle_data.sts[i] = FW::Vec2f(0, 0);
		}
	}

	return triangle_data;
}


void AddPolygons(RibScene* scene,
	const std::vector<int>& nloops,
	const std::vector<int>& nvertices,
	const std::vector<int>& vertices,
	const std::map<std::string,std::vector<float>>& params)
{
	if (params.find("P") == params.end())
		return;

	FW::Array<FW::Vec3f>* scene_vtx_positions = scene->vtx_positions_ptr();
	FW::Array<unsigned int>* scene_tri_indices = scene->tri_indices_ptr();
	FW::Array<TriangleData>* scene_tri_data = scene->triangle_data_ptr();

	int material_id = scene->current_material_id();
	TransformMatrix4f* basis = scene->basis_ptr();
	FW::Vec3f* bounds = scene->bounds();

	int offset = scene_vtx_positions->getSize();

	for (int i = 0; i < params.at("P").size() / 3; i++) {
		FW::Vec3f vertex;
		for (int j = 0; j < 3; j++) {
			vertex[j] = params.at("P")[i * 3 + j];
		}
		vertex = basis->transformP(vertex);
		bounds[0] = FW::max(bounds[0], vertex);
		bounds[1] = FW::min(bounds[1], vertex);
		scene_vtx_positions->add(vertex);
	}


	int polygon_offset = 0;
	for (int i = 0; i < nloops.size() ; i++) {
		geo<FW::Vec3f>::Polygon polygon;
		int current_offset = polygon_offset;
		std::vector<int> vertex_indices;
		for (int j = 0; j < nloops[i]; j++) {
			geo<FW::Vec3f>::Loop loop;
			for(int k = 0; k < nvertices[i + j]; k++) {
				int index = vertices[polygon_offset + k];
				loop.push_back(
					scene_vtx_positions->get(offset + index)
				);
				vertex_indices.push_back(index);
			}
			polygon.push_back(loop);
			polygon_offset += nvertices[i + j];
		}

		std::vector<int> triangle_indices;
		Triangulate<FW::Vec3f>(polygon, triangle_indices);

		for (int j = 0; j < triangle_indices.size() / 3; j++) {
			FW::Vec3i attr_indices;
			FW::Vec3i tri_indices;
			for (int k = 0; k < 3; k++) {
				int index = triangle_indices[j * 3 + k];
				tri_indices[k] = offset + vertex_indices[index];
				attr_indices[k] = current_offset + index;
				scene_tri_indices->add(tri_indices[k]);
			}

			TriangleData triangle_data = GetAttribs(
				attr_indices, tri_indices,
				scene_vtx_positions, params,
				basis
			);
			triangle_data.material_id = material_id;

			scene_tri_data->add(triangle_data);
		}
	}
}


void SpherePoints(RibScene* scene,
	float radius, float zmin, float zmax, float thetamax)
{
	float args[] = { radius, zmin, zmax, thetamax };
	scene->tesselator()->tessellate(quadrics::SpherePoint<FW::Vec3f>, args);
	scene->tesselator()->addToScene(scene);
}

void ConePoints(RibScene* scene,
	float height, float radius, float thetamax)
{
	float args[] = { height, radius, thetamax };
	scene->tesselator()->tessellate(quadrics::ConePoint<FW::Vec3f>, args);
	scene->tesselator()->addToScene(scene);
}

void CylinderPoints(RibScene* scene,
	float radius, float zmin, float zmax, float thetamax)
{
	float args[] = { radius, zmin, zmax, thetamax };
	scene->tesselator()->tessellate(quadrics::CylinderPoint<FW::Vec3f>, args);
	scene->tesselator()->addToScene(scene);
}

void HyperboloidPoints(RibScene* scene,
	float x1, float y1, float z1,
	float x2, float y2, float z2,
	float thetamax)
{
	float args[] = { x1, y1, z1, x2, y2, z2, thetamax };
	scene->tesselator()->tessellate(quadrics::HyperboloidPoint<FW::Vec3f>, args);
	scene->tesselator()->addToScene(scene);
}

void ParaboloidPoints(RibScene* scene,
	float rmax, float zmin, float zmax, float thetamax)
{
	float args[] = { rmax, zmin, zmax, thetamax };
	scene->tesselator()->tessellate(quadrics::ParaboloidPoint<FW::Vec3f>, args);
	scene->tesselator()->addToScene(scene);
}

void DiskPoints(RibScene* scene,
	float height, float radius, float thetamax)
{
	float args[] = { height, radius, thetamax };
	scene->tesselator()->tessellate(quadrics::DiskPoint<FW::Vec3f>, args);
	scene->tesselator()->addToScene(scene);
}

void TorusPoints(RibScene* scene,
	float rmajor, float rminor,
	float phimin, float phimax, float thetamax)
{
	float args[] = { rmajor, rminor, phimin, phimax, thetamax };
	scene->tesselator()->tessellate(quadrics::TorusPoint<FW::Vec3f>, args);
	scene->tesselator()->addToScene(scene);
}


/*
// the previous way to tesselate the quadrics, isn't in use now
void PopulateArrays(RibScene* scene,
			FW::Vec3f (*f)(float, float, float *), float *args)
{
	// compute optimal sampling
	FW::Vec3f mid = f(0.5, 0.5, args);
	FW::Vec3f du1 = f(0.4, 0.5, args);
	FW::Vec3f dv1 = f(0.5, 0.4, args);
	FW::Vec3f du2 = f(0.6, 0.5, args);
	FW::Vec3f dv2 = f(0.5, 0.6, args);
	float nomu = (du1 - mid).length();
	float nomv = (dv1 - mid).length();
	float denomu = (du1 - du2).length();
	float denomv = (dv1 - dv2).length();
	float ratio = nomu / nomv;
	int numu = ceil(nomu / denomu * ratio * 10);
	int numv = ceil(nomv / denomv * 10);

	TransformMatrix4f* basis = scene->getBasisPtr();
	FW::Vec3f origin = basis->transform(FW::Vec3f(0, 0, 0));
	FW::Array<FW::Vec3f>* vtx_positions = scene->getVtxPositionsPtr();
	FW::Array<unsigned int>* tri_indices = scene->getTriIndicesPtr();

	int offset = vtx_positions->getSize();

	float step_u = 1.0 / ((float)numu - 1);
	float step_v = 1.0 / ((float)numv - 1);
	for (int i = 0; i < numu; i++) {
		for (int j = 0; j < numv; j++) {
			float u = step_u * i;
			float v = step_v * j;
			u = FW::clamp(u, 0, 1);
			v = FW::clamp(v, 0, 1);
			FW::Vec3f vtx = f(u, v, args);
			vtx = basis->transform(vtx);
			vtx_positions->add(vtx);
		}
	}
	
	for (int i = 1; i < numu; i++) {
		for (int j = 0; j < numv - 1; j++) {
			int t1 = i * numv + j;
			int t2 = i * numv + j + 1;
			int t3 = (i - 1) * numv + j;
			int t4 = (i - 1) * numv + j + 1;

			tri_indices->add(t1 + offset);
			tri_indices->add(t2 + offset);
			tri_indices->add(t3 + offset);

			tri_indices->add(t4 + offset);
			tri_indices->add(t2 + offset);
			tri_indices->add(t3 + offset);
		}
	}
}
*/

} // namespace rt_pathtracer
