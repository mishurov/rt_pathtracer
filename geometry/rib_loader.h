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

#ifndef RTPT_RIB_LOADER_H_
#define RTPT_RIB_LOADER_H_

#include "nvidia_base/Array.hpp"
#include "rib_lexer_parser/parser/rib_driver.h"
#include "geometry/transforms.h"
#include "geometry/quadrics_tesselator.h"
#include "geometry/quadrics.h"


namespace rt_pathtracer {

struct TexResources {
	float4* env_texture;
	FW::Array<cudaArray_t> tex_arrays;
	FW::Array<cudaGraphicsResource_t> mapped_res;
};

class RibScene {
public:
	RibScene();

	bool loadRib(const char *file_name);
	void loadTexturesGL();
	void loadTextures();
	void loadEnvTexture();

	FW::Array<FW::Vec3f> vtx_positions() { return vtx_positions_; }
	FW::Array<FW::Vec3f>* vtx_positions_ptr() { return &vtx_positions_; }

	FW::Array<unsigned int> tri_indices() { return tri_indices_; }
	FW::Array<unsigned int>* tri_indices_ptr() { return &tri_indices_; }

	FW::Array<unsigned int>* light_indices_ptr() { return &light_indices_; }
	FW::Array<FW::Vec3f>* light_vertices_ptr() { return &light_vertices_; }

	FW::Array<TriangleData>* triangle_data_ptr() { return &triangle_data_; }
	FW::Array<Material>* materials_ptr() { return &materials_; }

	FW::Array<Quadric>* quadrics_ptr() { return &quadrics_; }

	QuadricsTessellator* tesselator() { return &tesselator_; }
	FW::Vec3f* bounds() { return bounds_; }
	int current_material_id() { return current_material_id_; }
	TransformMatrix4f* basis_ptr() { return &basis_; }

	cudaTextureObject_t* cuda_textures() { return cuda_textures_; }

	FW::Vec3f env_color() { return env_color_; }
	float env_intensity() { return env_intensity_; }
	cudaTextureObject_t env_texture() { return gpu_env_texture_; }
	FW::Vec2i env_texture_size() { return env_texture_size_; }

	TexResources tex_resources() { return tex_resources_; }

	std::string dir_path() { return dir_path_; }
	std::string rib_name() { return rib_name_; }
private:
	std::string file_name_;
	std::string dir_path_;
	std::string rib_name_;

	FW::Array<FW::Vec3f> vtx_positions_;
	FW::Array<unsigned int> tri_indices_;
	FW::Array<TriangleData> triangle_data_;
	FW::Array<Quadric> quadrics_;

	FW::Array<unsigned int> light_indices_;
	FW::Array<FW::Vec3f> light_vertices_;

	QuadricsTessellator tesselator_;

	rib::Driver driver_;
	rib::Node root_;

	TransformMatrix4f basis_;
	FW::Array<TransformMatrix4f> transform_stack_;
	bool triangulate_;
	FW::Vec3f bounds_[2];

	Material default_meterial_;
	FW::Array<Material> materials_;
	int current_material_id_;

	std::map<std::string,std::pair<int,std::string>> textures_;
	int current_texture_id_;

	std::string env_texture_;
	cudaTextureObject_t gpu_env_texture_;
	FW::Vec2i env_texture_size_;
	FW::Vec3f env_color_;
	float env_intensity_;

	cudaTextureObject_t* cuda_textures_;
	
	TexResources tex_resources_;

private:
	void processNode(rib::Node *node);
	void dfs(rib::Node *node);
	void addBxdf(rib::BxdfNode *node);
	int addColorParam(const std::string& name,
		const std::map<std::string,std::vector<std::string>>& string_params,
		const std::map<std::string,std::vector<float>>& float_params,
		FW::Vec3f* param, bool check_texture);
	FW::Array<std::string> createTextureArray();
	void makeTextureObjects(const FW::Array<cudaArray_t>& cuda_arrays);
};


} // namespace rt_pathtracer

#endif // RTPT_RIB_LOADER_H_
