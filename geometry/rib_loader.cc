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

#include "geometry/rib_loader.h"
#include <cuda_gl_interop.h>
#include "geometry/rib_primitives.h"
#include "materials/texture_loader.h"


namespace rt_pathtracer {

RibScene::RibScene()
{
	Material default_material {
		.bxdf = kDiffuse,
		.color = FW::Vec3f(1, 1, 1),
		.kappa = FW::Vec3f(1, 1, 1),
		.value = 0.0,
		.texture_id = -1
	};
	materials_.reset();
	materials_.add(default_material);
	current_material_id_ = 0;
	current_texture_id_ = -1;
	env_texture_ = "";
	env_color_ = FW::Vec3f(0, 0, 0);
	env_intensity_ = -1;
	env_texture_size_ = FW::Vec2i(0, 0);

}

bool RibScene::loadRib(const char *file_name)
{
	file_name_ = file_name;
	size_t last_of = file_name_.find_last_of("/\\");
	dir_path_ = file_name_.substr(0, last_of);
	std::string base_name = file_name_.substr(
		last_of + 1, file_name_.length()
	);
	rib_name_ = base_name.substr(0, base_name.find_last_of("."));


	basis_.setIdentity();
	transform_stack_.reset();
	root_ = driver_.parse(file_name);
	printf("Processing geometry...\n");
	dfs(&root_);

	float comfortable = 5;

	FW::Vec3f diag = bounds_[0] - bounds_[1];
	FW::Vec3f mid = bounds_[1] + (bounds_[0] - bounds_[1]) / 2;
	float ratio = comfortable / diag.length();

	for (unsigned int i = 0; i < vtx_positions_.getSize(); i++) {
		vtx_positions_[i] -= mid;
		vtx_positions_[i] *= ratio;
	}

	for (unsigned int i = 0; i < quadrics_.getSize(); i++) {
		TransformMatrix4f t;
		t.setIdentity();
		t.addTranslation(
			-mid.x * ratio,
			-mid.y * ratio,
			-mid.z * ratio
		);
		t.addScale(ratio, ratio, ratio);
		t *= quadrics_[i].basis;
		quadrics_[i].basis = t;
		quadrics_[i].computeBounds();
	}

	for (unsigned int i = 0; i < 2; i++) {
		bounds_[i] -= mid;
		bounds_[i] *= ratio;
	}

	for (unsigned int i = 0; i < triangle_data_.getSize(); i++) {
		if (materials_[triangle_data_[i].material_id].bxdf == kLight) {
			light_indices_.add(i);
			for (int j = 0; j < 3; j++) {
				light_vertices_.add(
					vtx_positions_[tri_indices_[i + j]]
				);
			}
		}
	}

	return true;
}


void RibScene::loadEnvTexture()
{
	if (!env_texture_.compare(""))
		return;
	
	printf("Loading environment texture...\n");
	std::string ext = env_texture_.substr(
		env_texture_.find_last_of(".") + 1
	);
	std::string path = dir_path_ + "/" + env_texture_;
	int width;
	int height;
	float *pixels_float = NULL;
	unsigned char *pixels_bytes = NULL;
	FW::Array<FW::Vec4f> cpu_tex;

	bool is_hdr = ext.compare("hdr") == 0;
	bool is_jpg = ext.compare("jpg") == 0 || ext.compare("jpeg") == 0;
	bool is_tga = ext.compare("tga") == 0;

	if (is_hdr) {
		pixels_float = TextureLoader::readHdr(
			path.c_str(), &width, &height
		);
	} else if (is_jpg) {
		pixels_bytes = TextureLoader::readJpg(
			path.c_str(), &width, &height
		);
	} else if (is_tga) {
		pixels_bytes = TextureLoader::readTga(
			path.c_str(), &width, &height
		);
	} else {
		printf(
			"Environmnent texture extension is not supported: %s\n",
			env_texture_.c_str()
		);
	}

	if (pixels_float == NULL && pixels_bytes == NULL) {
		printf("Failed to load texture: %s\n", env_texture_.c_str());
	}

	cpu_tex.resize(width * height);

	if (is_hdr) {
		for (int i = 0; i < width * height; i++) {
			cpu_tex[i] = FW::Vec4f(
				pixels_float[3 * i],
				pixels_float[3 * i + 1],
				pixels_float[3 * i + 2],
				0.0f
			);
		}
		delete pixels_float;
	} else if (is_jpg) {
		for (int i = 0; i < width * height; i++) {
			cpu_tex[i] = FW::Vec4f(
				(float)pixels_bytes[3 * i] / 255.0,
				(float)pixels_bytes[3 * i + 1] / 255.0,
				(float)pixels_bytes[3 * i + 2] / 255.0,
				0.0f
			);
		}
		free(pixels_bytes);
	} else if (is_tga) {
		for (int i = 0; i < width * height; i++) {
			cpu_tex[i] = FW::Vec4f(
				(float)pixels_bytes[3 * i + 2] / 255.0,
				(float)pixels_bytes[3 * i + 1] / 255.0,
				(float)pixels_bytes[3 * i] / 255.0,
				0.0f
			);
		}
		free(pixels_bytes);
	}

	cudaMalloc(&tex_resources_.env_texture,
			width * height * sizeof(float4));
	cudaMemcpy(tex_resources_.env_texture, cpu_tex.getPtr(),
		width * height * sizeof(float4),
		cudaMemcpyHostToDevice);
	env_texture_size_ = FW::Vec2i(width, height);

	cudaTextureDesc td;
	memset(&td, 0, sizeof(td));

	td.addressMode[0] = cudaAddressModeClamp;
	td.readMode = cudaReadModeElementType;
	td.filterMode = cudaFilterModePoint;
	td.normalizedCoords = false;

	cudaResourceDesc rd;
	memset(&rd, 0, sizeof(rd));
	rd.resType = cudaResourceTypeLinear;
	rd.res.linear.devPtr = (void*)(tex_resources_.env_texture);
	rd.res.linear.sizeInBytes = width * height * sizeof(float4);
	rd.res.linear.desc.f = cudaChannelFormatKindFloat;
	rd.res.linear.desc.x = 32;
	rd.res.linear.desc.y = 32;
	rd.res.linear.desc.z = 32;
	rd.res.linear.desc.w = 32;

	cudaCreateTextureObject(&gpu_env_texture_, &rd, &td, 0);
}


FW::Array<std::string> RibScene::createTextureArray()
{
	FW::Array<std::string> textures;
	textures.resize(textures_.size());

	std::map<std::string,std::pair<int,std::string>>::iterator iter;
	for (iter = textures_.begin(); iter != textures_.end(); iter++) {
		textures[iter->second.first] = iter->second.second;
        }
	return textures;
}


void RibScene::makeTextureObjects(const FW::Array<cudaArray_t>& cuda_arrays)
{
	int len = cuda_arrays.getSize();

	cudaMalloc(
		(void**)(&cuda_textures_),
		sizeof(cudaTextureObject_t) * len
	);

	for (int i = 0; i < len; i++) {
		cudaTextureDesc tex_desc = {};
		tex_desc.addressMode[0] = cudaAddressModeWrap;
		tex_desc.addressMode[1] = cudaAddressModeWrap;
		tex_desc.filterMode = cudaFilterModeLinear;
		tex_desc.readMode = cudaReadModeNormalizedFloat;
		tex_desc.normalizedCoords = true;

		cudaResourceDesc res_desc = {};
		res_desc.resType = cudaResourceTypeArray;
		res_desc.res.array.array = cuda_arrays[i];

		cudaTextureObject_t h_tex_obj;

		cudaCreateTextureObject(&h_tex_obj, &res_desc, &tex_desc, 0);
	
		cudaMemcpy(
			&cuda_textures_[i],
			&h_tex_obj, sizeof(cudaTextureObject_t),
			cudaMemcpyHostToDevice
		);
	}
	cudaDeviceSynchronize();
}


void RibScene::loadTexturesGL()
{
	FW::Array<std::string> textures = createTextureArray();
	for (int i = 0; i < textures.getSize(); i++) {
		std::string path;
		path = dir_path_ + "/" + textures[i];

		GLuint gl_tex = TextureLoader::makeTexture(path);
		cudaGraphicsResource_t cuda_tex = 0;

		cudaGraphicsGLRegisterImage(&cuda_tex,
					gl_tex,
					GL_TEXTURE_2D,
					cudaGraphicsRegisterFlagsReadOnly);
		cudaArray_t gpu_array;
		cudaGraphicsMapResources(1, &cuda_tex, 0);
		cudaGraphicsSubResourceGetMappedArray(&gpu_array, cuda_tex, 0, 0);
		tex_resources_.mapped_res.add(cuda_tex);
		tex_resources_.tex_arrays.add(gpu_array);
		glDeleteTextures(1, &gl_tex);
	}
	makeTextureObjects(tex_resources_.tex_arrays);
}


void RibScene::loadTextures()
{
	FW::Array<std::string> textures = createTextureArray();
	FW::Array<cudaArray_t> cuda_arrays;

	for (int i = 0; i < textures.getSize(); i++) {
		int width, height;
		std::string path;
		path = dir_path_ + "/" + textures[i];
		unsigned char *pixelsBGR = (unsigned char*)TextureLoader::readTga(
			path.c_str(), &width, &height
		);


		int size = width * height * sizeof(unsigned char) * 4;
		unsigned char* pixelsRGBA = (unsigned char*)malloc(size);

		for (int i = 0; i <  width * height; i++) {
			pixelsRGBA[i * 4] = pixelsBGR[3 * i + 2];
			pixelsRGBA[i * 4 + 1] = pixelsBGR[3 * i + 1];
			pixelsRGBA[i * 4 + 2] = pixelsBGR[3 * i];
			pixelsRGBA[i * 4 + 3] = 0;
		}

		cudaArray_t gpu_array;
		cudaChannelFormatDesc chan_desc = cudaCreateChannelDesc<uchar4>();

		cudaMallocArray(&gpu_array, &chan_desc, width, height);
		cudaMemcpyToArray(gpu_array, 0, 0, pixelsRGBA,
					sizeof(uchar4) * width * height,
					cudaMemcpyHostToDevice);
		tex_resources_.tex_arrays.add(gpu_array);
		free(pixelsRGBA);
	}
	makeTextureObjects(tex_resources_.tex_arrays);
}


void RibScene::processNode(rib::Node *node)
{
	switch (node->type) {
	case rib::kJoint:
		break;
	case rib::kAttribute:
		{
			rib::AttributeNode *n = (rib::AttributeNode *) node;
			if (n->name == "tesselation" &&
			    n->float_params.find("int quadrics") !=
			    n->float_params.end()) {
				triangulate_ =
					n->float_params.at(
						"int quadrics"
					).at(0);
			}
		}
		break;
	case rib::kTranslate:
		{
			rib::TranslateNode *n = (rib::TranslateNode *) node;
			basis_.addTranslation(n->x, n->y, n->z);
		}
		break;
	case rib::kRotate:
		{
			rib::RotateNode *n = (rib::RotateNode *) node;
			FW::Vec3f axis(n->x, n->y, n->z);
			basis_.addRotate(axis, n->r);
		}
		break;
	case rib::kScale:
		{
			rib::ScaleNode *n = (rib::ScaleNode *) node;
			basis_.addScale(n->x, n->y, n->z);
		}
		break;
	case rib::kConcatTransform:
		{
			rib::ConcatTransformNode *n =
					(rib::ConcatTransformNode *) node;
			FW::Mat4f transformer;
			transformer.setIdentity();

			transformer.setCol(0,
		FW::Vec4f(n->matrix[0],  n->matrix[1],  n->matrix[2],  n->matrix[3])
			);
			transformer.setCol(1,
		FW::Vec4f(n->matrix[4],  n->matrix[5],  n->matrix[6],  n->matrix[7])
			);
			transformer.setCol(2,
		FW::Vec4f(n->matrix[8],  n->matrix[9],  n->matrix[10], n->matrix[11])
			);
			transformer.setCol(3,
		FW::Vec4f(n->matrix[12], n->matrix[13], n->matrix[14], n->matrix[15])
			);

			basis_ *= transformer;
		}
		break;
	case rib::kSphere:
		{
			rib::SphereNode *n = (rib::SphereNode *) node;
			
			if (triangulate_) {
				SpherePoints(
					this,
					n->radius,
					n->zmin, n->zmax, n->thetamax
				);
			} else {
				Quadric q;
				q.basis = basis_;
				q.material_id = current_material_id_;
				q.type = kqSphere;
				q.params[0] = n->radius;
				q.params[1] = n->zmin;
				q.params[2] = n->zmax;
				q.params[3] = n->thetamax;
				q.computeBounds();
				q.toRadians();
				quadrics_.add(q);
				bounds_[0] = FW::max(bounds_[0], q.b_max);
				bounds_[1] = FW::min(bounds_[1], q.b_min);
			
			}
		}
		break;
	case rib::kCone:
		{
			rib::ConeNode *n = (rib::ConeNode *) node;

			if (triangulate_) {
				ConePoints(
					this,
					n->height, n->radius, n->thetamax
				);
			} else {
				Quadric q;
				q.basis = basis_;
				q.material_id = current_material_id_;
				q.type = kqCone;
				q.params[0] = n->height;
				q.params[1] = n->radius;
				q.params[2] = n->thetamax;
				q.computeBounds();
				q.toRadians();
				quadrics_.add(q);
				bounds_[0] = FW::max(bounds_[0], q.b_max);
				bounds_[1] = FW::min(bounds_[1], q.b_min);
			}
		}
		break;
	case rib::kCylinder:
		{
			rib::CylinderNode *n = (rib::CylinderNode *) node;

			if (triangulate_) {
				CylinderPoints(
					this,
					n->radius, n->zmin, n->zmax,
					n->thetamax
				);
			} else {
				Quadric q;
				q.basis = basis_;
				q.material_id = current_material_id_;
				q.type = kqCylinder;
				q.params[0] = n->radius;
				q.params[1] = n->zmin;
				q.params[2] = n->zmax;
				q.params[3] = n->thetamax;
				q.computeBounds();
				q.toRadians();
				quadrics_.add(q);
				bounds_[0] = FW::max(bounds_[0], q.b_max);
				bounds_[1] = FW::min(bounds_[1], q.b_min);
			}
		}
		break;
	case rib::kHyperboloid:
		{
			rib::HyperboloidNode *n = (rib::HyperboloidNode *) node;

			if (triangulate_) {
				HyperboloidPoints(
					this,
					n->x1, n->y1, n->z1,
					n->x2, n->y2, n->z2,
					n->thetamax
				);
			} else {
				Quadric q;
				q.basis = basis_;
				q.material_id = current_material_id_;
				q.type = kqHyperboloid;
				q.params[0] = n->x1;
				q.params[1] = n->y1;
				q.params[2] = n->z1;
				q.params[3] = n->x2;
				q.params[4] = n->y2;
				q.params[5] = n->z2;
				q.params[6] = n->thetamax;
				q.computeBounds();
				q.toRadians();
				quadrics_.add(q);
				bounds_[0] = FW::max(bounds_[0], q.b_max);
				bounds_[1] = FW::min(bounds_[1], q.b_min);
			}
		}
		break;
	case rib::kParaboloid:
		{
			rib::ParaboloidNode *n = (rib::ParaboloidNode *) node;

			if (triangulate_) {
				ParaboloidPoints(
					this,
					n->rmax, n->zmin, n->zmax, n->thetamax
				);
			} else {
				Quadric q;
				q.basis = basis_;
				q.material_id = current_material_id_;
				q.type = kqParaboloid;
				q.params[0] = n->rmax;
				q.params[1] = n->zmin;
				q.params[2] = n->zmax;
				q.params[3] = n->thetamax;
				q.computeBounds();
				q.toRadians();
				quadrics_.add(q);
				bounds_[0] = FW::max(bounds_[0], q.b_max);
				bounds_[1] = FW::min(bounds_[1], q.b_min);
			}
		}
		break;
	case rib::kDisk:
		{
			rib::DiskNode *n = (rib::DiskNode *) node;

			if (triangulate_) {
				DiskPoints(
					this,
					n->height, n->radius, n->thetamax
				);
			} else {
				Quadric q;
				q.basis = basis_;
				q.material_id = current_material_id_;
				q.type = kqDisk;
				q.params[0] = n->height;
				q.params[1] = n->radius;
				q.params[2] = n->thetamax;
				q.computeBounds();
				q.toRadians();
				quadrics_.add(q);
				bounds_[0] = FW::max(bounds_[0], q.b_max);
				bounds_[1] = FW::min(bounds_[1], q.b_min);
			}
		}
		break;
	case rib::kTorus:
		{
			rib::TorusNode *n = (rib::TorusNode *) node;

			if (triangulate_) {
				TorusPoints(
					this,
					n->rmajor, n->rminor,
					n->phimin, n->phimax, n->thetamax
				);
			} else {
				Quadric q;
				q.basis = basis_;
				q.material_id = current_material_id_;
				q.type = kqTorus;
				q.params[0] = n->rmajor;
				q.params[1] = n->rminor;
				q.params[2] = n->phimin;
				q.params[3] = n->phimax;
				q.params[4] = n->thetamax;
				q.computeBounds();
				q.toRadians();
				quadrics_.add(q);
				bounds_[0] = FW::max(bounds_[0], q.b_max);
				bounds_[1] = FW::min(bounds_[1], q.b_min);
			}
			
		}
		break;
	case rib::kPointsGeneralPolygons:
		{
			rib::PointsGeneralPolygonsNode *n =
				(rib::PointsGeneralPolygonsNode *) node;
			AddPolygons(this,
				n->nloops,
				n->nvertices,
				n->vertices,
				n->params
			);
		}
		break;
	case rib::kPointsPolygons:
		{
			rib::PointsPolygonsNode *n =
				(rib::PointsPolygonsNode *) node;

			std::vector<int> nloops(n->nvertices.size(), 1);
			AddPolygons(this,
				nloops,
				n->nvertices,
				n->vertices,
				n->params
			);
		}
		break;
	case rib::kLight:
		{
			rib::LightNode *n = (rib::LightNode *) node;
			if (n->item_type == "PxrDomeLight") {
			
				if (n->string_params.find("string lightColorMap") !=
			            n->string_params.end()) {
					env_texture_ =
						n->string_params.at(
							"string lightColorMap"
						).at(0);
				}
				addColorParam(
					"lightColor",
					n->string_params,
					n->float_params,
					&(env_color_),
					false
				);
				if (n->float_params.find("float intensity") !=
				    n->float_params.end()) {
					env_intensity_ =
						n->float_params.at(
							"float intensity"
						).at(0);
				}

			} else if (n->item_type == "PxrMeshLight") {
				Material material = materials_.getFirst();
				material.bxdf = kLight;
				int td = addColorParam(
					"lightColor",
					n->string_params,
					n->float_params,
					&(material.color),
					true
				);
				if (n->float_params.find("float intensity") !=
				    n->float_params.end()) {
					material.value =
						n->float_params.at(
							"float intensity"
						).at(0);
				}
				material.texture_id = td;
				materials_.add(material);
				current_material_id_++;
			}
			
		}
		break;
	case rib::kPattern:
		{
			rib::PatternNode *n = (rib::PatternNode *) node;

			if (n->item_type == "PxrTexture" &&
			    n->string_params.find("string filename") !=
			    n->string_params.end() &&
			    textures_.find(n->name) == textures_.end()) {
				std::string filename =
					n->string_params.at(
						"string filename"
					).at(0);

				current_texture_id_++;
				textures_.insert(
					{n->name, {current_texture_id_, filename}}
				);
			}
		}
		break;
	case rib::kBxdf:
		{
			rib::BxdfNode *n = (rib::BxdfNode *) node;
			addBxdf(n);
		}
		break;
	}
}

int RibScene::addColorParam(const std::string& name,
	const std::map<std::string,std::vector<std::string>>& s,
	const std::map<std::string,std::vector<float>>& f,
	FW::Vec3f* param, bool check_texture)
{
	int texture_id = -1;

	if (check_texture && s.find("reference color " + name) != s.end()) {
		std::string texture_name = s.at("reference color " + name).at(0);
		std::string postfix = ":resultRGB";
		texture_name = texture_name.substr(
			0, texture_name.size() - postfix.size()
		);
		if (textures_.find(texture_name) != textures_.end()) {
			texture_id = textures_.at(texture_name).first;
		}
	}

	if (f.find("color " + name) != f.end()) {
		for (int i = 0; i < 3; i++) {
			param->set(i, f.at("color " + name).at(i));
		}
	}

	return texture_id;
}


void RibScene::addBxdf(rib::BxdfNode *n) {

	Material material = materials_.getFirst();
	std::string t = n->item_type;
	std::map<std::string,std::vector<std::string>> s = n->string_params;
	std::map<std::string,std::vector<float>> f = n->float_params;
	int td;

	if (t == "PxrLMDiffuse") {
		material.bxdf = kDiffuse;
		td = addColorParam("frontColor", s, f, &(material.color), true);
		material.texture_id = td;

	} else if (t == "PxrLMPlastic") {
		material.bxdf = kPlastic;
		td = addColorParam("diffuseColor", s, f, &(material.color), true);
		if (f.find("float diffuseRoughness") != f.end())
			material.value = f.at("float diffuseRoughness").at(0);
		material.texture_id = td;

	} else if (t == "PxrLMMetal") {
		material.bxdf = kMetal;
		addColorParam("eta", s, f, &(material.color), false);
		addColorParam("kappa", s, f, &(material.kappa), false);
		if (f.find("float roughness") != f.end())
			material.value = f.at("float roughness").at(0);

	} else if (t == "PxrLMGlass") {
		material.bxdf = kGlass;
		addColorParam("refractionColor", s, f, &(material.color), false);
		if (f.find("float roughness") != f.end())
			material.value = f.at("float roughness").at(0);

	}
	
	materials_.add(material);
	current_material_id_++;
}



void RibScene::dfs(rib::Node *node)
{
	processNode(node);
	for(std::vector<rib::Node *>::iterator it =
	    node->children.begin();
	    it != node->children.end(); ++it) {
		if (it == node->children.begin()) {
			TransformMatrix4f t = basis_;
			transform_stack_.add(t);
		}
		dfs(*it);
		if(std::next(it) == node->children.end()) {
			basis_ = transform_stack_.removeLast();
			triangulate_ = false;
		}
	}
}


} // namespace rt_pathtracer
