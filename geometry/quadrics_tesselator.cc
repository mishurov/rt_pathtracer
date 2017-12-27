#include "geometry/quadrics_tesselator.h"
#include "geometry/rib_loader.h"


namespace rt_pathtracer {


QuadricsTessellator::QuadricsTessellator() {
	lod_min_ = 3;
	lod_max_ = 7;
	geometry_threshold_ = 0.1;
}

QuadricsTessellator::~QuadricsTessellator() {}


void QuadricsTessellator::clear()
{
    vertices_.clear();
    edges_.clear();
    faces_.clear();
}

bool QuadricsTessellator::tessellate(FW::Vec3f (*f)(float, float, float *), float *args)
{
	clear();

	if (!initialize(f, args))
		return false;

	float dot_threshold = 0.5f / sqrtf(0.25f + geometry_threshold_ * geometry_threshold_);

	int remaining = edges_.getSize();

	for (int lod = 0; lod < lod_max_ - 1 && remaining > 0; lod++) {
		int count = edges_.getSize();

		for (int e0 = 0; e0 < count; e0++) {
			if ( edges_[e0].lod != lod )
				continue;

			remaining--;

			int v0 = edges_[e0].v[0];
			int v1 = edges_[e0].v[1];

			Vertex vertex;
			if (!calculateVertex(f, args, &vertex,
					0.5f * (vertices_[v0].param + vertices_[v1].param)))
				return false;

			if (lod < lod_min_ ||
				FW::dot(vertex.normal, vertices_[v0].normal) < dot_threshold ||
				FW::dot(vertex.normal, vertices_[v1].normal) < dot_threshold ||
				delta(vertices_[v0].pos, vertices_[v1].pos, vertex.pos) > geometry_threshold_) {

				int v2 = vertices_.getSize();
				vertices_.add(vertex);

				Edge edge;
				edge.lod = lod + 2;
				edge.v[0] = v2;
				edge.v[1] = v1;
				edge.f[0] = edges_[e0].f[0];
				edge.f[1] = edges_[e0].f[1];

				int e1 = edges_.getSize();
				edges_.add(edge);

				edges_[e0].lod = lod + 2;
				edges_[e0].v[1] = v2;

				remaining++;

				// process adjacent faces
				for (int i = 0; i < 2; i++) {
					int f0 = edges_[e0].f[i];

					if (f0 < 0)
						continue;

					// find the index of the common edge
					int j0 = -1;
					for (int j = 0; j < 3; j++) {
						if (faces_[f0].e[j] == e0) {
							j0 = j;
							break;
						}
					}

					//Q_ASSERT( j0 >= 0 );

					// next and previous edge in CW direction
					int j3 = (j0 + 2 - i) % 3;
					int j4 = (j0 + 1 + i) % 3;

					int e3 = faces_[f0].e[j3];
					int e4 = faces_[f0].e[j4];

					// third vertex of the face
					int v3;
					if (i == 0)
						v3 = edges_[e3].v[faces_[f0].dir[j3]];
					else
						v3 = edges_[e4].v[faces_[f0].dir[j4]];

					// add new edge and adjust two other edges
					int f1 = faces_.getSize();

					edge.lod = lod + 2;
					edge.v[0] = v2;
					edge.v[1] = v3;
					edge.f[i] = f0;
					edge.f[1 - i] = f1;

					int e2 = edges_.getSize();
					edges_.add(edge);

					edges_[e1].f[i] = f1;
					edges_[e4].f[faces_[f0].dir[j4]] = f1;

					remaining++;

					// divide the face
					Face face;
					face.e[j0] = e1;
					face.dir[j0] = i;
					face.e[j3] = e2;
					face.dir[j3] = 1 - i;
					face.e[j4] = e4;
					face.dir[j4] = faces_[f0].dir[j4];

					faces_.add(face);

					faces_[f0].e[j4] = e2;
					faces_[f0].dir[j4] = i;
				}
			}
		}
	}

	return true;
}


bool QuadricsTessellator::initialize(FW::Vec3f (*f)(float, float, float *), float *args)
{
	Vertex v0, v1, v2, v3;

	if (!calculateVertex(f, args, &v0, FW::Vec2f(0, 0)))
		return false;
	if (!calculateVertex(f, args, &v1, FW::Vec2f(1, 0)))
		return false;
	if (!calculateVertex(f, args, &v2, FW::Vec2f(0, 1)))
		return false;
	if (!calculateVertex(f, args, &v3, FW::Vec2f(1, 1)))
		return false;

	vertices_.add(v0);
	vertices_.add(v1);
	vertices_.add(v2);
	vertices_.add(v3);

	Edge e0 = { 1, { 0, 1 }, { 0, -1 } };
	Edge e1 = { 1, { 2, 3 }, { -1, 1 } };
	Edge e2 = { 1, { 0, 2 }, { -1, 1 } };
	Edge e3 = { 1, { 1, 3 }, { 0, -1 } };
	Edge e4 = { 0, { 0, 3 }, { 1, 0 } };

	edges_.add(e0);
	edges_.add(e1);
	edges_.add(e2);
	edges_.add(e3);
	edges_.add(e4);

	Face f0 = { { 0, 3, 4 }, { 0, 0, 1 } };
	Face f1 = { { 4, 1, 2 }, { 0, 1, 1 } };

	faces_.add(f0);
	faces_.add(f1);

	return true;
}


float QuadricsTessellator::distanceToLine(const FW::Vec3f& origin,
			const FW::Vec3f& point, const FW::Vec3f& direction)
{
	if (direction[0] == 0 && direction[1] == 0 && direction[2] == 0)
		return (origin - point).length();
	FW::Vec3f p = point + FW::dot(origin - point, direction) * direction;
	return (origin - p).length();
}


float QuadricsTessellator::delta(const FW::Vec3f& v0,
					const FW::Vec3f& v1,
					const FW::Vec3f& v2)
{
	FW::Vec3f line = v1 - v0;

	float length = line.length();

	if (fuzzyCompare(length, 0.0f))
		return 0.0f;

	return distanceToLine(v2, v0, line / length) / length;
}

bool QuadricsTessellator::fuzzyCompare(float p1, float p2)
{
	return (abs(p1 - p2) * 1000000000000. <=
					fmin(abs(p1), abs(p2)));
}


bool QuadricsTessellator::fuzzyCompare3f(
				const FW::Vec3f& p1, const FW::Vec3f& p2)
{
	return fuzzyCompare(p1[0], p2[0]) &&
			fuzzyCompare(p1[1], p2[1]) &&
			fuzzyCompare(p1[2], p2[2]);
}


FW::Vec3f QuadricsTessellator::getNormal(const FW::Vec3f& v1,
				const FW::Vec3f& v2, const FW::Vec3f& v3)
{
	FW::Vec3f v = FW::cross((v2 - v1), (v3 - v1));
	return v.normalized();
}


bool QuadricsTessellator::calculateVertex(
	FW::Vec3f (*f)(float, float, float *), float *args,
	Vertex* vertex, const FW::Vec2f& param)
{
	vertex->param = param;
	vertex->pos = f(param[0], param[1], args);

	/*
	    if pos is invalid return false;
	*/

	float du = 0.1f / (float)(1 << ( lod_max_ / 2 ));
	float dv = 0.1f / (float)(1 << ( lod_max_ / 2 ));

	if (param[0] == 1)
		du = -du;
	if (param[1] == 1)
		dv = -dv;

	FW::Vec2f p1 = param + FW::Vec2f(du, 0.0f);
	FW::Vec2f p2 = param + FW::Vec2f(0.0f, dv);
	
	if (param[1] == 0 || param[1] == 1)
		p1 = param + FW::Vec2f(du, dv);

	FW::Vec3f v1, v2;
	v1 = f(p1[0], p1[1], args);
	v2 = f(p2[0], p2[1], args);
	vertex->normal = getNormal(vertex->pos, v1, v2);

	if ((param[0] == 1 || param[1] == 1) && param[0] != param[1])
		vertex->normal = -vertex->normal;


	bool eq1 = fuzzyCompare3f(vertex->pos, v1);
	bool eq2 = fuzzyCompare3f(vertex->pos, v2);
	if (!eq1 && !eq2) {
	} else {
		FW::Vec2f p3 = param - FW::Vec2f(du, dv);
		FW::Vec3f v3;
		
		v3 = f(p3[0], p3[1], args);
		
		//    if v3 is not valid return false;
		
		vertex->normal = eq1 ?
				getNormal(vertex->pos, v3, v2)
				: getNormal(vertex->pos, v1, v3);
	}

	return true;
}

void QuadricsTessellator::addToScene(void* scene_ptr)
{
	RibScene* scene = (RibScene*)scene_ptr;
	FW::Array<FW::Vec3f>* scene_vtx_positions = scene->vtx_positions_ptr();
	FW::Array<unsigned int>* scene_tri_indices = scene->tri_indices_ptr();
	FW::Array<TriangleData>* scene_tri_data = scene->triangle_data_ptr();

	int offset = scene_vtx_positions->getSize();
	FW::Vec3f* scene_bounds = scene->bounds();
	int material_id = scene->current_material_id();
	TransformMatrix4f* basis = scene->basis_ptr();

	for (int i = 0; i < vertices_.getSize(); i++) {
		FW::Vec3f vertex = vertices_[i].pos;
		vertex = basis->transformP(vertex);
		scene_bounds[0] = FW::max(scene_bounds[0], vertex);
		scene_bounds[1] = FW::min(scene_bounds[1], vertex);
		scene_vtx_positions->add(vertex);
	}

	for (int i = 0; i < faces_.getSize(); i++) {
		TriangleData triangle_data;
		triangle_data.material_id = material_id;
		FW::Vec3i triangle;

		for (int j = 0; j < 3; j++) {
			int index = 0;
			index = edges_[faces_[i].e[j]].v[faces_[i].dir[j]];
			triangle[j] = offset + index;
			Vertex vertex = vertices_[index];
			//FW::Vec3f normal = basis->transformV(vertex.normal);
			//triangle_data.normals[j] = normal.normalized();
			scene_tri_indices->add(offset + index);
			triangle_data.sts[j] = vertex.param;
		}

		FW::Vec3f p[3];
		for (int j = 0; j < 3; j++)
			p[j] = scene_vtx_positions->get(triangle[j]);
		FW::Vec3f normal = FW::cross(p[1] - p[0], p[1] - p[2]);
		for (int j = 0; j < 3; j++)
			triangle_data.normals[j] = normal.normalized();

		scene_tri_data->add(triangle_data);
	}
}

} // namespace rt_pathtracer
