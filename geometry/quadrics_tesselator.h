#ifndef RTPT_QUADRICS_TESSELATOR_H_
#define RTPT_QUADRICS_TESSELATOR_H_

// http://www.tankonyvtar.hu/hu/tartalom/tamop425/0046_algorithms_of_informatics_volume2/ch10s03.html

#include "nvidia_base/Array.hpp"

namespace rt_pathtracer {


class QuadricsTessellator
{
public:
	QuadricsTessellator();
	~QuadricsTessellator();
	void clear();
	bool tessellate(FW::Vec3f (*f)(float, float, float *), float *args);
	void addToScene(void* scene);

private:
	struct Vertex
	{
		FW::Vec2f param;
		FW::Vec3f pos;
		FW::Vec3f normal;
	};

	struct Edge
	{
		int lod;
		int v[2];
		int f[2];
	};

	struct Face
	{
		int e[3];
		int dir[3];
	};

private:
	bool initialize(FW::Vec3f (*f)(float, float, float *), float *args);
	bool calculateVertex(FW::Vec3f (*f)(float, float, float *), float *args,
			Vertex* vertex, const FW::Vec2f& param);

	static float distanceToLine(const FW::Vec3f& origin,
			const FW::Vec3f& point, const FW::Vec3f& direction);
    	static float delta(const FW::Vec3f& v0,
				const FW::Vec3f& v1, const FW::Vec3f& v2);
	static bool fuzzyCompare(float p1, float p2);
	static bool fuzzyCompare3f(const FW::Vec3f& p1, const FW::Vec3f& p2);
	static FW::Vec3f getNormal(const FW::Vec3f& v1,
				const FW::Vec3f& v2, const FW::Vec3f& v3);
private:
	int lod_min_;
	int lod_max_;
	float geometry_threshold_;

	FW::Array<Vertex> vertices_;
	FW::Array<Edge> edges_;
	FW::Array<Face> faces_;
};

} // namespace rt_pathtracer

#endif // RTPT_QUADRICS_TESSELATOR_H_
