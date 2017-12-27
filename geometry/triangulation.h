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

#ifndef RTPT_TRIANGULATION_H_
#define RTPT_TRIANGULATION_H_

#include <cmath>
#include "earcut.hpp"


namespace rt_pathtracer {


template <typename T>
struct geo {
	typedef std::vector<T> Loop;
	typedef std::vector<Loop> Polygon;
	typedef std::vector<std::array<int, 3>> Triangulation;
	typedef std::array<T, 3> Triangle;
	typedef std::array<float, 4> Quaternion;
};


template<typename T>
T Cross(T a, T b) {
	return T(a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0]);
}


template<typename T>
float Dot(T a, T b) {
	return a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}


template<typename T>
float SqrLen(T a) {
	return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
}


template<typename T>
T Orthogonal(T v)
{
	float x = abs(v.x);
	float y = abs(v.y);
	float z = abs(v.z);

	T x_axis(1, 0, 0);
	T y_axis(0, 1, 0);
	T z_axis(0, 0, 1);

	T other = x < y ? (x < z ? x_axis : z_axis) : (y < z ? y_axis : z_axis);

	return Cross<T>(v, other);
}


template<typename T>
typename geo<T>::Quaternion GetXYQurtarnion(T v)
{
	// get a rotation quaternion onto the x, y plane
	// https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another

	typename geo<T>::Quaternion q;
	T z = T(0, 0, 1);
	float k_cos_theta = Dot<T>(v, z);
	float k = sqrt(SqrLen<T>(v) * SqrLen<T>(z));
	
	if (k_cos_theta / k == -1) {
		T orth = Orthogonal<T>(v);
		float orth_len = sqrt(SqrLen<T>(v));
		orth[0] /= orth_len;
		orth[1] /= orth_len;
		orth[2] /= orth_len;
		
		q[0] = 0;
		q[1] = orth[0];
		q[2] = orth[1];
		q[3] = orth[2];
  	} else {
		T cross_v_z = Cross<T>(v, z);
		q[0] = k_cos_theta + k;
		q[1] = cross_v_z[0];
		q[2] = cross_v_z[1];
		q[3] = cross_v_z[2];
		float q_len = sqrt(
			q[0] * q[0] + q[1] * q[1] +
			q[2] * q[2] + q[3] * q[2]
		);
		q[0] /= q_len;
		q[1] /= q_len;
		q[2] /= q_len;
		q[3] /= q_len;
	}
	return q;
}


template<typename T>
T RotateQ(T v, typename geo<T>::Quaternion q) {
	// Rotate by a quaternion
	// https://www.mathworks.com/help/aeroblks/quaternionrotation.html

	float rm00 = (1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]);
	float rm01 = 2 * (q[1] * q[2] + q[0] * q[3]);
	float rm02 = 2 * (q[1] * q[3] - q[0] * q[2]);

	float rm10 = 2 * (q[1] * q[2] - q[0] * q[3]);
	float rm11 = (1 - 2 * q[1] * q[1] - 2 * q[3] * q[3]);
	float rm12 = 2 * (q[2] * q[3] + q[0] * q[1]);

	float rm20 = 2 * (q[1] * q[3] + q[0] * q[2]);
	float rm21 = 2 * (q[2] * q[3] - q[0] * q[1]);
	float rm22 = (1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]);

	T ret;
	ret[0] = rm00 * v[0] + rm01 * v[1] + rm02 * v[2];
	ret[1] = rm10 * v[0] + rm11 * v[1] + rm12 * v[2];
	ret[2] = rm20 * v[0] + rm21 * v[1] + rm22 * v[2];

	return ret;
}


template<typename T>
T AvgCross(typename geo<T>::Loop& loop)
{
	T avg_cross(0, 0, 0);

	for (int i = 1; i < loop.size() - 1; i++) {
		T prev = loop.at(i - 1);
		T next = loop.at(i + 1);
		T cur = loop.at(i);
		T edge1(
			prev[0] - cur[0],
			prev[1] - cur[1],
			prev[2] - cur[2]
		);
		T edge2(
			next[0] - cur[0],
			next[1] - cur[1],
			next[2] - cur[2]
		);
		T cross = Cross<T>(edge1, edge2);
		avg_cross[0] += cross[0];
		avg_cross[1] += cross[1];
		avg_cross[2] += cross[2];
	}

	float avg_len = sqrt(SqrLen<T>(avg_cross));
	avg_cross[0] /= avg_len;
	avg_cross[1] /= avg_len;
	avg_cross[2] /= avg_len;

	return avg_cross;
}


template<typename T>
void Project(typename geo<T>::Polygon& poly)
{
	T avg_cross = AvgCross<T>(poly.at(0));
	typename geo<T>::Quaternion q = GetXYQurtarnion<T>(avg_cross);

	T vtx(0, 0, 0);
	for (int i = 0; i < poly.size(); i++) {
		for (int j = 0; j < poly.at(i).size(); j++) {
			vtx = poly.at(i).at(j);
			poly.at(i).at(j) = RotateQ<T>(vtx, q);
		}
	}
}


template<typename T>
void Triangulate(typename geo<T>::Polygon& poly,
		 std::vector<int>& indices)
{
	Project<T>(poly);
	
	using Coord = float;
	using Point = std::array<Coord, 2>;

	std::vector<std::vector<Point>> polygon;
	for (int i = 0; i < poly.size(); i++) {
		std::vector<Point> loop;
		for (int j = 0; j < poly.at(i).size(); j++) {
			Point p;
			p[0] = poly.at(i).at(j)[0];
			p[1] = poly.at(i).at(j)[1];
			loop.push_back(p);
		}
		polygon.push_back(loop);
	}

	indices = mapbox::earcut<int>(polygon);
}


} // namespace rt_pathtracer

#endif  // RIBPARSER_TRIANGULATION_H_
