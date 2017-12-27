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

#ifndef RTPT_TRANSFORMS_H_
#define RTPT_TRANSFORMS_H_

#include "nvidia_base/Math.hpp"

namespace rt_pathtracer {

class TransformMatrix4f : public FW::Mat4f {
public:
	FW_CUDA_FUNC void addTranslation(float x, float y, float z)
	{
		/*
			1, 0, 0, x,
			0, 1, 0, y,
			0, 0, 1, z,
			0, 0, 0, 1
		*/

		transformer_.setIdentity();
		transformer_.set(0, 3, x);
		transformer_.set(1, 3, y);
		transformer_.set(2, 3, z);

		*this *= transformer_;
	}


	FW_CUDA_FUNC void addScale(float x, float y, float z)
	{
		/*
			x, 0, 0, 0,
			0, y, 0, 0,
			0, 0, z, 0,
			0, 0, 0, 1
		*/

		transformer_.setIdentity();
		transformer_.set(0, 0, x);
		transformer_.set(1, 1, y);
		transformer_.set(2, 2, z);

		*this *= transformer_;
	}


	FW_CUDA_FUNC void addRotate(FW::Vec3f axis, float theta)
	{
		transformer_.setIdentity();
		theta = theta * M_PI / 180;
		float sin_theta = sin(theta);
		float cos_theta = cos(theta);
		axis.normalize();

		/*
			x * x + (1 - x * x) * cos;
			x * y * (1 - cos) - z * sin;
			x * z * (1 - cos) + y * sin;
		*/
		float m_00 = axis[0] * axis[0] + (1 - axis[0] * axis[0]) * cos_theta;
		float m_01 = axis[0] * axis[1] * (1 - cos_theta) - axis[2] * sin_theta;
		float m_02 = axis[0] * axis[2] * (1 - cos_theta) + axis[1] * sin_theta;
		transformer_.set(0, 0, m_00);
		transformer_.set(0, 1, m_01);
		transformer_.set(0, 2, m_02);
		/*
			x * y * (1 - cos) + z * sin;
			y * y + (1 - y * y) * cos;
			y * z * (1 - cos) - a.x * sin;
		*/
		float m_10 = axis[0] * axis[1] * (1 - cos_theta) + axis[2] * sin_theta;
		float m_11 = axis[1] * axis[1] + (1 - axis[1] * axis[1]) * cos_theta;
		float m_12 = axis[1] * axis[2] * (1 - cos_theta) - axis[0] * sin_theta;
		transformer_.set(1, 0, m_10);
		transformer_.set(1, 1, m_11);
		transformer_.set(1, 2, m_12);

		/*
			x * z * (1 - cos) - y * sin;
			y * z * (1 - cos) + x * sin;
			z * z + (1 - z * z) * cos;
		*/
		float m_20 = axis[0] * axis[2] * (1 - cos_theta) - axis[1] * sin_theta;
		float m_21 = axis[1] * axis[2] * (1 - cos_theta) + axis[0] * sin_theta;
		float m_22 = axis[2] * axis[2] + (1 - axis[2] * axis[2]) * cos_theta;
		transformer_.set(2, 0, m_20);
		transformer_.set(2, 1, m_21);
		transformer_.set(2, 2, m_22);
		
		*this *= transformer_;
	}

	FW_CUDA_FUNC FW::Vec3f transformP(FW::Vec3f op) const {
		return FW::Vec3f(get(0, 0) * op[0] + get(0, 1) * op[1] + get(0, 2) * op[2] + get(0, 3),
				get(1, 0) * op[0] + get(1, 1) * op[1] + get(1, 2) * op[2] + get(1, 3),
				get(2, 0) * op[0] + get(2, 1) * op[1] + get(2, 2) * op[2] + get(2, 3));
	}

	FW_CUDA_FUNC FW::Vec3f transformV(FW::Vec3f op) const {
		return FW::Vec3f(get(0, 0) * op[0] + get(0, 1) * op[1] + get(0, 2) * op[2],
				get(1, 0) * op[0] + get(1, 1) * op[1] + get(1, 2) * op[2],
				get(2, 0) * op[0] + get(2, 1) * op[1] + get(2, 2) * op[2]);
	}

	FW_CUDA_FUNC TransformMatrix4f getInverted() const {
		TransformMatrix4f ret;
		FW::Mat4f inv = inverted();
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				ret.set(i, j, inv.get(i, j));
			}
		}
		return ret;
	}


private:
	FW::Mat4f transformer_;
};


} // namespace rt_pathtracer

#endif // RTPT_TRANSFORMS_H_
