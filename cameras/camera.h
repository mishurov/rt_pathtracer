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

#ifndef RTPT_CAMERA_H_
#define RTPT_CAMERA_H_

#include <GLFW/glfw3.h>
#include "nvidia_base/Math.hpp"

#if !FW_CUDA
#include "network/server.h"
#endif

namespace rt_pathtracer {

struct CameraInfo
{
	FW::Vec3f position;
	FW::Vec3f view;
	FW::Vec3f up;
	FW::Vec2f fov;
	FW::Vec2f resolution;
	float aperture_radius;
	float focal_distance;
	float lens_radius;
	float A;
	bool is_stereo;
	float inter_lens;
};


class BaseCamera {
public:
	CameraInfo* camera_info_ptr();
	bool is_dirty;
protected:
	void computeA();
	CameraInfo c_;
};


class MayaCamera : public BaseCamera {
public:
	MayaCamera(int im_width, int im_height);

	static MayaCamera* instance;
	static void onMouseMove(GLFWwindow* window, double xpos, double ypos);
	static void onMouseScroll(GLFWwindow* window,
				double xoffset, double yoffset);
	static void onMouseButton(GLFWwindow* window,
				int button, int action, int mods);
private:
	void setLastPointer(double xpos, double ypos);
	void dolly(double offset);
	void dragDolly(double xpos, double ypos);
	void track(double xpos, double ypos);
	void arcBall(double xpos, double ypos);
	void tumble(double xpos, double ypos);
	void update();
	void matchRotation();
	FW::Vec3f getPointOnSphere(FW::Vec2f point);
	FW::Vec3f multQuatVec(FW::Vec4f q, FW::Vec3f v);
	FW::Vec4f multQuatQuat(const FW::Vec4f& ql, const FW::Vec4f& qr);
	FW::Mat4f lookAt(FW::Vec3f view, FW::Vec3f pos, FW::Vec3f up);
	FW::Vec4f invertQuat(FW::Vec4f q);
	FW::Vec4f getQuatBetween(FW::Vec3f v1, FW::Vec3f v2);
	FW::Vec4f getQuatFromMat(FW::Mat4f m);
	FW::Vec4f getQuatFromAxis(FW::Vec3f axis, float theta);
	FW::Vec4f getQuatFromEuler(float x, float y, float z);
	FW::Vec4f normalizeQuat(FW::Vec4f q);
	FW::Vec3f eulerFromQuat(FW::Vec4f q);
	float mod(float x, float y);

	void updateLookUp(bool look, bool up);
	void computeCameraLook(FW::Vec3f &look);
private:
	FW::Vec3f target_;
	FW::Mat4f matrix_;
	FW::Vec4f rotation_;
	float distance_;
	FW::Vec2f last_pointer_;
	FW::Vec3f last_radial_vec_;
};


#if !FW_CUDA
class CardboardCamera : public BaseCamera {
public:
	CardboardCamera(int im_width, int im_height);
	void updateCam(CameraParams* cam_params);
private:
	void updateFov(CameraParams* cam_params);
};
#endif

} // namespace rt_pathtracer

#endif // RTPT_CAMERA_H_
