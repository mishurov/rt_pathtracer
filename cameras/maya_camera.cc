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


#include "cameras/camera.h"
#include <math.h>

namespace rt_pathtracer {

CameraInfo* BaseCamera::camera_info_ptr()
{
	return &c_;
}


inline float radians(float d) { return d * M_PI / 180; }
inline float degrees(float r) { return r * 180/ M_PI; }


MayaCamera* MayaCamera::instance;


MayaCamera::MayaCamera(int im_width, int im_height)
{
	c_.position = FW::Vec3f(-7, 0.7, 0);
	c_.up = FW::Vec3f(0, 1, 0);

	c_.aperture_radius = 0.04;
	c_.lens_radius = 1;
	c_.A = 5; 

	c_.focal_distance = 4.0f;
	c_.resolution = FW::Vec2f(im_width, im_height);
	c_.fov[0] = 45;
	c_.fov[1] = degrees(
		atan(
			tan(radians(c_.fov[0]) * 0.5) *
			(c_.resolution[1] / c_.resolution[0])
		) * 2.0
	);
	target_ = FW::Vec3f(0, 0, 0);
	c_.view = target_ - c_.position;
	rotation_ = getQuatFromEuler(0, 0, 0);
	distance_ = 0;
	update();
	matchRotation();
	setLastPointer(-1, -1);
}


void MayaCamera::onMouseButton(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_RELEASE) {
		instance->setLastPointer(-1, -1);
	}
}


void MayaCamera::onMouseScroll(GLFWwindow* window, double xoffset, double yoffset)
{
	instance->dolly(yoffset);
}

void MayaCamera::onMouseMove(GLFWwindow* window, double xpos, double ypos)
{
	int lb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	int rb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
	int mb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
	int ctl = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL);
	int shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT);

	bool dolly = (lb == GLFW_PRESS
				&& ctl == GLFW_PRESS) || rb == GLFW_PRESS;
	bool track = (lb == GLFW_PRESS
				&& shift == GLFW_PRESS) || mb == GLFW_PRESS;
	bool tumble = lb == GLFW_PRESS && !dolly && !track;

	if (dolly)
		instance->dragDolly(xpos, ypos);
	else if (track)
		instance->track(xpos, ypos);
	else if (tumble)
		instance->tumble(xpos, ypos);

	if (dolly || track || tumble)
		instance->setLastPointer(xpos, ypos);
}


void MayaCamera::setLastPointer(double xpos, double ypos)
{
	last_pointer_ = FW::Vec2f(xpos, ypos);
}


void MayaCamera::dolly(double offset)
{
	distance_ -= offset;
	FW::Vec3f z(0, 0, 1);
	c_.position = distance_ * multQuatVec(rotation_, z) + target_;
	update();
	matchRotation();
}


void MayaCamera::dragDolly(double xpos, double ypos)
{
	if (last_pointer_[0] < 0 && last_pointer_[1] < 0)
		return;

	FW::Vec2f current_pointer(xpos, ypos);
	FW::Vec2f offset = last_pointer_ - current_pointer;

	float delta_x = fabs(offset.x);
	float delta_y = fabs(offset.y);

	offset *= 0.03;
	
	if (delta_x > delta_y)
		dolly(-offset.x);
	else
		dolly(offset.y);
}

void MayaCamera::track(double xpos, double ypos)
{
	if (last_pointer_[0] < 0 && last_pointer_[1] < 0)
		return;

	FW::Vec2f current_pointer(xpos, ypos);
	FW::Vec2f offset = last_pointer_ - current_pointer;

	FW::Vec3f view = target_ - c_.position;

	FW::Vec3f x(1, 0, 0);
	FW::Vec3f right = multQuatVec(rotation_, x).normalized();

	FW::Vec3f pan = (c_.up * -offset[1] + right * offset[0]) *
				view.length() * 0.001;
	
	c_.position += pan;
	target_ += pan;

	update();
	matchRotation();
}

void MayaCamera::tumble(double xpos, double ypos)
{
	FW::Vec2f current_pointer(xpos, ypos);
	if (last_pointer_ == current_pointer)
		return;

	if (last_pointer_[0] < 0 && last_pointer_[1] < 0) {
		return;
	}

	FW::Vec2f offset = last_pointer_ - current_pointer;

	float rad_per_pixel = M_PI / 450;
	float yaw = offset[0] * rad_per_pixel;
	float pitch = offset[1] * rad_per_pixel;

	FW::Vec3f up(0, 0, 0);
	up.y = c_.up.y;

	float limit = 0.05;

	if (up.y < limit && up.y > 0) {
		yaw = 0;
		up.y = limit;
	}
	if (up.y > -limit && up.y < 0) {
		yaw = 0;
		up.y = -limit;
	}

	FW::Vec4f qy = getQuatFromAxis(up, yaw);
	c_.position = multQuatVec(qy, c_.position);
	matrix_ = lookAt(target_, c_.position, c_.up);
	matchRotation();


	FW::Vec3f view = target_ - c_.position;
	FW::Vec3f right = FW::cross(view, up);
	FW::Vec4f qx = getQuatFromAxis(right, pitch);
	c_.position = multQuatVec(qx, c_.position);
	view = target_ - c_.position;
	c_.up = FW::cross(-view, right).normalized();

	update();
	matchRotation();
}


void MayaCamera::arcBall(double xpos, double ypos)
{
	FW::Vec2f current_pointer(xpos, ypos);
	if (last_pointer_ == current_pointer)
		return;
	
	FW::Vec3f current_radial_vec = getPointOnSphere(current_pointer);

	if (last_pointer_[0] < 0 && last_pointer_[1] < 0) {
		last_radial_vec_ = current_radial_vec;
		return;
	}

	FW::Vec4f q = getQuatBetween(last_radial_vec_, current_radial_vec);
	q = invertQuat(q);

	last_radial_vec_ = current_radial_vec;

	rotation_ = multQuatQuat(rotation_, q);

	FW::Vec3f y(0, 1, 0);
	c_.up = multQuatVec(rotation_, y).normalized();

	FW::Vec3f z(0, 0, 1);
	c_.position = distance_ * multQuatVec(rotation_, z) + target_;
	update();
}


void MayaCamera::update() {
	c_.view = target_ - c_.position;
	matrix_ = lookAt(target_, c_.position, c_.up);
	is_dirty = true;
}

void MayaCamera::matchRotation()
{
	FW::Vec4f q = getQuatFromMat(matrix_);
	rotation_ = invertQuat(q);
	distance_ = (c_.position - target_).length();
}

float MayaCamera::mod(float x, float y)
{
	return x - y * floorf(x / y);
}

FW::Vec3f MayaCamera::getPointOnSphere(FW::Vec2f point)
{
	FW::Vec3f res;

	float x = (2.f * point.x - c_.resolution[0]) / c_.resolution[0];
	float y = (c_.resolution[1] - 2.f * point.y) / c_.resolution[1];
	
	float length2 = x * x + y * y;

	if (length2 <= .5) {
		res.z = sqrt(1.0 - length2);
	} else {
		res.z = 0.5 / sqrt(length2);
	}

	float norm = 1.0 / sqrt(length2 + res.z * res.z);

	res.x = x * norm;
	res.y = y * norm;
	res.z *= norm;

	return res;
}


FW::Mat4f MayaCamera::lookAt(FW::Vec3f view, FW::Vec3f pos, FW::Vec3f up)
{
	FW::Mat4f res;
		
	FW::Vec3f zaxis = (pos - target_).normalized();
	FW::Vec3f xaxis = cross(up, zaxis).normalized();
	FW::Vec3f yaxis = cross(zaxis, xaxis); 

	res.setRow(0, FW::Vec4f(
		xaxis.x,             yaxis.x,              zaxis.x,          0
	));
	res.setRow(1, FW::Vec4f(
		xaxis.y,             yaxis.y,              zaxis.y,          0
	));
	res.setRow(2, FW::Vec4f(
		xaxis.z,             yaxis.z,              zaxis.z,          0
	));
	res.setRow(3, FW::Vec4f(
		-FW::dot(xaxis, target_),-FW::dot(xaxis, target_), -dot(xaxis, target_), 1
	));

	return res;
}


FW::Vec4f MayaCamera::getQuatBetween(FW::Vec3f rotateFrom, FW::Vec3f rotateTo)
{
	FW::Vec3f p1, p2;
	float alpha;

	p1 = rotateFrom.normalized();
	p2 = rotateTo.normalized();

	alpha = FW::dot(p1, p2);

	if (alpha == float(1.0)) {
		return FW::Vec4f(1, 0, 0, 0);
	}
	if (alpha == float(-1.0)) {
		FW::Vec3f v;

		if (p1[0] != p1[1] || p1[0] != p1[2]) {
			v = FW::Vec3f(p1[1], p1[2], p1[0]);
		} else {
			v = FW::Vec3f(-p1[0], p1[1], p1[2]);
		}

		v -= p1 * FW::dot(p1, v);
		v.normalize();
		return getQuatFromAxis(v, 3.1415926);
	}
	p1 = normalize(cross(p1, p2));
	return getQuatFromAxis(p1, acos(alpha));
}


FW::Vec3f MayaCamera::multQuatVec(FW::Vec4f q, FW::Vec3f src)
{
	FW::Vec3f dst;
	float v_coef = q.w * q.w - q.x * q.x - q.y * q.y - q.z * q.z;
	float u_coef = float(2.0) * (src[0] * q.x + src[1] * q.y + src[2] * q.z);
	float c_coef = float(2.0) * q.w;

	dst[0] = v_coef * src[0] + u_coef * q.x + c_coef *
						(q.y * src[2] - q.z * src[1]);
	dst[1] = v_coef * src[1] + u_coef * q.y + c_coef *
						(q.z * src[0] - q.x * src[2]);
	dst[2] = v_coef * src[2] + u_coef * q.z + c_coef *
						(q.x * src[1] - q.y * src[0]);
	return dst;
}


FW::Vec4f MayaCamera::multQuatQuat(const FW::Vec4f& ql, const FW::Vec4f& qr)
{
	FW::Vec4f r;
	r.w = ql.w * qr.w - ql.x * qr.x - ql.y * qr.y - ql.z * qr.z;
	r.x = ql.w * qr.x + ql.x * qr.w + ql.y * qr.z - ql.z * qr.y;
	r.y = ql.w * qr.y + ql.y * qr.w + ql.z * qr.x - ql.x * qr.z;
	r.z = ql.w * qr.z + ql.z * qr.w + ql.x * qr.y - ql.y * qr.x;
	return r;
}


FW::Vec4f MayaCamera::invertQuat(FW::Vec4f q)
{
	q.x *= -1.0;
	q.y *= -1.0;
	q.z *= -1.0;
	return q;
}


FW::Vec4f MayaCamera::getQuatFromMat(FW::Mat4f m)
{
	FW::Vec4f res;

	float tr, s;
	int i, j, k;
	const int nxt[3] = { 1, 2, 0 };

	tr = m(0,0) + m(1,1) + m(2,2);

	if (tr > float(0)) {
		s = float(sqrt(tr + m(3,3)));
		res[3] = float(s * 0.5);
		s = float(0.5) / s;

		res[0] = float((m(1,2) - m(2,1)) * s);
		res[1] = float((m(2,0) - m(0,2)) * s);
		res[2] = float((m(0,1) - m(1,0)) * s);
	} else {
		i = 0;

		if (m(1,1) > m(0,0)) {
			i = 1;
		}

		if (m(2,2) > m(i,i)) {
			i = 2;
		}

		j = nxt[i];
		k = nxt[j];

		s = float(sqrt((m(i,j) - (m(j,j) + m(k,k))) + float(1.0)));

		res[i] = float(s * 0.5);
		s = float(0.5 / s);

		res[3] = float((m(j,k) - m(k,j)) * s);
		res[j] = float((m(i,j) + m(j,i)) * s);
		res[k] = float((m(i,k) + m(k,i)) * s);
	}
	return res;
}


FW::Vec4f MayaCamera::getQuatFromAxis(FW::Vec3f axis, float theta)
{
	FW::Vec4f r;
	float sqnorm = axis.lenSqr();

	if (sqnorm == float(0.0))
	{
		r.x = r.y = r.z = float(0.0);
		r.w = float(1.0);
	} else {
		theta *= float(0.5);
		float sin_theta = float(FW::sin(theta));

		if (sqnorm != float(1)) {
			sin_theta /= float(sqrt(sqnorm));
		}

		r.x = sin_theta * axis[0];
		r.y = sin_theta * axis[1];
		r.z = sin_theta * axis[2];
		r.w = float(FW::cos(theta));
	}

	return r;
}

FW::Vec4f MayaCamera::getQuatFromEuler(float x, float y, float z) {
    // angles are in radians
    FW::Vec4f r;
    float c1 = cos(x);
    float s1 = sin(x);
    float c2 = cos(y);
    float s2 = sin(y);
    float c3 = cos(z);
    float s3 = sin(z);
    r.w = sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
    float w4 = (4.0 * r.w);
    r.x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4 ;
    r.y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4 ;
    r.z = (-s1 * s3 + c1 * s2 * c3 +s2) / w4 ;
    return r;
}


FW::Vec4f MayaCamera::normalizeQuat(FW::Vec4f q)
{
	FW::Vec4f r;
	float rnorm = float(1.0) / float(sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z));

	r.x *= rnorm;
	r.y *= rnorm;
	r.z *= rnorm;
	r.w *= rnorm;
	return r;
}

FW::Vec3f MayaCamera::eulerFromQuat(FW::Vec4f q)
{
	FW::Vec3f e;
	float x = q.x;
	float y = q.y;
	float z = q.z;
	float w = q.w;
	e.x = atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z);
	e.y = atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z);
 	e.z = asin(2 * x * y + 2 * z * w);
	return e;
}

} // namespace rt_pathtracer
