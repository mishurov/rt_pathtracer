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

#ifndef RTPT_INTEGRATOR_CAMERA_H_
#define RTPT_INTEGRATOR_CAMERA_H_

#include "integrator/utils.h"
#include "cameras/camera.h"

namespace rt_pathtracer {

#define DISPARITY 3.5
#define OFFSET_COEFF 0.5355

FW_CUDA_FUNC float RayToCameraPlane(
		const FW::Vec3f& wi,
		CameraInfo* cam,
		FW::Vec3f& p
	)
{
	// returns a PDF of the direction
	FW::Vec3f c_pos = cam->position;
	FW::Vec3f c_view = cam->view.normalized();
	float cos_theta = FW::dot(c_view, -wi);

	if (cos_theta > TMIN) {
		float t = cam->focal_distance / cos_theta;
		p = c_pos - wi * t;
		float dist2 = wi.lenSqr();
		float lens_area = cam->lens_radius != 0 ?
			(PB_PI * cam->lens_radius * cam->lens_radius) : 1;
		return dist2 / (AbsDot(c_view, wi) * lens_area);
	}
	return 0;
}


FW_CUDA_FUNC float CameraPlaneToImIdx(
		const FW::Vec3f& p,
		const FW::Vec2i& im_size,
		CameraInfo* cam,
		int& index
	)
{
	// returns a PDF of the position
	FW::Vec3f c_pos = cam->position;
	FW::Vec3f c_view = cam->view.normalized();
	FW::Vec3f c_up = cam->up.normalized();

	FW::Vec3f h_axis = cross(c_view, c_up).normalized();
	FW::Vec3f v_axis = cross(h_axis, c_view).normalized();
	FW::Vec3f c_mid = c_pos + c_view;

	FW::Vec3f horiz = h_axis * tanf(cam->fov.x * 0.5 * (M_PI / 180));
	FW::Vec3f vert = v_axis * tanf(-cam->fov.y * 0.5 * (M_PI / 180));

	FW::Vec3f unit_p = (p - c_pos) / cam->focal_distance + c_pos;

	FW::Vec3f vec_h = (((unit_p - c_mid) / horiz + 1) * 0.5);
	FW::Vec3f vec_v = (((unit_p - c_mid) / vert + 1) * 0.5);

	float sx = vec_h.z;
	float sy = vec_v.y;

	float pixel_x = sx * (cam->resolution.x - 1);
	float pixel_y = sy * (cam->resolution.y - 1);

	int x = (int)pixel_x;
	int y = im_size.y - (int)pixel_y - 1;

	if (x >= im_size.x || y >= im_size.y || x < 0 || y < 0)
		return 0;

	index = (im_size.y - y - 1) * im_size.x + x;

	float lens_radius = cam->lens_radius;
	float lens_area = PB_PI * lens_radius * lens_radius;

	return 1 / lens_area;
}


FW_CUDA_FUNC void CameraPdfs(
		const FW::Vec3f& dir,
		CameraInfo* cam,
		float& pdf_pos,
		float& pdf_dir
	)
{
	FW::Vec3f c_up = cam->up.normalized();
	float lens_radius = cam->lens_radius;
	float lens_area = PB_PI * lens_radius * lens_radius;
	float cos_theta = FW::dot(c_up, dir);

	pdf_pos = 0;
	pdf_dir = 0;

	if (cos_theta > TMIN) {
		pdf_pos = 1 / lens_area;
		pdf_dir = 1 / (cam->A * cos_theta * cos_theta * cos_theta);
	}
}


FW_CUDA_FUNC void SampleCameraPoint(
		float x,
		float y,
		CameraInfo* cam,
		FW::Vec3f& o,
		FW::Vec3f& d,
		float& pdf_pos,
		float& pdf_dir
	)
{
	FW::Vec3f c_pos = cam->position;
	FW::Vec3f c_view = cam->view.normalized();
	FW::Vec3f c_up = cam->up.normalized();

	FW::Vec3f h_axis = cross(c_view, c_up).normalized();
	FW::Vec3f v_axis = cross(h_axis, c_view).normalized();
	FW::Vec3f c_mid = c_pos + c_view;

	FW::Vec3f horiz = h_axis * tanf(cam->fov.x * 0.5 * (M_PI / 180));
	FW::Vec3f vert = v_axis * tanf(-cam->fov.y * 0.5 * (M_PI / 180));


	FW::Vec3f unit_point = c_mid + (horiz * ((2 * x) - 1)) +
						(vert * ((2 * y) - 1));

	o = c_pos + ((unit_point - c_pos) * cam->focal_distance);
	d = (o - c_pos).normalized();
	o = c_pos;

	CameraPdfs(d, cam, pdf_pos, pdf_dir);
}


FW_CUDA_FUNC void SampleStereoCameraPoint(
		float x,
		float y,
		CameraInfo* cam,
		bool is_left,
		FW::Vec3f& o,
		FW::Vec3f& d,
		float& pdf_pos,
		float& pdf_dir
	)
{
	FW::Vec3f c_pos = cam->position;
	FW::Vec3f c_view = cam->view.normalized();
	FW::Vec3f c_up = cam->up.normalized();
	FW::Vec3f h_axis = cross(c_view, c_up).normalized();
	FW::Vec3f v_axis = cross(h_axis, c_view).normalized();

	FW::Vec3f horiz = h_axis * tanf(cam->fov.x * 0.5 * (M_PI / 180));
	FW::Vec3f vert = v_axis * tanf(-cam->fov.y * 0.5 * (M_PI / 180));

	
	FW::Vec3f mid_offset = horiz * OFFSET_COEFF;
	FW::Vec3f pos_offset = mid_offset * (DISPARITY * cam->inter_lens);

	is_left ? c_pos -= pos_offset : c_pos += pos_offset;

	FW::Vec3f c_mid = c_pos + c_view;

	is_left ? c_mid += mid_offset : c_mid -= mid_offset;
	
	FW::Vec3f unit_point = c_mid + (horiz * ((2 * x) - 1)) +
						(vert * ((2 * y) - 1));

	d = ((unit_point - c_pos) * cam->focal_distance).normalized();
	o = c_pos;

	CameraPdfs(d, cam, pdf_pos, pdf_dir);
}

} // namespace rt_pathtracer

#endif // RTPT_INTEGRATOR_CAMERA_H_
