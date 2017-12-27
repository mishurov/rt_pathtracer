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

#define LOWPASS 0.0001

inline float radians(float d) { return d * M_PI / 180; }
inline float degrees(float r) { return r * 180/ M_PI; }


CardboardCamera::CardboardCamera(int im_width, int im_height)
{
	c_.up = FW::Vec3f(0, 1, 0);
	c_.view = FW::Vec3f(0, 0, -1);
	c_.position = FW::Vec3f(0, 0.0, 2.7);

	c_.aperture_radius = 0.04;
	c_.lens_radius = 1;
	c_.A = 5;
	c_.inter_lens = 0.05;

	c_.focal_distance = 1.f;
	c_.resolution = FW::Vec2f(im_width, im_height);
}


void CardboardCamera::updateFov(CameraParams* cam_params)
{
	if (c_.is_stereo) {
		c_.inter_lens = cam_params->inter_lens;
		c_.fov[0] = cam_params->fov_horiz;
		c_.fov[1] = cam_params->fov_vert * 0.5;
	} else {
		c_.fov[0] = 45;
		c_.fov[1] = degrees(
			atan(
				tan(radians(c_.fov[0]) * 0.5) *
				(c_.resolution[1] / c_.resolution[0])
			) * 2.0
		);
	}
}

void CardboardCamera::updateCam(CameraParams* cam_params)
{
	FW::Vec3f up, view;
	for (int i = 0; i < 3; i++) {
		up[i] = cam_params->up[i];
		view[i] = cam_params->view[i];
	}

	if (c_.is_stereo != cam_params->is_stereo ||
	    (c_.view - view).lenSqr() > LOWPASS ||
	    (c_.up - up).lenSqr() > LOWPASS) {
		c_.is_stereo = cam_params->is_stereo;
		c_.view = view;
		c_.up = up;
		updateFov(cam_params);
		is_dirty = true;
	}
}


}
