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

#ifndef RTPT_SERVER_H_
#define RTPT_SERVER_H_

#include <vector>
#include <atomic>
#include <string>
#include "network/PracticalSocket.h"

namespace rt_pathtracer {


struct ClientData
{
	float up[3];
	float view[3];
	float inter_lens;
	float fov_horiz;
	float fov_vert;
	int width;
	int height;
	int is_stereo;
};

struct CameraParams
{
	std::atomic<float> up[3];
	std::atomic<float> view[3];
	std::atomic<float> inter_lens;
	std::atomic<float> fov_horiz;
	std::atomic<float> fov_vert;
	std::atomic<bool> is_stereo;
};

class RenderServer {
public:
	std::atomic<int> width;
	std::atomic<int> height;
	std::atomic<bool> is_scene_ready;
public:
	void send(char* buffer, int width, int height);
	void startListening();
	CameraParams* cam_params_ptr() { return &cam_params_; }
private:
	static void *listen(void *arg);
	void compress(char* buffer, int width, int height,
				std::vector<unsigned char>& res);
private:
	CameraParams cam_params_;
	string client_address_;
	UDPSocket send_soc_;
	UDPSocket recv_soc_;
};


} // namespace rt_pathtracer


#endif // RTPT_RENDERER_H_
