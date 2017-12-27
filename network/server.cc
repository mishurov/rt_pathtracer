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

#include "network/server.h"
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <pthread.h>
#include <jpeglib.h>


namespace rt_pathtracer {


#define LISTEN_PORT 5000
#define SEND_PORT 5001
#define PACK_SIZE 4096
#define JPEG_QUALITY 80


void RenderServer::startListening()
{
	is_scene_ready = false;
	width = 0;
	height = 0;
	client_address_ = "";
	pthread_t t;
	pthread_create(&t, NULL, &RenderServer::listen, this);
}


void *RenderServer::listen(void *arg)
{
	RenderServer* instance = (RenderServer*) arg;
	UDPSocket sock(LISTEN_PORT);

	char buffer[sizeof(ClientData)];
	unsigned short port;

	while (1) {
		sock.recvFrom(
			buffer,
			sizeof(ClientData),
			instance->client_address_,
			port
		);

		ClientData* data = (ClientData*) buffer;

		for (int i = 0; i < 3; i++) {
			instance->cam_params_.up[i] = data->up[i];
			instance->cam_params_.view[i] = data->view[i];
		}
		instance->cam_params_.inter_lens = data->inter_lens;
		instance->cam_params_.fov_horiz = data->fov_horiz;
		instance->cam_params_.fov_vert = data->fov_vert;
		instance->cam_params_.is_stereo = data->is_stereo;
		if (data->is_stereo)
			instance->width = data->width * 2;
		else
			instance->width = data->width;
		instance->height = data->height;

		if (!instance->is_scene_ready) continue;

		printf("\nRecieved client data from %s:%i\n",
			instance->client_address_.c_str(),
			port);

		printf("mode: %s\n", data->is_stereo ? "stereo" : "mono");
		printf("width %i, height %i\n", data->width, data->height);
		printf("camera up [%f, %f, %f]\n",
			data->up[0], data->up[1], data->up[2]);
		printf("camera view [%f, %f, %f]\n",
			data->view[0], data->view[1], data->view[2]);
		printf("FOV: %f, %f \n", data->fov_horiz, data->fov_vert);
		printf("inter lens distance: %f\n", data->inter_lens);
	}

}


void RenderServer::send(char* buffer, int width, int height)
{
	std::vector<unsigned char> compressed;
	compress(buffer, width, height, compressed);

	int total_pack = 1 + (compressed.size() - 1) / PACK_SIZE;
	string address = client_address_ != "" ? client_address_ : "127.0.0.1";

	int buf[1];
	buf[0] = total_pack;
	
	send_soc_.sendTo(buf, sizeof(int), address, SEND_PORT);

	for (int i = 0; i < total_pack; i++) {
		try {
			send_soc_.sendTo(
				&compressed[i * PACK_SIZE],
				PACK_SIZE,
				address,
				SEND_PORT
			);
		} catch (SocketException) {
			return;
		}
	}

}

void RenderServer::compress(char* buffer, int width, int height,
					std::vector<unsigned char>& res)
{
	JSAMPARRAY image;
	image = (JSAMPARRAY)calloc(height, sizeof(JSAMPROW));

	for (int i = 0; i < height; i++) {
		image[i] = (JSAMPROW)calloc(width * 3, sizeof (JSAMPLE));
		for (int j = 0; j < width; j++) {
			image[i][j * 3 + 0] = buffer[(i * width + j) * 4 + 0];
			image[i][j * 3 + 1] = buffer[(i * width + j) * 4 + 1];
			image[i][j * 3 + 2] = buffer[(i * width + j) * 4 + 2];
		}
	}

	struct jpeg_compress_struct compress;
	struct jpeg_error_mgr error;
	compress.err = jpeg_std_error(&error);
	jpeg_create_compress(&compress);
	
	unsigned char* mem;
	unsigned long mem_size = 0;
	jpeg_mem_dest(&compress, &mem, &mem_size);

	compress.image_width = width;
	compress.image_height = height;
	compress.input_components = 3;
	compress.in_color_space = JCS_RGB;

	jpeg_set_defaults(&compress);
	jpeg_set_quality(&compress, JPEG_QUALITY, TRUE);
	jpeg_start_compress(&compress, TRUE);
	jpeg_write_scanlines(&compress, image, height);
	jpeg_finish_compress(&compress);

	jpeg_destroy_compress(&compress);

	for (int i = 0; i < height; i++) {
		free(image[i]);
	}
	free(image);
	res.assign(mem, mem + mem_size);
}


} // namespace rt_pathtracer
