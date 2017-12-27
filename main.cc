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

#include "renderer.h"

#define DEFAULT_WIDTH 800
#define DEFAULT_HEIGHT 600
//#define DEFAULT_WIDTH 1500
//#define DEFAULT_HEIGHT 900
#define HEADLESS_ARG "headless"


int main(int argc, char** argv)
{
	int width = DEFAULT_WIDTH;
	int height = DEFAULT_HEIGHT;
	unsigned char flags = RT_GUI;
	std::string filename;


	std::vector <std::string> arguments;
	for (int i = 1; i < argc; i++) {
		arguments.push_back(argv[i]);
	}


	if (arguments.size() == 1) {
		filename = arguments.at(0);
	} else if (arguments.size() == 2 && arguments.at(0) == HEADLESS_ARG) {
		flags = RT_HEADLESS;
		filename = arguments.at(1);
	} else if (arguments.size() == 3) {
		width = std::stoi(arguments.at(0));
		height = std::stoi(arguments.at(1));
		filename = arguments.at(2);
	} else {
		printf("args: [<width> <height> or \"headless\"] <file>\n");
		return 0;
	}


	rt_pathtracer::Renderer renderer(width, height, flags);
	renderer.loadScene(filename.c_str());
	renderer.startRendering();
	renderer.clean();

	return 0;
}

