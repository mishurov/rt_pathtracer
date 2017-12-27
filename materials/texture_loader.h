#ifndef RTPT_TEXTURE_LOADER_H_
#define RTPT_TEXTURE_LOADER_H_

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <string>

namespace rt_pathtracer {

typedef unsigned char Rgbe[4];

class TextureLoader {
public:
	static GLuint makeTexture(const std::string& filename);

	static unsigned char* readTga(
		const char *filename, int *width, int *height
	);
	static unsigned char* readJpg(
		const char *filename, int *width, int *height
	);
	static float* readHdr(const char *filename, int *width, int *height);

private:
	struct TgaHeader
	{
		char  id_length;
		char  color_map_type;
		char  data_type_code;
		unsigned char  color_map_origin[2];
		unsigned char  color_map_length[2];
		char  color_map_depth;
		unsigned char  x_origin[2];
		unsigned char  y_origin[2];
		unsigned char  width[2];
		unsigned char  height[2];
		char  bits_per_pixel;
		char  image_descriptor;
	};

	static short leShort(unsigned char *bytes);
	static void WorkOnRgbe(Rgbe *scan, int len, float *cols);
	static bool Decrunch(Rgbe *scanline, int len, FILE *file);
	static bool OldDecrunch(Rgbe *scanline, int len, FILE *file);
	static float ConvertComponent(int expo, int val);
};


} // namespace rt_pathtracer

#endif // RTPT_TEXTURE_LOADER_H_
