#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <memory.h>
#include <jpeglib.h>

#include "materials/texture_loader.h"

#define R 0
#define G 1
#define B 2
#define E 3

#define MINELEN 8
#define MAXELEN 0x7fff

namespace rt_pathtracer {


GLuint TextureLoader::makeTexture(const std::string& filename)
{
	int width, height;
	unsigned char *pixels = NULL;
	GLenum format;
	std::string ext = filename.substr(filename.find_last_of(".") + 1);
	if (!ext.compare("tga")) {
		pixels = readTga(filename.c_str(), &width, &height);
		format = GL_BGR;
	} else if (!ext.compare("jpg") || !ext.compare("jpeg")) {
		pixels = readJpg(filename.c_str(), &width, &height);
		format = GL_RGB;
	} else {
		printf(
			"Texture extension is not supported: %s\n",
			filename.c_str()
		);
	}
	GLuint texture;

	if (!pixels)
		return 0;

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_CLAMP_TO_EDGE);
	glTexImage2D(
		GL_TEXTURE_2D, 0,           /* target, level */
		GL_RGB8,                    /* internal format */
		width, height, 0,           /* width, height, border */
		format, GL_UNSIGNED_BYTE,   /* external format, type */
		(void *)pixels                      /* pixels */
	);
	free(pixels);

	return texture;
}

short TextureLoader::leShort(unsigned char *bytes)
{
	return bytes[0] | ((char)bytes[1] << 8);
}

unsigned char* TextureLoader::readTga(const char *filename,
						int *width, int *height)
{
	TgaHeader header;
	int i, color_map_size;
	unsigned pixels_size;
	FILE *f;
	size_t read;
	unsigned char *pixels;

	f = fopen(filename, "rb");

	if (!f) {
		fprintf(stderr, "Unable to open %s for reading\n", filename);
		return NULL;
	}

	read = fread(&header, 1, sizeof(header), f);

	if (read != sizeof(header)) {
		fprintf(stderr, "%s has incomplete tga header\n", filename);
		fclose(f);
		return NULL;
	}
	if (header.data_type_code != 2) {
		fprintf(stderr, "%s is not an uncompressed RGB tga file\n", filename);
		fclose(f);
		return NULL;
	}
	if (header.bits_per_pixel != 24) {
		fprintf(stderr, "%s is not a 24-bit uncompressed RGB tga file\n", filename);
		fclose(f);
		return NULL;
	}

	for (i = 0; i < header.id_length; ++i)
	if (getc(f) == EOF) {
		    fprintf(stderr, "%s has incomplete id string\n", filename);
		    fclose(f);
		    return NULL;
	}

	color_map_size = leShort(header.color_map_length) * (header.color_map_depth/8);
	for (i = 0; i < color_map_size; ++i)
	if (getc(f) == EOF) {
		    fprintf(stderr, "%s has incomplete color map\n", filename);
		    fclose(f);
		    return NULL;
	}

	*width = leShort(header.width);
	*height = leShort(header.height);
	pixels_size = *width * *height * (header.bits_per_pixel/8);
	pixels = (unsigned char *)malloc(pixels_size);

	read = fread(pixels, 1, pixels_size, f);
	fclose(f);

	if (read != pixels_size) {
		fprintf(stderr, "%s has incomplete image\n", filename);
		free(pixels);
		return NULL;
	}

	return pixels;
}


unsigned char * TextureLoader::readJpg(const char *filename,
						int *width, int *height)
{
	unsigned long data_size;
	unsigned char *rowptr[1];
	unsigned char *jdata;
	struct jpeg_decompress_struct info;
	struct jpeg_error_mgr err;

	FILE* file = fopen(filename, "rb");

	info.err = jpeg_std_error(&err);
	jpeg_create_decompress(&info);

	if(!file) {
		fprintf(stderr, "Error reading JPEG file %s\n", filename);
		return NULL;
	}

	jpeg_stdio_src(&info, file);
	jpeg_read_header(&info, TRUE);

	jpeg_start_decompress(&info);

	*width = info.output_width;
	*height = info.output_height;

	//channels = info.num_components;
	//type = GL_RGB;
	//if (channels == 4) type = GL_RGBA;

	data_size = *width * *height * 3;

	jdata = (unsigned char *)malloc(data_size);
	while (info.output_scanline < info.output_height) {
		rowptr[0] = (unsigned char *)jdata +
			3 * info.output_width * info.output_scanline; 

		jpeg_read_scanlines(&info, rowptr, 1);
	}
	jpeg_finish_decompress(&info);

	return jdata;
}

float* TextureLoader::readHdr(const char *filename, int *width, int *height)
{
	int i;
	char str[200];
	FILE *file;

	file = fopen(filename, "rb");
	if (!file)
		return NULL;

	fread(str, 10, 1, file);
	if (memcmp(str, "#?RADIANCE", 10)) {
		fclose(file);
		return NULL;
	}

	fseek(file, 1, SEEK_CUR);

	i = 0;
	char c = 0, oldc;
	while (true) {
		oldc = c;
		c = fgetc(file);
		if (c == 0xa && oldc == 0xa)
			break;
	}

	char reso[200];
	i = 0;
	while (true) {
		c = fgetc(file);
		reso[i++] = c;
		if (c == 0xa)
			break;
	}

	long w, h;
	if (!sscanf(reso, "-Y %ld +X %ld", &h, &w)) {
		fclose(file);
		return NULL;
	}

	*width = w;
	*height = h;

	float *cols = new float[w * h * 3];
	float *res = cols;

	Rgbe *scanline = new Rgbe[w];
	if (!scanline) {
		fclose(file);
		return NULL;
	}

	for (int y = h - 1; y >= 0; y--) {
		if (Decrunch(scanline, w, file) == false)
			break;
		WorkOnRgbe(scanline, w, cols);
		cols += w * 3;
	}

	delete[] scanline;
	fclose(file);

	return res;
}


float TextureLoader::ConvertComponent(int expo, int val)
{
	float v = val / 256.0f;
	float d = (float)powf(2, expo);
	return v * d;
}


void TextureLoader::WorkOnRgbe(Rgbe *scan, int len, float *cols)
{
	while (len-- > 0) {
		int expo = scan[0][E] - 128;
		cols[0] = ConvertComponent(expo, scan[0][R]);
		cols[1] = ConvertComponent(expo, scan[0][G]);
		cols[2] = ConvertComponent(expo, scan[0][B]);
		cols += 3;
		scan++;
	}
}


bool TextureLoader::Decrunch(Rgbe *scanline, int len, FILE *file)
{
	int  i, j;

	if (len < MINELEN || len > MAXELEN)
		return OldDecrunch(scanline, len, file);

	i = fgetc(file);
	if (i != 2) {
		fseek(file, -1, SEEK_CUR);
		return OldDecrunch(scanline, len, file);
	}

	scanline[0][G] = fgetc(file);
	scanline[0][B] = fgetc(file);
	i = fgetc(file);

	if (scanline[0][G] != 2 || scanline[0][B] & 128) {
		scanline[0][R] = 2;
		scanline[0][E] = i;
		return OldDecrunch(scanline + 1, len - 1, file);
	}

	for (i = 0; i < 4; i++) {
		for (j = 0; j < len;) {
			unsigned char code = fgetc(file);
			if (code > 128) {
				code &= 127;
				unsigned char val = fgetc(file);
				while (code--)
					scanline[j++][i] = val;
			} else  {
				while (code--)
					scanline[j++][i] = fgetc(file);
			}
		}
	}

	return feof(file) ? false : true;
}


bool TextureLoader::OldDecrunch(Rgbe *scanline, int len, FILE *file)
{
	int i;
	int rshift = 0;

	while (len > 0) {
		scanline[0][R] = fgetc(file);
		scanline[0][G] = fgetc(file);
		scanline[0][B] = fgetc(file);
		scanline[0][E] = fgetc(file);
		if (feof(file))
			return false;

		if (scanline[0][R] == 1 &&
			scanline[0][G] == 1 &&
			scanline[0][B] == 1) {
			for (i = scanline[0][E] << rshift; i > 0; i--) {
				memcpy(&scanline[0][0], &scanline[-1][0], 4);
				scanline++;
				len--;
			}
			rshift += 8;
		}
		else {
			scanline++;
			len--;
			rshift = 0;
		}
	}
	return true;
}

} // namespace rt_pathtracer
