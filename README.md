# CUDA accelerated unidirectional pathtracer

Numerical Monte Carlo integral estimation and bla bla bla.

The project was developed and tested on a laptop with NVidia GeForce 940MX in Debian 9 with the stable and backports repositories, CUDA 8 is from Debian's repositories. Since CUDA 8 doesn't support the newer GCC versions from the official repos, the CMake project was initialised with Clang. CUDA architecture for the target hardware is also specified in the CMake file, CUDA_NVCC_FLAGS.

I didn't test or compile the project on other systems thus I'm not sure whether it works flawlessly out the box on a different rigs so it may require some modifications.

Note that the repository depends on [the submodule](https://github.com/mishurov/rib_lexer_parser) so
```shell
git clone --recurse-submodules
```

Initially the project was started from [this tutorial](http://raytracey.blogspot.ru/2015/10/gpu-path-tracing-tutorial-1-drawing.html) by Samuel Lapere. Lately it was totally rewritten, BVH and linear maths were replaced with the original code from NVidia, replaced OpenGL framework for GUI, kernels, bxdf, cameras, geometry processing, etc. There're only minor parts remain such as drawing into OpenGL framebuffer and BVH caching.

## Features
![](http://mishurov.co.uk/images/github/rt_pathtracer/vr.png)

*demo.rib*

* Headless mode and sending compressed image stream to a remote client over UDP, there's an example of [a client  implementation](https://github.com/mishurov/rt_client_android) displaying the rendered images on Google Cardboard devices. The protocol is sort of naive and straightforward but on a real device it looks pretty much fascinating and gives a notion how a production tier pathtracer for VR could look.

* GUI mode with an interactive window and Maya-style navigation.

* CUDA Persistent Threads in order to keep CUDA cores busy.

* Kepler's Texture objects for BVH data and image textures.

* ASCII RIB as an input format, only some fraction of the syntax is supported.

### NVidia's BVH
![](http://mishurov.co.uk/images/github/rt_pathtracer/stanford.png)

*stanford.rib*

Nvidia's BVH implementation optimised for Kepler architecture from **Understanding the Efficiency of Ray Traversal on GPUs** by Timo Aila and Samuli Laine. The good part of the code for the renderer is also based on framework's data structures and utility functions from Nvidia's code for the paper.

### Importance sampling
![](http://mishurov.co.uk/images/github/rt_pathtracer/microfacet.png)

*microfacet.rib*

Sampling rays according to BxDF probability distributions, e.g. Beckmann distribution for the micro-facet model. The chunk of code for the micro-facet model is taken from [PBRT](https://github.com/mmp/pbrt-v3) and adapted for CUDA.

### Multiple importance sampling
![](http://mishurov.co.uk/images/github/rt_pathtracer/cornell_box.png)

*cornell_box.rib*

Multiple Importance Sampling for direct lightning and bxdf: power heuristic and so on, it's a quite standard solution for such a type of tasks.

### Quadric surfaces
![](http://mishurov.co.uk/images/github/rt_pathtracer/tessellation.png)

*tessellation.rib*

The quadric surfaces can be either tested for a ray intersection analytically or triangulated using the adaptive iterative tessellation.

The quadric-ray intersections are somewhat limited, they don't use any acceleration structure, only brute-force checks for every bounding box and then a surface within it. Thus I've hardcoded the maximum number of quadrics which can appear in a scene in order to do not overhead computations.

### Concave polygons with holes
![](http://mishurov.co.uk/images/github/rt_pathtracer/concave_maya.png)

*concave.ma*

![](http://mishurov.co.uk/images/github/rt_pathtracer/concave.png)

*concave.rib*

Triangulation via Earcut library for concave polygons with holes. It's a tiny, concise library. All I do is just map a polygon onto 2d subspace using quaternions and apply library's functions.

### Textures
![](http://mishurov.co.uk/images/github/rt_pathtracer/textures.png)

*textures.rib*

Image based lightning and image textures from HDR, JPEG and TGA files. Actually I wanted to use as less external libraries as possible but since I'm using jpeg library for image streaming, I've made it possible to use jpeg images as textures as well.


