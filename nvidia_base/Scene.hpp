/*
 *  Copyright (c) 2009-2011, NVIDIA Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of NVIDIA Corporation nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include "nvidia_base/Array.hpp"
#include "geometry/rib_loader.h"

namespace FW
{
//------------------------------------------------------------------------

class Scene
{
public:
                    Scene               (rt_pathtracer::RibScene *rs);
                    ~Scene              (void);

    int             getNumTriangles             (void) const    { return m_numTriangles; }
    int             getNumVertices              (void) const    { return m_numVertices; }

    Array<Vec3i>&         getTriVtxIndexBuffer        (void)    { return m_triVtxIndex; }
    Array<Vec3f>&         getTriNormalBuffer          (void)    { return m_triNormal; }
    Array<U32>&           getTriMaterialColorBuffer   (void)    { return m_triMaterialColor; }
    Array<Vec3f>&         getVtxPosBuffer             (void)    { return m_vtxPos; }
    Array<Vec2f>&         getStsBuffer             (void)    { return m_sts; }

    U32             hash                        (void);

private:
                    Scene               (const Scene&); // forbidden
    Scene&          operator=           (const Scene&); // forbidden

private:
    S32                   m_numTriangles;
    S32                   m_numVertices;
    Array<Vec3i>          m_triVtxIndex;      // Vec3i[numTriangles]
    Array<Vec3f>          m_triNormal;        // Vec3f[numTriangles]
    Array<U32>            m_triMaterialColor; // U32[numTriangles], ABGR
    Array<Vec3f>          m_vtxPos;           // Vec3f[numVertices]
    Array<Vec2f>          m_sts;
};

//------------------------------------------------------------------------
}
