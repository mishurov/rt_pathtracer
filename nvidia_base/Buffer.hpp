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

#include "nvidia_base/Defs.hpp"
#include "nvidia_kernels/CudaTracerKernels.hpp"


namespace FW
{
//------------------------------------------------------------------------

class Buffer
{
public:
    explicit        Buffer              ()                 { init(0); }
    virtual         ~Buffer             (void)                                  {}

    S64             getSize             (void) const                            { return m_size; }
    void            reset               (const void* ptr, S64 size)             { deinit(); init(size); if (ptr) setRange(0, ptr, size); }
    void            resizeDiscard       (S64 size)                              { if (m_size != size) reset(NULL, size); }
    void            setRange            (S64 dstOfs, const void* src, S64 size);
    void            set                 (const void* ptr, S64 size) { resizeDiscard(size); setRange(0, ptr, size); }
    U8*             getMutablePtr       (S64 ofs = 0)                           { FW_ASSERT(ofs >= 0 && ofs <= m_size); return m_cpuPtr + ofs; }
    CUdeviceptr     getCudaPtr          (S64 ofs = 0);
private:
    void            init                (S64 size);
    void            deinit              (void);
private:
    S32             m_align;
    S64             m_size;

    U8*             m_cpuPtr;
    U8*             m_cpuBase;

    CUdeviceptr     m_cudaPtr;
    CUdeviceptr     m_cudaBase;
};

//------------------------------------------------------------------------
}
