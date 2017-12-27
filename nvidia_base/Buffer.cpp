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

#include "nvidia_base/Buffer.hpp"

using namespace FW;

//------------------------------------------------------------------------

#define FW_IO_BUFFER_SIZE 65536

void Buffer::init(S64 size)
{
    FW_ASSERT(size >= 0);

    m_size      = size;
    m_cpuBase = new U8[(size_t)(size)];
    m_cpuPtr = m_cpuBase;
    //m_cpuPtr -= (UPTR)m_cpuPtr % (UPTR)1;

    m_cudaPtr = 0;
    m_cudaBase = 0;
}

//------------------------------------------------------------------------

void Buffer::deinit(void)
{
    if (m_cpuPtr) {
        delete[] m_cpuBase;
        m_cpuBase = NULL;
        m_cpuPtr = NULL;
    }

    if (m_cudaPtr) {
        cuMemFree(m_cudaBase);
        m_cudaPtr = 0;
        m_cudaBase = 0;
    }
    m_size      = 0;
}

//------------------------------------------------------------------------

void Buffer::setRange(S64 dstOfs, const void* src, S64 size)
{
    FW_ASSERT(dstOfs >= 0 && dstOfs <= m_size - size);
    FW_ASSERT(src || !size);
    FW_ASSERT(size >= 0);

    if (!size)
        return;

    m_size      = size;
    memcpy(getMutablePtr(dstOfs), src, (size_t)size);
}

//------------------------------------------------------------------------

CUdeviceptr Buffer::getCudaPtr(S64 ofs)
{
    FW_ASSERT(ofs >= 0 && ofs <= m_size);

    if (!m_cudaPtr) {
       cuMemAlloc(&m_cudaBase, max(1U, (U32)(m_size)));
       m_cudaPtr = m_cudaBase;
     //  m_cudaPtr -= (U32)m_cudaPtr % (U32)1;
       cuMemcpyHtoD(m_cudaPtr, m_cpuPtr, (U32)m_size);
    }

    return m_cudaPtr + (U32)ofs;
}

