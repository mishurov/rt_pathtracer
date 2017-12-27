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

#include "nvidia_base/Sort.hpp"

using namespace FW;

//------------------------------------------------------------------------

#define QSORT_STACK_SIZE    32
#define QSORT_MIN_SIZE      16

//------------------------------------------------------------------------

namespace FW
{

struct TaskSpec
{
    S32             low;
    S32             high;
    void*           data;
    SortCompareFunc compareFunc;
    SortSwapFunc    swapFunc;
};

static inline void  insertionSort   (int start, int size, void* data, SortCompareFunc compareFunc, SortSwapFunc swapFunc);
static inline int   median3         (int low, int high, void* data, SortCompareFunc compareFunc);
static int          partition       (int low, int high, void* data, SortCompareFunc compareFunc, SortSwapFunc swapFunc);
static void         qsort           (int low, int high, void* data, SortCompareFunc compareFunc, SortSwapFunc swapFunc);

}

//------------------------------------------------------------------------

void FW::insertionSort(int start, int size, void* data, SortCompareFunc compareFunc, SortSwapFunc swapFunc)
{
    FW_ASSERT(compareFunc && swapFunc);
    FW_ASSERT(size >= 0);

    for (int i = 1; i < size; i++)
    {
        int j = start + i - 1;
        while (j >= start && compareFunc(data, j + 1, j))
        {
            swapFunc(data, j, j + 1);
            j--;
        }
    }
}

//------------------------------------------------------------------------

int FW::median3(int low, int high, void* data, SortCompareFunc compareFunc)
{
    FW_ASSERT(compareFunc);
    FW_ASSERT(low >= 0 && high >= 2);

    int l = low;
    int c = (low + high) >> 1;
    int h = high - 2;

    if (compareFunc(data, h, l)) swap(l, h);
    if (compareFunc(data, c, l)) c = l;
    return (compareFunc(data, h, c)) ? h : c;
}

//------------------------------------------------------------------------

int FW::partition(int low, int high, void* data, SortCompareFunc compareFunc, SortSwapFunc swapFunc)
{
    // Select pivot using median-3, and hide it in the highest entry.

    swapFunc(data, median3(low, high, data, compareFunc), high - 1);

    // Partition data.

    int i = low - 1;
    int j = high - 1;
    for (;;)
    {
        do
            i++;
        while (compareFunc(data, i, high - 1));
        do
            j--;
        while (compareFunc(data, high - 1, j));

        FW_ASSERT(i >= low && j >= low && i < high && j < high);
        if (i >= j)
            break;

        swapFunc(data, i, j);
    }

    // Restore pivot.

    swapFunc(data, i, high - 1);
    return i;
}

//------------------------------------------------------------------------

void FW::qsort(int low, int high, void* data, SortCompareFunc compareFunc, SortSwapFunc swapFunc)
{
    FW_ASSERT(compareFunc && swapFunc);
    FW_ASSERT(low <= high);

    int stack[QSORT_STACK_SIZE];
    int sp = 0;
    stack[sp++] = high;

    while (sp)
    {
        high = stack[--sp];
        FW_ASSERT(low <= high);

        // Small enough or stack full => use insertion sort.

        if (high - low < QSORT_MIN_SIZE || sp + 2 > QSORT_STACK_SIZE)
        {
            insertionSort(low, high - low, data, compareFunc, swapFunc);
            low = high + 1;
            continue;
        }

        // Partition and sort sub-partitions.

        int i = partition(low, high, data, compareFunc, swapFunc);
        FW_ASSERT(sp + 2 <= QSORT_STACK_SIZE);
        if (high - i > 2)
            stack[sp++] = high;
        if (i - low > 1)
            stack[sp++] = i;
        else
            low = i + 1;
    }
}

//------------------------------------------------------------------------

void FW::sort(void* data, int start, int end, SortCompareFunc compareFunc, SortSwapFunc swapFunc)
{
    FW_ASSERT(start <= end);
    FW_ASSERT(compareFunc && swapFunc);

    // Nothing to do => skip.

    if (end - start < 2)
        return;

    qsort(start, end, data, compareFunc, swapFunc);
}

//------------------------------------------------------------------------
