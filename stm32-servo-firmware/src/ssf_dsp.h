#ifndef SSF_DSP_H
#define SSF_DSP_H

#include <stddef.h>

static inline void zmean(float dst[], size_t dstStride, const float src[], const size_t srcStride , size_t len)
{
	float mean = 0.0f;
	for (size_t i = 0; i < len; ++i)
	{
		mean += src[srcStride*i];
	}
	for (size_t i = 0; i < len; ++i)
	{
		dst[dstStride*i] = src[srcStride*i] - mean;
	}
}

static inline float linterprep(const float const y[], const float i, const size_t len)
{
	long i0 = ((long)i) % len;
    float ifrac = i - i0;
    long i1 = (i0+1) % len;
    float y0 = y[i0+1];
    float y1 = y[i1+1];
    return y1*ifrac + y0*(1.0-ifrac);

}




#endif // SSF_DSP_H
