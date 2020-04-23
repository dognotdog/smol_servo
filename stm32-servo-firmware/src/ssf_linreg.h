#ifndef SSF_LINREG_H
#define SSF_LINREG_H

#include <stddef.h>
#include <stdbool.h>

typedef struct {
    float sumx;		// sum of x     
    float sumx2;	// sum of x**2  
    float sumxy;	// sum of x * y 
    float sumy;		// sum of y     
    float sumy2;	// sum of y**2 
    size_t n;		// number of samples

} incrementalLinreg_t;

typedef struct {
	float intercept, slope, rsqr;
	bool ok;
} linregResult_t;

incrementalLinreg_t linreg_addSample(incrementalLinreg_t self, float x, float y);

linregResult_t linreg_solve(incrementalLinreg_t self);

int linreg(size_t n, const float x[], const float y[], float* m, float* b, float* r);



#endif // SSF_LINREG_H
