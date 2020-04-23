
#include "ssf_linreg.h"

#include <float.h>
#include <math.h>

incrementalLinreg_t linreg_addSample(incrementalLinreg_t self, float x, float y)
{
	self.sumx  += x;       
	self.sumx2 += x*x;  
	self.sumxy += x*y;
	self.sumy  += y;      
	self.sumy2 += y*y; 
	++self.n;
	return self;
}

linregResult_t linreg_solve(incrementalLinreg_t self)
{
	float denom = (self.n * self.sumx2 - (self.sumx*self.sumx));
	if (fabsf(denom) <= FLT_EPSILON) 
	{
	    return (linregResult_t){.ok = false};
	}
	float nom = (self.n*self.sumxy - self.sumx*self.sumy);
	float den = (self.n*self.sumx2 - self.sumx*self.sumx) * (self.n*self.sumy2 - self.sumy*self.sumy);
	linregResult_t result = {
		.intercept = (self.n * self.sumxy  -  self.sumx*self.sumy) / denom,
		.slope = (self.sumy * self.sumx2  -  self.sumx*self.sumxy) / denom,
	    .rsqr =  nom*nom / den,
	    .ok = true,
	};
	return result;
}

// https://stackoverflow.com/questions/5083465/fast-efficient-least-squares-fit-algorithm-in-c
int linreg(size_t n, const float x[], const float y[], float* m, float* b, float* r)
{
    float sumx = 0.0;	// sum of x     
    float sumx2 = 0.0;	// sum of x**2  
    float sumxy = 0.0;	// sum of x * y 
    float sumy = 0.0;	// sum of y     
    float sumy2 = 0.0;	// sum of y**2 

    for (size_t i = 0; i < n; ++i)
    { 
        sumx  += x[i];       
        sumx2 += (x[i]*x[i]);  
        sumxy += x[i] * y[i];
        sumy  += y[i];      
        sumy2 += (y[i]*y[i]); 
    } 

    float denom = (n * sumx2 - (sumx*sumx));
    if (fabsf(denom) <= FLT_EPSILON) 
    {
        *m = 0.0f;
        *b = 0.0f;
        if (r)
         	*r = 0.0f;
        return -1;
    }

    *m = (n * sumxy  -  sumx*sumy) / denom;
    *b = (sumy * sumx2  -  sumx*sumxy) / denom;
    if (r) 
    {
    	float nom = (n*sumxy - sumx*sumy);
    	float den = ((n*sumx2 - sumx*sumx) *
              (n*sumy2 - sumy*sumy));
        *r =  nom*nom / den;
    }

    return 0; 
}
