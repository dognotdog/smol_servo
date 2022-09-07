#ifndef SSF_KALMAN_H
#define SSF_KALMAN_H

#include <stddef.h>

typedef struct kalman_model_s {
	float* x_tmp; // x-by-1 state vector temp storage
	float* z_tmp; // z-by-1 state vector temp storage
	float* P_tmp; // x-by-x process covariance temp storage
	float* IKH_tmp; // x-by-xprocess covariance update temp storage
	float* x; // x-by-1 state vector
	float* u; // u-by-1 input vector
	float* r; // z-by-1 measurement noise vector
	size_t x_dim;
	size_t z_dim;
	size_t u_dim;
	float* A; // x-by-x state matrix
	float* B; // x-by-u update matrix
	float* H; // z-by-x observation matrix
	float* P; // x-by-x state covariance matrix
	float* Q; // x-by-x process covariance matrix
	float* K; // x-by-z Kalman gain
} kalman_model_t;

typedef struct {
	size_t rows;
	size_t cols;
} kalman_matrix_dims_t;


// math utils
void mmuladd(float dst[], kalman_matrix_dims_t dstdims, const float a[], kalman_matrix_dims_t adims, const float b[], kalman_matrix_dims_t bdims);
void mmulsub(float dst[], kalman_matrix_dims_t dstdims, const float a[], kalman_matrix_dims_t adims, const float b[], kalman_matrix_dims_t bdims);
void mmultransposeadd(float dst[], kalman_matrix_dims_t dstdims, const float a[], kalman_matrix_dims_t adims, const float b[], kalman_matrix_dims_t bdims);


void kalman_predict(kalman_model_t* model, float dt);
void kalman_measure(kalman_model_t* model, float zk, size_t k);

#endif // SSF_KALMAN_H
