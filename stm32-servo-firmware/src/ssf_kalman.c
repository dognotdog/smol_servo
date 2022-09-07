/**
 *
 */
#include "ssf_kalman.h"

#include <assert.h>
#include <string.h>
#include <math.h>


void mmuladd(float dst[], kalman_matrix_dims_t dstdims, const float a[], kalman_matrix_dims_t adims, const float b[], kalman_matrix_dims_t bdims)
{
	kalman_matrix_dims_t _dstdims = {adims.rows, bdims.cols};
	assert((_dstdims.rows == dstdims.rows) && (_dstdims.cols == dstdims.cols));
	assert(adims.cols == bdims.rows);
	assert(dstdims.rows == adims.rows);
	assert(dstdims.cols == bdims.cols);

	// memset(dst, 0, sizeof(float)*dstdims.rows*dstdims.cols);

	for (size_t i = 0; i < adims.rows; ++i)
	{
		for (size_t j = 0; j < bdims.cols; ++j)
		{
			float ab = 0.0f;
			for (size_t k = 0; k < adims.cols; ++k)
			{
				ab += a[adims.cols*i+k]*b[bdims.cols*k+j];
			}
			dst[dstdims.cols*i+j] += ab;
		}
	}
}

void mmulsub(float dst[], kalman_matrix_dims_t dstdims, const float a[], kalman_matrix_dims_t adims, const float b[], kalman_matrix_dims_t bdims)
{
	kalman_matrix_dims_t _dstdims = {adims.rows, bdims.cols};
	assert((_dstdims.rows == dstdims.rows) && (_dstdims.cols == dstdims.cols));
	assert(adims.cols == bdims.rows);
	assert(dstdims.rows == adims.rows);
	assert(dstdims.cols == bdims.cols);

	// memset(dst, 0, sizeof(float)*dstdims.rows*dstdims.cols);

	for (size_t i = 0; i < adims.rows; ++i)
	{
		for (size_t j = 0; j < bdims.cols; ++j)
		{
			float ab = 0.0f;
			for (size_t k = 0; k < adims.cols; ++k)
			{
				ab += a[adims.cols*i+k]*b[bdims.cols*k+j];
			}
			dst[dstdims.cols*i+j] -= ab;
		}
	}
}

void mmultransposeadd(float dst[], kalman_matrix_dims_t dstdims, const float a[], kalman_matrix_dims_t adims, const float b[], kalman_matrix_dims_t bdims)
{
	kalman_matrix_dims_t _dstdims = {adims.rows, bdims.rows};
	assert((_dstdims.rows == dstdims.rows) && (_dstdims.cols == dstdims.cols));
	assert(adims.cols == bdims.cols);
	assert(dstdims.rows == adims.rows);
	assert(dstdims.cols == bdims.rows);

	// memset(dst, 0, sizeof(float)*dstdims.rows*dstdims.cols);

	for (size_t i = 0; i < adims.rows; ++i)
	{
		for (size_t j = 0; j < bdims.rows; ++j)
		{
			float abat = 0.0f;
			// k is over cols of A
			for (size_t k = 0; k < adims.cols; ++k)
			{
				float bat = 0.0f;
				for (size_t l = 0; l < adims.cols; ++l)
				{
					bat += b[adims.cols*k + l]*a[bdims.cols*j + l];
				}
				abat += a[adims.cols*i + k]*bat;
			}
			// i is over dst rows, j over dst cols
			dst[dstdims.cols*i + j] += abat;
		}
	}
}

static void _clearMatrix(float m[], kalman_matrix_dims_t dims)
{
	memset(m, 0, sizeof(float)*dims.rows*dims.cols);
}

static inline size_t minz(size_t a, size_t b)
{
	return (a < b) ? a : b;
}

static void _addEyeMatrix(float m[], kalman_matrix_dims_t dims)
{
	for (size_t i = 0; i < minz(dims.rows, dims.cols); ++i)
	{
		m[i*dims.rows + i] += 1.0f;
	}
}


/**
 * do an A*B*A^T transform
 */
static void mtransformadd(float dst[], kalman_matrix_dims_t dstdims, const float a[], kalman_matrix_dims_t adims, const float b[], kalman_matrix_dims_t bdims)
{
	kalman_matrix_dims_t _dstdims = {adims.rows, bdims.cols};
	assert((_dstdims.rows == dstdims.rows) && (_dstdims.cols == dstdims.cols));
	assert(adims.cols == bdims.rows);
	assert(dstdims.rows == adims.rows);
	assert(dstdims.cols == bdims.cols);

	// memset(dst, 0, sizeof(float)*dstdims.rows*dstdims.cols);

	for (size_t i = 0; i < adims.cols; ++i)
	{
		for (size_t j = 0; j < bdims.rows; ++j)
		{
			float ab = 0.0f;
			for (size_t k = 0; k < adims.rows; ++k)
			{
				float ba = 0.0f;
				for (size_t l = 0; l < adims.rows; ++k)
				{
					ba += b[bdims.rows*k+l]*a[adims.rows*j+l];
				}
				ab += a[adims.rows*i+k]*ba;
			}
			dst[dstdims.rows*i+j] += ab;
		}
	}
}

/**
 * 
 */
void kalman_predict(kalman_model_t* model, float dt)
{
	assert(model);

	kalman_matrix_dims_t x_dims = {model->x_dim, 1};
	kalman_matrix_dims_t a_dims = {model->x_dim, model->x_dim};
	kalman_matrix_dims_t u_dims = {model->u_dim, 1};
	kalman_matrix_dims_t b_dims = {model->x_dim, model->u_dim};

	// x = A*x + B*u
	memcpy(model->x_tmp, model->x, sizeof(float)*model->x_dim);
	mmuladd(model->x, x_dims, model->A, a_dims, model->x_tmp, x_dims);
	mmuladd(model->x, x_dims, model->B, b_dims, model->u, u_dims);


	kalman_matrix_dims_t P_dims = {model->x_dim, model->x_dim};

	// P = A P A^T + Q
	_clearMatrix(model->P_tmp, P_dims);
	mtransformadd(model->P_tmp, P_dims, model->A, a_dims, model->P, P_dims);
}

/**
 * sequential measure
 */
void kalman_measure(kalman_model_t* model, float zi, size_t i)
{
	kalman_matrix_dims_t s_dims = {1, 1};
	kalman_matrix_dims_t H_dims = {model->z_dim, model->x_dim};
	kalman_matrix_dims_t h_dims = {1, model->x_dim};
	kalman_matrix_dims_t P_dims = {model->x_dim, model->x_dim};
	kalman_matrix_dims_t K_dims = {model->x_dim, model->z_dim};
	kalman_matrix_dims_t x_dims = {model->x_dim, 1};
	kalman_matrix_dims_t k_dims = x_dims;

	// s_i = H P H^T + r_i
	// but the H here is only a subset of the H matrix
	float* h = &(model->H[H_dims.rows*i]);
	float s = model->r[i];
	mtransformadd(&s, s_dims, h, h_dims, model->P, P_dims);

	// K = P H^T / s
	// but k here is a column vector, so not dense in K
	// though it has the same dims as x
	float* k = model->x_tmp;
	_clearMatrix(k, x_dims);
	mmultransposeadd(k, k_dims, model->P, P_dims, h, h_dims);
	float sinv = 1.0f/s;
	for (size_t j = 0; j < k_dims.rows; ++j)
	{
		model->K[K_dims.rows*j + i] = k[j] * sinv;
	}

	// x = x + K (z - Hx)
	float hx = 0.0f;
	mmuladd(&hx, s_dims, h, h_dims, model->x, x_dims);
	hx = zi - hx;

	mmuladd(model->x, x_dims, k, k_dims, &hx, s_dims);

	// P = (I-KH)*P*(I-KH)^T + K R K^T
	// _copyMatrix(model->P_tmp, model->P, P_dims);
	_clearMatrix(model->P_tmp, P_dims);

	// 2nd part of equation first, so we don't need additional temp matrix
	float ri = model->r[i];
	mmultransposeadd(model->P_tmp, P_dims, k, k_dims, &ri, s_dims);

	_clearMatrix(model->IKH_tmp, P_dims);
	_addEyeMatrix(model->IKH_tmp, P_dims);
	mmulsub(model->IKH_tmp, P_dims, k, k_dims, h, h_dims);

	mmultransposeadd(model->P_tmp, P_dims, model->P, P_dims, model->IKH_tmp, P_dims);


	// swap P <-> P_tmp
	{ 
		float* tmp = model->P; model->P = model->P_tmp; model->P_tmp = tmp;
	}



}

