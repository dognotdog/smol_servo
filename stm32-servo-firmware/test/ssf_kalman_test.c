
#include "unity.h"

#include "ssf_kalman.h"
#include <math.h>
#include <float.h>


// mock
// void ssf_ui1sTask(uint32_t now_us) {
//  (void)now_us;
// }

void setUp(void) 
{
    // set stuff up here
}

void tearDown(void) 
{
    // clean stuff up here
}

void test_mmul_scalars(void) 
{
    //test stuff
    float a = 1.0f;
    float b = 2.0f;
    float c = 0.0f;
    kalman_matrix_dims_t adims = {1,1};
    kalman_matrix_dims_t bdims = {1,1};
    kalman_matrix_dims_t cdims = {1,1};

    mmuladd(&c, cdims, &a, adims, &b, bdims);

    TEST_ASSERT_FLOAT_WITHIN(2.0f*FLT_EPSILON, 2.0f, c);

    c = 2.0f;

    mmuladd(&c, cdims, &a, adims, &b, bdims);

    TEST_ASSERT_FLOAT_WITHIN(4.0f*FLT_EPSILON, 4.0f, c);

    c = 4.0f;
    mmulsub(&c, cdims, &a, adims, &b, bdims);

    TEST_ASSERT_FLOAT_WITHIN(4.0f*FLT_EPSILON, 2.0f, c);

}

void test_mmul_vectors(void) 
{
    //test vector ops
    float a[] = {1.0f, 2.0f};
    float b[] = {3.0f, 
                 4.0f};
    float c[] = {0.0f};
    float d[] = {0.0f, 0.0f, 0.0f, 0.0f};
    kalman_matrix_dims_t adims = {.rows = 1, .cols = 2};
    kalman_matrix_dims_t bdims = {.rows = 2, .cols = 1};
    kalman_matrix_dims_t cdims = {1,1};
    kalman_matrix_dims_t ddims = {2,2};

    // dot product
    mmuladd(c, cdims, a, adims, b, bdims);

    TEST_ASSERT_FLOAT_WITHIN(11.0f*FLT_EPSILON, 11.0f, c[0]);

    // outer product
    mmuladd(d, ddims, b, bdims, a, adims);

    TEST_ASSERT_FLOAT_WITHIN(3.0f*FLT_EPSILON, 3.0f, d[0]);
    TEST_ASSERT_FLOAT_WITHIN(6.0f*FLT_EPSILON, 6.0f, d[1]);
    TEST_ASSERT_FLOAT_WITHIN(4.0f*FLT_EPSILON, 4.0f, d[2]);
    TEST_ASSERT_FLOAT_WITHIN(8.0f*FLT_EPSILON, 8.0f, d[3]);

}

void test_mmul_matrix(void) 
{
    // test stuff
    float a[] = {1.0f, 
                 2.0f};
    float b[] = {3.0f, 4.0f, 
                 5.0f, 6.0f};
    float bb[] = {7.0f,  8.0f, 
                  9.0f, 10.0f};
    float c[] = {0.0f, 
                 0.0f};
    float d[] = {0.0f, 0.0f, 0.0f, 0.0f};
    kalman_matrix_dims_t adims = {.rows = 2, .cols = 1};
    kalman_matrix_dims_t bdims = {.rows = 2, .cols = 2};
    kalman_matrix_dims_t bbdims = {2,2};
    kalman_matrix_dims_t cdims = {2,1};
    kalman_matrix_dims_t ddims = {2,2};

    // vector transform C = B A
    mmuladd(c, cdims, b, bdims, a, adims);

    TEST_ASSERT_FLOAT_WITHIN(11.0f*FLT_EPSILON, 11.0f, c[0]);
    TEST_ASSERT_FLOAT_WITHIN(17.0f*FLT_EPSILON, 17.0f, c[1]);

    // matrix mul D = B BB
    mmuladd(d, ddims, b, bdims, bb, bbdims);

    TEST_ASSERT_FLOAT_WITHIN(57.0f*FLT_EPSILON, 57.0f, d[0]);
    TEST_ASSERT_FLOAT_WITHIN(64.0f*FLT_EPSILON, 64.0f, d[1]);
    TEST_ASSERT_FLOAT_WITHIN(89.0f*FLT_EPSILON, 89.0f, d[2]);
    TEST_ASSERT_FLOAT_WITHIN(100.0f*FLT_EPSILON, 100.0f, d[3]);

}

void test_mmul_transform(void) 
{
    float a[] = {1.0f, 2.0f,
                 3.0f, 4.0f};
    float at[] = {1.0f, 3.0f,
                 2.0f, 4.0f};
    float b[] = {5.0f, 6.0f,
                 7.0f, 8.0f};
    float ab[] = {0.0, 0.0,
                  0.0, 0.0};
    float bat[] = {0.0, 0.0,
                   0.0, 0.0};
    float abat[] = {0.0, 0.0,
                    0.0, 0.0};
    float c[] = {0.0, 0.0,
                 0.0, 0.0};
    kalman_matrix_dims_t adims   = {2,2};
    kalman_matrix_dims_t atdims  = {2,2};
    kalman_matrix_dims_t bdims   = {2,2};
    kalman_matrix_dims_t cdims   = {2,2};
    kalman_matrix_dims_t abdims  = {2,2};
    kalman_matrix_dims_t batdims = {2,2};
    kalman_matrix_dims_t abatdims = {2,2};

    // computing A B A^T
    mmuladd(ab, abdims, a, adims, b, bdims);
    mmuladd(bat, batdims, b, adims, at, atdims);
    mmuladd(abat, abatdims, a, adims, bat, batdims);

    mmultransposeadd(c, cdims, a, adims, b, bdims);

    // test that the separate mul(A, mul(B, A^T)) ops are the same as the
    // A B A^T combined transform
    TEST_ASSERT_FLOAT_WITHIN(abat[0]*FLT_EPSILON, abat[0], c[0]);
    TEST_ASSERT_FLOAT_WITHIN(abat[1]*FLT_EPSILON, abat[1], c[1]);
    TEST_ASSERT_FLOAT_WITHIN(abat[2]*FLT_EPSILON, abat[2], c[2]);
    TEST_ASSERT_FLOAT_WITHIN(abat[3]*FLT_EPSILON, abat[3], c[3]);

}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_mmul_scalars);
    RUN_TEST(test_mmul_vectors);
    RUN_TEST(test_mmul_matrix);
    RUN_TEST(test_mmul_transform);
    return UNITY_END();
}