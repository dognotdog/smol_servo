
#include "unity.h"

// mock
void ssf_ui1sTask(uint32_t now_us) {
 (void)now_us;
}

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_function_should_doBlahAndBlah(void) {
    //test stuff
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_function_should_doBlahAndBlah);
    return UNITY_END();
}