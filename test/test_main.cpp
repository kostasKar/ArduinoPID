#include <unity.h>
#include "ArduinoPID.h"





void test_pid_basic(){
    ArduinoPID pid(1, -100, 100, LOW_FILTERING);
    pid.setParameters(1.0, 0.1, 0.1);
    TEST_ASSERT_EQUAL(50, pid.compute(50, 0));
    TEST_ASSERT_EQUAL(100, pid.compute(1000, 0));     
    TEST_ASSERT_EQUAL(-100, pid.compute(-1000, 0));     
}

void test_pid_wrap_high(){
    ArduinoPID pid(1, -100, 100, LOW_FILTERING);
    pid.setParameters(1.0, 0.1, 0.1);
    int16_t sp = INT16_MAX;
    int16_t meas = sp + 1;
    pid.reset(meas);
    TEST_ASSERT_EQUAL(-1, pid.compute(sp, meas));    
}

void test_pid_wrap_low(){
    ArduinoPID pid(1, -100, 100, LOW_FILTERING);
    pid.setParameters(1.0, 0.1, 0.1);
    int16_t sp = INT16_MIN;
    int16_t meas = sp - 1;
    pid.reset(meas);
    TEST_ASSERT_EQUAL(1, pid.compute(sp, meas));    
}

void test_pid_high_gains(){
    ArduinoPID pid(1, -1000, 1000, LOW_FILTERING);
    pid.setParameters(255, 0.1, 0.1);
    int16_t sp = INT16_MAX;
    TEST_ASSERT_EQUAL(1000, pid.compute(sp, 0));    
    TEST_ASSERT_EQUAL(1000, pid.compute(sp, 0));
    TEST_ASSERT_EQUAL(255, pid.compute(1, 0));
}

void test_pid_all_gains(){
    ArduinoPID pid(1, -100, 100, NO_FILTERING);
    pid.setParameters(1, 1, 1);               // p  i  d  
    TEST_ASSERT_EQUAL(1, pid.compute(1, 0));  // 1  0  0
    TEST_ASSERT_EQUAL(2, pid.compute(1, 0));  // 1  1  0
    TEST_ASSERT_EQUAL(3, pid.compute(1, 0));  // 1  2  0    
    TEST_ASSERT_EQUAL(4, pid.compute(1, 0));  // 1  3  0
    TEST_ASSERT_EQUAL(3, pid.compute(1, 1));  // 0  4 -1    
    TEST_ASSERT_EQUAL(4, pid.compute(1, 1));  // 0  4  0
}

void test_pid_anti_windup(){
    ArduinoPID pid(1, -100, 100, NO_FILTERING);
    pid.setParameters(1, 1, 1);
    TEST_ASSERT_EQUAL(100, pid.compute(1000, 0));
    TEST_ASSERT_EQUAL(100, pid.compute(1000, 0));
    TEST_ASSERT_EQUAL(10, pid.compute(10, 0));  
    TEST_ASSERT_EQUAL(20, pid.compute(10, 0)); 
}




void setUp() {
    
}   

void tearDown() {

}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_pid_basic);
    RUN_TEST(test_pid_wrap_high);
    RUN_TEST(test_pid_wrap_low);
    RUN_TEST(test_pid_high_gains);
    RUN_TEST(test_pid_all_gains);
    RUN_TEST(test_pid_anti_windup);
    return UNITY_END();
}