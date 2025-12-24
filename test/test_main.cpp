#include <unity.h>
#include "CorePID.h"





void test_pid_basic(){
    CorePID pid(1, -100, 100, LOW_FILTERING);
    pid.setParameters(1.0, 0.1, 0.1);
    TEST_ASSERT_EQUAL(50, pid.compute(50, 0));
    TEST_ASSERT_EQUAL(100, pid.compute(1000, 0));     
    TEST_ASSERT_EQUAL(-100, pid.compute(-1000, 0));     
}

void test_pid_wrap_high(){
    CorePID pid(1, -100, 100, LOW_FILTERING);
    pid.setParameters(1.0, 0.1, 0.1);
    int16_t sp = INT16_MAX;
    int16_t meas = sp + 1;
    pid.reset(meas);
    TEST_ASSERT_EQUAL(-1, pid.compute(sp, meas));    
}

void test_pid_wrap_low(){
    CorePID pid(1, -100, 100, LOW_FILTERING);
    pid.setParameters(1.0, 0.1, 0.1);
    int16_t sp = INT16_MIN;
    int16_t meas = sp - 1;
    pid.reset(meas);
    TEST_ASSERT_EQUAL(1, pid.compute(sp, meas));    
}

void test_pid_high_gains(){
    CorePID pid(1, -1000, 1000, LOW_FILTERING);
    pid.setParameters(255, 0.1, 0.1);
    int16_t sp = INT16_MAX;
    TEST_ASSERT_EQUAL(1000, pid.compute(sp, 0));    
    TEST_ASSERT_EQUAL(1000, pid.compute(sp, 0));
    TEST_ASSERT_EQUAL(255, pid.compute(1, 0));
}

void test_pid_all_gains(){
    CorePID pid(1, -100, 100, NO_FILTERING);
    pid.setParameters(1, 1, 1);               // p  i  d  
    TEST_ASSERT_EQUAL(1, pid.compute(1, 0));  // 1  0  0
    TEST_ASSERT_EQUAL(2, pid.compute(1, 0));  // 1  1  0
    TEST_ASSERT_EQUAL(3, pid.compute(1, 0));  // 1  2  0    
    TEST_ASSERT_EQUAL(4, pid.compute(1, 0));  // 1  3  0
    TEST_ASSERT_EQUAL(3, pid.compute(1, 1));  // 0  4 -1    
    TEST_ASSERT_EQUAL(4, pid.compute(1, 1));  // 0  4  0
}

void test_pid_anti_windup(){
    CorePID pid(1, -100, 100, NO_FILTERING);
    pid.setParameters(1, 1, 1);
    TEST_ASSERT_EQUAL(100, pid.compute(1000, 0));
    TEST_ASSERT_EQUAL(100, pid.compute(1000, 0));
    TEST_ASSERT_EQUAL(10, pid.compute(10, 0));  
    TEST_ASSERT_EQUAL(20, pid.compute(10, 0)); 
}

void test_iir_filter(){
    FirstOrderIIRFilter filter;
    filter.setParams(100, 1.0, 10.0); // gain=1, cutoff=1Hz, sampling=10Hz
    TEST_ASSERT_EQUAL(466, filter.run(10) >> 8);  //raw would be 1000
    TEST_ASSERT_EQUAL(247, filter.run(0) >> 8); 
    TEST_ASSERT_EQUAL(131, filter.run(0) >> 8);
    TEST_ASSERT_EQUAL(69, filter.run(0) >> 8);
    TEST_ASSERT_EQUAL(36, filter.run(0) >> 8);
    TEST_ASSERT_EQUAL(19, filter.run(0) >> 8);
    TEST_ASSERT_EQUAL(10, filter.run(0) >> 8);
    TEST_ASSERT_EQUAL(5, filter.run(0) >> 8);
    TEST_ASSERT_EQUAL(2, filter.run(0) >> 8);
    TEST_ASSERT_EQUAL(1, filter.run(0) >> 8);
    TEST_ASSERT_EQUAL(0, filter.run(0) >> 8);
    TEST_ASSERT_EQUAL(0, filter.run(0) >> 8);
}   

void test_derivative_filter(){
    CorePID pid(10.0, -1000, 1000, CUSTOM_CUTOFF_HZ, 1.0);
    pid.setParameters(0, 0, 10.0);
    FirstOrderIIRFilter filter;
    filter.setParams(100, 1.0, 10.0);
    TEST_ASSERT_EQUAL(pid.compute(0, -10), filter.run(10) / 256);  
} 

void test_pid_value_limits(){
    CorePID pid(1, INT16_MIN, INT16_MAX, LOW_FILTERING);
    pid.setParameters(255, 255, 255);
    TEST_ASSERT_EQUAL(INT16_MAX, pid.compute(INT16_MAX, 0));
    TEST_ASSERT_EQUAL(INT16_MAX, pid.compute(INT16_MAX, 0));  
    TEST_ASSERT_EQUAL(INT16_MAX, pid.compute(INT16_MAX, 0)); 
    TEST_ASSERT_EQUAL(INT16_MIN, pid.compute(INT16_MIN, 0)); 
    TEST_ASSERT_EQUAL(INT16_MIN, pid.compute(INT16_MIN, 0)); 
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
    RUN_TEST(test_iir_filter);
    RUN_TEST(test_derivative_filter);
    RUN_TEST(test_pid_value_limits);
    return UNITY_END();
}