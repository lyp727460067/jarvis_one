/*
 * author : zhangxiangtong
 * description : vslam app
 * data : 2024/4/18
 */

#include <memory>
#include "fio/DataCapturer.h"

using namespace VSLAM;

// Main function
int main(int argc, char **argv) 
{

    // 限制核数
    // //绑定cpu核
	// // pid_t selfid = syscall(__NR_gettid);
	// cpu_set_t mask;
	// CPU_ZERO(&mask);
	// CPU_SET(0, &mask);   //id0
	// // CPU_SET(1, &mask);   //id1
	// sched_setaffinity(0, sizeof(mask), &mask);

    // Run
    std::cout << "start run." << std::endl;
    DataCapturer data_capturer;

    data_capturer.Run();

    // Done!
    return EXIT_SUCCESS;
}