#include "opencl_handler.h"
#include "jarvis/utility/tic_toc.h"
namespace jarvis_pic{
namespace {

constexpr char kernels[] = R"(
__kernel void fast_pyra_down(__global const uchar* input, __global uchar* output,
                             const int width) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    int x_row = x + x;
    int y_row = y + y;
    int width2 = width + width;
    int index = y_row * width2 + x_row;

    int sum = (int)(input[index]) + (int)(input[index+1]) +
              (int)(input[index + width2]) + (int)(input[index + width2 +1]);
    sum = sum / 4;

    output[y * width + x] = (uchar)(sum);
}

__kernel void calc_scharr_deriv(__global const uchar* input, __global short* output,
                                const int width, const int height) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    
    int row1 = y * width;
    int row0 = (y > 0) ? (row1 - width) : (row1 + width);
    int row2 = (y < height - 1) ? (row1 + width) : (row1 - width);

    int col0 = (x > 0) ? (x - 1) : 1;
    int col2 = (x < width - 1) ? (x + 1) : (width - 2);

    int deriv_r = (input[row0 + col2] - input[row0 + col0] + input[row2 + col2]
                 - input[row2 + col0]) * 3 + (input[row1 + col2] - input[row1 + col0])
                 * 10;
    int deriv_c = (input[row2 + col2] - input[row0 + col2] + input[row2 + col0]
                 - input[row0 + col0]) * 3 + (input[row2 + x] - input[row0 + x]) * 10;

    output[row1 + row1 + x + x] = (short)deriv_r;
    output[row1 + row1 + x + x + 1] = (short)deriv_c;
}

__kernel void fast_pyra_down_with_border(__global const uchar* input, __global uchar* output,
                                         const int width ,const int height, const int winSize) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    int winSize2 = winSize + winSize;
    int onerow = width + winSize2;
    int onerow2 = onerow + width;

    int col = (x < winSize) ? (winSize2 - x) : ((x >= width + winSize) ? (onerow2 - x) : x);
    int row = (y < winSize) ? (winSize2 - y) : ((y >= height + winSize) ? 
              (height + height + winSize2 - y) : y);

    int index = (row + row - winSize) * onerow2 + col + col - winSize;

    int sum = ((int)(input[index]) + (int)(input[index+1]) +
              (int)(input[index + onerow2]) + (int)(input[index + onerow2 +1])) >> 2;

    output[y * onerow + x] = (uchar)(sum);
}

__kernel void calc_scharr_deriv_with_border(__global const uchar* input,
                                            __global short* output,
                                            const int width, const int height,
                                            const int winSize) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    int winSize2 = winSize + winSize;
    int onerow = width + winSize2;
    int onecol = height + winSize2;

    int col = (x < winSize) ? (winSize2 - x) : ((x >= width + winSize) ?
              (col = onerow + width - x) : x);
    int row = (y < winSize) ? (winSize2 - y) : ((y >= height + winSize) ?
              (row = onecol + height - y) : y);
    
    int row1 = row * onerow;
    int row0 = row1 - onerow;
    int row2 = row1 + onerow;

    int col0 = col - 1;
    int col2 = col + 1;

    // int row1 = row * onerow;
    // int row0 = (row > winSize) ? (row1 - onerow) : (row1 + onerow);
    // int row2 = (row < height + winSize - 1) ? (row1 + onerow) : (row1 - onerow);

    // int col0 = (col > winSize) ? (col - 1) : 1;
    // int col2 = (col < width + winSize - 1) ? (col + 1) : (width + winSize - 2);

    int temp1 = input[row0 + col2] - input[row2 + col0];
    int temp2 = input[row2 + col2] - input[row0 + col0];

    int deriv_r = (temp1 + temp2) * 3 + (input[row1 + col2] - input[row1 + col0]) * 10;
    int deriv_c = (temp2 - temp1) * 3 + (input[row2 + col] - input[row0 + col]) * 10;

    // int deriv_r = (input[row0 + col2] - input[row0 + col0] + input[row2 + col2]
    //              - input[row2 + col0]) * 3 + (input[row1 + col2] - input[row1 + col0])
    //              * 10;
    // int deriv_c = (input[row2 + col2] - input[row0 + col2] + input[row2 + col0]
    //              - input[row0 + col0]) * 3 + (input[row2 + col] - input[row0 + col]) * 10;

    int rowsid = y * onerow;
    output[rowsid + rowsid + x + x] = (short)deriv_r;
    output[rowsid + rowsid + x + x + 1] = (short)deriv_c;    
}
)";
}

OpenCLHandler::OpenCLHandler() { initOpenCL(std::string(kernels)); }

OpenCLHandler::~OpenCLHandler() {
    clReleaseKernel(kernelDownSampling);
    clReleaseKernel(kernelCalcDeriv);
    clReleaseKernel(kernelDownSamplingWithBorder);
    clReleaseProgram(program);
    // clFinish(queue);
    clReleaseCommandQueue(queue);
    clReleaseContext(context);
}

void OpenCLHandler::executeKernel(const cv::Mat &img, int level, std::vector<cv::Mat> &pyramids) {
    // 加载数据
    level_ = level;
    width_ = new int[level_ + 1];
    height_ = new int[level_ + 1];
    buffers = new cl_mem[level_ + 1];
    buffers_deriv = new cl_mem[level_ + 1];

    // jarvis::estimator::TicToc timer;
    loadData(img, pyramids);
    // double duration = timer.toc();
    // std::cout << "load data time: " << duration << " ms " << std::endl;


    // 并行计算
    // timer.tic();
    size_t localSize[2] = {static_cast<size_t>(20),
                           static_cast<size_t>(17)};
    clSetKernelArg(kernelCalcDeriv, 0, sizeof(cl_mem), &(buffers[0]));
    clSetKernelArg(kernelCalcDeriv, 1, sizeof(cl_mem), &(buffers_deriv[0]));
    clSetKernelArg(kernelCalcDeriv, 2, sizeof(int), &(width_[0]));
    clSetKernelArg(kernelCalcDeriv, 3, sizeof(int), &(height_[0]));
    size_t globalSize[2] = {static_cast<size_t>(width_[0]),
                            static_cast<size_t>(height_[0])};
    err = clEnqueueNDRangeKernel(queue, kernelCalcDeriv, 2, nullptr,
                                 globalSize, localSize, 0, nullptr, nullptr);
    CHECK_ERROR(err);

    for (int i = 1; i <= level_; i++) {
        clSetKernelArg(kernelDownSampling, 0, sizeof(cl_mem), &(buffers[i - 1]));
        clSetKernelArg(kernelDownSampling, 1, sizeof(cl_mem), &(buffers[i]));
        clSetKernelArg(kernelDownSampling, 2, sizeof(int), &(width_[i]));
        size_t globalSize[2] = {static_cast<size_t>(width_[i]),
                                static_cast<size_t>(height_[i])};
        err = clEnqueueNDRangeKernel(queue, kernelDownSampling, 2, nullptr,
                                     globalSize, localSize, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clFinish(queue);
        CHECK_ERROR(err);

        clSetKernelArg(kernelCalcDeriv, 0, sizeof(cl_mem), &(buffers[i]));
        clSetKernelArg(kernelCalcDeriv, 1, sizeof(cl_mem), &(buffers_deriv[i]));
        clSetKernelArg(kernelCalcDeriv, 2, sizeof(int), &(width_[i]));
        clSetKernelArg(kernelCalcDeriv, 3, sizeof(int), &(height_[i]));
        err = clEnqueueNDRangeKernel(queue, kernelCalcDeriv, 2, nullptr,
                                     globalSize, localSize, 0, nullptr, nullptr);
    }
    err = clFinish(queue);
    CHECK_ERROR(err);
    // duration = timer.toc();
    // std::cout << "cal time: " << duration << std::endl;



    // 读取数据
    // cl_int clEnqueueReadBuffer(
    //     cl_command_queue command_queue, cl_mem buffer, cl_bool
    //     blocking_read, size_t offset, size_t cb, void *ptr, cl_uint
    //     num_events_in_wait_list, const cl_event *event_wait_list, cl_event
    //     *event);
    // buffer：要读取数据的OpenCL缓冲区对象
    // blocking_read：一个布尔值，指示读取操作是否应阻塞直到数据被读取。
    // ptr：一个指向主机内存区域的指针，数据将被读取到这个内存区域。
    // timer.tic();

    err = clEnqueueReadBuffer(queue, buffers_deriv[0], CL_TRUE, 0,
                              sizeof(deriv_type) * width_[0] * height_[0] * 2,
                              pyramids[1].data, 0, nullptr, nullptr);
    CHECK_ERROR(err);

    for (int i = 1; i <= level_; i++) {
        err = clEnqueueReadBuffer(queue, buffers[i], CL_TRUE, 0,
                                  sizeof(uchar) * width_[i] * height_[i],
                                  pyramids[i * 2].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clEnqueueReadBuffer(queue, buffers_deriv[i], CL_TRUE, 0,
                                  sizeof(deriv_type) * width_[i] * height_[i] * 2,
                                  pyramids[i * 2 + 1].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }
    // duration = timer.toc();
    // std::cout << "get result time: " << duration << std::endl;

    // 释放內存
    // timer.tic();
    for (int i = 0; i <= level_; i++) {
        clReleaseMemObject(buffers[i]);
        clReleaseMemObject(buffers_deriv[i]);
    }
    delete[] width_;
    delete[] height_;
    delete[] buffers;
    delete[] buffers_deriv;
    // duration = timer.toc();
    // std::cout << "release time: " << duration << std::endl;
}

void OpenCLHandler::executeKernelWithBorder(const cv::Mat &img, int level,
                                            std::vector<cv::Mat> &pyramids, cv::Size winSize) {
    // 加载数据
    level_ = level;
    winSize_ = winSize;
    width_ = new int[level_ + 1];
    height_ = new int[level_ + 1];
    width_with_border_ = new int[level_ + 1];
    height_with_border_ = new int[level_ + 1];
    buffers = new cl_mem[level_ + 1];
    buffers_deriv = new cl_mem[level_ + 1];
    jarvis::estimator::TicToc timer;
    loadDataWithBorder(img, pyramids);
    double duration = timer.toc();
    std::cout << "load data time: " << duration << " ms " << std::endl;

    // 并行计算
    timer.tic();
    clSetKernelArg(kernelCalcDerivWithBorder, 0, sizeof(cl_mem), &(buffers[0]));
    clSetKernelArg(kernelCalcDerivWithBorder, 1, sizeof(cl_mem), &(buffers_deriv[0]));
    clSetKernelArg(kernelCalcDerivWithBorder, 2, sizeof(int), &(width_[0]));
    clSetKernelArg(kernelCalcDerivWithBorder, 3, sizeof(int), &(height_[0]));
    clSetKernelArg(kernelCalcDerivWithBorder, 4, sizeof(int), &(winSize_.height));
    size_t globalSize[2] = {static_cast<size_t>(width_with_border_[0]),
                            static_cast<size_t>(height_with_border_[0])};
    err = clEnqueueNDRangeKernel(queue, kernelCalcDerivWithBorder, 2, nullptr,
                                 globalSize, nullptr, 0, nullptr, nullptr);
    CHECK_ERROR(err);
    for (int i = 1; i <= level_; i++) {
        clSetKernelArg(kernelDownSamplingWithBorder, 0, sizeof(cl_mem), &(buffers[i - 1]));
        clSetKernelArg(kernelDownSamplingWithBorder, 1, sizeof(cl_mem), &(buffers[i]));
        clSetKernelArg(kernelDownSamplingWithBorder, 2, sizeof(int), &(width_[i]));
        clSetKernelArg(kernelDownSamplingWithBorder, 3, sizeof(int), &(height_[i]));
        clSetKernelArg(kernelDownSamplingWithBorder, 4, sizeof(int), &(winSize_.height));
        size_t globalSize[2] = {static_cast<size_t>(width_with_border_[i]),
                                static_cast<size_t>(height_with_border_[i])};
        err = clEnqueueNDRangeKernel(queue, kernelDownSamplingWithBorder, 2, nullptr,
                                     globalSize, nullptr, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clFinish(queue);
        CHECK_ERROR(err);

        clSetKernelArg(kernelCalcDerivWithBorder, 0, sizeof(cl_mem), &(buffers[i]));
        clSetKernelArg(kernelCalcDerivWithBorder, 1, sizeof(cl_mem), &(buffers_deriv[i]));
        clSetKernelArg(kernelCalcDerivWithBorder, 2, sizeof(int), &(width_[i]));
        clSetKernelArg(kernelCalcDerivWithBorder, 3, sizeof(int), &(height_[i]));
        clSetKernelArg(kernelCalcDerivWithBorder, 4, sizeof(int), &(winSize_.height));
        err = clEnqueueNDRangeKernel(queue, kernelCalcDerivWithBorder, 2, nullptr,
                                     globalSize, nullptr, 0, nullptr, nullptr);
    }
    err = clFinish(queue);
    CHECK_ERROR(err);
    duration = timer.toc();
    std::cout << "cal time: " << duration << std::endl;

    // cl_int clEnqueueReadBuffer(
    //     cl_command_queue command_queue, cl_mem buffer, cl_bool
    //     blocking_read, size_t offset, size_t cb, void *ptr, cl_uint
    //     num_events_in_wait_list, const cl_event *event_wait_list, cl_event
    //     *event);
    // buffer：要读取数据的OpenCL缓冲区对象
    // blocking_read：一个布尔值，指示读取操作是否应阻塞直到数据被读取。
    // ptr：一个指向主机内存区域的指针，数据将被读取到这个内存区域。
    timer.tic();

    err = clEnqueueReadBuffer(queue, buffers_deriv[0], CL_TRUE, 0,
                              sizeof(deriv_type) * width_with_border_[0] * height_with_border_[0] * 2,
                              pyramids[1].data, 0, nullptr, nullptr);
    CHECK_ERROR(err);
    
    for (int i = 1; i <= level_; i++) {
        err = clEnqueueReadBuffer(queue, buffers[i], CL_TRUE, 0,
                                  sizeof(uchar) * width_with_border_[i] * height_with_border_[i],
                                  pyramids[i * 2].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clEnqueueReadBuffer(queue, buffers_deriv[i], CL_TRUE, 0,
                                  sizeof(deriv_type) * width_with_border_[i] * height_with_border_[i] * 2,
                                  pyramids[i * 2 + 1].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }
    duration = timer.toc();
    std::cout << "get result time: " << duration << std::endl;

    // 释放內存
    timer.tic();
    for (int i = 0; i <= level_; i++) {
        clReleaseMemObject(buffers[i]);
        clReleaseMemObject(buffers_deriv[i]);
    }
    delete[] width_;
    delete[] height_;
    delete[] width_with_border_;
    delete[] height_with_border_;
    delete[] buffers;
    delete[] buffers_deriv;
    duration = timer.toc();
    std::cout << "release time: " << duration << std::endl;
}

void OpenCLHandler::loadData(const cv::Mat &img, std::vector<cv::Mat> &pyramids) {
    // Create buffers
    pyramids.resize((level_ + 1) * 2);
    pyramids[0] = img.clone();
    cv::Size sz = img.size();

    for (size_t i = 0; i < pyramids.size(); i++) {
        if (i % 2 == 0) {
            width_[i / 2] = sz.width;
            height_[i / 2] = sz.height;
            if (i != 0) {
                cv::Mat img_i(sz, img.type());
                pyramids[i] = img_i;
            }
        } else {
            cv::Mat deriv_i(sz, CV_16SC2);
            pyramids[i] = deriv_i;
            sz = cv::Size(sz.width / 2, sz.height / 2);
        }
    }

    buffers[0] = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
                             sizeof(uchar) * width_[0] * height_[0],
                             pyramids[0].data, &err);
    CHECK_ERROR(err);
    buffers_deriv[0] = clCreateBuffer(
        context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
        sizeof(deriv_type) * width_[0] * height_[0] * 2, nullptr, &err);
    CHECK_ERROR(err);

    for (int i = 1; i <= level_; i++) {
        buffers[i] = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR,
                                    sizeof(uchar) * width_[i] * height_[i],
                                    nullptr, &err);
        CHECK_ERROR(err);
        buffers_deriv[i] = clCreateBuffer(
            context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
            sizeof(deriv_type) * width_[i] * height_[i] * 2, nullptr, &err);
        CHECK_ERROR(err);
    }
}

void OpenCLHandler::loadDataWithBorder(const cv::Mat &img, std::vector<cv::Mat> &pyramids) {
    // Create buffers
    cv::Size sz = img.size();
    pyramids.resize((level_ + 1) * 2);
    pyramids[0].create(img.rows + winSize_.height * 2, img.cols + winSize_.width * 2, img.type());
    cv::copyMakeBorder(img, pyramids[0], winSize_.height, winSize_.height, winSize_.width,
                       winSize_.width, 4);

    for (size_t i = 0; i < pyramids.size(); i++) {
        if (i % 2 == 0) {
            width_[i / 2] = sz.width;
            height_[i / 2] = sz.height;
            width_with_border_[i / 2] = sz.width + winSize_.width * 2;
            height_with_border_[i / 2] = sz.height + winSize_.height * 2;
            if (i != 0) {
                cv::Mat img_i(height_with_border_[i / 2], width_with_border_[i / 2], img.type());
                pyramids[i] = img_i;
            }
        } else {
            cv::Mat deriv_i(height_with_border_[i / 2], width_with_border_[i / 2], CV_16SC2);
            pyramids[i] = deriv_i;
            sz = cv::Size(sz.width / 2, sz.height / 2);
        }
    }

    // for (int i = 0; i <= level_; i++) {
    //     std::cout << width_[i] << " / " << height_[i] << std::endl;
    //     std::cout << width_with_border_[i] << " / " << height_with_border_[i] << std::endl
    //               << std::endl;
    // }

    buffers[0] = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
                                sizeof(uchar) * width_with_border_[0] * height_with_border_[0],
                                pyramids[0].data, &err);
    CHECK_ERROR(err);
    buffers_deriv[0] = clCreateBuffer(
        context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
        sizeof(deriv_type) * width_with_border_[0] * height_with_border_[0] * 2, nullptr, &err);
    CHECK_ERROR(err);

    for (int i = 1; i <= level_; i++) {
        buffers[i] = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR,
                                    sizeof(uchar) * width_with_border_[i] * height_with_border_[i],
                                    nullptr, &err);
        CHECK_ERROR(err);
        buffers_deriv[i] = clCreateBuffer(
            context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
            sizeof(deriv_type) * width_with_border_[i] * height_with_border_[i] * 2, nullptr, &err);
        CHECK_ERROR(err);
    }
}

std::string OpenCLHandler::loadKernelSource(const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open kernel source file.");
    }
    std::stringstream source;
    source << file.rdbuf();
    return source.str();
}

void OpenCLHandler::printPlatformInfo(cl_platform_id platform) {
    char buffer[1024];
    clGetPlatformInfo(platform, CL_PLATFORM_NAME, sizeof(buffer), buffer,
                      nullptr);
    std::cout << "Platform Name: " << buffer << "\n";
    clGetPlatformInfo(platform, CL_PLATFORM_VENDOR, sizeof(buffer), buffer,
                      nullptr);
    std::cout << "Platform Vendor: " << buffer << "\n";
    clGetPlatformInfo(platform, CL_PLATFORM_VERSION, sizeof(buffer), buffer,
                      nullptr);
    std::cout << "Platform Version: " << buffer << "\n";
    clGetPlatformInfo(platform, CL_PLATFORM_PROFILE, sizeof(buffer), buffer,
                      nullptr);
    std::cout << "Platform Profile: " << buffer << "\n";
}

void OpenCLHandler::printDeviceInfo(cl_device_id device) {
    char buffer[1024];
    clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(buffer), buffer, nullptr);
    std::cout << "Device Name: " << buffer << "\n";
    clGetDeviceInfo(device, CL_DEVICE_VENDOR, sizeof(buffer), buffer, nullptr);
    std::cout << "Device Vendor: " << buffer << "\n";
    clGetDeviceInfo(device, CL_DEVICE_VERSION, sizeof(buffer), buffer, nullptr);
    std::cout << "Device Version: " << buffer << "\n";
    clGetDeviceInfo(device, CL_DRIVER_VERSION, sizeof(buffer), buffer, nullptr);
    std::cout << "Driver Version: " << buffer << "\n";
    clGetDeviceInfo(device, CL_DEVICE_OPENCL_C_VERSION, sizeof(buffer), buffer,
                    nullptr);
    std::cout << "OpenCL C Version: " << buffer << "\n";

    cl_uint computeUnits;
    clGetDeviceInfo(device, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(computeUnits),
                    &computeUnits, nullptr);
    std::cout << "Max Compute Units: " << computeUnits << "\n";

    size_t workGroupSize;
    clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(workGroupSize),
                    &workGroupSize, nullptr);
    std::cout << "Max Work Group Size: " << workGroupSize << "\n";

    cl_uint maxDimensions;
    std::vector<size_t> maxWorkItemSizes;

    // 查询设备支持的最大工作项尺寸
    clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS,
                    sizeof(maxDimensions), &maxDimensions, nullptr);

    maxWorkItemSizes.resize(maxDimensions);
    clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_ITEM_SIZES,
                    maxDimensions * sizeof(size_t), maxWorkItemSizes.data(),
                    nullptr);

    // 输出每个维度上的最大工作项数量
    std::cout << "Max Work Item Sizes:";
    for (cl_uint i = 0; i < maxDimensions; ++i) {
        std::cout << " " << maxWorkItemSizes[i];
    }
    std::cout << "\n";

    cl_ulong globalMemSize;
    clGetDeviceInfo(device, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(globalMemSize),
                    &globalMemSize, nullptr);
    std::cout << "Global Memory Size: " << globalMemSize / (1024 * 1024)
              << " MB\n";

    cl_ulong localMemSize;
    clGetDeviceInfo(device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(localMemSize),
                    &localMemSize, nullptr);
    std::cout << "Local Memory Size: " << localMemSize / 1024 << " KB\n";
}

void OpenCLHandler::initOpenCL(const std::string &kernelFile) {
    // Get platforms and devices
    cl_uint numPlatforms;
    clGetPlatformIDs(0, nullptr, &numPlatforms);
    std::vector<cl_platform_id> platforms(numPlatforms);
    clGetPlatformIDs(numPlatforms, platforms.data(), nullptr);

    if (platforms.empty()) {
        throw std::runtime_error("No OpenCL platforms found.");
    }

    for (const auto &platform : platforms) {
        // printPlatformInfo(platform);
        // std::cout << "\n";

        cl_uint numDevices;
        clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 0, nullptr, &numDevices);
        std::vector<cl_device_id> devices(numDevices);
        clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, numDevices, devices.data(),
                       nullptr);

        if (devices.empty()) {
            std::cerr << "No OpenCL devices found.\n";
            continue;
        }

        // for (const auto &device : devices) {
        //   printDeviceInfo(device);
        //   std::cout << "\n";
        // }
    }

    cl_platform_id platform = platforms[0];
    cl_uint numDevices;
    clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 0, nullptr, &numDevices);
    std::vector<cl_device_id> devices(numDevices);
    clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, numDevices, devices.data(),
                   nullptr);

    if (devices.empty()) {
        throw std::runtime_error("No OpenCL devices found.");
    }
    cl_device_id device = devices[0];

    // Create context and command queue
    /* cl_context clCreateContext(
            const cl_context_properties *properties,
       一组用于指定上下文属性的属性/值对 cl_uint num_devices, 指定 devices
       数组中设备的数量 const cl_device_id *devices,
       指向设备ID数组的指针，这些设备ID用于标识要在上下文中包含的设备 void
       (*pfn_notify)(const char *, const void *, size_t, void *),
       指向一个回调函数的指针，OpenCL可以用来报告运行时错误 void *user_data,
       指向用户数据的指针，可以传递给回调函数 cl_int *errcode_ret)
       返回错误码的指针  */
    context = clCreateContext(nullptr, 1, &device, nullptr, nullptr, nullptr);

    /* cl_int clCreateCommandQueueWithProperties(
            cl_context context,
            const cl_device_id* devices,
            const cl_command_queue_properties* properties,
            cl_command_queue* command_queue,
            cl_int* errcode_ret);  用于接收操作的状态  */
    queue = clCreateCommandQueueWithProperties(context, device, 0, &err);
    CHECK_ERROR(err);

    // Load and build program
    // std::string kernelSource = loadKernelSource(kernelFile);
    std::string kernelSource =(kernelFile);
    const char *source = kernelSource.c_str();
    size_t sourceSize = kernelSource.length();
    // Use the clCreateProgramWithBinary() function to do this.
    // Use the clGetProgramInfo() function to obtain the binary after you have
    // generated it
    program =
        clCreateProgramWithSource(context, 1, &source, &sourceSize, nullptr);
    cl_int err =
        clBuildProgram(program, 1, &device, "-cl-std=CL3.0", nullptr, nullptr);
    if (err != CL_SUCCESS) {
        size_t logSize;
        clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, nullptr,
                              &logSize);
        std::vector<char> log(logSize);
        clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, logSize,
                              log.data(), nullptr);
        throw std::runtime_error("Error during program build: " +
                                 std::string(log.data()));
    }

    // Create kernels
    kernelDownSampling = clCreateKernel(program, "fast_pyra_down", nullptr);
    kernelDownSamplingWithBorder = clCreateKernel(program, "fast_pyra_down_with_border",
                                                  nullptr);
    kernelCalcDeriv = clCreateKernel(program, "calc_scharr_deriv", nullptr);
    kernelCalcDerivWithBorder = clCreateKernel(program, "calc_scharr_deriv_with_border", nullptr);
}
}