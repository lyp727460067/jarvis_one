#include "opencl_handler.h"
#include "jarvis/utility/tic_toc.h"
namespace jarvis_pic{
OpenCLHandler::OpenCLHandler() {
    initOpenCL("/oem/mowpack/vslam_param/kernels/kernels.cl");
}

OpenCLHandler::~OpenCLHandler() {
    clReleaseKernel(kernelDownSampling);
    clReleaseKernel(kernelCalcDeriv);
    clReleaseKernel(kernelDownSamplingWithBorder);
    clReleaseProgram(program);
    clReleaseCommandQueue(queue);
    clReleaseContext(context);
}

void OpenCLHandler::executeKernel(const cv::Mat &img, int level, std::vector<cv::Mat> &pyramids) {
    // 加载数据
    level_ = level;
    pyramids.resize((level + 1) * 2);
    jarvis::estimator::TicToc timer;
    pyramids[0] = img.clone();
    loadData(img, pyramids);
    double duration = timer.toc();
    std::cout << "load data time: " << duration << " ms " << std::endl;
    size_t localSize[2] = {static_cast<size_t>(20),
                           static_cast<size_t>(17)};

    // 并行计算
    timer.tic();
    clSetKernelArg(kernelCalcDeriv, 0, sizeof(cl_mem), &buffer0);
    clSetKernelArg(kernelCalcDeriv, 1, sizeof(cl_mem), &buffer_deriv_0);
    clSetKernelArg(kernelCalcDeriv, 2, sizeof(int), &(width_[0]));
    clSetKernelArg(kernelCalcDeriv, 3, sizeof(int), &(height_[0]));
    size_t globalSize[2] = {static_cast<size_t>(width_[0]),
                            static_cast<size_t>(height_[0])};
    err = clEnqueueNDRangeKernel(queue, kernelCalcDeriv, 2, nullptr,
                                 globalSize, localSize, 0, nullptr, nullptr);
    CHECK_ERROR(err);

    // err = clFinish(queue);
    // CHECK_ERROR(err);

    if (level_ > 0) {
        clSetKernelArg(kernelDownSampling, 0, sizeof(cl_mem), &buffer0);
        clSetKernelArg(kernelDownSampling, 1, sizeof(cl_mem), &buffer1);
        clSetKernelArg(kernelDownSampling, 2, sizeof(int), &(width_[1]));

        size_t globalSize[2] = {static_cast<size_t>(width_[1]),
                                static_cast<size_t>(height_[1])};
        err = clEnqueueNDRangeKernel(queue, kernelDownSampling, 2, nullptr,
                                     globalSize, localSize, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clFinish(queue);
        CHECK_ERROR(err);

        clSetKernelArg(kernelCalcDeriv, 0, sizeof(cl_mem), &buffer1);
        clSetKernelArg(kernelCalcDeriv, 1, sizeof(cl_mem), &buffer_deriv_1);
        clSetKernelArg(kernelCalcDeriv, 2, sizeof(int), &(width_[1]));
        clSetKernelArg(kernelCalcDeriv, 3, sizeof(int), &(height_[1]));
        err = clEnqueueNDRangeKernel(queue, kernelCalcDeriv, 2, nullptr,
                                     globalSize, localSize, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }

    if (level_ > 1) {
        clSetKernelArg(kernelDownSampling, 0, sizeof(cl_mem), &buffer1);
        clSetKernelArg(kernelDownSampling, 1, sizeof(cl_mem), &buffer2);
        clSetKernelArg(kernelDownSampling, 2, sizeof(int), &(width_[2]));

        size_t globalSize[2] = {static_cast<size_t>(width_[2]),
                                static_cast<size_t>(height_[2])};
        err = clEnqueueNDRangeKernel(queue, kernelDownSampling, 2, nullptr,
                                     globalSize, localSize, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clFinish(queue);
        CHECK_ERROR(err);

        clSetKernelArg(kernelCalcDeriv, 0, sizeof(cl_mem), &buffer2);
        clSetKernelArg(kernelCalcDeriv, 1, sizeof(cl_mem), &buffer_deriv_2);
        clSetKernelArg(kernelCalcDeriv, 2, sizeof(int), &(width_[2]));
        clSetKernelArg(kernelCalcDeriv, 3, sizeof(int), &(height_[2]));
        err = clEnqueueNDRangeKernel(queue, kernelCalcDeriv, 2, nullptr,
                                     globalSize, localSize, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }

    if (level_ > 2) {
        clSetKernelArg(kernelDownSampling, 0, sizeof(cl_mem), &buffer2);
        clSetKernelArg(kernelDownSampling, 1, sizeof(cl_mem), &buffer3);
        clSetKernelArg(kernelDownSampling, 2, sizeof(int), &(width_[3]));

        size_t globalSize[2] = {static_cast<size_t>(width_[3]),
                                static_cast<size_t>(height_[3])};
        err = clEnqueueNDRangeKernel(queue, kernelDownSampling, 2, nullptr,
                                     globalSize, localSize, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clFinish(queue);
        CHECK_ERROR(err);

        clSetKernelArg(kernelCalcDeriv, 0, sizeof(cl_mem), &buffer3);
        clSetKernelArg(kernelCalcDeriv, 1, sizeof(cl_mem), &buffer_deriv_3);
        clSetKernelArg(kernelCalcDeriv, 2, sizeof(int), &(width_[3]));
        clSetKernelArg(kernelCalcDeriv, 3, sizeof(int), &(height_[3]));
        err = clEnqueueNDRangeKernel(queue, kernelCalcDeriv, 2, nullptr,
                                     globalSize, localSize, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }

    if (level_ > 3) {
        clSetKernelArg(kernelDownSampling, 0, sizeof(cl_mem), &buffer3);
        clSetKernelArg(kernelDownSampling, 1, sizeof(cl_mem), &buffer4);
        clSetKernelArg(kernelDownSampling, 2, sizeof(int), &(width_[4]));

        size_t globalSize[2] = {static_cast<size_t>(width_[4]),
                                static_cast<size_t>(height_[4])};
        err = clEnqueueNDRangeKernel(queue, kernelDownSampling, 2, nullptr,
                                     globalSize, localSize, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clFinish(queue);
        CHECK_ERROR(err);

        clSetKernelArg(kernelCalcDeriv, 0, sizeof(cl_mem), &buffer4);
        clSetKernelArg(kernelCalcDeriv, 1, sizeof(cl_mem), &buffer_deriv_4);
        clSetKernelArg(kernelCalcDeriv, 2, sizeof(int), &(width_[4]));
        clSetKernelArg(kernelCalcDeriv, 3, sizeof(int), &(height_[4]));
        err = clEnqueueNDRangeKernel(queue, kernelCalcDeriv, 2, nullptr, globalSize,
                                     localSize, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }
    clFinish(queue);
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

    err = clEnqueueReadBuffer(queue, buffer_deriv_0, CL_TRUE, 0,
                              sizeof(deriv_type) * width_[0] * height_[0] * 2,
                              pyramids[1].data, 0, nullptr, nullptr);
    CHECK_ERROR(err);

    if (level_ > 0) {
        err = clEnqueueReadBuffer(queue, buffer1, CL_TRUE, 0,
                                  sizeof(uchar) * width_[1] * height_[1],
                                  pyramids[2].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clEnqueueReadBuffer(queue, buffer_deriv_1, CL_TRUE, 0,
                                  sizeof(deriv_type) * width_[1] * height_[1] * 2,
                                  pyramids[3].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }

    if (level_ > 1) {
        err = clEnqueueReadBuffer(queue, buffer2, CL_TRUE, 0,
                                  sizeof(uchar) * width_[2] * height_[2],
                                  pyramids[4].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clEnqueueReadBuffer(queue, buffer_deriv_2, CL_TRUE, 0,
                                  sizeof(deriv_type) * width_[2] * height_[2] * 2,
                                  pyramids[5].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }

    if (level_ > 2) {
        err = clEnqueueReadBuffer(queue, buffer3, CL_TRUE, 0,
                                  sizeof(uchar) * width_[3] * height_[3],
                                  pyramids[6].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clEnqueueReadBuffer(queue, buffer_deriv_3, CL_TRUE, 0,
                                  sizeof(deriv_type) * width_[3] * height_[3] * 2,
                                  pyramids[7].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }

    if (level_ > 3) {
        err = clEnqueueReadBuffer(queue, buffer4, CL_TRUE, 0,
                                  sizeof(uchar) * width_[4] * height_[4],
                                  pyramids[8].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);

        err = clEnqueueReadBuffer(queue, buffer_deriv_4, CL_TRUE, 0,
                                  sizeof(deriv_type) * width_[4] * height_[4] * 2,
                                  pyramids[9].data, 0, nullptr, nullptr);
        CHECK_ERROR(err);
    }
    duration = timer.toc();
    std::cout << "get result time: " << duration << std::endl;

    // 释放內存
    timer.tic();
    clReleaseMemObject(buffer0);
    clReleaseMemObject(buffer1);
    clReleaseMemObject(buffer2);
    clReleaseMemObject(buffer3);
    // clReleaseMemObject(buffer4);
    clReleaseMemObject(buffer_deriv_0);
    clReleaseMemObject(buffer_deriv_1);
    clReleaseMemObject(buffer_deriv_2);
    clReleaseMemObject(buffer_deriv_3);
    // clReleaseMemObject(buffer_deriv_4);
    delete[] width_;
    delete[] height_;
    duration = timer.toc();
    std::cout << "release time: " << duration << std::endl;
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
    width_ = new int[level_ + 1];
    height_ = new int[level_ + 1];
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

    buffer0 = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR,
                             sizeof(uchar) * width_[0] * height_[0],
                             pyramids[0].data, &err);
    CHECK_ERROR(err);
    buffer_deriv_0 = clCreateBuffer(
        context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
        sizeof(deriv_type) * width_[0] * height_[0] * 2, nullptr, &err);
    CHECK_ERROR(err);

    if (level_ > 0) {
        buffer1 =
            clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR,
                           sizeof(uchar) * width_[1] * height_[1], nullptr, &err);
        CHECK_ERROR(err);
        buffer_deriv_1 = clCreateBuffer(
            context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
            sizeof(deriv_type) * width_[1] * height_[1] * 2, nullptr, &err);
        CHECK_ERROR(err);
    }

    if (level_ > 1) {
        buffer2 =
            clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR,
                           sizeof(uchar) * width_[2] * height_[2], nullptr, &err);
        CHECK_ERROR(err);
        buffer_deriv_2 = clCreateBuffer(
            context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
            sizeof(deriv_type) * width_[2] * height_[2] * 2, nullptr, &err);
        CHECK_ERROR(err);
    }

    if (level_ > 2) {
        buffer3 =
            clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR,
                           sizeof(uchar) * width_[3] * height_[3], nullptr, &err);
        CHECK_ERROR(err);
        buffer_deriv_3 = clCreateBuffer(
            context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
            sizeof(deriv_type) * width_[3] * height_[3] * 2, nullptr, &err);
        CHECK_ERROR(err);
    }

    if (level_ > 3) {
        buffer4 =
            clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR,
                           sizeof(uchar) * width_[4] * height_[4], nullptr, &err);
        CHECK_ERROR(err);
        buffer_deriv_4 = clCreateBuffer(
            context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR,
            sizeof(deriv_type) * width_[4] * height_[4] * 2, nullptr, &err);
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
    std::string kernelSource = loadKernelSource(kernelFile);
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