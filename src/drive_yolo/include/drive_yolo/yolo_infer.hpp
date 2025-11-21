#pragma once
#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <NvOnnxParser.h>

// DriveWorks includes (conditional)
#ifdef USE_DRIVEWORKS
#include <dw/core/base/Version.h>
#include <dw/core/context/Context.h>
#include <dw/core/logger/Logger.h>
#include <dw/sensors/Sensors.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/image/Image.h>
#endif

struct Detection {
    float x, y, width, height;  // Bounding box
    float confidence;           // Detection confidence
    int class_id;              // Class ID
};

class YoloInfer {
public:
    explicit YoloInfer(const std::string &model_path);
    ~YoloInfer();
    
    // Disable copy constructor and assignment
    YoloInfer(const YoloInfer&) = delete;
    YoloInfer& operator=(const YoloInfer&) = delete;
    
    // Main inference method
    std::vector<Detection> detect(const cv::Mat& image, float conf_threshold = 0.5f, float nms_threshold = 0.4f);
    
    // Get model info
    void printModelInfo() const;
    
private:
    class Logger : public nvinfer1::ILogger {
    public:
        void log(Severity severity, const char* msg) noexcept override {
            if (severity <= Severity::kWARNING) {
                std::cout << "[TensorRT] " << msg << std::endl;
            }
        }
    };
    
    Logger logger_;
    std::unique_ptr<nvinfer1::ICudaEngine, void(*)(nvinfer1::ICudaEngine*)> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext, void(*)(nvinfer1::IExecutionContext*)> context_;
    int original_image_width_;
    int original_image_height_;
    // Model dimensions
    int input_width_;
    int input_height_;
    int input_channels_;
    size_t input_size_;
    size_t output_size_;
    int num_classes_;
    // GPU memory
    void* input_device_buffer_;
    void* output_device_buffer_;
    void* host_input_buffer_;
    void* host_output_buffer_;
    
    void loadModel(const std::string &model_path);
    void loadEngineFile(const std::string &engine_path);
    void loadOnnxFile(const std::string &onnx_path);
    void allocateBuffers();
    void preprocessImage(const cv::Mat& image, float* input_data);
    std::vector<Detection> postprocessOutput(float* output_data, float conf_threshold, float nms_threshold);
    std::vector<Detection> fastNMS(const std::vector<Detection>& detections, float nms_threshold);
    std::vector<Detection> applyNMS(const std::vector<Detection>& detections, float nms_threshold);
    float fastIoU(const Detection& a, const Detection& b);
    float calculateIoU(const Detection& a, const Detection& b);
};

// DriveWorks Camera wrapper (conditional compilation)
#ifdef USE_DRIVEWORKS
class DriveWorksCamera {
public:
    DriveWorksCamera();
    ~DriveWorksCamera();
    
    bool initialize(const std::string& camera_params);
    bool start();
    bool stop();
    bool getNextFrame(cv::Mat& image, uint64_t& timestamp);
    void release();
    
    bool isInitialized() const { return initialized_; }
    
private:
    dwContextHandle_t context_;
    dwSALHandle_t sal_;
    dwSensorHandle_t camera_;
    dwCameraFrameHandle_t frame_;
    dwImageHandle_t image_;
    
    bool initialized_;
    bool started_;
    
    // Image properties
    uint32_t image_width_;
    uint32_t image_height_;
    
    bool initializeDriveWorks();
    void cleanup();
    cv::Mat dwImageToCvMat(dwImageHandle_t dwImage);
};
#endif // USE_DRIVEWORKS