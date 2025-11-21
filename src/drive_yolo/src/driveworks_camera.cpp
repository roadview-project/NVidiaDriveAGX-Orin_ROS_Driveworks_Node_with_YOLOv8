#ifdef USE_DRIVEWORKS

#include "drive_yolo/yolo_infer.hpp"
#include <ros/ros.h>
#include <cuda_runtime.h>
#include <cstring>  // for memset

DriveWorksCamera::DriveWorksCamera() 
    : context_(DW_NULL_HANDLE)
    , sal_(DW_NULL_HANDLE)
    , camera_(DW_NULL_HANDLE)
    , frame_(DW_NULL_HANDLE)
    , image_(DW_NULL_HANDLE)
    , initialized_(false)
    , started_(false)
    , image_width_(0)
    , image_height_(0)
{
}

DriveWorksCamera::~DriveWorksCamera() {
    cleanup();
}

bool DriveWorksCamera::initialize(const std::string& camera_params) {
    ROS_INFO("=== ENHANCED DRIVEWORKS CAMERA INITIALIZATION ===");
    ROS_INFO("Camera params: %s", camera_params.c_str());
    
    try {
        if (!initializeDriveWorks()) {
            ROS_ERROR("Failed to initialize DriveWorks context/SAL");
            return false;
        }
        ROS_INFO("✓ DriveWorks context and SAL initialized successfully");
        
        // Parse camera parameters for validation
        ROS_INFO("Parsing camera parameters:");
        std::stringstream ss(camera_params);
        std::string param;
        while (std::getline(ss, param, ',')) {
            ROS_INFO("  - %s", param.c_str());
        }
        
        // Create sensor parameters with detailed logging
        dwSensorParams params{};
        params.protocol = "camera.gmsl";
        params.parameters = camera_params.c_str();
        
        ROS_INFO("Creating camera sensor...");
        ROS_INFO("  Protocol: %s", params.protocol);
        ROS_INFO("  Parameters: %s", params.parameters);
        
        // Try creating the sensor
        dwStatus result = dwSAL_createSensor(&camera_, params, sal_);
        
        // Detailed error reporting
        switch (result) {
            case DW_SUCCESS:
                ROS_INFO("✓ Camera sensor created successfully");
                break;
            case DW_INVALID_ARGUMENT:
                ROS_ERROR("✗ DW_INVALID_ARGUMENT: Invalid camera parameters");
                ROS_ERROR("  Check: camera name, interface, link number");
                return false;
            case DW_INVALID_HANDLE:
                ROS_ERROR("✗ DW_INVALID_HANDLE: Invalid DriveWorks handle");
                ROS_ERROR("  Check: DriveWorks initialization");
                return false;
            case DW_NOT_AVAILABLE:
                ROS_ERROR("✗ DW_NOT_AVAILABLE: Camera not available");
                ROS_ERROR("  Check: camera connection, power, permissions");
                ROS_ERROR("  Check: no other process using camera");
                return false;
            case DW_NOT_SUPPORTED:
                ROS_ERROR("✗ DW_NOT_SUPPORTED: Camera configuration not supported");
                ROS_ERROR("  Try: different output-format or interface");
                return false;
            case DW_INTERNAL_ERROR:
                ROS_ERROR("✗ DW_INTERNAL_ERROR: Internal DriveWorks error");
                ROS_ERROR("  This usually indicates:");
                ROS_ERROR("  1. Camera already in use by another process");
                ROS_ERROR("  2. Hardware initialization failed");
                ROS_ERROR("  3. Incorrect camera module connection");
                ROS_ERROR("  4. Power supply issues");
                ROS_ERROR("  5. Permission issues with /dev/video devices");
                
                // Additional diagnostics
                ROS_ERROR("Diagnostic suggestions:");
                ROS_ERROR("  - Kill any existing camera processes: pkill -f camera");
                ROS_ERROR("  - Check video devices: ls -la /dev/video*");
                ROS_ERROR("  - Try different link number (0 instead of 1)");
                ROS_ERROR("  - Try native output format instead of processed");
                ROS_ERROR("  - Verify camera module physical connection");
                return false;
            default:
                ROS_ERROR("✗ Unknown DriveWorks error: %s (%d)", dwGetStatusName(result), static_cast<int>(result));
                return false;
        }
        
        // Try to get image properties to verify camera is working
        ROS_INFO("Testing camera image properties...");
        dwImageProperties imageProperties;
        
        // Try different output formats
        std::vector<dwCameraOutputType> output_formats = {
            DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8,
            DW_CAMERA_OUTPUT_NATIVE_PROCESSED, 
            DW_CAMERA_OUTPUT_NATIVE_RAW
        };
        
        bool format_found = false;
        for (auto format : output_formats) {
            result = dwSensorCamera_getImageProperties(&imageProperties, format, camera_);
            if (result == DW_SUCCESS) {
                ROS_INFO("✓ Image properties retrieved with format %d", static_cast<int>(format));
                ROS_INFO("  Image size: %dx%d", imageProperties.width, imageProperties.height);
                format_found = true;
                break;
            } else {
                ROS_WARN("  Format %d failed: %s", static_cast<int>(format), dwGetStatusName(result));
            }
        }
        
        if (!format_found) {
            ROS_ERROR("✗ Could not get image properties with any format");
            return false;
        }
        
        image_width_ = imageProperties.width;
        image_height_ = imageProperties.height;
        
        ROS_INFO("✓ Camera initialized successfully: %dx%d", image_width_, image_height_);
        initialized_ = true;
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("✗ Exception during camera initialization: %s", e.what());
        return false;
    } catch (...) {
        ROS_ERROR("✗ Unknown exception during camera initialization");
        return false;
    }
}


bool DriveWorksCamera::initializeDriveWorks() {
    try {
        ROS_INFO("Initializing DriveWorks context...");
        
        // Try different initialization approaches for DRIVE OS 6.0.6
        dwStatus result;
        
        // Approach 1: Minimal initialization
        ROS_INFO("Trying minimal context initialization...");
        result = dwInitialize(&context_, DW_VERSION, nullptr);
        if (result == DW_SUCCESS) {
            ROS_INFO("Minimal DriveWorks context initialized successfully");
        } else {
            ROS_WARN("Minimal initialization failed: %s (%d)", dwGetStatusName(result), static_cast<int>(result));
            
            // Approach 2: With empty context parameters
            ROS_INFO("Trying with empty context parameters...");
            dwContextParameters contextParams;
            memset(&contextParams, 0, sizeof(contextParams));
            
            result = dwInitialize(&context_, DW_VERSION, &contextParams);
            if (result != DW_SUCCESS) {
                ROS_ERROR("All DriveWorks context initialization methods failed: %s (%d)", 
                         dwGetStatusName(result), static_cast<int>(result));
                return false;
            }
            ROS_INFO("Empty params DriveWorks context initialized successfully");
        }
        
        // Initialize SAL (Sensor Abstraction Layer)
        ROS_INFO("Initializing SAL...");
        result = dwSAL_initialize(&sal_, context_);
        if (result != DW_SUCCESS) {
            ROS_ERROR("Failed to initialize SAL: %s (%d)", dwGetStatusName(result), static_cast<int>(result));
            return false;
        }
        ROS_INFO("SAL initialized successfully");
        
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception during DriveWorks initialization: %s", e.what());
        return false;
    } catch (...) {
        ROS_ERROR("Unknown exception during DriveWorks initialization");
        return false;
    }
}

bool DriveWorksCamera::start() {
    if (!initialized_) {
        ROS_ERROR("Camera not initialized");
        return false;
    }
    
    dwStatus result = dwSensor_start(camera_);
    if (result != DW_SUCCESS) {
        ROS_ERROR("Failed to start camera: %s", dwGetStatusName(result));
        return false;
    }
    
    started_ = true;
    ROS_INFO("Camera started successfully");
    return true;
}

bool DriveWorksCamera::stop() {
    if (!started_) return true;
    
    dwStatus result = dwSensor_stop(camera_);
    if (result != DW_SUCCESS) {
        ROS_ERROR("Failed to stop camera: %s", dwGetStatusName(result));
        return false;
    }
    
    started_ = false;
    ROS_INFO("Camera stopped");
    return true;
}

bool DriveWorksCamera::getNextFrame(cv::Mat& image, uint64_t& timestamp) {
    if (!started_) {
        ROS_ERROR("Camera not started");
        return false;
    }
    
    // Get next frame with correct API for 5.10 (3 parameters only)
    dwStatus result = dwSensorCamera_readFrame(&frame_, 33000, camera_); // timeout in microseconds, then camera handle
    if (result != DW_SUCCESS) {
        if (result == DW_TIME_OUT) {
            ROS_WARN_THROTTLE(1, "Camera frame timeout");
        } else {
            ROS_ERROR_THROTTLE(1, "Failed to read camera frame: %s", dwGetStatusName(result));
        }
        return false;
    }
    
    // Get processed image from frame
    result = dwSensorCamera_getImage(&image_, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, frame_);
    if (result != DW_SUCCESS) {
        ROS_ERROR("Failed to get processed image: %s", dwGetStatusName(result));
        dwSensorCamera_returnFrame(&frame_);
        return false;
    }
    
    // Get timestamp with correct type
    dwTime_t dw_timestamp;
    result = dwSensorCamera_getTimestamp(&dw_timestamp, frame_);
    if (result != DW_SUCCESS) {
        ROS_WARN_THROTTLE(5, "Failed to get timestamp: %s", dwGetStatusName(result));
        timestamp = 0;
    } else {
        timestamp = static_cast<uint64_t>(dw_timestamp);
    }
    
    // Convert to OpenCV Mat
    image = dwImageToCvMat(image_);
    
    // Return the frame
    dwSensorCamera_returnFrame(&frame_);
    
    return !image.empty();
}


cv::Mat DriveWorksCamera::dwImageToCvMat(dwImageHandle_t dwImage) {
    dwImageCUDA* cudaImage;
    dwStatus result = dwImage_getCUDA(&cudaImage, dwImage);
    if (result != DW_SUCCESS) {
        ROS_ERROR("Failed to get CUDA image: %s", dwGetStatusName(result));
        return cv::Mat();
    }
    
    // Get image properties to understand the actual format
    dwImageProperties props;
    result = dwImage_getProperties(&props, dwImage);
    if (result != DW_SUCCESS) {
        ROS_ERROR("Failed to get image properties: %s", dwGetStatusName(result));
        return cv::Mat();
    }
    
    //ROS_INFO("CUDA Image properties: %dx%d, pitch=%d, format=%d", 
       //     props.width, props.height, cudaImage->pitch[0], props.format);
    
    // Calculate expected stride vs actual pitch
    int expected_stride = props.width * 4; // 4 bytes for RGBA
    int actual_pitch = cudaImage->pitch[0];
    
    //ROS_INFO("Expected stride: %d, Actual pitch: %d", expected_stride, actual_pitch);
    
    if (actual_pitch == expected_stride) {
        // No padding, can copy directly
        cv::Mat cuda_image(props.height, props.width, CV_8UC4); // RGBA
        
        cudaError_t cuda_result = cudaMemcpy(cuda_image.data, cudaImage->dptr[0], 
                                            props.height * actual_pitch, cudaMemcpyDeviceToHost);
        
        if (cuda_result != cudaSuccess) {
            ROS_ERROR("CUDA memcpy failed: %s", cudaGetErrorString(cuda_result));
            return cv::Mat();
        }
        
        // Convert RGBA to BGR for OpenCV compatibility
        cv::Mat bgr_image;
        cv::cvtColor(cuda_image, bgr_image, cv::COLOR_RGBA2BGR);
        return bgr_image;
        
    } else {
        // Has padding, need to copy row by row to remove padding
        //ROS_INFO("Image has padding, copying row by row");
        
        // Allocate temporary buffer for padded data
        std::vector<uint8_t> padded_data(props.height * actual_pitch);
        
        cudaError_t cuda_result = cudaMemcpy(padded_data.data(), cudaImage->dptr[0], 
                                            props.height * actual_pitch, cudaMemcpyDeviceToHost);
        
        if (cuda_result != cudaSuccess) {
            ROS_ERROR("CUDA memcpy failed: %s", cudaGetErrorString(cuda_result));
            return cv::Mat();
        }
        
        // Create OpenCV matrix and copy row by row without padding
        cv::Mat cuda_image(props.height, props.width, CV_8UC4); // RGBA
        
        for (uint32_t row = 0; row < props.height; ++row) {
            const uint8_t* src_row = padded_data.data() + row * actual_pitch;
            uint8_t* dst_row = cuda_image.ptr<uint8_t>(row);
            memcpy(dst_row, src_row, props.width * 4); // Copy only valid pixels, skip padding
        }
        
        // Convert RGBA to BGR for OpenCV compatibility
        cv::Mat bgr_image;
        cv::cvtColor(cuda_image, bgr_image, cv::COLOR_RGBA2BGR);
        return bgr_image;
    }
}

void DriveWorksCamera::cleanup() {
    if (started_) {
        stop();
    }
    
    if (image_ != DW_NULL_HANDLE) {
        dwImage_destroy(image_);
        image_ = DW_NULL_HANDLE;
    }
    
    if (camera_ != DW_NULL_HANDLE) {
        dwSAL_releaseSensor(camera_);
        camera_ = DW_NULL_HANDLE;
    }
    
    if (sal_ != DW_NULL_HANDLE) {
        dwSAL_release(sal_);
        sal_ = DW_NULL_HANDLE;
    }
    
    if (context_ != DW_NULL_HANDLE) {
        dwRelease(context_);
        context_ = DW_NULL_HANDLE;
    }
    
    initialized_ = false;
    started_ = false;
    
    ROS_INFO("DriveWorks camera cleanup completed");
}

#endif // USE_DRIVEWORKS