#include "drive_yolo/yolo_infer.hpp"
#include <iostream>
#include <stdexcept>
#include <ros/ros.h>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <cuda_runtime.h>

YoloInfer::YoloInfer(const std::string &model_path) 
    : engine_(nullptr, [](nvinfer1::ICudaEngine* p) { if(p) p->destroy(); })
    , context_(nullptr, [](nvinfer1::IExecutionContext* p) { if(p) p->destroy(); })
    , input_device_buffer_(nullptr)
    , output_device_buffer_(nullptr)
    , host_input_buffer_(nullptr)
    , host_output_buffer_(nullptr)
{
    ROS_INFO("Loading YOLO model from: %s", model_path.c_str());
    loadModel(model_path);
    allocateBuffers();
    ROS_INFO("YOLO model loaded successfully");
}

YoloInfer::~YoloInfer() {
    // Free CUDA memory
    if (input_device_buffer_) cudaFree(input_device_buffer_);
    if (output_device_buffer_) cudaFree(output_device_buffer_);
    if (host_input_buffer_) free(host_input_buffer_);
    if (host_output_buffer_) free(host_output_buffer_);
}

void YoloInfer::loadModel(const std::string &model_path) {
    // Check if it's a .engine file (pre-compiled TensorRT)
    if (model_path.find(".engine") != std::string::npos) {
        ROS_INFO("Loading pre-compiled TensorRT engine");
        loadEngineFile(model_path);
        printModelInfo();
    } else if (model_path.find(".onnx") != std::string::npos) {
        ROS_INFO("Converting ONNX to TensorRT (this may take a while...)");
        loadOnnxFile(model_path);
    } else {
        throw std::runtime_error("Unsupported model format. Use .engine or .onnx files");
    }
}

void YoloInfer::loadEngineFile(const std::string &engine_path) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        throw std::runtime_error("Failed to open engine file: " + engine_path);
    }
    
    // Get file size
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    // Read engine data
    std::vector<char> engine_data(size);
    file.read(engine_data.data(), size);
    file.close();
    
    // Create TensorRT runtime
    std::unique_ptr<nvinfer1::IRuntime, void(*)(nvinfer1::IRuntime*)> 
        runtime(nvinfer1::createInferRuntime(logger_), 
                [](nvinfer1::IRuntime* p) { if(p) p->destroy(); });
    
    if (!runtime) {
        throw std::runtime_error("Failed to create TensorRT runtime");
    }
    
    // Deserialize engine
    engine_.reset(runtime->deserializeCudaEngine(engine_data.data(), size));
    if (!engine_) {
        throw std::runtime_error("Failed to deserialize TensorRT engine");
    }
    
    // Create execution context
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
        throw std::runtime_error("Failed to create execution context");
    }
    
    ROS_INFO("TensorRT engine loaded successfully with %d layers", engine_->getNbLayers());
}

void YoloInfer::loadOnnxFile(const std::string &onnx_path) {
    // Create builder
    std::unique_ptr<nvinfer1::IBuilder, void(*)(nvinfer1::IBuilder*)> 
        builder(nvinfer1::createInferBuilder(logger_), 
                [](nvinfer1::IBuilder* p) { if(p) p->destroy(); });
    
    if (!builder) {
        throw std::runtime_error("Failed to create TensorRT builder");
    }

    // Create network
    const auto explicitBatch = 1U << static_cast<uint32_t>(
        nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    
    std::unique_ptr<nvinfer1::INetworkDefinition, void(*)(nvinfer1::INetworkDefinition*)>
        network(builder->createNetworkV2(explicitBatch),
                [](nvinfer1::INetworkDefinition* p) { if(p) p->destroy(); });
    
    if (!network) {
        throw std::runtime_error("Failed to create TensorRT network");
    }

    // Create parser
    std::unique_ptr<nvonnxparser::IParser, void(*)(nvonnxparser::IParser*)>
        parser(nvonnxparser::createParser(*network, logger_),
               [](nvonnxparser::IParser* p) { if(p) p->destroy(); });
    
    if (!parser) {
        throw std::runtime_error("Failed to create ONNX parser");
    }

    // Parse ONNX file
    ROS_INFO("Parsing ONNX file...");
    if (!parser->parseFromFile(onnx_path.c_str(), 
                              static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        throw std::runtime_error("Failed to parse ONNX file: " + onnx_path);
    }

    // Create builder config
    std::unique_ptr<nvinfer1::IBuilderConfig, void(*)(nvinfer1::IBuilderConfig*)>
        config(builder->createBuilderConfig(),
               [](nvinfer1::IBuilderConfig* p) { if(p) p->destroy(); });
    
    if (!config) {
        throw std::runtime_error("Failed to create builder config");
    }

    // Set max workspace size (1GB)
    config->setMaxWorkspaceSize(1ULL << 30);
    
    // Enable FP16 if available
    if (builder->platformHasFastFp16()) {
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
        ROS_INFO("FP16 mode enabled");
    }

    // Build engine (this is the slow part)
    ROS_INFO("Building TensorRT engine... This may take several minutes");
    engine_.reset(builder->buildEngineWithConfig(*network, *config));
    if (!engine_) {
        throw std::runtime_error("Failed to build TensorRT engine");
    }

    // Create execution context
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
        throw std::runtime_error("Failed to create execution context");
    }

    ROS_INFO("TensorRT engine built successfully with %d layers", engine_->getNbLayers());
}
void YoloInfer::allocateBuffers() {
    // Get binding dimensions
    auto input_dims = engine_->getBindingDimensions(0);
    auto output_dims = engine_->getBindingDimensions(1);
    
    // Input dimensions [batch, channels, height, width]
    input_channels_ = input_dims.d[1];
    input_height_ = input_dims.d[2];
    input_width_ = input_dims.d[3];
    input_size_ = input_channels_ * input_height_ * input_width_;
    
    // Output dimensions [batch, features, detections]
    // features = 4 (bbox) + num_classes
    int num_features = output_dims.d[1];
    int num_detections = output_dims.d[2];
    
    // AUTO-DETECT number of classes from output shape!
    num_classes_ = num_features - 4;  // Subtract 4 bbox coordinates
    
    output_size_ = num_features * num_detections;
    
    ROS_INFO("Model Configuration:");
    ROS_INFO("  Input: %dx%dx%d", input_width_, input_height_, input_channels_);
    ROS_INFO("  Output: [%d, %d, %d]", output_dims.d[0], num_features, num_detections);
    ROS_INFO("  Detected %d classes (features=%d - 4 bbox)", num_classes_, num_features);
    
    if (num_classes_ == 1) {
        ROS_INFO("  -> Single-class detection model (probably cars only)");
    } else if (num_classes_ == 80) {
        ROS_INFO("  -> COCO 80-class detection model");
    }
    
    // Allocate GPU memory
    cudaMalloc(&input_device_buffer_, input_size_ * sizeof(float));
    cudaMalloc(&output_device_buffer_, output_size_ * sizeof(float));
    
    // Allocate CPU memory
    host_input_buffer_ = malloc(input_size_ * sizeof(float));
    host_output_buffer_ = malloc(output_size_ * sizeof(float));
    
    if (!input_device_buffer_ || !output_device_buffer_ || 
        !host_input_buffer_ || !host_output_buffer_) {
        throw std::runtime_error("Failed to allocate memory for inference");
    }
}

std::vector<Detection> YoloInfer::detect(const cv::Mat& image, float conf_threshold, float nms_threshold) {
    // Store original image dimensions
    original_image_width_ = image.cols;
    original_image_height_ = image.rows;
    
    // Preprocess image
    preprocessImage(image, static_cast<float*>(host_input_buffer_));
    
    // Copy input to GPU
    cudaMemcpy(input_device_buffer_, host_input_buffer_, input_size_ * sizeof(float), cudaMemcpyHostToDevice);
    
    // Run inference
    void* bindings[] = {input_device_buffer_, output_device_buffer_};
    bool success = context_->executeV2(bindings);
    
    if (!success) {
        ROS_ERROR("TensorRT inference failed");
        return {};
    }
    
    // Copy output back to CPU
    cudaMemcpy(host_output_buffer_, output_device_buffer_, output_size_ * sizeof(float), cudaMemcpyDeviceToHost);
    // Print class scores for first detection
    float max_score = 0;
    int best_class = -1;
    for (int i = 0; i < 80; i++) {
        float score = static_cast<float*>(host_output_buffer_)[(4+i) * 8400 + 0];
        if (score > max_score) {
            max_score = score;
            best_class = i;
        }
    }
    
    // Post-process results (now with original image dimensions)
    return postprocessOutput(static_cast<float*>(host_output_buffer_), conf_threshold, nms_threshold);
}
void YoloInfer::preprocessImage(const cv::Mat& image, float* input_data) {
    // Store original dimensions for scaling calculations
    original_image_width_ = image.cols;
    original_image_height_ = image.rows;
    
    // Method 1: Simple resize (matches your Python approach initially)
    // This eliminates padding-related coordinate issues
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(input_width_, input_height_));
    
    // Convert BGR to RGB and normalize
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
    
    // Convert to float and normalize to [0,1]
    cv::Mat normalized;
    rgb.convertTo(normalized, CV_32F, 1.0/255.0);
    
    // Convert from HWC to CHW format for TensorRT
    std::vector<cv::Mat> channels(3);
    cv::split(normalized, channels);
    
    // Copy to input buffer in CHW format
    for (int c = 0; c < 3; ++c) {
        memcpy(input_data + c * input_width_ * input_height_, 
               channels[c].data, 
               input_width_ * input_height_ * sizeof(float));
    }
}

// For YOLOv8, modify the postprocessOutput function:

std::vector<Detection> YoloInfer::postprocessOutput(float* output_data, 
                                                     float conf_threshold, 
                                                     float nms_threshold) {
    std::vector<Detection> detections;
    detections.reserve(100);
    
    // These values are determined by YOLO architecture
    int num_detections = 8400;  // Fixed for YOLOv8/v11 architecture
    int num_features = 4 + num_classes_;  // 4 bbox + N classes (auto-detected)
    
    // Calculate scaling factors from model input to original image
    float scale_x = static_cast<float>(original_image_width_) / static_cast<float>(input_width_);
    float scale_y = static_cast<float>(original_image_height_) / static_cast<float>(input_height_);
    
    // First pass: find detections above threshold
    for (int det_idx = 0; det_idx < num_detections; ++det_idx) {
        // Extract class scores for this detection
        float max_score = 0.0f;
        int best_class = -1;
        
        // Look through all class scores for this detection
        for (int class_idx = 0; class_idx < num_classes_; ++class_idx) {
            // CRITICAL: Access pattern for [features, detections] layout
            int score_index = (4 + class_idx) * num_detections + det_idx;
            float score = output_data[score_index];
            
            if (score > max_score) {
                max_score = score;
                best_class = class_idx;
            }
        }
        
        // Skip low confidence detections
        if (max_score <= conf_threshold) continue;
        
        // Extract bounding box coordinates for this detection
        float center_x = output_data[0 * num_detections + det_idx];  // x_center
        float center_y = output_data[1 * num_detections + det_idx];  // y_center  
        float width = output_data[2 * num_detections + det_idx];     // width
        float height = output_data[3 * num_detections + det_idx];    // height
        
        // Scale from model input coordinates to original image coordinates
        float scaled_x = center_x * scale_x;
        float scaled_y = center_y * scale_y;
        float scaled_width = width * scale_x;
        float scaled_height = height * scale_y;
        
        // Skip tiny boxes
        if (scaled_width < 5 || scaled_height < 5) {
            continue;
        }
        
        // Basic boundary check (allow some margin for partially visible objects)
        if (scaled_x < -original_image_width_*0.2 || scaled_x > original_image_width_*1.2 ||
            scaled_y < -original_image_height_*0.2 || scaled_y > original_image_height_*1.2) {
            continue;
        }
        
        // Create detection
        Detection det;
        det.x = scaled_x;
        det.y = scaled_y;
        det.width = scaled_width;
        det.height = scaled_height;
        det.confidence = max_score;
        det.class_id = best_class;
        
        detections.push_back(det);
    }
    
    // NMS (Non-Maximum Suppression)
    if (detections.empty()) {
        return detections;
    }
    
    // Sort by confidence (highest first)
    std::sort(detections.begin(), detections.end(), 
              [](const Detection& a, const Detection& b) {
                  return a.confidence > b.confidence;
              });
    
    std::vector<Detection> nms_results;
    std::vector<bool> suppressed(detections.size(), false);
    
    for (size_t i = 0; i < detections.size(); ++i) {
        if (suppressed[i]) continue;
        
        nms_results.push_back(detections[i]);
        
        // Calculate IoU with remaining boxes
        float x1_i = detections[i].x - detections[i].width / 2;
        float y1_i = detections[i].y - detections[i].height / 2;
        float x2_i = detections[i].x + detections[i].width / 2;
        float y2_i = detections[i].y + detections[i].height / 2;
        float area_i = detections[i].width * detections[i].height;
        
        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (suppressed[j]) continue;
            
            // Only suppress detections of same class
            if (detections[i].class_id != detections[j].class_id) continue;
            
            float x1_j = detections[j].x - detections[j].width / 2;
            float y1_j = detections[j].y - detections[j].height / 2;
            float x2_j = detections[j].x + detections[j].width / 2;
            float y2_j = detections[j].y + detections[j].height / 2;
            float area_j = detections[j].width * detections[j].height;
            
            // Calculate intersection
            float x1_inter = std::max(x1_i, x1_j);
            float y1_inter = std::max(y1_i, y1_j);
            float x2_inter = std::min(x2_i, x2_j);
            float y2_inter = std::min(y2_i, y2_j);
            
            float inter_width = std::max(0.0f, x2_inter - x1_inter);
            float inter_height = std::max(0.0f, y2_inter - y1_inter);
            float inter_area = inter_width * inter_height;
            
            // Calculate IoU
            float union_area = area_i + area_j - inter_area;
            float iou = inter_area / union_area;
            
            if (iou > nms_threshold) {
                suppressed[j] = true;
            }
        }
    }
    
    return nms_results;
}

// Fast NMS implementation
std::vector<Detection> YoloInfer::fastNMS(const std::vector<Detection>& detections, float nms_threshold) {
    if (detections.size() <= 1) return detections;
    
    std::vector<Detection> result;
    result.reserve(detections.size());
    
    std::vector<int> indices(detections.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    // Partial sort for top candidates only
    int top_k = std::min(15, static_cast<int>(detections.size()));
    std::partial_sort(indices.begin(), indices.begin() + top_k, indices.end(), 
                     [&](int a, int b) {
                         return detections[a].confidence > detections[b].confidence;
                     });
    
    std::vector<bool> suppressed(detections.size(), false);
    
    for (int i = 0; i < top_k; ++i) {
        int idx_i = indices[i];
        if (suppressed[idx_i]) continue;
        
        result.push_back(detections[idx_i]);
        
        // Suppress overlapping detections
        for (int j = i + 1; j < top_k; ++j) {
            int idx_j = indices[j];
            if (suppressed[idx_j]) continue;
            
            if (fastIoU(detections[idx_i], detections[idx_j]) > nms_threshold) {
                suppressed[idx_j] = true;
            }
        }
        
        if (result.size() >= 8) break;  // Limit final results
    }
    
    return result;
}

// Fast IoU calculation
float YoloInfer::fastIoU(const Detection& a, const Detection& b) {
    // Quick distance check
    float dx = std::abs(a.x - b.x);
    float dy = std::abs(a.y - b.y);
    
    if (dx > (a.width + b.width) / 2 || dy > (a.height + b.height) / 2) {
        return 0.0f;
    }
    
    // Convert to corners
    float a_x1 = a.x - a.width * 0.5f;
    float a_y1 = a.y - a.height * 0.5f;
    float a_x2 = a.x + a.width * 0.5f;
    float a_y2 = a.y + a.height * 0.5f;
    
    float b_x1 = b.x - b.width * 0.5f;
    float b_y1 = b.y - b.height * 0.5f;
    float b_x2 = b.x + b.width * 0.5f;
    float b_y2 = b.y + b.height * 0.5f;
    
    // Calculate intersection
    float inter_x1 = std::max(a_x1, b_x1);
    float inter_y1 = std::max(a_y1, b_y1);
    float inter_x2 = std::min(a_x2, b_x2);
    float inter_y2 = std::min(a_y2, b_y2);
    
    if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1) {
        return 0.0f;
    }
    
    float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
    float a_area = a.width * a.height;
    float b_area = b.width * b.height;
    float union_area = a_area + b_area - inter_area;
    
    return inter_area / union_area;
}


void YoloInfer::printModelInfo() const {
    if (!engine_) return;
    
    ROS_INFO("=== DETAILED Model Information ===");
    ROS_INFO("Number of bindings: %d", engine_->getNbBindings());
    
    for (int i = 0; i < engine_->getNbBindings(); ++i) {
        auto dims = engine_->getBindingDimensions(i);
        std::string shape = "";
        size_t total_elements = 1;
        
        for (int j = 0; j < dims.nbDims; ++j) {
            shape += std::to_string(dims.d[j]);
            total_elements *= dims.d[j];
            if (j < dims.nbDims - 1) shape += "x";
        }
        
        const char* format_str = "";
        auto format = engine_->getBindingFormat(i);
        switch(format) {
            case nvinfer1::TensorFormat::kLINEAR: format_str = "LINEAR"; break;
            case nvinfer1::TensorFormat::kCHW2: format_str = "CHW2"; break;
            case nvinfer1::TensorFormat::kHWC8: format_str = "HWC8"; break;
            case nvinfer1::TensorFormat::kCHW4: format_str = "CHW4"; break;
            default: format_str = "UNKNOWN"; break;
        }
        
        ROS_INFO("Binding %d: %s, shape: %s, elements: %zu, format: %s", 
                 i, 
                 engine_->bindingIsInput(i) ? "INPUT" : "OUTPUT", 
                 shape.c_str(),
                 total_elements,
                 format_str);
                 
        // For output binding, print expected access pattern
        if (!engine_->bindingIsInput(i) && dims.nbDims == 3) {
            ROS_INFO("  Expected output format: [batch=%d, features=%d, detections=%d]", 
                     dims.d[0], dims.d[1], dims.d[2]);
            if (dims.d[1] == 84 && dims.d[2] == 8400) {
                ROS_INFO("  -> YOLOv11 format confirmed: access as output[feature*8400 + detection]");
            }
        }
    }
    
    // Print some memory layout info
    ROS_INFO("Input size: %zu floats (%zu bytes)", input_size_, input_size_ * sizeof(float));
    ROS_INFO("Output size: %zu floats (%zu bytes)", output_size_, output_size_ * sizeof(float));
    ROS_INFO("Expected output dimensions: input=%dx%dx%d", input_width_, input_height_, input_channels_);
    ROS_INFO("===============================");
}