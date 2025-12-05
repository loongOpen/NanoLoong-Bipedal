#include "rl_onnx.h"
#include <vector>
#include <iostream>
#include <cstring>

namespace OnnxRuntime {

class OnnxRuntimeClass::ImpClass {
public:
    ImpClass(OnnxRuntimeClass* omp);
    ~ImpClass();
    bool init(const std::string& modelFile, int numIn1, int numIn2, int numOut);
    void step();
    
    OnnxRuntimeClass& omp;
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    int numIn1, numIn2, numOut;
    int memIn1, memIn2, memOut;
    std::string input_name1, input_name2, output_name;
    std::vector<int64_t> input_dims1, input_dims2, output_dims;
};

OnnxRuntimeClass::ImpClass::ImpClass(OnnxRuntimeClass* omp) : 
    omp(*omp),
    env(ORT_LOGGING_LEVEL_WARNING, "OnnxRuntime")
{
    printf("ONNX Runtime: ImpClass构造函数\n");
}

OnnxRuntimeClass::ImpClass::~ImpClass() {
    printf("ONNX Runtime: ImpClass析构函数\n");
}

bool OnnxRuntimeClass::ImpClass::init(const std::string& modelFile, int numIn1, int numIn2, int numOut) {
    printf("ONNX Runtime: 开始初始化, 模型: %s\n", modelFile.c_str());
    
    // 设置维度参数
    this->numIn1 = numIn1;
    this->numIn2 = numIn2;
    this->numOut = numOut;
    memIn1 = numIn1 * sizeof(float);
    memIn2 = numIn2 * sizeof(float);
    memOut = numOut * sizeof(float);
    
    printf("ONNX Runtime: 维度设置 - 输入1特征:%d, 输入2总元素:%d, 输出:%d\n", numIn1, numIn2, numOut);

    try {
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        
        printf("ONNX Runtime: 开始加载模型\n");
        session = std::make_unique<Ort::Session>(env, modelFile.c_str(), session_options);
        printf("ONNX Runtime: 模型加载成功\n");

        input_name1 = "obs";
        input_name2 = "obs_hist"; 
        output_name = "action_tensor";
        
        printf("ONNX Runtime: 使用固定名称 - 输入1:%s, 输入2:%s, 输出:%s\n", 
               input_name1.c_str(), input_name2.c_str(), output_name.c_str());

        omp.in1.setZero(numIn1);
        omp.in2.setZero(numIn2);
        omp.out.setZero(numOut);
        
        printf("ONNX Runtime: 输入输出向量初始化完成\n");
        printf("ONNX Runtime: 初始化完成 - 成功\n");
        return true;
        
    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime: 初始化错误 - " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "ONNX Runtime: 初始化异常 - " << e.what() << std::endl;
        return false;
    }
}

void OnnxRuntimeClass::ImpClass::step() {
    if (!session) {
        std::cerr << "ONNX Runtime: 错误 - 会话未初始化" << std::endl;
        return;
    }
    
    if (!omp.in1.data() || !omp.in2.data() || !omp.out.data()) {
        std::cerr << "ONNX Runtime: 错误 - 输入输出数据指针无效" << std::endl;
        return;
    }

    try {
        std::vector<Ort::Value> input_tensors;
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        
        // 输入1 - [1, 39]
        std::vector<int64_t> obs_shape = {1, numIn1};  // [1, 39]
        Ort::Value input_tensor1 = Ort::Value::CreateTensor<float>(
            memory_info, omp.in1.data(), numIn1, obs_shape.data(), obs_shape.size());
        input_tensors.push_back(std::move(input_tensor1));
        
        // 输入2 - [1, 10, 39]
        std::vector<int64_t> obs_buf_shape = {1, 10, 39};  // [1, 10, 39]
        Ort::Value input_tensor2 = Ort::Value::CreateTensor<float>(
            memory_info, omp.in2.data(), numIn2, obs_buf_shape.data(), obs_buf_shape.size());
        input_tensors.push_back(std::move(input_tensor2));
        
        // 运行推理
        Ort::RunOptions run_options;
        const char* input_names[] = {input_name1.c_str(), input_name2.c_str()};
        const char* output_names[] = {output_name.c_str()};
        
        auto output_tensors = session->Run(run_options, 
                                          input_names, 
                                          input_tensors.data(), 
                                          input_tensors.size(),
                                          output_names, 
                                          1);
        
        // 处理输出
        if (!output_tensors.empty() && output_tensors[0].IsTensor()) {
            float* output_data = output_tensors[0].GetTensorMutableData<float>();
            
            // action_vec
            std::memcpy(omp.out.data(), output_data, memOut);
        } else {
            std::cerr << "ONNX Runtime: 输出张量无效，清零输出" << std::endl;
            std::memset(omp.out.data(), 0, memOut);
        }
        
    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX Runtime: 推理错误 - " << e.what() << std::endl;
        std::memset(omp.out.data(), 0, memOut);
    } catch (const std::exception& e) {
        std::cerr << "ONNX Runtime: 推理异常 - " << e.what() << std::endl;
        std::memset(omp.out.data(), 0, memOut);
    }
}

OnnxRuntimeClass::OnnxRuntimeClass() : imp(std::make_unique<ImpClass>(this)) {
    printf("ONNX Runtime: OnnxRuntimeClass构造函数\n");
}

OnnxRuntimeClass::~OnnxRuntimeClass() {
    printf("ONNX Runtime: OnnxRuntimeClass析构函数\n");
}

bool OnnxRuntimeClass::init(const std::string& modelFile, int numIn1, int numIn2, int numOut) {
    printf("ONNX Runtime: 调用init函数\n");
    if (!imp) {
        std::cerr << "ONNX Runtime: 错误 - ImpClass未初始化" << std::endl;
        return false;
    }
    return imp->init(modelFile, numIn1, numIn2, numOut);
}

void OnnxRuntimeClass::run() {
    if (!imp) {
        std::cerr << "ONNX Runtime: 错误 - ImpClass未初始化" << std::endl;
        return;
    }
    imp->step();
}

} // namespace OnnxRuntime