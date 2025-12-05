#pragma once
#include "eigen.h"
#include <string>
#include <memory>
#include <onnxruntime_cxx_api.h>// 添加onnxruntime头文件

namespace OnnxRuntime {
class OnnxRuntimeClass {
public:
    OnnxRuntimeClass();
    ~OnnxRuntimeClass();
    bool init(const std::string& modelFile, int numIn1, int numIn2, int numOut);
    void run();
    vecXf in1, in2, out;

private:
    class ImpClass;
    std::unique_ptr<ImpClass> imp;  // 改为智能指针，延迟初始化
};
} // namespace OnnxRuntime