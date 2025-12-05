import torch
import torch.onnx

# 加载原始模型
model = torch.jit.load("model_jitt.pt", map_location=torch.device('cpu'))
model.eval()

# 将模型转换为float32类型，避免与输入数据类型不匹配
model = model.to(dtype=torch.float32)

dummy_obs_tensor  = torch.randn(1, 39, dtype=torch.float32)  # 第一个输入: obs_tensor [1, 39]
dummy_obs_buf_batch  = torch.randn(1, 10, 39, dtype=torch.float32)  # 第二个输入: obs_buf_batch [1, history_length, 39]

torch.onnx.export(
    model,
    (dummy_obs_tensor, dummy_obs_buf_batch),
    "model.onnx",
    input_names=["obs_tensor", "obs_buf_batch"],
    output_names=["action_tensor"],
    # dynamic_axes={
    #     'obs': {0: 'batch_size'},
    #     'obs_buf': {0: 'batch_size', 1: 'history_length'},
    #     'output': {0: 'batch_size'}
    # },
    dynamic_axes=None, # 禁用动态纬度
    opset_version=11,  # 低版本兼容性更好，适合嵌入式
    do_constant_folding=True,  # 启用常量折叠优化
    export_params=True  # 导出模型权重
    )

print("Model converted to ONNX format and saved as model.onnx")

# 验证模型输入输出形状
import onnx
onnx_model = onnx.load("model.onnx")
onnx.checker.check_model(onnx_model)

print("模型输入形状：")
for input in onnx_model.graph.input:
    dims = [dim.dim_value for dim in input.type.tensor_type.shape.dim]
    print(f"输入 {input.name}：{dims}")  # 应输出 [1,39] 和 [1,10,39]

print("模型输出形状：")
for output in onnx_model.graph.output:
    dims = [dim.dim_value for dim in output.type.tensor_type.shape.dim]
    print(f"输出 {output.name}：{dims}")  # 应输出 [1,10]