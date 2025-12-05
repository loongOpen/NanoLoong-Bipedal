import torch
import torch.onnx

model = torch.jit.load("model_jitt.pt", map_location=torch.device('cpu'))
model.eval()

model = model.to(dtype=torch.float32)

dummy_obs_tensor  = torch.randn(1, 39, dtype=torch.float32)
dummy_obs_buf_batch  = torch.randn(1, 10, 39, dtype=torch.float32)

def infer_input_names_from_scriptmodule(script_mod):
    # try schema
    try:
        schema = script_mod.forward.schema  # may throw if not present
        arg_names = [arg.name for arg in schema.arguments if arg.name != 'self']
        if len(arg_names) > 0:
            return arg_names
    except Exception:
        pass

    # try graph inputs
    try:
        g = script_mod.graph
        names = []
        for inp in g.inputs():
            nm = inp.debugName()
            if nm == 'self':
                continue
            if nm and not nm.startswith('%'):
                names.append(nm)
            else:
                # use the value_name if available
                try:
                    # Some torch versions provide .debugName() like '%x.1'; still accept it
                    names.append(nm)
                except Exception:
                    pass
        # if we found at least 1 non-self input, return them
        if len(names) >= 1:
            return names
    except Exception:
        pass
    return ['input0', 'input1']

input_names = infer_input_names_from_scriptmodule(model)
if len(input_names) != 2:
    print(f"警告：推断到的输入名数量为 {len(input_names)}，期望 2 个。推断到的名字：{input_names}")
    if len(input_names) > 2:
        input_names = input_names[:2]
    else:
        while len(input_names) < 2:
            input_names.append(f"input{len(input_names)}")

print("使用的 input_names:", input_names)

torch.onnx.export(
    model,
    (dummy_obs_tensor, dummy_obs_buf_batch),
    "model.onnx",
    input_names=input_names,# 动态获取names
    output_names=["action_tensor"],
    dynamic_axes=None,
    opset_version=11,
    do_constant_folding=True,
    export_params=True
)

print("Model converted to ONNX format and saved as model.onnx")

# 验证
import onnx
onnx_model = onnx.load("model.onnx")
onnx.checker.check_model(onnx_model)

print("模型输入形状：")
for input in onnx_model.graph.input:
    dims = []
    for dim in input.type.tensor_type.shape.dim:
        if dim.dim_value > 0:
            dims.append(dim.dim_value)
        elif dim.dim_param:
            dims.append(dim.dim_param)
        else:
            dims.append(None)
    print(f"输入 {input.name}：{dims}")

print("模型输出形状：")
for output in onnx_model.graph.output:
    dims = []
    for dim in output.type.tensor_type.shape.dim:
        if dim.dim_value > 0:
            dims.append(dim.dim_value)
        elif dim.dim_param:
            dims.append(dim.dim_param)
        else:
            dims.append(None)
    print(f"输出 {output.name}：{dims}")
