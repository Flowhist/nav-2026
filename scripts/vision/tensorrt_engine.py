"""Fixed-shape TensorRT engines with reusable CUDA buffers; no CUDA PyTorch required."""
import ctypes as ct
from pathlib import Path

import numpy as np


class Engine:
    def __init__(self, path):
        import tensorrt as trt
        self.cuda = ct.CDLL('libcudart.so.12')
        self.cuda.cudaMalloc.argtypes = [ct.POINTER(ct.c_void_p), ct.c_size_t]
        self.cuda.cudaFree.argtypes = [ct.c_void_p]
        self.cuda.cudaMemcpy.argtypes = [ct.c_void_p, ct.c_void_p, ct.c_size_t, ct.c_int]
        self.cuda.cudaStreamCreate.argtypes = [ct.POINTER(ct.c_void_p)]
        self.cuda.cudaStreamSynchronize.argtypes = [ct.c_void_p]
        self.cuda.cudaStreamDestroy.argtypes = [ct.c_void_p]
        self.stream = ct.c_void_p()
        self.buffers = {}
        self.logger = trt.Logger(trt.Logger.ERROR)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(Path(path).read_bytes())
        if self.engine is None:
            raise RuntimeError('Invalid engine; rebuild with this device/runtime: '+str(path))
        self.context = self.engine.create_execution_context()
        try:
            self.check(self.cuda.cudaStreamCreate(ct.byref(self.stream)))
            inputs, outputs = [], []
            for i in range(self.engine.num_io_tensors):
                name = self.engine.get_tensor_name(i)
                shape = tuple(self.engine.get_tensor_shape(name))
                if min(shape) <= 0:
                    raise ValueError('Only fixed-shape engines are supported')
                array = np.empty(shape, dtype=trt.nptype(self.engine.get_tensor_dtype(name)))
                ptr = ct.c_void_p()
                self.check(self.cuda.cudaMalloc(ct.byref(ptr), array.nbytes))
                self.buffers[name] = (array, ptr)
                self.context.set_tensor_address(name, ptr.value)
                (inputs if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT else outputs).append(name)
            if len(inputs) != 1 or len(outputs) != 1:
                raise ValueError('Expected one input and one output')
            self.input, self.output = inputs[0], outputs[0]
        except Exception:
            self.close()
            raise

    @staticmethod
    def check(code):
        if code:
            raise RuntimeError(f'CUDA runtime error {code}')

    def __call__(self, value):
        target, ptr = self.buffers[self.input]
        if value.shape != target.shape:
            raise ValueError(f'Expected {target.shape}, got {value.shape}')
        np.copyto(target, value)
        self.check(self.cuda.cudaMemcpy(ptr, target.ctypes.data, target.nbytes, 1))
        if not self.context.execute_async_v3(self.stream.value):
            raise RuntimeError('TensorRT execution failed')
        self.check(self.cuda.cudaStreamSynchronize(self.stream))
        result, ptr = self.buffers[self.output]
        self.check(self.cuda.cudaMemcpy(result.ctypes.data, ptr, result.nbytes, 2))
        return result

    def close(self):
        for _, ptr in self.buffers.values():
            self.cuda.cudaFree(ptr)
        self.buffers.clear()
        if self.stream.value:
            self.cuda.cudaStreamDestroy(self.stream)
            self.stream = ct.c_void_p()


def build(onnx, output):
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    if not parser.parse(Path(onnx).read_bytes()):
        raise RuntimeError('\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    config = builder.create_builder_config()
    config.builder_optimization_level = 1
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 256 << 20)
    config.set_flag(trt.BuilderFlag.FP16)
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError('TensorRT build failed')
    temporary = Path(str(output)+'.tmp')
    temporary.write_bytes(bytes(serialized))
    temporary.replace(output)
