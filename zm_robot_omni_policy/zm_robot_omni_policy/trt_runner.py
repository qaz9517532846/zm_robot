# SPDX-License-Identifier: Apache-2.0
#
# Minimal TensorRT engine wrapper (static shapes, host numpy in/out) on top of pycuda.
# Device buffers are allocated once at load time, not per inference.

import numpy as np

try:
    import pycuda.autoinit    # noqa: F401  pylint: disable=unused-import  (creates the CUDA context)
    import pycuda.driver as cuda
    import tensorrt as trt
except ImportError as exc:    # raised with an install hint when an engine is actually loaded
    cuda, trt, _IMPORT_ERROR = None, None, exc
else:
    _IMPORT_ERROR = None


class TrtEngine:
    """Load a serialized TensorRT engine and run it: outputs = engine({name: ndarray})."""

    def __init__(self, engine_path):
        if _IMPORT_ERROR is not None:
            raise ModuleNotFoundError(
                "TensorRT / pycuda are not importable (pip3 install --break-system-packages "
                f"pycuda; TensorRT python bindings): {_IMPORT_ERROR}")
        self.path = engine_path
        self._logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(self._logger).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(
                f"could not deserialize {engine_path} - engines are tied to the TensorRT version "
                "and GPU they were built on; rebuild with scripts/build_engines.sh")
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        self.inputs, self.outputs = {}, {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            host = np.zeros(shape, dtype=trt.nptype(self.engine.get_tensor_dtype(name)))
            dev = cuda.mem_alloc(max(host.nbytes, 1))
            self.context.set_tensor_address(name, int(dev))
            is_input = self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT
            (self.inputs if is_input else self.outputs)[name] = (host, dev)

    def __call__(self, feeds):
        for name, (host, dev) in self.inputs.items():
            np.copyto(host, np.asarray(feeds[name], dtype=host.dtype).reshape(host.shape))
            cuda.memcpy_htod_async(dev, host, self.stream)
        self.context.execute_async_v3(self.stream.handle)
        for name, (host, dev) in self.outputs.items():
            cuda.memcpy_dtoh_async(host, dev, self.stream)
        self.stream.synchronize()
        return {name: host.copy() for name, (host, _) in self.outputs.items()}
