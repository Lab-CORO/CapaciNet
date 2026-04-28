"""TensorRT inference wrapper for UNet3D on Jetson Orin.

Uses TensorRT 8.6 Python API (execute_async_v3 + set_tensor_address).
Input and output tensors must already be on CUDA — no CPU roundtrip.

Note: the engine expects fixed spatial dims (152³ by default) because
F.interpolate skip-connection sizes are baked as constants during ONNX export.
"""

import torch
import tensorrt as trt

# Register all built-in TRT plugins (e.g. InstanceNormalization_TRT).
# trtexec does this automatically; the Python Runtime does not.
_trt_logger = trt.Logger(trt.Logger.WARNING)
trt.init_libnvinfer_plugins(_trt_logger, "")

_TRT_DTYPE_TO_TORCH = {
    trt.DataType.FLOAT: torch.float32,
    trt.DataType.HALF:  torch.float16,
    trt.DataType.INT32: torch.int32,
    trt.DataType.INT8:  torch.int8,
}


class TRTModel:
    def __init__(self, engine_path: str):
        runtime = trt.Runtime(_trt_logger)

        with open(engine_path, 'rb') as f:
            self._engine_data = f.read()  # must stay alive for the engine's lifetime

        self.engine = runtime.deserialize_cuda_engine(self._engine_data)
        if self.engine is None:
            raise RuntimeError(f'Failed to deserialize TRT engine: {engine_path}')

        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError('Failed to create TRT execution context')

        self.input_name = 'input'
        self.output_name = 'output'

        # Detect the dtype the engine expects at its I/O boundaries.
        # With --fp16, TRT may still use float32 at graph boundaries and convert internally.
        trt_in_dtype = self.engine.get_tensor_dtype(self.input_name)
        trt_out_dtype = self.engine.get_tensor_dtype(self.output_name)
        self.input_dtype = _TRT_DTYPE_TO_TORCH.get(trt_in_dtype, torch.float32)
        self.output_dtype = _TRT_DTYPE_TO_TORCH.get(trt_out_dtype, torch.float32)

        # Maximum batch the engine accepts (set at build time to avoid int32 overflow
        # at the full-resolution decoder concat layer: B×96×D³ < 2^31).
        # get_tensor_profile_shape returns (min, opt, max) tuples.
        max_shape = self.engine.get_tensor_profile_shape(self.input_name, 0)[2]
        self.max_batch = max_shape[0]

    def _infer_single(self, x: torch.Tensor) -> torch.Tensor:
        """Run one TRT forward pass. x must fit within max_batch."""
        self.context.set_input_shape(self.input_name, tuple(x.shape))
        output = torch.empty(x.shape, dtype=self.output_dtype, device='cuda')
        self.context.set_tensor_address(self.input_name, x.data_ptr())
        self.context.set_tensor_address(self.output_name, output.data_ptr())
        stream = torch.cuda.current_stream().cuda_stream
        ok = self.context.execute_async_v3(stream_handle=stream)
        if not ok:
            raise RuntimeError('TRT execute_async_v3 returned False')
        return output

    def infer(self, x: torch.Tensor) -> torch.Tensor:
        """Run inference on a CUDA tensor, splitting oversized batches transparently.

        Args:
            x: input tensor on CUDA, shape (B, 1, D, H, W)

        Returns:
            output tensor on CUDA, same shape as x, dtype = self.output_dtype
        """
        assert x.is_cuda, 'Input must be on CUDA'
        x = x.to(self.input_dtype).contiguous()

        if x.shape[0] <= self.max_batch:
            return self._infer_single(x)

        # TRT int32 volume limit: split into chunks of max_batch and reassemble.
        chunks = x.split(self.max_batch, dim=0)
        return torch.cat([self._infer_single(c) for c in chunks], dim=0)

    def warmup(self, spatial: int = 152):
        """Run dummy inferences to prime cuDNN kernel selection.

        Should be called once at node init before real data arrives.
        Warms up both batch=1 and the maximum batch the engine was built for.
        """
        for batch in (1, self.max_batch):
            dummy = torch.zeros(batch, 1, spatial, spatial, spatial,
                                dtype=self.input_dtype, device='cuda')
            self.infer(dummy)
        torch.cuda.synchronize()
