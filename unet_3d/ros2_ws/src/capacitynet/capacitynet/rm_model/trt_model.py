"""TensorRT inference wrapper for UNet3D on Jetson Orin.

Uses TensorRT 8.6 Python API (execute_async_v3 + set_tensor_address).
Input and output tensors must already be on CUDA — no CPU roundtrip.

Note: the engine expects fixed spatial dims (128³ by default) because
F.interpolate skip-connection sizes are baked as constants during ONNX export.

CUDA Graph: captured during warmup() for batch=1 only. Replaying a CUDA Graph
eliminates per-call CPU kernel-dispatch overhead (~10–20 ms on Jetson). Batch≠1
(gradient mode, batch=9) falls back to the standard execute_async_v3 path.

Stream discipline:
  torch.cuda.graph(g, stream=s) makes s the active capture stream inside the
  with-block. execute_async_v3 must receive s.cuda_stream so its kernels are
  recorded on s (not the default stream). At replay: copy_ input on s (ordered
  before replay), replay() on s, wait_stream() so default stream waits before
  clone() reads the output buffer.
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
        # at the full-resolution decoder concat layer: B×48×D³ < 2^31).
        # get_tensor_profile_shape returns (min, opt, max) tuples.
        max_shape = self.engine.get_tensor_profile_shape(self.input_name, 0)[2]
        self.max_batch = max_shape[0]

        # CUDA Graph state — populated by _capture_graph() called from warmup().
        self._cuda_graph = None
        self._graph_stream = None
        self._graph_input_buf = None
        self._graph_output_buf = None

    def _capture_graph(self, spatial: int):
        """Capture a CUDA Graph for batch=1 inference.

        A dedicated stream is created and passed to torch.cuda.graph() so that
        execute_async_v3 submits kernels onto the capture stream — the only way
        they get recorded. Getting the stream handle outside the with-block (as
        torch.cuda.current_stream()) would yield the normal stream and produce an
        empty graph.
        """
        self._graph_stream = torch.cuda.Stream()

        self._graph_input_buf = torch.zeros(
            1, 1, spatial, spatial, spatial, dtype=self.input_dtype, device='cuda')
        self._graph_output_buf = torch.zeros(
            1, 1, spatial, spatial, spatial, dtype=self.output_dtype, device='cuda')

        # Fix tensor addresses permanently for this graph.
        self.context.set_input_shape(self.input_name, (1, 1, spatial, spatial, spatial))
        self.context.set_tensor_address(self.input_name,  self._graph_input_buf.data_ptr())
        self.context.set_tensor_address(self.output_name, self._graph_output_buf.data_ptr())

        # Warmup on dedicated stream before capture (stabilises cuDNN kernel state).
        with torch.cuda.stream(self._graph_stream):
            self.context.execute_async_v3(self._graph_stream.cuda_stream)
        torch.cuda.synchronize()

        # Capture: stream=_graph_stream makes it the active capture stream inside
        # the with-block, so execute_async_v3(_graph_stream.cuda_stream) is recorded.
        self._cuda_graph = torch.cuda.CUDAGraph()
        with torch.cuda.graph(self._cuda_graph, stream=self._graph_stream):
            self.context.execute_async_v3(self._graph_stream.cuda_stream)
        torch.cuda.synchronize()

    def _infer_single(self, x: torch.Tensor) -> torch.Tensor:
        """Run one TRT forward pass. x must fit within max_batch."""
        if x.shape[0] == 1 and self._cuda_graph is not None:
            # copy_, replay() and clone() must all run on the SAME stream so they
            # execute strictly in order. The previous version copied the input on a
            # dedicated stream but replayed on the current stream with no sync, so
            # replay() raced the copy and read the STALE input buffer on ~half the
            # calls (e.g. last frame / zeros warmup) -> intermittently wrong output.
            # Running everything on the current stream serialises copy -> replay ->
            # read with no race. A graph captured on _graph_stream may be replayed
            # on any stream.
            self._graph_input_buf.copy_(x)
            self._cuda_graph.replay()
            return self._graph_output_buf.clone()

        # Fallback: dynamic batch or graph not yet captured.
        self.context.set_input_shape(self.input_name, tuple(x.shape))
        output = torch.empty(x.shape, dtype=self.output_dtype, device='cuda')
        self.context.set_tensor_address(self.input_name,  x.data_ptr())
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

    def warmup(self, spatial: int = 128):
        """Run dummy inferences to prime cuDNN kernel selection, then capture CUDA Graph.

        Should be called once at node init before real data arrives.
        Warms up both batch=1 and the maximum batch the engine was built for,
        then captures a CUDA Graph for batch=1 to eliminate future dispatch overhead.
        """
        for batch in (1, self.max_batch):
            dummy = torch.zeros(batch, 1, spatial, spatial, spatial,
                                dtype=self.input_dtype, device='cuda')
            self.infer(dummy)
        torch.cuda.synchronize()
        self._capture_graph(spatial)
