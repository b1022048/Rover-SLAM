#!/bin/bash

export OpenCV_DIR=$HOME/opt/opencv-3.4.13/share/OpenCV

export LD_LIBRARY_PATH=$HOME/opt/opencv-3.4.13/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$HOME/henry/onnxruntime-linux-x64-gpu-1.16.3/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/targets/x86_64-linux/lib:/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH

# 先用 RTX 3080 Ti，避免 RTX 5070 Ti 跟 CUDA 11.8 不相容
export CUDA_VISIBLE_DEVICES=2
