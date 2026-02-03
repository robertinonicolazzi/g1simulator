# ==============================
# Stage 1: Builder
# ==============================
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
ENV CONDA_DIR=/opt/conda
ENV PATH=$CONDA_DIR/bin:$PATH

# 设置 HTTP/HTTPS 代理（可选）
ARG http_proxy
ARG https_proxy
ENV http_proxy=${http_proxy}
ENV https_proxy=${https_proxy}

# 使用阿里云源
RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu/|http://mirrors.aliyun.com/ubuntu/|g' /etc/apt/sources.list


# 安装构建依赖（GCC 12 + GLU + Vulkan）
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc-12 g++-12 cmake build-essential unzip git-lfs \
    libglu1-mesa-dev vulkan-tools  wget\
    libxt6 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 100 \
    && rm -rf /var/lib/apt/lists/*
# 安装 Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh && \
    bash miniconda.sh -b -p $CONDA_DIR && \
    rm miniconda.sh && \
    $CONDA_DIR/bin/conda clean -afy
# 接受 Conda TOS + 创建环境
RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda create -n unitree_sim_env python=3.11 -y && \
    conda clean -afy

# 切换到 Conda 环境
SHELL ["conda", "run", "-n", "unitree_sim_env", "/bin/bash", "-c"]  



RUN conda install -y -c conda-forge "libgcc-ng>=12" "libstdcxx-ng>=12" && \
    apt-get update && apt-get install -y libvulkan1 vulkan-tools && rm -rf /var/lib/apt/lists/*


# 安装 PyTorch（CUDA 12.6 对应）
RUN pip install --upgrade pip && \
    pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu126


# 安装 Isaac Sim
RUN pip install "isaacsim[all,extscache]==5.0.0" --extra-index-url https://pypi.nvidia.com

# 创建工作目录
RUN mkdir -p /home/code
WORKDIR /home/code

# 克隆并安装 IsaacLab
RUN git clone https://github.com/isaac-sim/IsaacLab.git && \
    cd IsaacLab && \
    git checkout v2.2.0 && \
    ./isaaclab.sh --install
    
# 构建 CycloneDDS
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x /cyclonedds && \
    cd /cyclonedds && mkdir build install && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=../install && \
    cmake --build . --target install

# 设置 CycloneDDS 环境变量
ENV CYCLONEDDS_HOME=/cyclonedds/install

# 安装 unitree_sdk2_python
RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python && \
    cd unitree_sdk2_python && pip install -e .

# Use local repository instead of cloning from GitHub
RUN git clone --recurse-submodules https://github.com/robertinonicolazzi/g1simulator.git /home/code/unitree_sim_isaaclab && \
    cd /home/code/unitree_sim_isaaclab && \
    git submodule update --init --recursive && \
    pip install -r requirements.txt && \
    pip install -e "teleimager[server]" --ignore-requires-python
# Pre-accept NVIDIA Omniverse EULA (creates acceptance file)
# This avoids interactive prompt on first run
RUN mkdir -p /root/.nvidia-omniverse/config /root/.local/share/ov/data/Kit && \
    echo 'eula_accepted = true' > /root/.nvidia-omniverse/config/privacy.toml && \
    echo 'privacy_consent = true' >> /root/.nvidia-omniverse/config/privacy.toml && \
    echo '{"eula_accepted": true}' > /root/.local/share/ov/data/Kit/user.config.json && \
    (echo "Yes" | timeout 30 python -c "import isaacsim" 2>/dev/null || true)


# ==============================
# Stage 2: Runtime
# ==============================
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
ENV CONDA_DIR=/opt/conda
ENV PATH=$CONDA_DIR/bin:$PATH

# Isaac Sim configuration
ENV OMNI_KIT_ALLOW_ROOT=1
ENV OMNI_KIT_DISABLE_STARTUP=1
# Accept NVIDIA Omniverse EULA for unattended operation
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=Y

# 安装运行时依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglu1-mesa git-lfs zenity unzip \
    && rm -rf /var/lib/apt/lists/*

# 复制 Conda 环境和代码
COPY --from=builder /home/code/IsaacLab /home/code/IsaacLab
COPY --from=builder /home/code/unitree_sdk2_python /home/code/unitree_sdk2_python

COPY --from=builder /cyclonedds /cyclonedds
COPY --from=builder /opt/conda /opt/conda
COPY --from=builder /home/code/unitree_sim_isaaclab /home/code/unitree_sim_isaaclab

# Copy NVIDIA Omniverse config (EULA acceptance)
COPY --from=builder /root/.nvidia-omniverse /root/.nvidia-omniverse
COPY --from=builder /root/.local/share/ov /root/.local/share/ov

ENV CYCLONEDDS_HOME=/cyclonedds/install

# 写入 bashrc 初始化
RUN echo 'source /opt/conda/etc/profile.d/conda.sh' >> ~/.bashrc && \
    echo 'conda activate unitree_sim_env' >> ~/.bashrc && \
    echo 'export OMNI_KIT_ALLOW_ROOT=1' >> ~/.bashrc && \
    echo 'export OMNI_KIT_DISABLE_STARTUP=1' >> ~/.bashrc && \
    echo 'export ACCEPT_EULA=Y' >> ~/.bashrc && \
    echo 'export PRIVACY_CONSENT=Y' >> ~/.bashrc

WORKDIR /home/code

# 默认进入 Conda 环境 bash
CMD ["conda", "run", "-n", "unitree_sim_env", "/bin/bash"]
