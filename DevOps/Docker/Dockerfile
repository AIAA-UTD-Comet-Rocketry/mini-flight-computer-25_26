# Use Ubuntu 24.04 as the base image
FROM ubuntu:24.04

# Set environment variable to non-interactive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Remove ubuntu user
RUN userdel -r ubuntu || true

# Update the package list and install tools
# The '&& rm -rf /var/lib/apt/lists/*' part helps keep the image size small by removing cached data

RUN apt update && apt install -y --no-install-recommends \
    # Pre-added tools for development
    bash-completion \
    bison \
    wget \
    flex \
    gperf \
    python3-venv \
    ninja-build \
    ccache \
    libffi-dev \
    libssl-dev \
    dfu-util \
    libusb-1.0-0-dev \
    make \
    cmake \
    fd-find \
    imagemagick \
    ca-certificates \
    jq \
    fzf \
    poppler-utils \
    7zip \
    unzip \
    ripgrep \
    ffmpeg \
    file \
    gcc \
    g++ \
    python3 \
    python3-pip \
    pipx \
    sudo \
    vim \
    git \
    tmux \
    curl \
    xsel \
    adduser \ 
    zoxide \
    libgcrypt20 \
    libglib2.0-0 \
    libpixman-1-0 \
    libsdl2-2.0-0 \
    libslirp0 \
    htop 


# Helps keep the image size small by removing cached data
RUN rm -rf /var/lib/apt/lists/* 

######################################################
#                  Create User
######################################################
# Create dev user and add to sudo group
# NOTE: this will create the home directory for the user
RUN useradd -m -s /bin/bash dev && adduser dev sudo && usermod -aG root dev

WORKDIR /home/dev

######################################################
#                  Collect Yazi
######################################################
# Collect yazi binary and install it
RUN curl -LJO https://github.com/sxyazi/yazi/releases/download/v26.1.22/yazi-x86_64-unknown-linux-gnu.deb
RUN apt install ./yazi-x86_64-unknown-linux-gnu.deb

# Cleanup
RUN rm -rf ./yazi-x86_64-unknown-linux-gnu.deb

######################################################
#                  Collect Dotfiles
######################################################
# Using knockout101's dotfiles for neovim and tmux
RUN git -c http.sslVerify=false clone --single-branch https://github.com/knockout101/dotfiles.git

# Now install these files into the .config within home directory
RUN mkdir -p /home/dev/.config
RUN mv /home/dev/dotfiles/tmux /home/dev/dotfiles/nvim /home/dev/.config/

# Fix ownership of directories for dev user (required for getting all plugins)
RUN chown -R dev:dev /home/dev /home/dev/.config

# Cleanup
RUN rm -rf dotfiles

######################################################
#                  Collect Neovim
######################################################
# cURL the tarball
RUN curl -LJO https://github.com/neovim/neovim/releases/download/v0.11.6/nvim-linux-x86_64.tar.gz
# Extract the tarball
RUN tar xzvf nvim-linux-x86_64.tar.gz
# Manually install all the required files
ARG NVIM_PATH=/home/dev/nvim-linux-x86_64
RUN mv ${NVIM_PATH}/bin/* /usr/bin/nvim && \
    mv ${NVIM_PATH}/lib/* /usr/lib/ && \
    mv ${NVIM_PATH}/share/man/* /usr/share/man/ && \
    mv ${NVIM_PATH}/share/nvim /usr/share/

# Cleanup
RUN rm -rf nvim-linux*

######################################################
#             Setup Environment for User
######################################################
# Set up the users of the sudo group to not need to use a password for calling sudo
RUN echo '%sudo ALL=(ALL:ALL) NOPASSWD:ALL' > /etc/sudoers.d/sudo-nopasswd \
    && chmod 0440 /etc/sudoers.d/sudo-nopasswd

# Switch to dev user
USER dev

# Define the command to run when the container starts
CMD ["/bin/bash"]

######################################################
#             Install ESPTool
######################################################
RUN pipx install esptool
# Add the .local binaries to dev's PATH
ENV PATH="/home/dev/.local/bin:$PATH"

######################################################
#             Install ESP-IDF (SDK)
######################################################
# Collect the ESP-IDF SDK from github
RUN git -c http.sslVerify=false clone -b release/v5.5 https://github.com/espressif/esp-idf.git --recursive
# Run install script for the SDK, set to all but can be changed to specific architecture
RUN ~/esp-idf/install.sh
# Force IDF to use the IDF_PATH variable when trying to use SDK
ENV IDF_PATH_FORCE=1
# Setup the IDF_PATH environment varialbe
ENV IDF_PATH="/home/dev/esp-idf"
RUN echo ". $IDF_PATH/export.sh 1>/dev/null 2>&1" >> ~/.bashrc

######################################################
#             Setup For ESP32 QEMU
######################################################
RUN python3 ${IDF_PATH}/tools/idf_tools.py install qemu-xtensa
