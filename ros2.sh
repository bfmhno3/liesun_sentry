function ros2::build() {
    mamba activate ros2

    if [[ ! -d "src" ]]; then
        >&2 echo "Error: This is not a ROS2 workspace."
        return 1
    fi

    local cmake_args=(
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
        -DCMAKE_SKIP_RPATH=ON
    )

    colcon build \
        --symlink-install \
        --parallel-workers 1 \
        --cmake-args "${cmake_args[@]}" \
        --event-handlers console_cohesion+

    if [[ -f "build/compile_commands.json" ]]; then
        ln -sf build/compile_commands.json compile_commands.json
    else
        >&2 echo "Warning: compile_commands.json not found in build directory"
    fi

    ros2::source
}

function ros2::clean() {
    if [[ -f "compile_commands.json" ]]; then
        unlink compile_commands.json || {echo $?}
    fi

    colcon clean workspace -y
}

function ros2::source() {
    if [[ -f "install/setup.zsh" ]]; then
        source install/setup.zsh
    elif [[ -f "install/setup.sh" ]]; then
        source install/setup.sh
    else
        echo "未找到 ROS2 的 setup 脚本文件"
    fi
}
