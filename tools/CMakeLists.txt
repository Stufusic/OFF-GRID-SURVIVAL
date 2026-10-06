cmake_minimum_required(VERSION 3.20)

# Công cụ replay/đánh giá dataset thật (Sensor Logger folder).
# Dùng: replay_evaluation <dataset_folder> [--csv <out.csv>] [--out <out.txt>]
# Không đối số -> chạy 2 dataset mặc định, in ra stdout.
add_executable(replay_evaluation
    replay_evaluation.cpp
)

target_link_libraries(replay_evaluation PRIVATE
    survival_core
    survival_harness
    Eigen3::Eigen
)

target_compile_features(replay_evaluation PRIVATE cxx_std_20)
