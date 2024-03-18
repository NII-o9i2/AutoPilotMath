find cilqr -type f -name "*.cpp" -exec clang-format -i {} \;
find cilqr -type f -name "*.h" -exec clang-format -i {} \;

find dcp_tree -type f -name "*.cpp" -exec clang-format -i {} \;
find dcp_tree -type f -name "*.h" -exec clang-format -i {} \;

find env_simulator -type f -name "*.cpp" -exec clang-format -i {} \;
find env_simulator -type f -name "*.h" -exec clang-format -i {} \;

find idm -type f -name "*.cpp" -exec clang-format -i {} \;
find idm -type f -name "*.h" -exec clang-format -i {} \;

find math_common -type f -name "*.cpp" -exec clang-format -i {} \;
find math_common -type f -name "*.h" -exec clang-format -i {} \;