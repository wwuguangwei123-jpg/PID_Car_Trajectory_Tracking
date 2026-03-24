# 1. 声明 CMake 最低版本要求
cmake_minimum_required(VERSION 3.10)

# 2. 设置项目名称（随便起，比如你的 PID 小车）
project(PID_CAR_PROJECT)

# 3. 指定 C++ 标准 (推荐使用 C++17)
set(CMAKE_CXX_STANDARD 17)

# 4. 自动搜寻当前目录下所有的 .cpp 和 .h 文件
# 这行代码最关键，以后你增加再多的文件，它都会自动抓取
file(GLOB SOURCES "*.cpp" "*.h")

# 5. 生成可执行文件 (第一个参数是生成的文件名，第二个是源代码列表)
add_executable(my_pid_car ${SOURCES})