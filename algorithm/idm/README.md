##1 修改 idm_case_manager.py下的path为本地仓库下idm的路径，如：/home/sensetime/ws/common_math/algorithm/idm/
##2 修改idm_debug.py下的sys.path为pybind生成的c++库的路径，如：/home/sensetime/ws/common_math/algorithm/idm/build
##3 如何编译修改之后的idm模型：
cd /home/sensetime/ws/common_math/algorithm/idm
mkdir build
cmake ..
make 
##4 usage
python3 idm_debug.py  -o ./idm.html -s case_1.json中的case_name