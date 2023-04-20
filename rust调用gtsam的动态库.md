rust调用gtsam动态库遇到的一些问题:
1. CXX这个rust crate只能编译成静态库所以如果是调用C++的动态库的话就不能用这个crate
2. gtsam没有编译成静态库
3. gtsam的动态库依赖于其他的动态库

方法:
1. 在头文件中定义暴露C接口的函数
2. 在C++文件中实现暴露的C接口的函数
3. 在CMakeLists.txt中添加编译成动态库的命令
示例可以参考 gtsam_build_demo/ 目录下的文件 gtam_build.h gtam_build.cpp CMakeLists.txt
4. 在build.rs中使用bindgen将头文件转换成bind.rs文件
5. 在build.rs中指定动态库的路径和名称
可以参考build_gtsam.rs 这文件的示例

编译生成的动态库可能需要放到指定的目录下面才能被rust调用 

安装gtsam
### install gtsam
git clone git@github.com:borglab/gtsam.git
git checkout 4.1.1
mkdir build
cd build

修改CMakelists.txt文件，将
cmake .. -DGTSAM_WITH_TBB=0 -DCMAKE_CXX_STANDARD=11 -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++
make install
