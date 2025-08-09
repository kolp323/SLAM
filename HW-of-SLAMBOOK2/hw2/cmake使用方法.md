### 标准的 CMake 项目目录结构
```
MyProject/
├── CMakeLists.txt         # 项目根目录的 CMakeLists 文件
├── main.cpp
├── src/                   # 存放源文件 (.cpp)
│   └── my_library.cpp
└── include/               # 存放头文件 (.h, .hpp)
    └── my_library.h
```
### 编译项目
创建编译目录  

```
mkdir ./build
```

生成构建文件  

```
cd build
cmake .. -G "MinGW Makefiles"
```  

编译项目  

```  
cmake --build .
cmake --build . -- -j8
# 这会强制使用 8 个核心进行编译。
```  

运行程序  

```
./HelloWorld
```  
-------------
    

### 创建库文件
对于没有main函数的文件，在cmakelist中加入：
```  
add_library( hello libHelloSLAM.cpp )
```
重新cmake 编译后，生成一个 libhello.a 文件。

库文件分成静态库和共享库（动态库）两种。静态库以.a作为后缀名，共享库以.so结尾。所有库都是一些函数打包后的集合，差别在于静态库每次被调用都会生成一个副本，而共享库则只有一个副本，更省空间。  
  
如果我们想生成共享库而不是静态库，只需用：
```  
add_library( hello_shared SHARED ​libHelloSLAM.cpp )
```
此时得到的文件是 libhello_shared.so
  
库文件是一个压缩包，里头带有编译好的二进制函数。不过，仅有.a或.so库文件的话，我们并不知道它里头的函数到底是什么，调用的形式又是什么样的。为了让别人（或者自己）使用这个库，我们需要提供一个头文件，说明这些库里都有些什么。  
下面写 libhello 的头文件libHelloSLAM.h:
```
#ifndef LIBHELLOSLAM_H_ 
#define LIBHELLOSLAM_H_ 
void printHello();
#endif
```
--------------
### 调用库函数
写一个可执行程序useHello.cpp，调用这个简单的函数。

```
#include "libHelloSLAM.h" 
int main( int argc, char** argv )
{ 
  printHello(); 
  return 0; 
}
```
在CMakeLists.txt中添加一个可执行程序的生成命令，链接到刚才我们使用的库上：
```
add_executable( useHello useHello.cpp ) 
target_link_libraries( useHello hello_shared )
```
通过这两句话，useHello程序就能顺利使用hello_shared库中的代码了。这个小例子演示了如何生成并调用一个库。对于他人提供的库，我们也可用同样的方式对它们进行调用，整合到自己的程序中。