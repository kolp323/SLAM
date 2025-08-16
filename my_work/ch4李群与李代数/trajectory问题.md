## 在编译时加地址消毒器（ASan）这样会直接告诉你是哪里越界
```cmake -DCMAKE_CXX_FLAGS="-fsanitize=address -g" ..```

## Sophus/Eigen 的类里有 SSE/AVX 对齐要求，错配就会导致“corrupted double-linked list”这种堆链表损坏。
需要在cmakelists加上：
```add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)```
这个宏的作用是禁止 Eigen 在静态对象中做内存对齐。

### 背景
Eigen 默认会对某些类型（例如 Eigen::Vector3d, Eigen::Quaterniond 等）进行16 字节内存对齐（为了 SSE/AVX 向量化优化），这样它们在堆或栈中分配时性能会更好。

但是，这种对齐方式在静态分配（例如全局变量、静态局部变量）时，在某些平台或编译器组合中会引起内存分配问题，甚至触发**corrupted double-linked list** 这种错误，因为静态存储区的对齐可能无法满足 Eigen 的要求。  

### 宏的作用
定义 EIGEN_DONT_ALIGN_STATICALLY 后：

Eigen 不会对静态分配的对象做 16 字节对齐，而是按普通内存分配规则来存放。

避免了在静态对象中由于对齐要求导致的崩溃（特别是在旧的 GCC 或某些嵌入式平台）。

对动态分配（new）和局部变量的对齐优化仍然保留。