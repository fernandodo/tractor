# C++ 中 `extern` 关键字详解讲义

---

## 一、`extern` 的基本概念

### 1.1 关键词定义  
`extern` 是 C/C++ 中的**声明修饰符**，用于告诉编译器：  
> “这个变量或函数的定义在别的源文件中，请不要在当前文件分配空间。”

换句话说：  
- **声明（declaration）**：只是说明变量或函数的类型和名称；  
- **定义（definition）**：不仅说明，还**分配内存空间或提供实现**。  

`extern` 只做“声明”，不做“定义”。

---

## 二、主要用途

### 2.1 声明外部变量
```cpp
// file1.cpp
int g_value = 100;   // 定义，分配内存

// file2.cpp
extern int g_value;  // 声明，不分配内存
```
链接器会在其他目标文件中找到 `g_value` 的定义并解析符号。  

如果不加 `extern` 而直接写 `int g_value;`，这就是**定义**，会导致重复定义错误。

### 2.2 声明外部函数
函数声明默认带有 `extern`：
```cpp
extern void foo();
void foo();  // 等价
```

### 2.3 与 `static` 的区别

| 关键字 | 可见性 | 存储位置 | 是否可跨文件访问 |
|:--------|:----------|:-------------|:------------------|
| `extern` | 外部链接（external linkage） | 全局区 (.data 或 .bss) | ✅ |
| `static` | 内部链接（internal linkage） | 全局区 (.data 或 .bss) | ❌ |

两者不能同时使用。

---

## 三、典型使用场景

### 3.1 跨文件共享全局变量
```cpp
// global.h
extern int counter;

// global.cpp
int counter = 0;

// main.cpp
#include "global.h"
int main() {
    counter++;
}
```

### 3.2 跨文件调用函数
```cpp
// utils.h
void say_hello();

// utils.cpp
#include <iostream>
void say_hello() { std::cout << "Hello!"; }
```

---

## 四、`extern "C"` 与 C/C++ 混合编程

### 4.1 背景
C++ 会对函数名进行**Name Mangling**，C 不会。  
若 C++ 调用 C 函数可能找不到符号。

### 4.2 解决方案
```cpp
extern "C" {
#include "c_api.h"
}
```
或：
```cpp
extern "C" void c_function(int);
```

通用写法：
```cpp
#ifdef __cplusplus
extern "C" {
#endif

void c_function(int);

#ifdef __cplusplus
}
#endif
```

---

## 五、`extern` 与内存分区

| 类型 | 内存区段 | 初始化状态 |
|------|-----------|-------------|
| 已初始化全局/静态变量 | `.data` 段 | 有初值 |
| 未初始化全局/静态变量 | `.bss` 段 | 默认值为 0 |
| 局部变量 | 栈（stack） | 函数内存自动管理 |
| 动态分配变量 | 堆（heap） | 程序员控制 |

`extern` 自身不分配内存，只声明。

```cpp
int a = 5;      // .data
int b;          // .bss
extern int c;   // 声明，不分配
```

---

## 六、常见面试陷阱

| 错误写法 | 原因 |
|-----------|------|
| `extern int x = 10;` | 实际是定义，会重复定义。 |
| 修饰局部变量 | ❌ 无效。 |
| 改变存储位置 | ❌ 不会。 |
| 与 `static` 一起用 | ❌ 冲突。 |

---

## 七、总结

- `extern` 用于声明外部定义，不分配内存。  
- 主要用途：跨文件变量、函数、C 混编。  
- 链接阶段由链接器解析符号。  
- 与 `static` 相反，一个外部一个内部。  
- 内存位置由定义决定。

---

# 附录：视频时间线索引

| 时间 | 内容概要 |
|------|-----------|
| 0:00–0:03 | 开场介绍：主题为 extern 的作用。 |
| 0:10–0:27 | extern 用于声明变量或函数，不分配内存。 |
| 0:27–0:49 | 跨源文件访问变量或函数时使用 extern。 |
| 1:00–1:27 | 示例：跨文件访问全局变量。 |
| 1:42–1:48 | 示例：跨文件调用函数。 |
| 2:04–2:29 | 解释 extern "C" 作用及 name mangling。 |
| 4:07–4:58 | 讲解内存段：.data 与 .bss。 |
| 5:02–5:10 | extern vs static 区别。 |

---
