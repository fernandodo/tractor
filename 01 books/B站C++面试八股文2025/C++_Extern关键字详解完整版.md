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

如果一个全局变量定义在 `file1.cpp` 中，而你想在 `file2.cpp` 中使用它，就必须在 `file2.cpp` 里声明它：

```cpp
// file1.cpp
int g_value = 100;   // 定义，分配内存

// file2.cpp
extern int g_value;  // 声明，不分配内存
```

编译器在编译 `file2.cpp` 时不会报错，它只知道有这么个变量。  
**链接阶段**，链接器会在其他目标文件中找到 `g_value` 的定义，并完成符号解析。

如果不加 `extern`，直接使用 `int g_value;`，这在 C++ 中是“定义”而非声明，会造成重复定义错误（multiple definition）。

---

### 2.2 声明外部函数

函数的声明默认带有 `extern` 属性，因此以下两者等价：

```cpp
extern void foo();
void foo();
```

通常函数声明可以直接写成第二种形式。  
但如果函数定义在别的 `.cpp` 文件中，而要在当前文件调用，仍需要在头文件或源文件中提前声明它。

---

### 2.3 与 `static` 的区别

| 关键字 | 可见性 | 存储位置 | 是否可跨文件访问 |
|:--------|:----------|:-------------|:------------------|
| `extern` | 外部链接（external linkage） | 全局区 (.data 或 .bss) | ✅ 可以 |
| `static` | 内部链接（internal linkage） | 全局区 (.data 或 .bss) | ❌ 不可以 |

`static` 会把符号限制在当前编译单元内；  
`extern` 会把符号导出，供其他文件使用。

> ⚠️ 注意：同一个标识符不能同时被 `static` 和 `extern` 修饰。

---

## 三、典型使用场景

### 3.1 跨文件共享全局变量

**推荐做法：**

```cpp
// global.h
#pragma once
extern int counter;   // 声明

// global.cpp
int counter = 0;      // 定义

// main.cpp
#include "global.h"
#include <iostream>

int main() {
    counter++;
    std::cout << counter << std::endl;
}
```

这样结构清晰、可维护，不会引发重复定义。

---

### 3.2 跨文件调用函数

同理：

```cpp
// utils.h
#pragma once
void say_hello();

// utils.cpp
#include <iostream>
void say_hello() {
    std::cout << "Hello!
";
}

// main.cpp
#include "utils.h"
int main() {
    say_hello();
}
```

在函数声明时其实隐含 `extern` 语义。

---

## 四、`extern "C"` 与 C/C++ 混合编程

### 4.1 背景

C++ 支持函数重载，因此编译器会对函数名进行**名字修饰（Name Mangling）**：

```cpp
void func(int);
void func(double);
```
会分别被编译为不同的符号名，如：

```
_func_int@4
_func_double@8
```

而 C 语言没有重载机制，函数名在目标文件中不被修改（如 `_func`）。

如果你在 C++ 中调用一个由 C 编译的函数，链接器可能会找不到符号。

---

### 4.2 解决方案

使用 `extern "C"` 告诉编译器：  
> 按照 C 的方式编译、命名符号，不要做 C++ 的名字修饰。

例如：

```cpp
// C 库头文件 c_api.h
void c_function(int);

// C++ 调用方
extern "C" {
#include "c_api.h"
}
```

或者写成：

```cpp
extern "C" void c_function(int);
```

如果是头文件供 C 和 C++ 同时使用，应写成：

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

### 5.1 内存区域回顾

全局变量和静态变量都存储在**全局区（静态存储区）**，可细分为：

| 类型 | 内存区段 | 初始化状态 |
|------|-----------|-------------|
| 已初始化全局/静态变量 | `.data` 段 | 有初值 |
| 未初始化全局/静态变量 | `.bss` 段 | 默认值为 0 |
| 局部变量 | 栈（stack） | 函数内存自动管理 |
| 动态分配变量 | 堆（heap） | 由程序员分配释放 |

### 5.2 `extern` 与内存

`extern` 本身不决定变量在哪个段，只是声明引用别处的定义。  
真正决定变量位置的，是定义语句：

```cpp
int a = 5;      // 已初始化 → .data
int b;          // 未初始化 → .bss
extern int c;   // 仅声明，不分配内存
```

---

## 六、常见面试陷阱与误区

| 误区 | 解释 |
|------|------|
| `extern int x = 10;` | 这是**定义**而不是声明，会分配内存。若重复，会导致 multiple definition。 |
| `extern` 可以修饰局部变量 | ❌ 错误。只能用于全局变量或函数。 |
| `extern` 会改变存储位置 | ❌ 不会，仅影响链接可见性。 |
| `extern` 与 `static` 一起用 | ❌ 冲突，语义对立。 |

---

## 七、总结

- `extern` 用于**声明外部定义**，不分配内存；  
- 主要用途：
  1. 跨文件访问全局变量；
  2. 跨文件调用函数；
  3. 与 `extern "C"` 结合实现 C/C++ 混编；  
- 链接阶段由链接器解析外部符号；  
- 注意区分 `extern`（外部链接）与 `static`（内部链接）；  
- 注意 `extern` 不会改变变量所在段，变量的存储由定义决定。

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
