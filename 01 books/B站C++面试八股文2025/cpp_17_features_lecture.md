# C++17 常用特性讲义（完整版）

## Session 1：简要时间线索引
- **0:00–0:12** 开场介绍
- **0:12–0:23** C++17 新特性概览
- **0:23–0:40** shared_ptr 支持数组
- **0:40–1:20** 常用特性总览 + 进入结构化绑定
- **1:20–2:55** 结构化绑定基础、应用、限制、底层
- **2:55–7:44** 结构化绑定补充细节 + if/switch 初始化器 + inline 变量
- **7:44–8:47** inline 变量详细讲解
- **8:47–10:33** std::optional
- **10:33–15:00** std::string_view

---

# Session 2：C++17 常用特性详细讲义

## 1. shared_ptr<T[]> —— C++17 新增数组支持
### 背景问题
- C++17 之前 `shared_ptr<int>(new int[10])` 会错误地使用 `delete` 而非 `delete[]`。
- 必须手写 deleter。

### C++17 之后
```cpp
std::shared_ptr<int[]> p(new int[10]);
```
- 自动使用 `delete[]`。
- 不再需要自定义删除器。

### 使用场景
- 安全管理动态数组。
- 避免手动 new[]/delete[] 导致异常安全问题。

---

## 2. 结构化绑定（Structured Binding）
### 定义
允许从 tuple / pair / array / struct 解包多个成员至局部变量：
```cpp
auto [x, y] = std::pair{1, 2};
```

### 常见应用
#### （1）map.insert 返回值解包
```cpp
auto [it, ok] = m.insert({key, value});
```

#### （2）遍历关联容器
```cpp
for (auto& [k, v] : mymap) {}
```

#### （3）结构体成员解包
```cpp
struct Point { int x; int y; };
auto [x, y] = p;
```

### 底层机制（字幕 2:56–3:18）
- 编译器会：
  1. 生成一个临时对象（如 `auto tmp = p;`）
  2. 绑定 `[x, y]` 到其成员

### 限制（字幕 3:41–4:10）
- **必须全部解包**（不能跳过）
- **顺序必须一致**
- **仅支持 public 成员**

### 面试简答
结构化绑定让 tuple/pair/struct 多成员解包更简洁；要求顺序一致且必须解包全部成员，私有成员无法绑定。

---

## 3. if / switch 条件中的初始化器（C++17 新语法）
### 基本语法
```cpp
if (auto it = m.find(k); it != m.end()) { }
```

### 核心作用（字幕 4:38–5:12）
- **限制变量作用域**。
- 在 if / else / switch 内可见，外部不可见。

### 应用场景
#### （1）查找 map
#### （2）线程锁（RAII）
```cpp
if (std::lock_guard<std::mutex> lock(mtx); need_lock()) {}
```

### 面试简答
C++17 支持带初始化器的 if/switch，能限制变量作用域，常与 RAII 对象（如 mutex lock）一起使用。

---

## 4. inline 变量（inline variables）
### 背景（字幕 6:27–6:56）
- C++17 之前：静态成员必须在 .cpp 再定义一次。
- 全局变量不能在头文件定义。

### C++17 的改变
```cpp
inline int value = 5;

struct A {
    inline static int counter = 0;
};
```
- 允许在头文件定义并初始化。
- 由链接器合并定义（字幕 6:56–7:03）。

### 示例：单例模式（字幕 7:15–7:44）
```cpp
class Singleton {
private:
    inline static Singleton inst{};
public:
    static Singleton& instance() { return inst; }
};
```
- 不需要额外的 .cpp 定义。

### 面试简答
inline 变量允许在头文件初始化全局变量与静态成员，由链接器消除重复定义，典型用于单例。

---

## 5. std::optional
### 动机
- 旧方式：返回值 + bool、nullptr、空字符串等。
- 可读性差、语义不清晰。

### optional 的语义
> 表示“可能有 T，也可能没有”。

### 示例
```cpp
std::optional<int> v = parse(s);
if (v) { use(v.value()); }
```

### 注意事项
- `.value()` 前必须确认有值。
- 可用 `.value_or(default)`。

---

## 6. std::string_view
### 核心特性
- 非 owning（仅保存指针与长度）。
- 零拷贝、零分配。

### 优势
```cpp
const std::string& s = "abc"; // 会构造临时 string
std::string_view sv = "abc"; // 不构造 string
```

### 主要风险
- 指向的底层字符串必须有效，否则悬空。

### 应用场景
- HTTP 文本解析
- 字符串切片操作（split 等）
- 日志解析

### 面试简答
string_view 是零拷贝的轻量字符串视图，但需确保引用的底层数据仍有效。

---