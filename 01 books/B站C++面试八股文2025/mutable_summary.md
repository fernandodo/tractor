# C++ `mutable` 关键字讲义总结

## 1. 导入：mutable 的面试考点

mutable 的核心用途：

1. 允许 const 成员函数修改特定成员变量  
2. 允许 lambda 在值捕获时修改捕获副本

这两个用途构成了全部使用场景。

---

## 2. const 成员函数与 this 指针语义

### 2.1 const 修饰的是 this 指针

```cpp
int size() const;
```

等价于：

```cpp
int size() const {
    // this 的类型为 const ClassName* const
}
```

**含义：**

- const 成员函数不能修改成员变量  
- this 指向的对象视为 const

如果要修改成员变量 → 会直接编译错误。

---

## 3. mutable 修饰非静态成员变量  
### 案例：线程池的阻塞队列（Blocking Queue）

```cpp
int size() const {
    std::lock_guard<std::mutex> lk(mtx);
    return q.size();
}
```

加锁/解锁操作会修改 mutex 内部状态，但 `size()` 被 const 修饰，因此会报错。

### 解决：将 mutex 声明为 mutable

```cpp
class BlockQueue {
private:
    mutable std::mutex mtx;

public:
    int size() const {
        std::lock_guard<std::mutex> lk(mtx);
        return q.size();
    }
};
```

### 是否破坏 const 语义？

不会。  

- const 语义指“对外逻辑不可变”  
- mutex 的状态对外不可见  
- 属于内部实现细节

因此 mutable 不破坏 const 的 *logical constness*。

---

## 4. 惰性求值 lazy evaluation 案例

```cpp
class Widget {
private:
    mutable Other other_;

public:
    Other get_other() const {
        if (!other_.valid()) {
            other_ = Other(...);  // 惰性构造
        }
        return other_;
    }
};
```

修改缓存对象并不改变对外语义，只改变内部状态，因此可使用 mutable。

---

## 5. mutable 修饰 lambda 表达式

### 5.1 默认 lambda 的 operator() 是 const

按值捕获：

```cpp
auto f = [i]() { i++; };  // 错误：operator() 是 const
```

### 5.2 按引用捕获为何能修改？

```cpp
int i = 0;
auto f = [&i]() { i++; };  // OK
```

- 成员变量是引用  
- const 限制不能换引用对象，但可以通过引用修改外部值

### 5.3 解决按值捕获不能改的方式：使用 mutable

```cpp
int i = 0;
auto f = [i]() mutable {
    i++;
    return i;
};
```

mutable 去掉 lambda 的 const 限制 → 允许修改捕获副本。

### 5.4 为什么 lambda 默认不允许修改捕获变量？

为了防止误会开发者以为：

```cpp
[x]() { x++; }   // 修改了外部 x？
```

因此默认 operator() const，除非显式 mutable。

---

## 6. 终极总结：mutable 的真正语义

### ✔ mutable = const 的受控例外（Controlled Exception）  
允许修改：

- mutex  
- 缓存  
- 调试计数器  
- lambda 的 value capture 副本

### ✔ 不破坏 const 的“逻辑只读”（logical constness）

---

## 7. 最经典的一句话总结

> **mutable 表示：虽然函数是 const，但修改这个变量“算我没改”。**

---

