# Vibe Coding Workflow v3.0  
AI 协作工程工作流（含每阶段 AI 指令模板）

---

# 🟦 0. 项目名称
（填写你的项目名称）

---

# 🟦 1. 系统蓝图（System Blueprint）

用于描述整个系统的顶层结构。

---

## 1.1 系统目标
- （系统要解决什么问题？）
- （关键约束：RT、内存、平台、容错等）

---

## 1.2 顶层模块（Top-Level Modules）
- Module A
- Module B
- Module C

---

## 1.3 子模块拆解（Top-Down）
以树状结构描述：

```
Uploader
 ├── MessageQueue
 ├── ProtocolEncoder
 ├── Transport
 ├── RetryManager
 └── StateTracker
```

---

## 1.4 数据流（Data Flow）
```
Sensor → DataQueue → Uploader → MQTT Broker
```

---

## 1.5 控制流（Control Flow）
（必要时描述流程）

---

## 1.6 建议项目结构
```
project/
 ├── SystemBlueprint.md
 ├── ModuleSpecs/
 ├── include/
 ├── src/
 ├── tests/
 ├── TestPlan.md
 └── UnderstandingNotes.md
```

---

## 📤 给 AI 的指令（Blueprint Prompt）

```
我将描述一个系统，请你根据描述生成系统蓝图（System Blueprint）。

要求：
- 列出顶层模块
- 对每个模块进行子模块拆解（Top-Down）
- 给出数据流（Data Flow）
- 给出控制流（Control Flow）
- 给出项目文件结构建议
- 整体内容不超过 80 行

系统需求如下：
<在此粘贴需求>
```

---

# 🟩 2. 模块规范（Module Spec + Design Logic）

使用统一的“三段结构短文档格式”。

---

## 《ModuleSpecs_Module_spec.md》模板

# Module Spec

## 🟦 ① QuickView（≤10 行）
- Module:
- Purpose:
- Key Idea:
- Thread Model:
- Errors:
- Dependencies:
- Used By:

---

## 🟩 ② Structure（树状结构）
```
MessageQueue
 ├── buffer[QUEUE_SIZE][MAX_LEN]
 ├── head
 ├── tail
 ├── push()
 └── pop()
```

---

## 🟨 ③ Minimal Logic（≤20 行伪代码）
```
push():
    lock
    if next(head)==tail → false
    copy frame
    head=next(head)
    unlock

pop():
    lock
    if head==tail → false
    copy out
    tail=next(tail)
    unlock
```

---

## 📤 给 AI 的指令（Module Spec Prompt）

```
请为模块 <ModuleName> 生成模块规范（Module Spec）。

格式必须严格为：

# QuickView（≤10行）
# Structure（树状结构）
# Minimal Logic（≤20行伪代码）

要求：
- 文档必须短、可读、信息密度高
- 不要写长段解释
- 不要写背景介绍
- 不要超过 60 行
```

---

# 🟨 3. 模块实现（Implementation）

实现顺序必须 **Bottom-Up**  
（先子模块 → 后中间模块 → 最后大模块）

---

## 📤 给 AI 的指令（Implementation Prompt）

```
请根据以下 Module Spec 生成该模块的完整代码。

要求：
- 严格遵守 Spec 的 QuickView / Structure / Minimal Logic
- 不要添加 Spec 未提到的变量、函数、状态或逻辑
- 使用 <C 或 C++>
- 输出 <Module>.h 和 <Module>.cpp（或 .c）
- 在关键逻辑处写少量注释
- 不生成任何额外说明文档

模块 Spec 如下：
<粘贴 Module Spec>
```

---

# 🟧 4. 测试与验证（Validation）

确保代码完全符合 Spec。

---

## TestPlan.md（模板）

```
# Test Plan

## 模块：<Module>

### 测试目标
验证 Spec 与代码实现的一致性。

### 测试点
- 功能行为
- 边界条件（满/空/越界）
- 并发 / 线程安全
- 错误处理
- 异常输入
- 状态机转换

### 测试文件
tests/test_<Module>.cpp
```

---

## 📤 给 AI 的指令（Test Prompt）

```
请根据以下 Module Spec 和模块代码，为 <Module> 生成单元测试。

要求：
- 覆盖 Spec 中的所有边界条件
- 覆盖所有正常路径和异常路径
- 测试文件命名为 test_<Module>.cpp
- 使用 <gtest 或 ztest>
- 不测试 Spec 未定义的行为

Module Spec:
<粘贴 Spec>

模块代码：
<粘贴代码>
```

---

# 🟪 5. 理解与反思（Reflection）

让工程成为“你真正理解的工程”。

---

## UnderstandingNotes.md（模板）

```
# Understanding Notes

## 模块：<Module>

### 我的问题
- 为什么用锁而不是无锁？
- 状态机为什么这样设计？
- 模块边界是否合理？
- 如果扩展，会怎么修改？

### AI 的解释
（将 AI 的回答粘贴在此处）
```

---

## 📤 给 AI 的指令（Reflection Prompt）

```
请解释模块 <Module> 的以下问题：

<列出你的疑问>

要求：
- 尽量简短
- 列表化
- 信息密度高
- 不超过 40 行
```

---

# 🏁 总结：完整工作流结构

```
1. Blueprint（结构设计）
   → Prompt：生成系统蓝图

2. Module Spec（三段短文档）
   → Prompt：生成模块 Spec

3. Implementation（自底向上）
   → Prompt：根据 Spec 生成代码

4. Validation（测试）
   → Prompt：基于 Spec + code 生成单元测试

5. Reflection（理解）
   → Prompt：解释问题、补齐理解
```


