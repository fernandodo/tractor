# 🎯 C++ 工厂模式体系总结

## 一、简单工厂（Simple Factory）

### 🧩 核心思想  
把对象的创建逻辑集中在一个工厂中，  
客户端只需指定类型，不关心如何 `new`。

### 💻 示例
```cpp
#include <iostream>
#include <memory>
#include <string>

class Character {
public:
    virtual void attack() = 0;
    virtual ~Character() = default;
};

class Warrior : public Character {
public:
    void attack() override { std::cout << "Warrior slashes with a sword!\n"; }
};

class Mage : public Character {
public:
    void attack() override { std::cout << "Mage casts a fireball!\n"; }
};

class CharacterFactory {
public:
    static std::unique_ptr<Character> createCharacter(const std::string& type) {
        if (type == "warrior") return std::make_unique<Warrior>();
        else if (type == "mage") return std::make_unique<Mage>();
        else return nullptr;
    }
};

int main() {
    auto c = CharacterFactory::createCharacter("mage");
    c->attack();
}
```

### ⚙️ 特点
- 创建逻辑集中、易于理解；
- 但每新增产品（如 Archer）都要修改工厂代码；
- 不符合开放封闭原则（OCP）。

---

## 二、工厂方法（Factory Method）

### 🧩 核心思想  
让子类决定创建哪种产品对象。  
基类定义接口，具体工厂实现创建细节。

### 💻 示例
```cpp
#include <iostream>
#include <memory>

class Character {
public:
    virtual void attack() = 0;
    virtual ~Character() = default;
};

class Warrior : public Character {
public:
    void attack() override { std::cout << "Warrior attacks!\n"; }
};

class Mage : public Character {
public:
    void attack() override { std::cout << "Mage casts fireball!\n"; }
};

class CharacterFactory {
public:
    virtual std::unique_ptr<Character> createCharacter() = 0;
    virtual ~CharacterFactory() = default;
};

class WarriorFactory : public CharacterFactory {
public:
    std::unique_ptr<Character> createCharacter() override {
        return std::make_unique<Warrior>();
    }
};

class MageFactory : public CharacterFactory {
public:
    std::unique_ptr<Character> createCharacter() override {
        return std::make_unique<Mage>();
    }
};

int main() {
    std::unique_ptr<CharacterFactory> factory = std::make_unique<MageFactory>();
    auto character = factory->createCharacter();
    character->attack();
}
```

### ⚙️ 特点
- 客户端依赖抽象接口，而非具体类；
- 扩展新产品只需新增工厂类；
- 类数量增加，结构比简单工厂复杂。

---

## 三、抽象工厂（Abstract Factory）

### 🧩 核心思想  
一次性创建一整套**相互关联、风格一致**的对象（如角色 + 武器）。  
解决“物品配套生产”的问题。

### 💻 示例
```cpp
#include <iostream>
#include <memory>

class Character {
public:
    virtual void attack() = 0;
    virtual ~Character() = default;
};

class Weapon {
public:
    virtual void use() = 0;
    virtual ~Weapon() = default;
};

// Human 系列
class HumanKnight : public Character {
public:
    void attack() override { std::cout << "Human Knight charges forward!\n"; }
};
class HumanSword : public Weapon {
public:
    void use() override { std::cout << "Swinging a Human Sword!\n"; }
};

// Orc 系列
class OrcBerserker : public Character {
public:
    void attack() override { std::cout << "Orc Berserker smashes!\n"; }
};
class OrcAxe : public Weapon {
public:
    void use() override { std::cout << "Swinging an Orc Axe!\n"; }
};

class AbstractFactory {
public:
    virtual std::unique_ptr<Character> createCharacter() = 0;
    virtual std::unique_ptr<Weapon> createWeapon() = 0;
    virtual ~AbstractFactory() = default;
};

class HumanFactory : public AbstractFactory {
public:
    std::unique_ptr<Character> createCharacter() override {
        return std::make_unique<HumanKnight>();
    }
    std::unique_ptr<Weapon> createWeapon() override {
        return std::make_unique<HumanSword>();
    }
};

class OrcFactory : public AbstractFactory {
public:
    std::unique_ptr<Character> createCharacter() override {
        return std::make_unique<OrcBerserker>();
    }
    std::unique_ptr<Weapon> createWeapon() override {
        return std::make_unique<OrcAxe>();
    }
};

int main() {
    std::unique_ptr<AbstractFactory> factory = std::make_unique<HumanFactory>();
    auto character = factory->createCharacter();
    auto weapon = factory->createWeapon();
    character->attack();
    weapon->use();
}
```

### ⚙️ 特点
- 一次生成成套对象（角色 + 武器）；
- 保证风格一致；
- 新增套装（Human → Orc → Elf）无需改旧代码；
- 但新增物品类型（如盔甲）仍需修改所有工厂。

---

## 四、对比总结

| 模式 | 创建范围 | 扩展方式 | 优点 | 缺点 | 典型用途 |
|------|------------|-----------|------|------|-----------|
| **简单工厂** | 单个对象 | 修改工厂 | 简单直观 | 不符合 OCP | 小型项目 |
| **工厂方法** | 单类产品 | 新增工厂类 | 符合 OCP | 类增多 | 插件、策略、驱动 |
| **抽象工厂** | 一整套对象 | 新增套装 | 产品配套一致 | 扩展维度有限 | GUI 主题、游戏装备、跨平台接口 |

---

## 五、扩展与局限

- 抽象工厂解决了“成套产品一致性”的问题；  
- 但仍需修改工厂来添加新产品（如新职业）；  
- 若想完全避免修改源码，可结合**注册表机制**或**插件式子工厂**改进；  
- 在实际工程中，通常以“抽象工厂 + 注册机制”的方式实现动态扩展。

---

## ✅ 六、总结一句话

> - **简单工厂**：集中创建逻辑；  
> - **工厂方法**：子类决定产品类型；  
> - **抽象工厂**：创建成套配套对象。  
>  
> 抽象工厂是“成套生产”的解决方案，  
> 工厂方法是“可扩展生产”的方案，  
> 简单工厂则是“集中生产”的快速实现。

## 视频时间点
根据字幕文件《细讲工厂模式》中出现的时间标记，各种工厂模式讲解的时间点如下：

|工厂模式|视频时间段|内容摘要|
|---|---|---|
|**简单工厂（Simple Factory）**|⏰ **约 16:17 – 16:29**|讲解“通过参数来决定创建对象的实例”，适用于创建逻辑简单、产品类型稳定的情况。|
|**工厂方法（Factory Method）**|⏰ **约 16:29 – 16:45**|介绍“创建流程较复杂，需要分离生成与初始化”，用于对象类型会动态变化、经常需要修改的场景。|
|**抽象工厂（Abstract Factory）**|⏰ **约 16:45 – 17:10**|说明抽象工厂“用于创建相关或依赖对象的家族”，例如指定种族后生成对应职业和武器等一系列关联产品。|
