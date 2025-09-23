# Doubly Linked Lists: Bi-directional Traversal

## Overview
Data structure where each node contains data and pointers to both the next and previous nodes, enabling efficient bi-directional traversal.

## Structure Definition

### Node Structure
```cpp
template<typename T>
struct DoublyLinkedNode {
    T data;
    DoublyLinkedNode* next;
    DoublyLinkedNode* prev;

    DoublyLinkedNode(const T& value)
        : data(value), next(nullptr), prev(nullptr) {}
};

template<typename T>
class DoublyLinkedList {
private:
    DoublyLinkedNode<T>* head;
    DoublyLinkedNode<T>* tail;
    size_t size;

public:
    DoublyLinkedList() : head(nullptr), tail(nullptr), size(0) {}

    ~DoublyLinkedList() {
        clear();
    }

    void clear() {
        while (head) {
            DoublyLinkedNode<T>* temp = head;
            head = head->next;
            delete temp;
        }
        tail = nullptr;
        size = 0;
    }
};
```

## Core Operations

### Insertion Operations
```cpp
template<typename T>
class DoublyLinkedList {
public:
    // Insert at beginning
    void pushFront(const T& value) {
        DoublyLinkedNode<T>* newNode = new DoublyLinkedNode<T>(value);

        if (!head) {
            // Empty list
            head = tail = newNode;
        } else {
            newNode->next = head;
            head->prev = newNode;
            head = newNode;
        }
        size++;
    }

    // Insert at end
    void pushBack(const T& value) {
        DoublyLinkedNode<T>* newNode = new DoublyLinkedNode<T>(value);

        if (!tail) {
            // Empty list
            head = tail = newNode;
        } else {
            tail->next = newNode;
            newNode->prev = tail;
            tail = newNode;
        }
        size++;
    }

    // Insert at specific position
    void insert(size_t index, const T& value) {
        if (index == 0) {
            pushFront(value);
            return;
        }
        if (index >= size) {
            pushBack(value);
            return;
        }

        DoublyLinkedNode<T>* newNode = new DoublyLinkedNode<T>(value);
        DoublyLinkedNode<T>* current = getNode(index);

        // Insert before current
        newNode->next = current;
        newNode->prev = current->prev;
        current->prev->next = newNode;
        current->prev = newNode;

        size++;
    }

private:
    DoublyLinkedNode<T>* getNode(size_t index) {
        if (index >= size) return nullptr;

        DoublyLinkedNode<T>* current;

        // Optimize: start from head or tail based on index
        if (index < size / 2) {
            // Start from head
            current = head;
            for (size_t i = 0; i < index; ++i) {
                current = current->next;
            }
        } else {
            // Start from tail
            current = tail;
            for (size_t i = size - 1; i > index; --i) {
                current = current->prev;
            }
        }

        return current;
    }
};
```

### Deletion Operations
```cpp
template<typename T>
class DoublyLinkedList {
public:
    // Remove from beginning
    bool popFront() {
        if (!head) return false;

        DoublyLinkedNode<T>* temp = head;

        if (head == tail) {
            // Only one element
            head = tail = nullptr;
        } else {
            head = head->next;
            head->prev = nullptr;
        }

        delete temp;
        size--;
        return true;
    }

    // Remove from end
    bool popBack() {
        if (!tail) return false;

        DoublyLinkedNode<T>* temp = tail;

        if (head == tail) {
            // Only one element
            head = tail = nullptr;
        } else {
            tail = tail->prev;
            tail->next = nullptr;
        }

        delete temp;
        size--;
        return true;
    }

    // Remove at specific position
    bool remove(size_t index) {
        if (index >= size) return false;

        if (index == 0) return popFront();
        if (index == size - 1) return popBack();

        DoublyLinkedNode<T>* nodeToRemove = getNode(index);

        // Update links
        nodeToRemove->prev->next = nodeToRemove->next;
        nodeToRemove->next->prev = nodeToRemove->prev;

        delete nodeToRemove;
        size--;
        return true;
    }

    // Remove by value (first occurrence)
    bool removeValue(const T& value) {
        DoublyLinkedNode<T>* current = head;

        while (current) {
            if (current->data == value) {
                if (current == head) {
                    return popFront();
                } else if (current == tail) {
                    return popBack();
                } else {
                    current->prev->next = current->next;
                    current->next->prev = current->prev;
                    delete current;
                    size--;
                    return true;
                }
            }
            current = current->next;
        }
        return false;
    }
};
```

## Bi-directional Traversal

### Forward and Backward Iteration
```cpp
template<typename T>
class DoublyLinkedList {
public:
    // Forward traversal
    void printForward() const {
        cout << "Forward: ";
        DoublyLinkedNode<T>* current = head;
        while (current) {
            cout << current->data << " ";
            current = current->next;
        }
        cout << endl;
    }

    // Backward traversal
    void printBackward() const {
        cout << "Backward: ";
        DoublyLinkedNode<T>* current = tail;
        while (current) {
            cout << current->data << " ";
            current = current->prev;
        }
        cout << endl;
    }

    // Iterator support
    class Iterator {
    private:
        DoublyLinkedNode<T>* node;

    public:
        Iterator(DoublyLinkedNode<T>* n) : node(n) {}

        T& operator*() { return node->data; }

        Iterator& operator++() {
            node = node->next;
            return *this;
        }

        Iterator& operator--() {
            node = node->prev;
            return *this;
        }

        bool operator!=(const Iterator& other) const {
            return node != other.node;
        }
    };

    Iterator begin() { return Iterator(head); }
    Iterator end() { return Iterator(nullptr); }
    Iterator rbegin() { return Iterator(tail); }
    Iterator rend() { return Iterator(nullptr); }
};
```

### Advanced Traversal Patterns
```cpp
// Range-based operations
template<typename T>
class DoublyLinkedList {
public:
    // Find node with given value
    DoublyLinkedNode<T>* find(const T& value) {
        DoublyLinkedNode<T>* current = head;
        while (current && current->data != value) {
            current = current->next;
        }
        return current;
    }

    // Reverse find (search from tail)
    DoublyLinkedNode<T>* findReverse(const T& value) {
        DoublyLinkedNode<T>* current = tail;
        while (current && current->data != value) {
            current = current->prev;
        }
        return current;
    }

    // Get element at index (optimized with bi-directional access)
    T& at(size_t index) {
        if (index >= size) {
            throw out_of_range("Index out of bounds");
        }

        DoublyLinkedNode<T>* node = getNode(index);
        return node->data;
    }

    // Splice operation: move range from one list to another
    void splice(Iterator pos, DoublyLinkedList& other,
                Iterator first, Iterator last) {
        if (first == last) return;

        // Extract range from other list
        DoublyLinkedNode<T>* firstNode = first.node;
        DoublyLinkedNode<T>* lastNode = last.node->prev;

        // Remove from other list
        if (firstNode->prev) {
            firstNode->prev->next = last.node;
        } else {
            other.head = last.node;
        }

        if (last.node) {
            last.node->prev = firstNode->prev;
        } else {
            other.tail = firstNode->prev;
        }

        // Insert into this list
        DoublyLinkedNode<T>* posNode = pos.node;

        if (posNode) {
            lastNode->next = posNode;
            firstNode->prev = posNode->prev;

            if (posNode->prev) {
                posNode->prev->next = firstNode;
            } else {
                head = firstNode;
            }

            posNode->prev = lastNode;
        } else {
            // Insert at end
            if (tail) {
                tail->next = firstNode;
                firstNode->prev = tail;
            } else {
                head = firstNode;
            }
            tail = lastNode;
            lastNode->next = nullptr;
        }
    }
};
```

## Advantages and Use Cases

### Benefits of Doubly Linked Lists
```cpp
// Comparison with singly linked lists
void demonstrateAdvantages() {
    DoublyLinkedList<int> dll;

    // 1. Efficient deletion when you have a node pointer
    // No need to traverse from head to find previous node

    // 2. Bi-directional traversal
    dll.pushBack(1);
    dll.pushBack(2);
    dll.pushBack(3);

    dll.printForward();   // 1 2 3
    dll.printBackward();  // 3 2 1

    // 3. Better cache locality for certain access patterns
    // Can start from either end based on target position

    // 4. Efficient implementation of deque operations
    dll.pushFront(0);     // O(1)
    dll.pushBack(4);      // O(1)
    dll.popFront();       // O(1)
    dll.popBack();        // O(1)
}
```

### Memory Overhead Analysis
```cpp
// Memory comparison
struct MemoryAnalysis {
    // Singly linked node: data + 1 pointer = sizeof(T) + sizeof(void*)
    // Doubly linked node: data + 2 pointers = sizeof(T) + 2*sizeof(void*)

    void compareMemoryUsage() {
        cout << "Integer singly linked node: "
             << sizeof(int) + sizeof(void*) << " bytes" << endl;
        cout << "Integer doubly linked node: "
             << sizeof(int) + 2*sizeof(void*) << " bytes" << endl;

        // On 64-bit system: 4 + 8 = 12 vs 4 + 16 = 20 bytes
        // 67% memory overhead for the extra pointer
    }
};
```

## Common Interview Problems

### Problem 1: Reverse Doubly Linked List
```cpp
template<typename T>
void DoublyLinkedList<T>::reverse() {
    DoublyLinkedNode<T>* current = head;

    while (current) {
        // Swap next and prev pointers
        DoublyLinkedNode<T>* temp = current->next;
        current->next = current->prev;
        current->prev = temp;

        // Move to next node (which is now prev)
        current = temp;
    }

    // Swap head and tail
    DoublyLinkedNode<T>* temp = head;
    head = tail;
    tail = temp;
}
```

### Problem 2: LRU Cache Implementation
```cpp
class LRUCache {
private:
    struct Node {
        int key, value;
        Node* prev;
        Node* next;
        Node(int k, int v) : key(k), value(v), prev(nullptr), next(nullptr) {}
    };

    unordered_map<int, Node*> cache;
    Node* head;
    Node* tail;
    int capacity;

    void addToHead(Node* node) {
        node->prev = head;
        node->next = head->next;
        head->next->prev = node;
        head->next = node;
    }

    void removeNode(Node* node) {
        node->prev->next = node->next;
        node->next->prev = node->prev;
    }

    Node* removeTail() {
        Node* last = tail->prev;
        removeNode(last);
        return last;
    }

public:
    LRUCache(int cap) : capacity(cap) {
        head = new Node(0, 0);
        tail = new Node(0, 0);
        head->next = tail;
        tail->prev = head;
    }

    int get(int key) {
        auto it = cache.find(key);
        if (it != cache.end()) {
            Node* node = it->second;
            removeNode(node);
            addToHead(node);
            return node->value;
        }
        return -1;
    }

    void put(int key, int value) {
        auto it = cache.find(key);
        if (it != cache.end()) {
            Node* node = it->second;
            node->value = value;
            removeNode(node);
            addToHead(node);
        } else {
            Node* newNode = new Node(key, value);

            if (cache.size() >= capacity) {
                Node* tail_node = removeTail();
                cache.erase(tail_node->key);
                delete tail_node;
            }

            cache[key] = newNode;
            addToHead(newNode);
        }
    }
};
```

### Problem 3: Flatten Multilevel Doubly Linked List
```cpp
class MultilevelNode {
public:
    int val;
    MultilevelNode* prev;
    MultilevelNode* next;
    MultilevelNode* child;

    MultilevelNode(int value) : val(value), prev(nullptr),
                               next(nullptr), child(nullptr) {}
};

MultilevelNode* flatten(MultilevelNode* head) {
    if (!head) return head;

    stack<MultilevelNode*> stk;
    MultilevelNode* current = head;

    while (current) {
        if (current->child) {
            // Save next node for later
            if (current->next) {
                stk.push(current->next);
            }

            // Connect to child
            current->next = current->child;
            current->child->prev = current;
            current->child = nullptr;
        }

        // If we reach the end and have saved nodes
        if (!current->next && !stk.empty()) {
            MultilevelNode* nextNode = stk.top();
            stk.pop();

            current->next = nextNode;
            nextNode->prev = current;
        }

        current = current->next;
    }

    return head;
}
```

## Performance Characteristics

### Time Complexity Summary
```cpp
// Operation complexities for doubly linked list:
void complexityAnalysis() {
    // Access by index: O(n) - but can start from either end
    // Search: O(n)
    // Insertion at head/tail: O(1)
    // Insertion at arbitrary position: O(n) to find + O(1) to insert
    // Deletion at head/tail: O(1)
    // Deletion of known node: O(1) - major advantage over singly linked
    // Deletion by value: O(n) to find + O(1) to delete
}
```

---
*Related: [[Singly linked lists - head tail traversal]] | [[Circular linked lists - cycle detection]]*
*Part of: [[Linked Lists MOC]]*