# Circular Linked Lists: Cycle Detection

## Overview
Linked list data structure where the last node points back to a previous node, creating a cycle. Understanding cycle detection is crucial for both implementing circular lists and debugging corrupted linked lists.

## Types of Circular Lists

### Singly Circular Linked List
```cpp
template<typename T>
struct CircularNode {
    T data;
    CircularNode* next;

    CircularNode(const T& value) : data(value), next(this) {}  // Points to itself initially
};

template<typename T>
class CircularLinkedList {
private:
    CircularNode<T>* tail;  // Keep track of tail, not head
    size_t size;

public:
    CircularLinkedList() : tail(nullptr), size(0) {}

    // Insert at beginning (after tail)
    void insertFront(const T& value) {
        CircularNode<T>* newNode = new CircularNode<T>(value);

        if (!tail) {
            // First node
            tail = newNode;
            newNode->next = newNode;  // Points to itself
        } else {
            newNode->next = tail->next;  // Point to current head
            tail->next = newNode;       // Tail points to new head
        }
        size++;
    }

    // Insert at end (new tail)
    void insertBack(const T& value) {
        insertFront(value);  // Insert at front
        tail = tail->next;   // Move tail to new node
    }

    void printList() const {
        if (!tail) return;

        CircularNode<T>* current = tail->next;  // Start from head
        do {
            cout << current->data << " ";
            current = current->next;
        } while (current != tail->next);
        cout << endl;
    }
};
```

### Doubly Circular Linked List
```cpp
template<typename T>
struct DoublyCircularNode {
    T data;
    DoublyCircularNode* next;
    DoublyCircularNode* prev;

    DoublyCircularNode(const T& value) : data(value), next(this), prev(this) {}
};

template<typename T>
class DoublyCircularList {
private:
    DoublyCircularNode<T>* head;
    size_t size;

public:
    DoublyCircularList() : head(nullptr), size(0) {}

    void insert(const T& value) {
        DoublyCircularNode<T>* newNode = new DoublyCircularNode<T>(value);

        if (!head) {
            head = newNode;
        } else {
            // Insert after head
            newNode->next = head->next;
            newNode->prev = head;
            head->next->prev = newNode;
            head->next = newNode;
        }
        size++;
    }

    void printForward() const {
        if (!head) return;

        DoublyCircularNode<T>* current = head;
        do {
            cout << current->data << " ";
            current = current->next;
        } while (current != head);
        cout << endl;
    }

    void printBackward() const {
        if (!head) return;

        DoublyCircularNode<T>* current = head->prev;
        do {
            cout << current->data << " ";
            current = current->prev;
        } while (current != head->prev);
        cout << endl;
    }
};
```

## Cycle Detection Algorithms

### Floyd's Cycle Detection (Tortoise and Hare)
```cpp
template<typename T>
bool hasCycle(Node<T>* head) {
    if (!head || !head->next) return false;

    Node<T>* slow = head;      // Tortoise: moves 1 step
    Node<T>* fast = head;      // Hare: moves 2 steps

    while (fast && fast->next) {
        slow = slow->next;
        fast = fast->next->next;

        if (slow == fast) {
            return true;  // Cycle detected
        }
    }

    return false;  // No cycle
}

// Find the start of the cycle
template<typename T>
Node<T>* findCycleStart(Node<T>* head) {
    if (!hasCycle(head)) return nullptr;

    Node<T>* slow = head;
    Node<T>* fast = head;

    // Phase 1: Detect cycle
    while (fast && fast->next) {
        slow = slow->next;
        fast = fast->next->next;
        if (slow == fast) break;
    }

    // Phase 2: Find start of cycle
    slow = head;
    while (slow != fast) {
        slow = slow->next;
        fast = fast->next;
    }

    return slow;  // Start of cycle
}

// Find cycle length
template<typename T>
int getCycleLength(Node<T>* head) {
    Node<T>* meetingPoint = nullptr;
    Node<T>* slow = head;
    Node<T>* fast = head;

    // Find meeting point
    while (fast && fast->next) {
        slow = slow->next;
        fast = fast->next->next;
        if (slow == fast) {
            meetingPoint = slow;
            break;
        }
    }

    if (!meetingPoint) return 0;  // No cycle

    // Count nodes in cycle
    int length = 1;
    Node<T>* current = meetingPoint->next;
    while (current != meetingPoint) {
        current = current->next;
        length++;
    }

    return length;
}
```

### Brent's Cycle Detection Algorithm
```cpp
template<typename T>
bool brentCycleDetection(Node<T>* head) {
    if (!head) return false;

    Node<T>* tortoise = head;
    Node<T>* hare = head;
    int power = 1;
    int lambda = 1;  // Cycle length

    // Find cycle length
    while (hare->next) {
        if (power == lambda) {
            tortoise = hare;
            power *= 2;
            lambda = 0;
        }
        hare = hare->next;
        lambda++;

        if (tortoise == hare) {
            break;  // Cycle found
        }
    }

    if (!hare->next) return false;  // No cycle

    // Find cycle start
    tortoise = hare = head;
    for (int i = 0; i < lambda; ++i) {
        hare = hare->next;
    }

    while (tortoise != hare) {
        tortoise = tortoise->next;
        hare = hare->next;
    }

    return true;
}
```

### Hash Set Approach
```cpp
template<typename T>
bool hasCycleHashSet(Node<T>* head) {
    unordered_set<Node<T>*> visited;

    Node<T>* current = head;
    while (current) {
        if (visited.find(current) != visited.end()) {
            return true;  // Cycle detected
        }

        visited.insert(current);
        current = current->next;
    }

    return false;  // No cycle
}

// Time Complexity: O(n)
// Space Complexity: O(n) - requires extra space for hash set
```

## Advanced Cycle Problems

### Remove Cycle from Linked List
```cpp
template<typename T>
void removeCycle(Node<T>* head) {
    if (!head || !hasCycle(head)) return;

    Node<T>* slow = head;
    Node<T>* fast = head;

    // Find meeting point
    while (fast && fast->next) {
        slow = slow->next;
        fast = fast->next->next;
        if (slow == fast) break;
    }

    // Find start of cycle
    slow = head;
    while (slow->next != fast->next) {
        slow = slow->next;
        fast = fast->next;
    }

    // Remove cycle by setting the last node to nullptr
    fast->next = nullptr;
}
```

### Detect Cycle in Doubly Linked List
```cpp
template<typename T>
bool hasCycleDoubly(DoublyNode<T>* head) {
    if (!head) return false;

    unordered_set<DoublyNode<T>*> visited;
    DoublyNode<T>* current = head;

    while (current) {
        if (visited.find(current) != visited.end()) {
            return true;
        }

        visited.insert(current);

        // Check both forward and backward cycles
        if (current->next && visited.find(current->next) != visited.end()) {
            return true;
        }

        current = current->next;
    }

    return false;
}
```

## Interview Problems and Solutions

### Problem 1: Happy Number (Uses Cycle Detection)
```cpp
bool isHappy(int n) {
    auto getNext = [](int num) {
        int sum = 0;
        while (num > 0) {
            int digit = num % 10;
            sum += digit * digit;
            num /= 10;
        }
        return sum;
    };

    int slow = n;
    int fast = n;

    do {
        slow = getNext(slow);
        fast = getNext(getNext(fast));
    } while (slow != fast);

    return slow == 1;
}
```

### Problem 2: Find Duplicate Number (Uses Cycle Detection)
```cpp
int findDuplicate(vector<int>& nums) {
    // Treat array as linked list where nums[i] is the next pointer
    int slow = nums[0];
    int fast = nums[0];

    // Phase 1: Find intersection point in cycle
    do {
        slow = nums[slow];
        fast = nums[nums[fast]];
    } while (slow != fast);

    // Phase 2: Find entrance to cycle
    slow = nums[0];
    while (slow != fast) {
        slow = nums[slow];
        fast = nums[fast];
    }

    return slow;
}
```

### Problem 3: Circular Array Loop
```cpp
bool circularArrayLoop(vector<int>& nums) {
    int n = nums.size();

    for (int i = 0; i < n; ++i) {
        if (nums[i] == 0) continue;

        int slow = i, fast = i;
        bool forward = nums[i] > 0;

        // Floyd's algorithm
        while (true) {
            slow = getNext(nums, slow, forward);
            if (slow == -1) break;

            fast = getNext(nums, fast, forward);
            if (fast == -1) break;

            fast = getNext(nums, fast, forward);
            if (fast == -1) break;

            if (slow == fast) {
                // Check if cycle length > 1
                if (slow == getNext(nums, slow, forward)) break;
                return true;
            }
        }

        // Mark visited elements
        slow = i;
        int val = nums[i];
        while (nums[slow] * val > 0) {
            int next = getNext(nums, slow, val > 0);
            nums[slow] = 0;
            slow = next;
        }
    }

    return false;
}

private:
int getNext(vector<int>& nums, int i, bool forward) {
    bool direction = nums[i] > 0;
    if (direction != forward) return -1;  // Direction changed

    int next = (i + nums[i]) % nums.size();
    if (next < 0) next += nums.size();

    if (next == i) return -1;  // Self-loop
    return next;
}
```

## Performance Analysis

### Algorithm Comparison
```cpp
void compareAlgorithms() {
    cout << "Cycle Detection Algorithm Comparison:" << endl;
    cout << "Floyd's (Tortoise & Hare):" << endl;
    cout << "  Time: O(n), Space: O(1)" << endl;
    cout << "  Pros: Constant space, simple implementation" << endl;
    cout << "  Cons: Requires careful pointer arithmetic" << endl;

    cout << "\nBrent's Algorithm:" << endl;
    cout << "  Time: O(n), Space: O(1)" << endl;
    cout << "  Pros: Often faster in practice, constant space" << endl;
    cout << "  Cons: More complex implementation" << endl;

    cout << "\nHash Set Approach:" << endl;
    cout << "  Time: O(n), Space: O(n)" << endl;
    cout << "  Pros: Simple to understand and implement" << endl;
    cout << "  Cons: Requires extra space" << endl;
}
```

### Use Cases for Circular Lists
```cpp
// 1. Round-robin scheduling
class RoundRobinScheduler {
    CircularLinkedList<string> processes;
    CircularNode<string>* current;

public:
    void addProcess(const string& name) {
        processes.insertBack(name);
        if (!current) current = processes.getHead();
    }

    string getNextProcess() {
        if (!current) return "";
        string process = current->data;
        current = current->next;
        return process;
    }
};

// 2. Music playlist with repeat
class Playlist {
    CircularLinkedList<string> songs;
    CircularNode<string>* currentSong;

public:
    void addSong(const string& title) {
        songs.insertBack(title);
    }

    string playNext() {
        if (!currentSong) currentSong = songs.getHead();
        string song = currentSong->data;
        currentSong = currentSong->next;
        return song;
    }

    string playPrevious() {
        // In doubly circular list
        currentSong = currentSong->prev;
        return currentSong->data;
    }
};
```

---
*Related: [[Doubly linked lists - bi-directional traversal]] | [[Head pointer management]]*
*Part of: [[Linked Lists MOC]]*