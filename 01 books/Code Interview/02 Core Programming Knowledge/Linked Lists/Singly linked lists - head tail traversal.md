# Singly Linked Lists: Head, Tail, Traversal

## Overview
Basic linked list structure with forward-only traversal capability.

## Structure
```
[Data|Next] -> [Data|Next] -> [Data|Next] -> NULL
     ^                              ^
   Head                           Tail
```

## Key Components
### Head Pointer
- Points to first node in the list
- Entry point for all operations
- Critical for list access

### Tail Pointer (Optional)
- Points to last node in the list
- Enables O(1) append operations
- Must be maintained during modifications

### Node Structure
```cpp
struct ListNode {
    int val;
    ListNode* next;
    ListNode(int x) : val(x), next(nullptr) {}
};
```

## Traversal
```cpp
void traverse(ListNode* head) {
    ListNode* current = head;
    while (current != nullptr) {
        std::cout << current->val << " ";
        current = current->next;
    }
}
```

## Time Complexity
- **Access**: O(n) - must traverse from head
- **Search**: O(n) - linear search only
- **Insert at head**: O(1)
- **Insert at tail**: O(1) with tail pointer, O(n) without
- **Delete**: O(n) - need to find predecessor

---
*Related: [[Head pointer management]] | [[Doubly linked lists - bi-directional traversal]]*
*Part of: [[Linked Lists MOC]]*