# Array Basics: Contiguous Memory, O(1) Access, O(n) Insert/Delete

## Overview
Fundamental characteristics of arrays as a data structure.

## Memory Layout
- **Contiguous Memory**: Array elements stored in adjacent memory locations
- **Benefits**: Cache efficiency, predictable memory access patterns
- **Implications**: Fixed size in many implementations

## Time Complexity
### Access: O(1)
- Direct index calculation: `base_address + (index × element_size)`
- Constant time regardless of array size
- Random access capability

### Insert/Delete: O(n)
- **Insertion**: Must shift elements to make space
- **Deletion**: Must shift elements to fill gap
- **Worst Case**: Insert/delete at beginning requires shifting all elements

## Code Example
```cpp
#include <vector>
#include <iostream>

// O(1) access
int value = arr[index];

// O(n) insertion at beginning
std::vector<int> arr = {1, 2, 3, 4, 5};
arr.insert(arr.begin(), new_value);  // Shifts all existing elements

// O(n) deletion from middle
arr.erase(arr.begin() + middle_index);  // Shifts elements after index

// Static array example
int static_arr[5] = {1, 2, 3, 4, 5};
int val = static_arr[2];  // O(1) access
```

## Trade-offs
- Fast access vs slow modification
- Memory efficiency vs dynamic sizing

---
*Related: [[Static vs dynamic arrays]] | [[Language-specific pitfalls - C C++ Java C# JavaScript]]*
*Part of: [[Arrays and Strings MOC]]*