# Static vs Dynamic Arrays

## Overview
Understanding the fundamental differences between static and dynamic arrays, their use cases, and performance implications.

## Static Arrays

### Characteristics
**Fixed Size**:
- Size determined at compile time or declaration
- Cannot be resized during program execution
- Memory allocated on stack or as fixed heap allocation
- Size must be known in advance

**Memory Layout**:
- Contiguous memory allocation
- Predictable memory usage
- No metadata overhead for size tracking
- Direct hardware optimization possible

### Examples by Language
```c
// C - Stack allocated
int arr[100];  // Fixed size of 100 integers

// C - Heap allocated with fixed size
int *arr = malloc(100 * sizeof(int));
```

```java
// Java - Fixed size after creation
int[] arr = new int[100];  // Cannot be resized
```

```python
# Python - Arrays module for fixed-type arrays
import array
arr = array.array('i', [0] * 100)  # Fixed type, but still dynamic size
```

### Advantages
- **Performance**: No resizing overhead, predictable access times
- **Memory Efficiency**: No extra space for growth or metadata
- **Cache Locality**: Contiguous memory layout optimizes cache usage
- **Simplicity**: No complex memory management or reallocation logic

### Disadvantages
- **Inflexibility**: Cannot adapt to changing data requirements
- **Memory Waste**: May allocate more space than needed
- **Runtime Errors**: Buffer overflows if size is exceeded
- **Planning Required**: Must estimate maximum size in advance

## Dynamic Arrays

### Characteristics
**Variable Size**:
- Can grow and shrink during program execution
- Size determined at runtime based on usage
- Automatic memory management and reallocation
- Flexible capacity management

**Growth Strategy**:
- Typically double capacity when full (growth factor of 2)
- Some implementations use 1.5x or 1.618x (golden ratio)
- Amortized O(1) insertion time
- Trade-off between memory usage and reallocation frequency

### Examples by Language
```python
# Python - Lists are dynamic arrays
arr = []
arr.append(1)  # Grows automatically
arr.extend([2, 3, 4])  # Can add multiple elements
```

```java
// Java - ArrayList
ArrayList<Integer> arr = new ArrayList<>();
arr.add(1);  // Automatic resizing
arr.add(2);  // Capacity management handled internally
```

```cpp
// C++ - std::vector
std::vector<int> arr;
arr.push_back(1);  // Dynamic growth
arr.resize(100);   // Explicit resizing
```

```javascript
// JavaScript - Arrays are dynamic by default
let arr = [];
arr.push(1, 2, 3);  // Automatic growth
arr.length = 100;   // Can set length explicitly
```

### Advantages
- **Flexibility**: Adapts to changing data requirements
- **Memory Efficiency**: Only allocates what's needed (plus growth buffer)
- **Ease of Use**: No need to predict maximum size
- **Safety**: Automatic bounds checking in many implementations

### Disadvantages
- **Performance Overhead**: Reallocation and copying costs during growth
- **Memory Overhead**: Extra capacity for future growth
- **Unpredictable Performance**: Occasional O(n) operations during resize
- **Fragmentation**: May not utilize memory as efficiently

## Performance Comparison

### Time Complexity
| Operation | Static Array | Dynamic Array |
|-----------|--------------|---------------|
| Access    | O(1)         | O(1)          |
| Insert (end) | O(1)*     | O(1) amortized |
| Insert (middle) | O(n)*  | O(n)          |
| Delete    | O(n)*        | O(n)          |
| Search    | O(n)         | O(n)          |

*Assuming sufficient capacity

### Space Complexity
**Static Arrays**:
- Exactly n elements * element_size
- No additional metadata
- May waste space if not fully utilized

**Dynamic Arrays**:
- n elements * element_size + growth buffer
- Additional metadata (size, capacity)
- Typically 1.5-2x actual usage for growth capacity

## Use Case Guidelines

### Choose Static Arrays When:
- **Size is Known**: Fixed or predictable data requirements
- **Performance Critical**: Real-time systems, embedded programming
- **Memory Constrained**: Limited memory environments
- **Simple Requirements**: Basic data storage without complex operations

**Examples**:
- Game coordinates (fixed board size)
- Mathematical matrices (known dimensions)
- Embedded sensor readings (fixed sample rates)
- Graphics programming (fixed resolution buffers)

### Choose Dynamic Arrays When:
- **Size is Unknown**: Data size varies based on input or user interaction
- **Flexibility Needed**: Requirements may change during development
- **Ease of Use**: Rapid prototyping and development
- **General Purpose**: Most application programming scenarios

**Examples**:
- User input lists (shopping carts, playlists)
- Data processing pipelines (variable input sizes)
- Web application data (user-generated content)
- Algorithm implementations (general-purpose utilities)

## Implementation Considerations

### Growth Strategies
```python
# Example: Dynamic array implementation with 2x growth
class DynamicArray:
    def __init__(self):
        self.capacity = 1
        self.size = 0
        self.data = [None] * self.capacity

    def append(self, item):
        if self.size >= self.capacity:
            self._resize()
        self.data[self.size] = item
        self.size += 1

    def _resize(self):
        old_capacity = self.capacity
        self.capacity *= 2  # Double the capacity
        new_data = [None] * self.capacity
        for i in range(self.size):
            new_data[i] = self.data[i]
        self.data = new_data
```

### Memory Management
**Manual Management** (C/C++):
```c
// Static allocation
int static_arr[100];

// Dynamic allocation
int *dynamic_arr = malloc(100 * sizeof(int));
// Must call free(dynamic_arr) when done

// Reallocation for growth
dynamic_arr = realloc(dynamic_arr, 200 * sizeof(int));
```

**Automatic Management** (Java, Python):
- Garbage collection handles memory cleanup
- Language runtime manages reallocation
- Developer focuses on logic rather than memory management

## Hybrid Approaches

### Pre-allocated Dynamic Arrays
```java
// ArrayList with initial capacity
ArrayList<Integer> list = new ArrayList<>(1000);
// Avoids initial reallocations if size is roughly known
```

### Array Lists with Size Hints
```python
# Python list with size hint (not enforced)
arr = [None] * 1000  # Pre-allocate space
# Can still grow beyond initial size
```

### Memory Pools
```cpp
// C++ with custom allocator
std::vector<int, pool_allocator<int>> arr;
// Uses pre-allocated memory pool to reduce allocation overhead
```

---
*Related: [[Array basics - contiguous memory O(1) access O(n) insert-delete]] | [[Language-specific pitfalls - C C++ Java C# JavaScript]]*
*Part of: [[Arrays and Strings MOC]]*