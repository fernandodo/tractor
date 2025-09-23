# Language-Specific Pitfalls: C, C++, Java, C#, JavaScript

## Overview
Common programming pitfalls and gotchas specific to different languages when working with arrays and strings.

## C++ Pitfalls

### Array Bounds and Memory Management
```cpp
// PITFALL 1: Array decay to pointer
void printArray(int arr[10]) {  // Actually receives int* arr
    // sizeof(arr) returns sizeof(int*), not sizeof(array)
    cout << sizeof(arr) << endl;  // Wrong! Returns 8 on 64-bit, not 40
}

// CORRECT: Use references or pass size
void printArray(int (&arr)[10]) {  // True array reference
    cout << sizeof(arr) << endl;  // Correct! Returns 40
}

template<size_t N>
void printArray(int (&arr)[N]) {  // Template for any size
    cout << "Array size: " << N << endl;
}
```

### Vector vs Array Confusion
```cpp
// PITFALL 2: Assuming vector behaves like C array
vector<int> vec(5);  // Creates vector with 5 elements (all 0)
int arr[5];          // Creates array with 5 uninitialized elements

// PITFALL 3: Iterator invalidation
vector<int> vec = {1, 2, 3, 4, 5};
for (auto it = vec.begin(); it != vec.end(); ++it) {
    if (*it == 3) {
        vec.erase(it);  // DANGER: Invalidates iterator
        // ++it;        // UNDEFINED BEHAVIOR
    }
}

// CORRECT: Use return value of erase
for (auto it = vec.begin(); it != vec.end();) {
    if (*it == 3) {
        it = vec.erase(it);  // erase returns next valid iterator
    } else {
        ++it;
    }
}
```

### String Handling
```cpp
// PITFALL 4: C-style string vs std::string confusion
char* cstr = "Hello";         // String literal in read-only memory
// cstr[0] = 'h';             // UNDEFINED BEHAVIOR
string stdstr = "Hello";      // std::string is mutable
stdstr[0] = 'h';             // OK

// PITFALL 5: String concatenation performance
string result = "";
for (int i = 0; i < 1000; ++i) {
    result += "text";  // O(n²) behavior due to repeated copying
}

// BETTER: Reserve space or use stringstream
string result;
result.reserve(4000);  // Pre-allocate space
for (int i = 0; i < 1000; ++i) {
    result += "text";  // Much faster
}
```

## C Pitfalls

### Memory Management
```c
// PITFALL 1: Buffer overflows
char buffer[10];
strcpy(buffer, "This string is too long");  // BUFFER OVERFLOW

// CORRECT: Use safe functions
char buffer[10];
strncpy(buffer, source, sizeof(buffer) - 1);
buffer[sizeof(buffer) - 1] = '\0';  // Ensure null termination

// PITFALL 2: Dangling pointers
char* createString() {
    char local[100] = "Hello";
    return local;  // WRONG: Returns pointer to local variable
}

// CORRECT: Use dynamic allocation or static
char* createString() {
    char* str = malloc(100);
    strcpy(str, "Hello");
    return str;  // Caller must free()
}
```

### Array Parameter Decay
```c
// PITFALL 3: Array size lost in function parameters
void processArray(int arr[100]) {  // Actually int* arr
    int size = sizeof(arr) / sizeof(arr[0]);  // WRONG! Always 1 or 2
}

// CORRECT: Always pass size separately
void processArray(int* arr, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        // Process arr[i]
    }
}
```

## Java Pitfalls

### String Immutability
```java
// PITFALL 1: String concatenation in loops
String result = "";
for (int i = 0; i < 1000; i++) {
    result += "text";  // Creates new String object each time - O(n²)
}

// CORRECT: Use StringBuilder
StringBuilder sb = new StringBuilder();
for (int i = 0; i < 1000; i++) {
    sb.append("text");  // O(n) amortized
}
String result = sb.toString();

// PITFALL 2: String comparison
String str1 = new String("hello");
String str2 = new String("hello");
if (str1 == str2) {  // WRONG: Compares references, not content
    // This will be false
}

// CORRECT: Use equals()
if (str1.equals(str2)) {  // Compares content
    // This will be true
}
```

### Array Initialization
```java
// PITFALL 3: Array vs ArrayList confusion
int[] arr = new int[5];     // Fixed size, initialized to 0
ArrayList<Integer> list = new ArrayList<>();  // Dynamic size

// PITFALL 4: Array copying
int[] original = {1, 2, 3, 4, 5};
int[] copy = original;  // WRONG: Both refer to same array
copy[0] = 10;  // Changes original[0] too!

// CORRECT: Create new array
int[] copy = original.clone();  // Shallow copy
// Or for deep copying of object arrays:
Arrays.copyOf(original, original.length);
```

## C# Pitfalls

### String Handling
```csharp
// PITFALL 1: String concatenation performance (similar to Java)
string result = "";
for (int i = 0; i < 1000; i++) {
    result += "text";  // O(n²) - creates new string each time
}

// CORRECT: Use StringBuilder
StringBuilder sb = new StringBuilder();
for (int i = 0; i < 1000; i++) {
    sb.Append("text");
}
string result = sb.ToString();

// PITFALL 2: Null vs empty string confusion
string str = null;
if (str == "") {  // NullReferenceException if str is null
    // Handle empty string
}

// CORRECT: Check for null first
if (string.IsNullOrEmpty(str)) {
    // Handles both null and empty
}
```

### Array vs List
```csharp
// PITFALL 3: Array resizing misconception
int[] arr = new int[5];
// arr.Length = 10;  // COMPILE ERROR: Length is read-only

// Use List<T> for dynamic sizing
List<int> list = new List<int>();
list.Add(1);  // Dynamic resizing

// PITFALL 4: Foreach modification
List<int> numbers = new List<int> {1, 2, 3, 4, 5};
foreach (int num in numbers) {
    if (num == 3) {
        numbers.Remove(num);  // InvalidOperationException
    }
}

// CORRECT: Use for loop or create copy
for (int i = numbers.Count - 1; i >= 0; i--) {
    if (numbers[i] == 3) {
        numbers.RemoveAt(i);
    }
}
```

## JavaScript Pitfalls

### Array Quirks
```javascript
// PITFALL 1: Array length is mutable
let arr = [1, 2, 3];
arr.length = 10;  // Now arr = [1, 2, 3, empty × 7]
console.log(arr[5]);  // undefined

// PITFALL 2: Sparse arrays
let sparse = [];
sparse[100] = "value";  // Creates array with length 101, mostly empty
console.log(sparse.length);  // 101, not 1

// PITFALL 3: Array vs object confusion
let notArray = {0: "a", 1: "b", length: 2};
console.log(Array.isArray(notArray));  // false
// Use Array.isArray() to check, not typeof

// PITFALL 4: Falsy array elements
let arr = [0, false, "", null, undefined, NaN];
let filtered = arr.filter(Boolean);  // Removes all falsy values
console.log(filtered);  // [] - empty array!
```

### String Peculiarities
```javascript
// PITFALL 5: Type coercion
console.log("5" + 3);   // "53" (string concatenation)
console.log("5" - 3);   // 2 (numeric subtraction)
console.log("5" * 3);   // 15 (numeric multiplication)

// PITFALL 6: Comparing arrays/objects
let arr1 = [1, 2, 3];
let arr2 = [1, 2, 3];
console.log(arr1 == arr2);   // false (different objects)
console.log(arr1 === arr2);  // false (different objects)

// CORRECT: Compare content
console.log(JSON.stringify(arr1) === JSON.stringify(arr2));  // true
// Or use a deep comparison library
```

## Cross-Language Interview Tips

### Best Practices
```cpp
// 1. Always clarify language requirements
// "Should I implement this in C++ or is pseudocode fine?"

// 2. Mention language-specific considerations
// "In C++, I need to be careful about memory management here"
// "In Java, strings are immutable so I'll use StringBuilder"

// 3. Know your language's standard library
vector<int> vec = {3, 1, 4, 1, 5};
sort(vec.begin(), vec.end());  // C++

// 4. Understand performance characteristics
// C++: Manual memory management, very fast
// Java: Garbage collected, good performance
// JavaScript: Interpreted/JIT, variable performance
```

### Common Interview Questions
1. **"What language would you prefer for this problem?"**
   - Consider: Performance requirements, built-in data structures, your expertise

2. **"How would this differ in [other language]?"**
   - Show knowledge of language trade-offs and design differences

3. **"What are the memory implications here?"**
   - Discuss stack vs heap allocation, garbage collection, manual management

---
*Related: [[Static vs dynamic arrays]] | [[Strings - encoding ASCII UTF-8 UTF-16 immutability manipulation]]*
*Part of: [[Arrays and Strings MOC]]*