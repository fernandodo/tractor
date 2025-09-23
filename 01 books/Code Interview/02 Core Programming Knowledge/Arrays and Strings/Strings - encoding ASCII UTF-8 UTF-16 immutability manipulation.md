# Strings: Encoding (ASCII, UTF-8, UTF-16), Immutability, Manipulation

## Overview
Understanding string representation, character encoding, and manipulation strategies crucial for programming interviews.

## Character Encoding Systems

### ASCII (American Standard Code for Information Interchange)
```cpp
// ASCII: 7-bit encoding, 128 characters (0-127)
char ascii_char = 'A';  // ASCII value 65
cout << "ASCII value of 'A': " << static_cast<int>(ascii_char) << endl;  // 65

// ASCII limitations: Only English characters, no accents, emojis, etc.
vector<char> ascii_range = {'A', 'Z', 'a', 'z', '0', '9', ' ', '!'};
for (char c : ascii_range) {
    cout << c << " = " << static_cast<int>(c) << endl;
}
```

### UTF-8 (8-bit Unicode Transformation Format)
```cpp
// UTF-8: Variable length encoding (1-4 bytes per character)
// Backward compatible with ASCII
string utf8_string = "Hello 世界 🌍";  // Mix of ASCII, Chinese, emoji

// UTF-8 characteristics:
// - ASCII characters: 1 byte each
// - European characters: 2 bytes each
// - Asian characters: 3 bytes each
// - Emojis/symbols: 4 bytes each

void analyzeUTF8(const string& str) {
    cout << "String: " << str << endl;
    cout << "Byte length: " << str.length() << endl;  // Number of bytes

    // Count actual characters (not bytes)
    int char_count = 0;
    for (size_t i = 0; i < str.length();) {
        unsigned char byte = str[i];
        if (byte < 0x80) {          // ASCII (0xxxxxxx)
            i += 1;
        } else if (byte < 0xE0) {   // 2-byte character (110xxxxx)
            i += 2;
        } else if (byte < 0xF0) {   // 3-byte character (1110xxxx)
            i += 3;
        } else {                    // 4-byte character (11110xxx)
            i += 4;
        }
        char_count++;
    }
    cout << "Character count: " << char_count << endl;
}
```

### UTF-16 (16-bit Unicode Transformation Format)
```cpp
// UTF-16: Uses 16-bit code units, can require 1 or 2 units per character
// Common in Windows (wchar_t), Java (String), C# (string)

#include <codecvt>
#include <locale>

void demonstrateUTF16() {
    // Basic Multilingual Plane (BMP): 1 code unit
    wstring utf16_bmp = L"Hello 世界";  // Each character fits in 16 bits

    // Supplementary Planes: 2 code units (surrogate pairs)
    wstring utf16_emoji = L"🌍🚀";     // Each emoji needs 2 × 16-bit units

    cout << "UTF-16 BMP length: " << utf16_bmp.length() << endl;      // Code units
    cout << "UTF-16 emoji length: " << utf16_emoji.length() << endl;  // 4 (2 per emoji)
}
```

## String Immutability

### Immutable Strings (Java, C#, Python)
```cpp
// Simulating immutable string behavior in C++
class ImmutableString {
private:
    string data;

public:
    ImmutableString(const string& str) : data(str) {}

    // No direct modification methods
    ImmutableString concat(const string& other) const {
        return ImmutableString(data + other);  // Returns new object
    }

    ImmutableString substring(size_t start, size_t length) const {
        return ImmutableString(data.substr(start, length));
    }

    char charAt(size_t index) const {
        return data[index];  // Read-only access
    }

    size_t length() const { return data.length(); }

    string toString() const { return data; }
};

// Performance implications of immutability
void demonstrateImmutabilityPerformance() {
    // INEFFICIENT: O(n²) due to creating new strings
    string result = "";
    for (int i = 0; i < 1000; ++i) {
        result = result + "a";  // Creates new string each time
    }

    // EFFICIENT: O(n) using mutable buffer
    string result2;
    result2.reserve(1000);  // Pre-allocate space
    for (int i = 0; i < 1000; ++i) {
        result2 += "a";  // Modifies existing string
    }
}
```

### Mutable Strings (C++, C)
```cpp
// C++ strings are mutable
string mutable_str = "Hello";
mutable_str[0] = 'h';           // Modifies in place
mutable_str += " World";        // Appends to existing string
mutable_str.insert(5, ",");     // Inserts character

// Benefits: Efficient modification
// Drawbacks: Can lead to unexpected side effects in function calls
void modifyString(string& str) {  // Pass by reference
    str += " Modified";           // Changes original string
}
```

## String Manipulation Techniques

### Basic Operations
```cpp
class StringManipulation {
public:
    // Case conversion
    static string toLowerCase(string str) {
        transform(str.begin(), str.end(), str.begin(), ::tolower);
        return str;
    }

    static string toUpperCase(string str) {
        transform(str.begin(), str.end(), str.begin(), ::toupper);
        return str;
    }

    // Trimming whitespace
    static string trim(const string& str) {
        size_t start = str.find_first_not_of(" \t\n\r");
        if (start == string::npos) return "";

        size_t end = str.find_last_not_of(" \t\n\r");
        return str.substr(start, end - start + 1);
    }

    // String reversal
    static string reverse(string str) {
        reverse(str.begin(), str.end());
        return str;
    }

    // Character frequency counting
    static map<char, int> charFrequency(const string& str) {
        map<char, int> freq;
        for (char c : str) {
            freq[c]++;
        }
        return freq;
    }
};
```

### Advanced String Algorithms
```cpp
// Pattern matching: KMP Algorithm
class StringSearch {
private:
    static vector<int> computeLPS(const string& pattern) {
        int m = pattern.length();
        vector<int> lps(m, 0);
        int len = 0;
        int i = 1;

        while (i < m) {
            if (pattern[i] == pattern[len]) {
                len++;
                lps[i] = len;
                i++;
            } else {
                if (len != 0) {
                    len = lps[len - 1];
                } else {
                    lps[i] = 0;
                    i++;
                }
            }
        }
        return lps;
    }

public:
    static vector<int> KMPSearch(const string& text, const string& pattern) {
        vector<int> result;
        int n = text.length();
        int m = pattern.length();

        vector<int> lps = computeLPS(pattern);

        int i = 0;  // Index for text
        int j = 0;  // Index for pattern

        while (i < n) {
            if (pattern[j] == text[i]) {
                i++;
                j++;
            }

            if (j == m) {
                result.push_back(i - j);
                j = lps[j - 1];
            } else if (i < n && pattern[j] != text[i]) {
                if (j != 0) {
                    j = lps[j - 1];
                } else {
                    i++;
                }
            }
        }
        return result;
    }
};
```

## Interview-Relevant String Problems

### Common Problem Patterns
```cpp
// 1. Palindrome detection
bool isPalindrome(const string& str) {
    int left = 0, right = str.length() - 1;
    while (left < right) {
        if (str[left] != str[right]) return false;
        left++;
        right--;
    }
    return true;
}

// 2. Anagram detection
bool areAnagrams(string str1, string str2) {
    if (str1.length() != str2.length()) return false;

    sort(str1.begin(), str1.end());
    sort(str2.begin(), str2.end());
    return str1 == str2;
}

// 3. First non-repeating character
char firstNonRepeating(const string& str) {
    unordered_map<char, int> freq;

    // Count frequencies
    for (char c : str) {
        freq[c]++;
    }

    // Find first character with frequency 1
    for (char c : str) {
        if (freq[c] == 1) {
            return c;
        }
    }
    return '\0';  // No non-repeating character found
}

// 4. String compression
string compress(const string& str) {
    if (str.empty()) return str;

    string result;
    char current = str[0];
    int count = 1;

    for (size_t i = 1; i < str.length(); ++i) {
        if (str[i] == current) {
            count++;
        } else {
            result += current + to_string(count);
            current = str[i];
            count = 1;
        }
    }
    result += current + to_string(count);  // Last group

    return result.length() < str.length() ? result : str;
}
```

## Performance Considerations

### Time Complexity Analysis
```cpp
// String operation complexities:
void complexityAnalysis() {
    string str = "example";

    // Access: O(1)
    char c = str[0];

    // Length: O(1) - usually cached
    size_t len = str.length();

    // Concatenation: O(n + m) where n, m are string lengths
    string result = str + "text";

    // Substring: O(k) where k is substring length
    string sub = str.substr(2, 3);

    // Search: O(n×m) naive, O(n+m) KMP
    size_t pos = str.find("amp");

    // Comparison: O(min(n,m)) where n, m are string lengths
    bool equal = (str == "example");
}
```

### Memory Optimization
```cpp
// String interning simulation
class StringIntern {
private:
    static unordered_set<string> pool;

public:
    static const string& intern(const string& str) {
        auto it = pool.find(str);
        if (it != pool.end()) {
            return *it;  // Return existing string
        }

        auto result = pool.insert(str);
        return *result.first;  // Return newly inserted string
    }
};

unordered_set<string> StringIntern::pool;

// Benefits: Reduces memory usage for repeated strings
// Drawbacks: Memory never freed, hash lookup overhead
```

---
*Related: [[Language-specific pitfalls - C C++ Java C# JavaScript]] | [[Common string problems - first non-repeated char substring search]]*
*Part of: [[Arrays and Strings MOC]]*