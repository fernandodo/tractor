# Common String Problems: First Non-Repeated Character, Substring Search

## Overview
Essential string manipulation problems frequently encountered in programming interviews with optimal solution approaches.

## First Non-Repeated Character

### Problem Statement
Given a string, find the first character that doesn't repeat. Return the character or a special value if no such character exists.

### Solution Approaches

#### Approach 1: Hash Map (Two Pass)
```cpp
char firstNonRepeating(const string& str) {
    unordered_map<char, int> freq;

    // First pass: Count character frequencies
    for (char c : str) {
        freq[c]++;
    }

    // Second pass: Find first character with frequency 1
    for (char c : str) {
        if (freq[c] == 1) {
            return c;
        }
    }

    return '\0';  // No non-repeating character found
}

// Time Complexity: O(n)
// Space Complexity: O(k) where k is number of unique characters
```

#### Approach 2: Array for ASCII (Optimized)
```cpp
char firstNonRepeatingASCII(const string& str) {
    int freq[256] = {0};  // ASCII character set

    // Count frequencies
    for (char c : str) {
        freq[static_cast<unsigned char>(c)]++;
    }

    // Find first non-repeating
    for (char c : str) {
        if (freq[static_cast<unsigned char>(c)] == 1) {
            return c;
        }
    }

    return '\0';
}

// Time Complexity: O(n)
// Space Complexity: O(1) - fixed size array
```

#### Approach 3: Single Pass with Ordered Map
```cpp
#include <queue>

char firstNonRepeatingSinglePass(const string& str) {
    unordered_map<char, int> freq;
    queue<char> candidates;

    for (char c : str) {
        freq[c]++;

        if (freq[c] == 1) {
            candidates.push(c);
        }

        // Remove characters that are now repeating
        while (!candidates.empty() && freq[candidates.front()] > 1) {
            candidates.pop();
        }
    }

    return candidates.empty() ? '\0' : candidates.front();
}

// Time Complexity: O(n)
// Space Complexity: O(k) where k is number of unique characters
```

### Edge Cases and Variations
```cpp
class FirstNonRepeatingVariations {
public:
    // Case-insensitive version
    static char firstNonRepeatingIgnoreCase(const string& str) {
        unordered_map<char, int> freq;

        // Convert to lowercase and count
        string lower_str = str;
        transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);

        for (char c : lower_str) {
            freq[c]++;
        }

        // Find first non-repeating in original case
        for (char c : str) {
            char lower_c = tolower(c);
            if (freq[lower_c] == 1) {
                return c;  // Return original case
            }
        }

        return '\0';
    }

    // Return index instead of character
    static int firstNonRepeatingIndex(const string& str) {
        unordered_map<char, int> freq;

        for (char c : str) {
            freq[c]++;
        }

        for (int i = 0; i < str.length(); ++i) {
            if (freq[str[i]] == 1) {
                return i;
            }
        }

        return -1;  // Not found
    }

    // Unicode support
    static string firstNonRepeatingUnicode(const string& str) {
        unordered_map<string, int> freq;

        // Split into UTF-8 characters
        vector<string> chars;
        for (size_t i = 0; i < str.length();) {
            size_t char_len = 1;
            unsigned char byte = str[i];

            if (byte >= 0xF0) char_len = 4;
            else if (byte >= 0xE0) char_len = 3;
            else if (byte >= 0xC0) char_len = 2;

            string utf8_char = str.substr(i, char_len);
            chars.push_back(utf8_char);
            freq[utf8_char]++;
            i += char_len;
        }

        // Find first non-repeating
        for (const string& ch : chars) {
            if (freq[ch] == 1) {
                return ch;
            }
        }

        return "";  // Not found
    }
};
```

## Substring Search

### Problem Statement
Find all occurrences of a pattern string within a text string. Return positions or boolean indicating presence.

### Naive Approach
```cpp
vector<int> naiveSearch(const string& text, const string& pattern) {
    vector<int> positions;
    int n = text.length();
    int m = pattern.length();

    for (int i = 0; i <= n - m; ++i) {
        int j = 0;
        while (j < m && text[i + j] == pattern[j]) {
            j++;
        }

        if (j == m) {  // Found match
            positions.push_back(i);
        }
    }

    return positions;
}

// Time Complexity: O(n×m) worst case
// Space Complexity: O(1) excluding output
```

### KMP Algorithm (Knuth-Morris-Pratt)
```cpp
class KMPSearch {
private:
    static vector<int> computeLPS(const string& pattern) {
        int m = pattern.length();
        vector<int> lps(m, 0);
        int len = 0;  // Length of previous longest prefix suffix
        int i = 1;

        while (i < m) {
            if (pattern[i] == pattern[len]) {
                len++;
                lps[i] = len;
                i++;
            } else {
                if (len != 0) {
                    len = lps[len - 1];  // Fallback to previous LPS
                } else {
                    lps[i] = 0;
                    i++;
                }
            }
        }

        return lps;
    }

public:
    static vector<int> search(const string& text, const string& pattern) {
        vector<int> positions;
        int n = text.length();
        int m = pattern.length();

        if (m == 0) return positions;

        vector<int> lps = computeLPS(pattern);

        int i = 0;  // Index for text
        int j = 0;  // Index for pattern

        while (i < n) {
            if (pattern[j] == text[i]) {
                i++;
                j++;
            }

            if (j == m) {
                positions.push_back(i - j);
                j = lps[j - 1];
            } else if (i < n && pattern[j] != text[i]) {
                if (j != 0) {
                    j = lps[j - 1];
                } else {
                    i++;
                }
            }
        }

        return positions;
    }
};

// Time Complexity: O(n + m)
// Space Complexity: O(m) for LPS array
```

### Rabin-Karp Algorithm (Rolling Hash)
```cpp
class RabinKarpSearch {
private:
    static const int PRIME = 101;

    static long long computeHash(const string& str, int start, int length) {
        long long hash = 0;
        long long pow = 1;

        for (int i = 0; i < length; ++i) {
            hash += (str[start + i] * pow) % PRIME;
            hash %= PRIME;
            pow = (pow * 256) % PRIME;
        }

        return hash;
    }

    static long long rollingHash(const string& text, int start, int length,
                                long long prevHash, long long pow) {
        // Remove leftmost character
        long long newHash = prevHash - text[start - 1];
        newHash /= 256;

        // Add rightmost character
        newHash += (text[start + length - 1] * pow / 256) % PRIME;
        newHash %= PRIME;

        return newHash;
    }

public:
    static vector<int> search(const string& text, const string& pattern) {
        vector<int> positions;
        int n = text.length();
        int m = pattern.length();

        if (m > n) return positions;

        long long patternHash = computeHash(pattern, 0, m);
        long long textHash = computeHash(text, 0, m);

        // Precompute 256^(m-1) % PRIME
        long long pow = 1;
        for (int i = 0; i < m - 1; ++i) {
            pow = (pow * 256) % PRIME;
        }

        for (int i = 0; i <= n - m; ++i) {
            if (textHash == patternHash) {
                // Hash collision possible, verify character by character
                if (text.substr(i, m) == pattern) {
                    positions.push_back(i);
                }
            }

            if (i < n - m) {
                textHash = rollingHash(text, i + 1, m, textHash, pow);
            }
        }

        return positions;
    }
};

// Time Complexity: O(n + m) average, O(n×m) worst case (many hash collisions)
// Space Complexity: O(1)
```

### Boyer-Moore Algorithm (Simplified)
```cpp
class BoyerMooreSearch {
private:
    static vector<int> buildBadCharTable(const string& pattern) {
        vector<int> badChar(256, -1);

        for (int i = 0; i < pattern.length(); ++i) {
            badChar[static_cast<unsigned char>(pattern[i])] = i;
        }

        return badChar;
    }

public:
    static vector<int> search(const string& text, const string& pattern) {
        vector<int> positions;
        int n = text.length();
        int m = pattern.length();

        vector<int> badChar = buildBadCharTable(pattern);

        int shift = 0;
        while (shift <= n - m) {
            int j = m - 1;

            while (j >= 0 && pattern[j] == text[shift + j]) {
                j--;
            }

            if (j < 0) {
                positions.push_back(shift);
                shift += (shift + m < n) ?
                         m - badChar[static_cast<unsigned char>(text[shift + m])] : 1;
            } else {
                shift += max(1, j - badChar[static_cast<unsigned char>(text[shift + j])]);
            }
        }

        return positions;
    }
};

// Time Complexity: O(n/m) best case, O(n×m) worst case
// Space Complexity: O(1) with fixed alphabet size
```

## Advanced String Problems

### Multiple Pattern Search
```cpp
// Aho-Corasick Algorithm for multiple pattern matching
class AhoCorasick {
    struct TrieNode {
        unordered_map<char, TrieNode*> children;
        TrieNode* failure;
        vector<int> output;

        TrieNode() : failure(nullptr) {}
    };

    TrieNode* root;

public:
    AhoCorasick(const vector<string>& patterns) {
        root = new TrieNode();
        buildTrie(patterns);
        buildFailureFunction();
    }

    vector<pair<int, int>> search(const string& text) {
        vector<pair<int, int>> matches;  // (position, pattern_id)
        TrieNode* current = root;

        for (int i = 0; i < text.length(); ++i) {
            char c = text[i];

            while (current != root && current->children.find(c) == current->children.end()) {
                current = current->failure;
            }

            if (current->children.find(c) != current->children.end()) {
                current = current->children[c];
            }

            // Check for matches
            for (int patternId : current->output) {
                matches.push_back({i, patternId});
            }
        }

        return matches;
    }

private:
    void buildTrie(const vector<string>& patterns) {
        for (int id = 0; id < patterns.size(); ++id) {
            TrieNode* node = root;
            for (char c : patterns[id]) {
                if (node->children.find(c) == node->children.end()) {
                    node->children[c] = new TrieNode();
                }
                node = node->children[c];
            }
            node->output.push_back(id);
        }
    }

    void buildFailureFunction() {
        queue<TrieNode*> q;

        for (auto& pair : root->children) {
            pair.second->failure = root;
            q.push(pair.second);
        }

        while (!q.empty()) {
            TrieNode* current = q.front();
            q.pop();

            for (auto& pair : current->children) {
                char c = pair.first;
                TrieNode* child = pair.second;
                TrieNode* failure = current->failure;

                while (failure != root && failure->children.find(c) == failure->children.end()) {
                    failure = failure->failure;
                }

                if (failure->children.find(c) != failure->children.end() && failure->children[c] != child) {
                    child->failure = failure->children[c];
                } else {
                    child->failure = root;
                }

                // Copy output from failure node
                for (int id : child->failure->output) {
                    child->output.push_back(id);
                }

                q.push(child);
            }
        }
    }
};
```

## Interview Tips and Common Variations

### Problem Variations
```cpp
// 1. Case-insensitive search
bool containsIgnoreCase(const string& text, const string& pattern) {
    string lowerText = text;
    string lowerPattern = pattern;

    transform(lowerText.begin(), lowerText.end(), lowerText.begin(), ::tolower);
    transform(lowerPattern.begin(), lowerPattern.end(), lowerPattern.begin(), ::tolower);

    return lowerText.find(lowerPattern) != string::npos;
}

// 2. Count occurrences
int countOccurrences(const string& text, const string& pattern) {
    int count = 0;
    size_t pos = 0;

    while ((pos = text.find(pattern, pos)) != string::npos) {
        count++;
        pos += pattern.length();  // Move past this occurrence
    }

    return count;
}

// 3. Find longest common substring
string longestCommonSubstring(const string& str1, const string& str2) {
    int m = str1.length();
    int n = str2.length();
    vector<vector<int>> dp(m + 1, vector<int>(n + 1, 0));

    int maxLength = 0;
    int endPos = 0;

    for (int i = 1; i <= m; ++i) {
        for (int j = 1; j <= n; ++j) {
            if (str1[i - 1] == str2[j - 1]) {
                dp[i][j] = dp[i - 1][j - 1] + 1;
                if (dp[i][j] > maxLength) {
                    maxLength = dp[i][j];
                    endPos = i;
                }
            }
        }
    }

    return str1.substr(endPos - maxLength, maxLength);
}
```

---
*Related: [[Strings - encoding ASCII UTF-8 UTF-16 immutability manipulation]] | [[Array basics - contiguous memory O(1) access O(n) insert-delete]]*
*Part of: [[Arrays and Strings MOC]]*