# Automated Screenings: MCQs and Coding Platforms

## Overview
Computer-based screening systems that filter candidates before human interaction using multiple choice questions and coding challenges.

## Types of Automated Screenings

### Multiple Choice Questions (MCQs)
- **Technical Knowledge**: Data structures, algorithms, language syntax
- **Problem Solving**: Logic puzzles, mathematical reasoning
- **Domain Specific**: Database queries, system design concepts
- **Behavioral**: Situational judgment questions

### Online Coding Platforms
- **HackerRank**: Algorithm challenges, domain-specific tests
- **CodeSignal**: General programming assessment
- **LeetCode**: Technical interview preparation platform
- **Codility**: Algorithmic problem solving

## Common Assessment Areas

### Programming Fundamentals
```cpp
// Example MCQ topic: Time Complexity
// What is the time complexity of this function?
int findMax(vector<int>& arr) {
    int max_val = arr[0];
    for (int i = 1; i < arr.size(); i++) {
        if (arr[i] > max_val) {
            max_val = arr[i];
        }
    }
    return max_val;
}
// Answer: O(n) - linear time complexity
```

### Data Structure Knowledge
```cpp
// Example coding challenge: Implement a stack
class Stack {
private:
    vector<int> data;

public:
    void push(int value) {
        data.push_back(value);
    }

    int pop() {
        if (data.empty()) {
            throw runtime_error("Stack is empty");
        }
        int result = data.back();
        data.pop_back();
        return result;
    }

    bool isEmpty() {
        return data.empty();
    }

    int top() {
        if (data.empty()) {
            throw runtime_error("Stack is empty");
        }
        return data.back();
    }
};
```

## Preparation Strategies

### Technical Review
- **Algorithm Complexity**: Big O notation fundamentals
- **Data Structures**: Arrays, linked lists, stacks, queues, trees
- **Language Syntax**: Common patterns and idioms
- **Problem Patterns**: Sorting, searching, graph traversal

### Platform Familiarization
```cpp
// Practice common coding patterns
// 1. Array manipulation
vector<int> reverseArray(vector<int>& arr) {
    int left = 0, right = arr.size() - 1;
    while (left < right) {
        swap(arr[left], arr[right]);
        left++;
        right--;
    }
    return arr;
}

// 2. String processing
bool isPalindrome(string s) {
    int left = 0, right = s.length() - 1;
    while (left < right) {
        if (s[left] != s[right]) return false;
        left++;
        right--;
    }
    return true;
}
```

## Time Management
- **Read Problems Carefully**: Understand requirements before coding
- **Start Simple**: Get basic solution working first
- **Test Edge Cases**: Empty inputs, single elements, maximum constraints
- **Optimize if Time Permits**: Improve time/space complexity

## Common Pitfalls
- **Rushing Through Questions**: Missing key details in problem statements
- **Not Testing Code**: Submitting without verifying basic functionality
- **Over-Engineering**: Complex solutions when simple ones suffice
- **Time Mismanagement**: Spending too long on difficult problems

## Platform-Specific Tips
- **HackerRank**: Focus on algorithm efficiency and clean code
- **CodeSignal**: Emphasizes practical programming skills
- **Codility**: Strong focus on correctness and performance
- **Company-Specific**: Some companies use custom platforms

---
*Related: [[Phone screens by recruiters - rigid Q&A]] | [[Preparation - quiet environment headset notes]]*
*Part of: [[The Phone Screen MOC]]*