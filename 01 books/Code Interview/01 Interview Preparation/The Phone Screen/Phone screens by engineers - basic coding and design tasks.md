# Phone Screens by Engineers: Basic Coding and Design Tasks

## Overview
Technical phone screens conducted by engineers focus on assessing coding ability, problem-solving approach, and technical communication skills through practical exercises.

## Common Assessment Formats

### Live Coding Exercises
**Platform-Based Coding**:
- Shared coding environments (CoderPad, HackerRank, Repl.it)
- Real-time collaboration and observation
- Syntax highlighting and basic debugging tools
- Save and review capabilities for later evaluation

**Typical Problem Types**:
- Array/string manipulation (find duplicates, reverse string)
- Basic algorithms (binary search, sorting, recursion)
- Data structure implementation (stack, queue, simple tree operations)
- Logic puzzles with programming solutions

### Code Review and Explanation
**Past Code Discussion**:
- Walk through a significant project or code sample
- Explain technical decisions and trade-offs made
- Discuss challenges encountered and solutions implemented
- Demonstrate understanding of code architecture and design

**Code Quality Assessment**:
- Review provided code snippet for bugs or improvements
- Suggest optimizations or alternative approaches
- Identify potential edge cases or failure points
- Discuss testing strategies and error handling

### System Design Discussions
**For Senior Roles**:
- High-level architecture design for simple systems
- Database schema design for given requirements
- API design and interface specifications
- Scalability considerations and bottlenecks

**Example Problems**:
- Design a URL shortening service (basic version)
- Create a simple social media feed architecture
- Design a basic chat application backend
- Plan a file storage and retrieval system

## Assessment Criteria

### Technical Competency
**Problem-Solving Approach**:
- Systematic breakdown of complex problems
- Logical progression from problem to solution
- Consideration of edge cases and error conditions
- Ability to optimize and improve initial solutions

**Coding Skills**:
- Syntactically correct and runnable code
- Appropriate use of data structures and algorithms
- Clean, readable code with good naming conventions
- Efficient solutions with reasonable time complexity

### Communication Skills
**Technical Explanation**:
- Clear articulation of thought process
- Ability to explain code and design decisions
- Effective use of technical terminology
- Teaching and mentoring potential demonstration

**Collaboration Indicators**:
- Asking clarifying questions when requirements are unclear
- Incorporating feedback and suggestions gracefully
- Working through problems interactively
- Showing openness to different approaches

## Preparation Strategies

### Technical Practice
**Coding Platform Familiarity**:
- Practice on common interview platforms
- Get comfortable with shared coding environments
- Learn keyboard shortcuts and basic features
- Test audio/video setup beforehand

**Problem Type Practice**:
- Review fundamental algorithms and data structures
- Practice explaining solutions while coding
- Time yourself on basic problems (20-30 minutes)
- Focus on clean, working code over complex optimizations

### Communication Practice
**Verbal Explanation Skills**:
- Practice talking through code while writing
- Explain technical concepts to non-technical friends
- Record yourself solving problems and review
- Prepare clear explanations for past project work

**Question Preparation**:
- Prepare thoughtful questions about the role and team
- Research the company's technical stack and challenges
- Understand the team structure and engineering practices
- Show genuine interest in their technical problems

## Common Coding Problems

### String and Array Problems
```python
# Example: Find first non-repeating character
def first_non_repeating(s):
    """
    Approach: Use hash map to count frequencies
    Time: O(n), Space: O(1) for limited alphabet
    """
    char_count = {}

    # Count character frequencies
    for char in s:
        char_count[char] = char_count.get(char, 0) + 1

    # Find first character with count 1
    for char in s:
        if char_count[char] == 1:
            return char

    return None
```

### Data Structure Implementation
```python
# Example: Implement a simple stack
class Stack:
    def __init__(self):
        self.items = []

    def push(self, item):
        self.items.append(item)

    def pop(self):
        if self.is_empty():
            raise IndexError("Pop from empty stack")
        return self.items.pop()

    def peek(self):
        if self.is_empty():
            raise IndexError("Peek from empty stack")
        return self.items[-1]

    def is_empty(self):
        return len(self.items) == 0
```

### Basic Algorithms
```python
# Example: Binary search implementation
def binary_search(arr, target):
    """
    Search for target in sorted array
    Time: O(log n), Space: O(1)
    """
    left, right = 0, len(arr) - 1

    while left <= right:
        mid = (left + right) // 2

        if arr[mid] == target:
            return mid
        elif arr[mid] < target:
            left = mid + 1
        else:
            right = mid - 1

    return -1  # Not found
```

## Success Strategies

### During the Interview
**Problem Approach**:
1. **Clarify Requirements**: Ask questions about input/output, constraints, edge cases
2. **Plan Solution**: Outline approach before coding
3. **Start Simple**: Implement basic solution first, then optimize
4. **Test and Debug**: Walk through examples and check for errors
5. **Optimize if Time**: Discuss improvements and trade-offs

**Communication Best Practices**:
- Think aloud throughout the process
- Explain why you're choosing specific approaches
- Ask for feedback and be open to suggestions
- Admit when you're unsure and ask for guidance

### Handling Challenges
**When Stuck**:
- Break down the problem into smaller parts
- Try working through a simple example manually
- Ask for a hint or clarification
- Discuss different approaches even if not perfect

**Technical Difficulties**:
- Have backup communication methods ready
- Test technology setup beforehand
- Stay calm and professional during technical issues
- Focus on problem-solving even with limited tools

## Red Flags to Avoid

### Technical Issues
- Unable to write working code for basic problems
- No systematic approach to problem-solving
- Can't explain or discuss their own code
- Significant gaps in fundamental CS knowledge

### Communication Problems
- Not asking clarifying questions
- Working in silence without explanation
- Getting defensive about feedback or suggestions
- Unable to explain technical concepts clearly

### Professional Concerns
- Unprepared or unprofessional setup
- Lack of genuine interest in the technical work
- Inability to discuss past experience meaningfully
- Poor time management during the interview

---
*Related: [[Preparation - quiet environment headset notes]] | [[Answering strategies - ambiguity handling multiple synonyms]]*
*Part of: [[The Phone Screen MOC]]*