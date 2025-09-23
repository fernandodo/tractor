---
created: 2025-09-23T20:38:52 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 4 Approaches to Programming Problems | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> 4Approaches to Programming Problems

 Coding questions are the heart of the process that most computer and software companies use to decide who to hire. How a candidate performs during the...

---
## THE PROCESS

The point of coding questions is to try to determine how well you can code. It’s the most important part of the interview because the code you write and the answers you give to the interviewer largely determine whether you’re recommended for the job.

### The Scenario

You usually work one-on-one with your interviewer. The interviewer may provide you a computer, but often will give you only a marker and a whiteboard (or pen and paper) and ask you to write some code. The interviewer usually wants you to talk through the question before you start coding. Generally, you are asked to write a function or method, but sometimes you need to write a class definition or a sequence of related code modules. In any case, you write code, either in an actual programming language or in some form of pseudocode. (The closer you can get to actual working code, the better.)

### The Problems

The problems used in interviews have specific requirements. They must be short enough to be explained and solved reasonably quickly, yet complex enough that not everyone can solve them. Therefore, it’s unlikely that you’ll be asked any real-world problems. Almost any worthy real-world problem would take too long to even explain, let alone solve. Instead, many of these problems require algorithmic tricks or uncommonly used features of a language.

The problems often prohibit you from using the most common way to do something or from using the ideal data structure. For example, you might be given a problem such as: “Write a function that determines whether two integers are equal without using any comparison operators.”

This is an outright silly and contrived problem. Almost every language that ever existed has some way to compare two integers. However, it’s not acceptable to respond, “This is a stupid question; I’d use the equality operator. I’d never have this problem.” Refusing to solve the question as asked, even if it’s silly, won’t lead to a job offer. Here, the interviewer is looking for a different way to compare two integers. (Hint: Try using bit operators.)

Instead, describe the way you would solve the problem in the absence of the restrictions and then solve it as it was asked. For example, if you are asked to solve a certain problem with a hash table, you might say, “This would be easy with a binary search tree because it’s much easier to extract the largest element, but let’s see how I can solve this with a hash table.”

Problems are generally presented in ascending order of difficulty. This is not a hard-and-fast rule, but you can expect them to get more difficult as you answer more of them correctly. Often, different interviewers communicate with each other about what they asked you, what you could answer, and what you couldn’t answer. If you solve all the problems in your early interviews but are stumped by harder problems later, this may indicate that earlier interviewers were impressed with your responses.

### Which Languages to Use

If you apply for a job with specific language requirements, you should know those languages and expect to use them to solve the problems. A good job description will usually make these requirements clear, but if you’re uncertain about what languages you’ll be expected to know, ask your recruiter. If you apply for a general programming or development position, a thorough knowledge of a mainstream language such as Java, Python, JavaScript, C#, and/or C++ is enough to get by. Your interviewer may permit you to use other popular languages, such as Ruby, PHP, Swift, or Objective-C. If you are given a choice, select the language you know best, but you may be required to solve some problems in a specific language. Interviewers are less likely to be amenable to you using languages such as Go, Scala, Perl, Lisp, or Fortran, but if you are particularly expert in one of these, there’s no harm in asking.

Before you go to your interview, make sure you are completely comfortable with the use and syntax of any language you plan to use. For example, if it has been a few years since you’ve done any C++ programming, you should at least thumb through a good C++ reference guide and refamiliarize yourself with the language.

### Interactivity Is Key

The code you write in the interview is probably the only example of your code that your interviewer sees. If you write ugly code, your interviewer will assume you always write ugly code. This is your chance to shine and show your best code. Take the time to make your code solid and pretty.

Programming questions are designed to see both how well you can code and how you solve problems. If all interviewers wanted to do was measure your coding ability, they could give you a piece of paper with problems and come back an hour later to evaluate how you did, as they do in programming contests. What the interviewer wants is to see your thought processes as you work through each stage of the programming problem.

The problem-solving process in these interviews is interactive, and if you have difficulty, the interviewer generally guides you to the correct answer via a series of hints. Of course, the less help you need to solve the problem, the better you look, but showing an intelligent thought process and responding well to the hints you are given is also important. If you don’t respond well to guidance, your interviewer might suspect that you won’t work well in a team environment.

Even when you immediately know the answer to a problem, don’t just blurt it out. Break down the answer into discrete steps and explain the thought processes behind each step. The point is to show the interviewer that you understand the underlying concepts, not that you’ve managed to memorize the answer to a programming puzzle.

If you know any additional information pertaining to the problem, you may want to mention it during the process to show your general knowledge of programming, even if it’s not directly applicable to the problem at hand. Demonstrate that you have logical thought processes, are generally knowledgeable about computers, and can communicate well.

## SOLVING THE PROBLEMS

When you begin solving a problem, don’t start writing code immediately. First, make sure you completely understand the problem. It may help to work through a simple, concrete example and then try to generalize the process to an algorithm. When you’re convinced you have the right algorithm, explain it clearly. Writing the code should be one of your final steps.

### The Basic Steps

The best way to solve an interview problem is to approach it methodically:

1.  **Make sure you understand the problem.** Your initial assumptions about the problem may be wrong, or the interviewer’s explanation may be brief or difficult to follow. You can’t demonstrate your skills if you don’t understand the problem. Don’t hesitate to ask your interviewer questions, and don’t start solving the problem until you understand it. The interviewer may be deliberately obscuring things to determine whether you can find and understand the actual problem. In these cases, asking the right clarifying questions is an important part of the correct solution.
2.  **When you understand the question, try a simple example.** This example may lead to insights about how to solve the general problem or bring to light any remaining misunderstandings that you have. Starting with an example also demonstrates a methodical, logical thought process. Examples are especially useful if you don’t see the solution right away.
3.  **Focus on the algorithm and data structures you will use to solve the problem.** This can take a long time and require additional examples. This is to be expected. _Interactivity is important during this process._ If you stand quietly staring at the whiteboard, the interviewer has no way of knowing whether you’re making productive headway or are simply clueless. Talk to your interviewer about what you are doing. For example, you might say something like, “I’m wondering whether I can store the values in an array and then sort them, but I don’t think that will work because I can’t quickly look up elements in an array by value.” This demonstrates your skill, which is the point of the interview, and may also lead to hints from the interviewer, who might respond, “You’re close to the solution. Do you really need to look up elements by value, or could you….”
    
    It may take a long time to solve the problem, and you may be tempted to begin coding before you figure out a complete solution. Resist this temptation. Consider who you would rather work with: someone who thinks about a problem for a long time and then codes it correctly the first time or someone who hastily jumps into a problem, makes several errors while coding, and doesn’t have any idea what direction to take. Not a difficult decision, is it?
    
4.  **After you figure out your algorithm and how you can implement it, explain your solution to the interviewer.** This provides an opportunity to evaluate your solution before you begin coding. Your interviewer may say, “Sounds great, go ahead and code it,” or something like, “That’s not quite right because you can’t look up elements in a hash table that way.” Another common response is “That sounds like it will work, but there’s a more efficient solution.” In any case, you gain valuable information about whether you should move on to coding or go back to working on the algorithm.
5.  **While you code, explain what you’re doing.** For example, you might say, “Here, I’m initializing the array to all zeros.” This narrative enables the interviewer to follow your code more easily.
6.  **Ask questions when necessary.** You generally won’t be penalized for asking factual questions that you might otherwise look up in a reference. You obviously can’t ask a question such as, “How do I solve this problem?” but it is acceptable to ask a question such as, “I can’t remember—what format string do I use to print out a localized date?” Although it’s better to know these things, it’s okay to ask this sort of question.
7.  **After you write the code for a problem, immediately verify that the code works by tracing through it with an example.** This step demonstrates clearly that your code is correct in at least one case. It also illustrates a logical thought process and your intention to check your work and search for bugs. The example may also help you flush out minor bugs in your solution.
8.  **Make sure you check your code for _all_ error and special cases, especially boundary conditions.** Programmers often overlook error and special cases; forgetting these cases in an interview indicates you might forget them on the job. If time does not allow for extensive checking, at least explain that you should check for such failures. Covering error and special cases can impress your interviewer and help you correctly solve the problem.

After you try an example and feel comfortable that your code is correct, the interviewer may ask you questions about what you wrote. These questions often focus on running time, alternative implementations, and complexity (discussed later in this chapter). If your interviewer does not ask you these questions, you should volunteer the information to show that you are cognizant of these issues. For example, you could say, “This implementation has linear running time, which is the best achievable running time since I need to check all the input values. The dynamic memory allocation will slow it down a little, as will the overhead of using recursion.”

### When You Get Stuck

Getting stuck on a problem is expected and an important part of the interviewing process. Interviewers want to see how you respond when you don’t recognize the answer to a question immediately. Giving up or getting frustrated is the worst thing to do if this happens to you. Instead, show interest in the problem and keep trying different ways to solve it:

-   **Go back to an example.** Try performing the task and analyzing what you are doing. Try extending your specific example to the general case. You may have to use detailed examples. This is okay, because it shows the interviewer your persistence in finding the correct solution.
-   **Try a different data structure.** Perhaps a linked list, an array, a hash table, or a binary search tree can help. If you’re given an unusual data structure, look for similarities between it and more familiar data structures. Using the right data structure often makes a problem much easier.
-   **Consider the less commonly used or more advanced aspects of a language.** Sometimes the key to a problem involves one of these features.

Even when you don’t feel stuck, you may not be converging on the optimal solution. You may miss an elegant or nonobvious way to implement something. Pause every once in a while to reconsider the bigger picture and whether there may be a better approach. One sign that you may be off track is if you find yourself writing too much code. Almost all interview coding questions have short answers. You rarely need to write more than 30 lines of code and almost never more than 50. If you start writing a lot of code, you may be heading in the wrong direction.

## ANALYZING YOUR SOLUTION

After you answer the problem, you may be asked about the efficiency of your implementation. Often, you have to compare trade-offs between your implementation and another possible solution and identify the conditions that make each option more favorable. Common questions focus on run time and memory usage.

A good understanding of Big-_O_ analysis is critical to making a good impression with the interviewer. _Big-O analysis_ is a form of runtime analysis that measures the efficiency of an algorithm in terms of the time it takes for the algorithm to run as a function of the input size. It’s not a formal benchmark, just a simple way to classify algorithms by relative efficiency when dealing with very large input sizes.

Most coding problem solutions in this book include a runtime analysis to help you solidify your understanding of the algorithms.

### Big-_O_ Analysis

Consider a simple function that returns the maximum value stored in an array of nonnegative integers. The size of the array is `n`. At least two easy ways exist to implement the function.

In the first alternative, you keep track of the current largest number as the function iterates through the array and return that value when you are done iterating. This implementation, called `CompareToMax`, looks like:

```
/* Returns the largest value in an array of n non-negative integers */
```

The second alternative compares each value to all the other values. If all other values are less than or equal to a given value, that value must be the maximum value. This implementation, called `CompareToAll`, looks like:

```
/* Returns the largest value in an array of n non-negative integers */
```

Both of these functions correctly return the maximum value. Which one is more efficient? You could try benchmarking them, but this would tie your measure of efficiency to the particular system and input size you used for benchmarking. It’s more useful to have a means of comparing the performance of different algorithms that depends only on the algorithm. In general, the relative performance of different algorithms only becomes relevant as the input size becomes large, because for small input sizes any reasonable algorithm will be fast. Big-**O** analysis provides a way to compare the predicted relative performance of different algorithms as the input size becomes large. (There are some important but less common cases where performance on small input sizes is relevant, typically when a very large number of small inputs must be processed. We’ll call attention to these unusual cases where Big-_O_ analysis may be misleading.)

### How Big-_O_ Analysis Works

In Big-_O_ analysis, input size is assumed to be an unknown value _n_. In this example, _n_ simply represents the number of elements in an array. In other problems, _n_ may represent the number of nodes in a linked list, the number of bits in a data type, or the number of entries in a hash table. After determining what _n_ means in terms of the input, you must determine how many operations are performed for each of the _n_ input items. “Operation” is a fuzzy word because algorithms differ greatly. Commonly, an operation is something that a real computer can do in a constant amount of time, like adding an input value to a constant, creating a new input item, or deleting an input value. In Big-_O_ analysis, the times for these operations are all considered equivalent. In both `CompareToMax` and `CompareToAll`, the operation of greatest interest is comparing an array value to another value.

In `CompareToMax`, each array element was compared once to a maximum value. Thus, the _n_ input items are each examined once, resulting in _n_ examinations. This is considered _O_(_n_), usually referred to as _linear time_: the time required to run the algorithm increases linearly with the number of input items.

You may notice that in addition to examining each element once, there is a check to ensure that the array is not empty and a step that initializes the `curMax` variable. It may seem more accurate to call this an _O_(_n_ + 2) function to reflect these extra operations. Big-_O_ analysis, however, is concerned with the asymptotic running time: the limit of the running time as _n_ gets very large. The justification for this is that when _n_ is small, almost any algorithm will be fast. It’s only when _n_ become large that the differences between algorithms are noticeable. As _n_ approaches infinity, the difference between _n_ and _n_ + 2 is insignificant, so the constant term can be ignored. Similarly, for an algorithm running in _n_ + _n_<sup>2</sup> time, the difference between _n_<sup>2</sup> and _n_ + _n_<sup>2</sup> is negligible for a very large _n_. Thus, in Big-_O_ analysis you eliminate all but the highest-order term: the term that is largest as _n_ gets very large. In this case, _n_ is the highest-order term. Therefore, the `CompareToMax` function is _O_(_n_).

The analysis of `CompareToAll` is a little more difficult. First, you need to make an assumption about where the largest number occurs in the array. For now, assume that the maximum element is at the end of the array. In this case, this function may compare each of _n_ elements to _n_ other elements. Thus you have _n_·_n_ examinations so this is an _O_(_n_<sup>2</sup>) algorithm.

The analysis so far has shown that `CompareToMax` is _O_(_n_) and `CompareToAll` is _O_(_n_<sup>2</sup>). This means that as the array grows, the number of comparisons in `CompareToAll` becomes much larger than in `CompareToMax`. Consider an array with 30,000 elements. `CompareToMax` compares on the order of 30,000 elements, whereas `CompareToAll` compares on the order of 900,000,000 elements. You would expect `CompareToMax` to be much faster because it examines 30,000 times fewer elements. In fact, one benchmark timed `CompareToMax` at less than .01 seconds, whereas `CompareToAll` took 23.99 seconds.

### Best, Average, and Worst Cases

You may think this comparison was stacked against `CompareToAll` because the maximum value was at the end. This is true, and it raises the important issues of best-case, average-case, and worst-case running times. The analysis of `CompareToAll` was a worst-case scenario: the maximum value was at the end of the array. Consider the average case, in which the largest value is in the middle. You end up checking only half the values _n_ times because the maximum value is in the middle. This results in checking _n_(_n_/2) = _n_<sup>2</sup>/2 times. This would appear to be an _O_(_n_<sup>2</sup>/2) running time. However, Big-_O_ analysis is concerned with the way the running time changes as the input becomes very large. As _n_ increases toward infinity, the difference between _n_<sup>2</sup>/2 and _n_<sup>2</sup> become negligible relative to the difference between _n_<sup>2</sup> and any other functional form (e.g., _n_ or _n_<sup>3</sup>). Therefore, in Big-_O_ analysis, you drop all constant factors, just as you drop all lower order terms. This is why you can consider the time for every operation to be equivalent: considering different constant time requirements for different operations would yield a constant multiplicative factor, which you would drop anyway. With this in mind, the average case for `CompareToAll` is no better than the worst case. It is still _O_(_n_<sup>2</sup>).

The best-case running time for `CompareToAll` is better than _O_(_n_<sup>2</sup>). In this case, the maximum value is at the beginning of the array. The maximum value is compared to all other values only once, so the result is an _O_(_n_) running time.

In `CompareToMax`, the best-case, average-case, and worst-case running times are identical. Regardless of the arrangement of the values in the array, the algorithm is always _O_(_n_).

Ask interviewers which scenario they’re most interested in. Sometimes you’ll find clues to this in the problem. Some sorting algorithms with terrible worst cases for unsorted data may nonetheless be well suited for a problem if the input is already sorted. These kinds of trade-offs are discussed in more detail in [Chapter 10](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c10.xhtml), which discusses general sorting algorithms.

### Optimizations and Big-_O_ Analysis

Algorithm optimizations do not always yield the expected changes in their overall running times. Consider the following optimization to `CompareToAll`: instead of comparing each number to every other number, compare each number only with the numbers that follow it in the array. Every number before the current number has already been compared to the current number. Thus, the algorithm is still correct if you compare only to numbers occurring after the current number.

What’s the worst-case running time for this implementation? The first number is compared to _n_ numbers, the second number to _n_ – 1 numbers, the third number to _n_ – 2, resulting in a number of comparisons equal to _n_ + (_n_ – 1) + (_n_ – 2) + (_n_ – 3) + ... + 1. This is a common result, a mathematical series with a sum of _n_<sup>2</sup>/2 + _n_/2. But because _n_<sup>2</sup> is the highest-order term, this version of the algorithm still has an _O_(_n_<sup>2</sup>) running time in the worst case! Although this optimization reduces the running time, it has no effect on the rate at which the running time increases as _n_ increases.

### How to Do Big-_O_ Analysis

The general procedure for Big-_O_ runtime analysis is as follows:

1.  Figure out what the input is and what _n_ represents.
2.  Express the number of operations the algorithm performs in terms of _n._
3.  Eliminate all but the highest-order terms.
4.  Remove all constant factors.

For the algorithms you’ll encounter in interviews, Big-_O_ analysis should be straightforward as long as you correctly identify the operations that are dependent on the input size.

If you’d like to learn more about runtime analysis, you can find a more extensive, mathematically rigorous discussion in the first chapters of any good algorithms textbook. This book defines Big-_O_ informally as it is most commonly used by professional programmers. The informal definition of Big-_O_ used in this book is generally closer to the textbook definition of Big-Theta than the textbook definition of Big-_O_.

### Which Algorithm Is Better?

The fastest possible running time for any runtime analysis is _O_(1), commonly referred to as _constant_ running time. An algorithm with constant running time always takes the same amount of time to execute, regardless of the input size. This is the ideal run time for an algorithm, but it’s rarely achievable.

The performance of most algorithms depends on _n_, the size of the input. Common running times of algorithms can be classified as follows from best-to-worse performance:

-   _O_**(log** _n_**).** An algorithm is said to be _logarithmic_ if its running time increases logarithmically in proportion to the input size.
-   _O_**(**_n_**).** A _linear algorithm_’s running time increases in direct proportion to the input size.
-   _O_**(**_n_ **log** _n_**).** A _quasilinear algorithm_ is midway between a linear algorithm and a polynomial algorithm.
-   _O_**(**_n_<sup>c</sup>**).** A _polynomial algorithm_ grows quickly based on the size of the input.
-   _O_**(c**_n_**).** An _exponential algorithm_ grows even faster than a polynomial algorithm.
-   _O_**(**_n!_**).** A _factorial algorithm_ grows the fastest and becomes quickly unusable for even small values of _n_.

The run times of different orders of algorithms separate rapidly as _n_ gets larger. Consider the run time for each of these algorithm classes with _n_ = 10:

-   log 10 = 1
-   10 = 10
-   10 log 10 = 10
-   10<sup>2</sup> = 100
-   2<sup>10</sup>\= 1,024
-   10! = 3,628,800

Now double it to n = 20:

-   log 20 ≈ 1.30
-   20 = 20
-   20 log 20 ≈ 26.02
-   20<sup>2</sup> = 400
-   2<sup>20</sup> = 1,048,576
-   20! ≈ 2.43 × 10<sup>18</sup>

Finding an algorithm that works in quasilinear time or better can make a huge difference in how well an application performs.

### Memory Footprint Analysis

Runtime analysis is not the only relevant metric for performance. A common request from interviewers is to analyze how much memory a program uses. This is sometimes referred to as the _memory footprint_ of the application. Memory use is sometimes as important as running time, particularly in constrained environments such as embedded systems.

In some cases, you will be asked about the memory usage of an _algorithm_. For this, the approach is to express the amount of memory required in terms of _n_, the size of the input, analogous to the preceding discussion of Big-_O_ runtime analysis. The difference is that instead of determining how many operations are required for each item of input, you determine the amount of storage required for each item.

Other times, you may be asked about the memory footprint of an _implementation_. This is usually an exercise in estimation, especially for languages such as Java and C# that run in a virtual machine. Interviewers don’t expect you to know to-the-byte exactly how much memory is used, but they like to see that you understand how the underlying data structures might be implemented. If you’re a C++ expert, don’t be surprised if you’re asked how much memory a struct or class requires—the interviewer may want to check that you understand memory alignment and structure packing issues.

There is often a trade-off between optimal memory use and runtime performance. The classic example of this is the Unicode string encodings discussed in [Chapter 7](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c07.xhtml), which enable more compact representations of strings while making it more expensive to perform many common string operations. Be sure to mention any trade-offs to the interviewer when discussing memory footprint issues.

## SUMMARY

How you solve programming problems during your interviews can determine whether you get a job offer, so you need to answer them as correctly and completely as you can. The problems usually get progressively harder as the day progresses, so don’t be surprised if you need an occasional hint from the interviewer. You normally code in a mainstream programming language, but the choice of language is ultimately dictated by the requirements of the job for which you apply, so be familiar with the right languages.

Interact with your interviewers as much as possible as you attempt each problem. Let them know what you’re thinking at each point in your analysis of the problem and your attempts at coding an answer. Start by making sure you understand the problem, and then try some examples to reinforce that understanding. Choose the algorithm and make sure it works for those examples. Remember to test for special cases. If you’re stuck, try more examples or choose a different algorithm. Keep obscure or advanced language features in mind when looking for alternative answers.

If asked to comment on the performance of a solution, a Big-_O_ runtime analysis is usually sufficient. Algorithms that run in constant, logarithmic, linear, or quasilinear time are preferred. You should also be prepared to comment on the memory footprint of an algorithm.
