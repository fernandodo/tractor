---
created: 2025-09-23T20:39:16 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 5 Linked Lists | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> 5Linked Lists

 The linked list, a deceptively simple data structure, is the basis for a surprising number of problems regarding the handling of dynamic data. Questions about efficient list...

---
## KINDS OF LINKED LISTS

Three basic kinds of linked lists exist: singly linked lists, doubly linked lists, and circular linked lists. Singly linked lists are the variety most commonly encountered in interviews.

### Singly Linked Lists

When interviewers say “linked list” they generally mean a linear _singly linked list_, where each data element in the list has a link (a pointer or reference) to the element that follows it in the list, as shown in [Figure 5-1](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c05-fig-0001). The first element in a singly linked list is referred to as the _head_ of the list. The last element in such a list is called the _tail_ of the list and has an empty or null link.

![[attachments/c05f001.jpg]]

[**FIGURE 5-1**](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c05-fig-0001)

Singly linked lists have a host of special cases and potential programming pitfalls. Because the links in a singly linked list consist only of next pointers (or references), the list can be traversed only in the forward direction. Therefore a complete traversal of the list must begin with the first element. In other words, you need a pointer or reference to the first element of a list to locate all the elements in the list. It’s common to store that pointer or reference in a separate data structure.

In C, the simplest singly linked list element is a `struct` with a pointer to a `struct` of the same type as its only member:

```
// The simplest singly linked list element
```

Because it has no data, it’s not a particularly useful list element. A more useful `struct` has at least one data member in addition to the pointer:

```
// A more useful singly linked list element
```

The `next` pointer can be anywhere in the `struct`, but placing it at the beginning makes it easier to write generic list-handling routines that work no matter what data an element holds by casting the pointer to be of the generic list element type.

In C++ you could define a class for the list element:

```
// A singly linked list in C++
```

However, it usually makes more sense to define a template for the list element:

```
// A templated C++ singly linked list
```

A Java implementation using generics is similar, but of course uses references instead of pointers:

```
// A templated Java singly linked list
```

### Doubly Linked Lists

A doubly linked list, as shown in [Figure 5-2](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c05-fig-0002), eliminates many of the difficulties of using a singly linked list. In a _doubly linked list_, each element has a link to the _previous_ element in the list as well as to the _next_ element in the list. This additional link makes it possible to traverse the list in either direction. The entire list can be traversed starting from any element. A doubly linked list has head and tail elements just like a singly linked list. The head of the list has an empty or null previous link, just as the tail of the list has a null or empty next link.

![[attachments/c05f002.jpg]]

[**FIGURE 5-2**](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c05-fig-0002)

Doubly linked lists are not frequently seen in interview problems. Many problems involve singly linked lists specifically because they are more difficult that way; they would be trivial with a doubly linked list. Other problems are difficult whether the list is singly or doubly linked, so there’s no point in using a doubly linked list, which adds complexity irrelevant to the problem.

### Circular Linked Lists

The final variation on the linked list theme is the circular linked list, which comes in singly and doubly linked varieties. _Circular linked_ lists have no ends—no head or tail. Each element in a circular linked list has non-null next (and previous, if it’s also doubly linked) pointers or references. A list with one element merely points to itself.

The primary traversal problem for these lists is cycle avoidance—if you don’t track where you start, you’ll cycle infinitely through the list.

You may encounter circular linked lists from time to time, but they are uncommon in interview problems.

## BASIC LINKED LIST OPERATIONS

Successfully solving linked list problems requires a thorough understanding of how to operate on linked lists. This includes tracking the head element so that the list doesn’t get lost, traversing the list, and inserting and deleting list elements. These operations are much more straightforward with a doubly linked list, so we focus on the pitfalls of implementing these operations for singly linked lists.

### Tracking the Head Element

The head element of a singly linked list must always be tracked; otherwise, the list will be lost—either garbage collected or leaked, depending on the language. This means that the pointer or reference to the head of the list must be updated when a new element is inserted ahead of the first element or when the existing first element is removed from the list.

Tracking the head element becomes a problem when you alter the list inside a function or method, because the caller must be made aware of the new head element. For example, the following Java code is incorrect because it fails to update the reference to the head of the list:

```
public void insertInFront( ListElement<Integer> list, int data ){
```

A correct solution is to return the new head element from the method:

```
public ListElement<Integer> insertInFront( ListElement<Integer> list, int data ){
```

The caller updates its reference to the head element accordingly:

```
int data = ....; // data to insert
```

In C or C++ it’s easier to make mistakes with pointer misuse. Consider this C code for inserting an element at the front of a list:

```
bool insertInFront( IntElement *head, int data ){
```

The preceding code is incorrect because it updates only the local copy of the head pointer. The correct version passes in a pointer to the head pointer:

```
bool insertInFront( IntElement **head, int data ){
```

This function uses the return value to indicate the success or failure of the memory allocation (because there are no exceptions in C), so it can’t return the new head pointer as the Java function did. In C++, the head pointer could also be passed in by reference, or the function could return the new head pointer.

### Traversing a List

Often, you need to work with list elements other than the head element. Operations on any but the first element of a linked list require traversal of some elements of the list. When traversing, you must always check that you haven’t reached the end of the list. The following traversal is unsafe:

```
public ListElement<Integer> find( ListElement<Integer> head, int data ){
```

This method works fine as long as the data to find is actually in the list. If it isn’t, an error occurs (a null reference exception) when you travel past the last element. A simple change to the loop fixes the problem:

```
public ListElement<Integer> find( ListElement<Integer> head, int data ){
```

With this implementation, the caller must detect an error condition by checking for a null return value. (Alternatively, it may make more sense to throw an exception if the end of the list is reached and the element cannot be found.)

### Inserting and Deleting Elements

Because links in a singly linked list are maintained exclusively with next pointers or references, any insertion or deletion of elements in the middle of a list requires modification of the previous element’s next pointer or reference. If you’re given only the element to delete (or before which to insert), this requires traversal of the list from the head because there’s no other way to find the preceding element. Special care must be taken when the element to be deleted is the head of the list.

This C function deletes an element from a list:

```
bool deleteElement( IntElement **head, IntElement *deleteMe )
```

Although the preceding is a common implementation, Linus Torvalds (creator of Linux) has pointed out that the special case for deleting the first element is inelegant and unnecessary. If instead of traversing the list using a pointer to the current element, you traverse using a pointer to a pointer to the next element, then your traversal pointer points at the pointer that you need to change when you delete the element, regardless of whether it’s the head pointer or the next pointer of a previous element. This double indirection approach is a little more complex to understand, but eliminates the special case and associated duplicated code:

```
bool deleteElement( IntElement **npp, IntElement *deleteMe ){
```

Performing deletions raises another issue in languages without garbage collection, like C or C++. Suppose you want to remove all the elements from a linked list. The natural inclination is to use a single pointer to traverse the list, freeing elements as you go. A problem arises, however, when this is implemented. Do you advance the pointer first or free the element first? If you advance the pointer first, then the freeing is impossible because you overwrote the pointer to the element to be freed. If you free the element first, advancing the pointer is impossible because it involves reading the next pointer in the element that was just freed. The solution is to use two pointers, as in the following example:

```
void deleteList( IntElement **head )
```

## LINKED LIST PROBLEMS

The solutions to the linked list problems that follow can be implemented in any language that supports dynamic memory, but because you rarely implement your own linked lists in languages like Java and C#, these problems make most sense in C.

### Stack Implementation

This problem is designed to determine three things:

1.  Your knowledge of basic data structures
2.  Your ability to write routines to manipulate these structures
3.  Your ability to design consistent interfaces to a group of routines

A stack is a _last-in-first-out_ (_LIFO_) data structure: elements are always removed in the reverse order in which they were added, much in the same way that you add or remove a dish from a stack of dishes. The add element and remove element operations are conventionally called _push_ and _pop_, respectively. Stacks are useful data structures for tasks that are divided into multiple subtasks. Tracking return addresses, parameters, and local variables for subroutines is one example of stack use; tracking tokens when parsing a programming language is another.

One of the ways to implement a stack is by using a _dynamic array_, an array that changes size as needed when elements are added. (See [Chapter 7](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c07.xhtml), “Arrays and Strings,” for a more complete discussion of arrays.) A primary advantage of dynamic arrays over linked lists is that arrays offer random access to the array elements—you can immediately access any element in the array if you know its index. However, operations on a stack always work on one end of the data structure (the top of the stack), so the random accessibility of a dynamic array gains you little. In addition, as a dynamic array grows, it must occasionally be resized, which can be a time-consuming operation as elements are copied from the old array to the new array.

Linked lists usually allocate memory dynamically for each element. Depending on the overhead of the memory allocator, these allocations are often more time consuming than the copies required by a dynamic array. Additionally, adjacent elements of a dynamic array are typically adjacent in memory, while adjacent elements of a linked list may not be, and dynamic arrays don’t have the overhead of a pointer for every element. This gives dynamic arrays better memory locality, which has increasingly important performance implications as processors have become much faster than memory. For these reasons a stack based on a dynamic array is usually faster than one based on a linked list. Implementing a linked list is less complicated than implementing a dynamic array, so in an interview, a linked list is probably the best choice for your solution. Whichever choice you make, be sure to explain the pros and cons of both approaches to your interviewer.

After explaining your choice, you can design the routines and their interfaces. If you take a moment to design your code before writing it, you can avoid mistakes and inconsistencies in implementation. More importantly, this shows you won’t skip right to coding on a larger project where good planning is essential to success. As always, talk to the interviewer about what you’re doing.

Your stack will need `push` and `pop` routines. What will the prototype for these functions be? Each function must be passed the stack it operates on. The `push` operation will be passed the data it is to push, and `pop` will return a piece of data from the stack.

The simplest way to pass the stack is to pass a pointer to the stack. Because the stack will be implemented as a linked list, the pointer to the stack will be a pointer to the head of the list. In addition to the pointer to the stack, you could pass the data as a second parameter to `push`. The `pop` function could take only the pointer to the stack as an argument and return the value of the data it popped from the stack.

To write the prototypes, you need to know the type of the data that will be stored on the stack. You should declare a `struct` for a linked list element with the appropriate data type. If the interviewer doesn’t make any suggestion, storing void pointers is a good general-purpose solution:

```
typedef struct Element {
```

The corresponding prototypes for `push` and `pop` follow:

```
void push( Element *stack, void *data );
```

Now consider what happens in these routines in terms of proper functionality and error handling. Both operations change the first element of the list. The calling routine’s stack pointer must be modified to reflect this change, but any change you make to the pointer that is passed to these functions won’t be propagated back to the calling routine. You can solve this problem by having both routines take a pointer to a pointer to the stack. This way, you can change the calling routine’s pointer so that it continues to point at the first element of the list. Implementing this change results in the following:

```
void push( Element **stack, void *data );
```

What about error handling? The `push` operation needs to dynamically allocate memory for a new element. Memory allocation in C is an operation that can fail, so remember to check that the allocation succeeded when you write this routine. (In C++ an exception is thrown when allocation fails, so the error handling is somewhat different.)

You also need some way to indicate to the calling routine whether the `push` succeeded or failed. In C, it’s generally most convenient to have a routine indicate success or failure by its return value. This way, the routine can be called from the condition of an `if` statement with error handling in the body. Have `push` return `true` for success and `false` for failure. (Throwing an exception is also an option in C++ and other languages with exception support.)

Can `pop` fail? It doesn’t have to allocate memory, but what if it’s asked to pop an empty stack? It should indicate that the operation was unsuccessful, but it still has to return data when it is successful. A C function has a single return value, but `pop` needs to return two values: the data it popped and an error code.

This problem has a number of possible solutions, none of which is entirely satisfactory. One approach is to use the single return value for both purposes. If `pop` is successful, have it return the data; if it is unsuccessful, return `NULL`. As long as your data is a pointer type and you never need to store null pointers on the stack, this works. If you have to store null pointers, however, there’s no way to determine whether the null pointer returned by `pop` represents a legitimate element that you stored or an empty stack. Another option is to return a special value that can’t represent a valid piece of data—a pointer to a reserved memory block, for example, or (for stacks dealing with nonnegative numbers only) a negative value. Although restricting the values that can be stored on the stack might be acceptable in some cases, assume that for this problem it is not.

You must return two distinct values. How else can a function return data? The same way the stack parameter is handled: by passing a pointer to a variable. The routine can return data by using the pointer to change the variable’s value, which the caller can access after popping the stack.

Two possibilities exist for the interface to `pop` that use this approach to return two values. You can have `pop` take a pointer to an error code variable as an argument and return the data, or you can have it take a pointer to a data variable and return an error code. Intuitively, most programmers would expect `pop` to return data. However, using `pop` is awkward if the error code is not its return value—instead of simply calling `pop` in the condition of an `if` or `while` statement, you must explicitly declare a variable for the error code and check its value in a separate statement after you call `pop`. Furthermore, `push` would take a data argument and return an error code, whereas `pop` would take an error code argument and return data. This may offend your sense of symmetry (it does ours).

Neither alternative is clearly correct; problems arise with either approach. In an interview, it wouldn’t matter much which alternative you chose as long as you identified the pros and cons of each and justified your choice. We think error code arguments are particularly irksome, so this discussion continues by assuming you chose to have `pop` return an error code. This results in the following prototypes:

```
bool push( Element **stack, void *data );
```

You also want to write `createStack` and `deleteStack` functions, even though neither of these is absolutely necessary in a linked list stack implementation; you could delete the stack by calling `pop` until the stack is empty and create a stack by passing `push` a null pointer as the stack argument. However, writing these functions provides a complete, implementation-independent interface to the stack. A stack implemented as a dynamic array would need `createStack` and `deleteStack` functions to properly manage the underlying array. By including these functions in your implementation, you create the possibility that someone could change the underlying implementation of the stack without needing to change the programs that use the stack—always a good thing.

With the goals of implementation independence and consistency in mind, it’s a good idea to have these functions return error codes, too. Even though in a linked list implementation neither `createStack` nor `deleteStack` can fail, they might fail under a different implementation, such as if `createStack` couldn’t allocate memory for a dynamic array. If you design the interface with no way for these functions to indicate failure, you severely handicap anyone who might want to change your implementation.

Again, you face the same problem as with `pop`: `createStack` must return both the empty stack and an error code. You can’t use a null pointer to indicate failure because a null pointer is the empty stack for a linked list implementation. In keeping with the previous decision, we write an implementation with an error code as the return value. Because `createStack` can’t return the stack as its value, it must take a pointer to a pointer to the stack. Because all the other functions take a pointer to the stack pointer, it makes sense to have `deleteStack` take its stack parameter in the same way. This way you don’t need to remember which functions require only a pointer to a stack and which take a pointer to a pointer to a stack—they all work the same way. This reasoning gives you the following prototypes:

```
bool createStack( Element **stack );
```

When everything is designed properly, the coding is fairly simple. The `createStack` routine sets the stack pointer to `NULL` and returns success:

```
bool createStack( Element **stack ){
```

The `push` operation allocates the new element, checks for failure, sets the data of the new element, places it at the top of the stack, and adjusts the stack pointer:

```
bool push( Element **stack, void *data ){
```

The `pop` operation checks that the stack isn’t empty, fetches the data from the top element, adjusts the stack pointer, and frees the element that is no longer on the stack, as follows:

```
bool pop( Element **stack, void **data ){
```

Although `deleteStack` could call `pop` repeatedly, it’s more efficient to simply traverse the data structure, freeing as you go. Don’t forget that you need a temporary pointer to hold the address of the next element while you free the current one:

```
bool deleteStack( Element **stack ){
```

Before the discussion of this problem is complete, it is worth noting (and probably worth mentioning to the interviewer) that the interface design would be much more straightforward in an object-oriented language. The `createStack` and `deleteStack` operations become the constructor and destructor, respectively. The `push` and `pop` routines are bound to the stack object, so they don’t need to have the stack explicitly passed to them, and the need for pointers to pointers evaporates. An exception can be thrown when there’s an attempt to pop an empty stack or a memory allocation fails, which enables you to use the return value of `pop` for data instead of an error code. You can use templates to allow the stack to be used to store different data types, eliminating the potentially error-prone type casting required when using the C implementation that stores `void *`. A minimal C++ version looks like the following:

```
template <class T>
```

A more complete C++ implementation should include a copy constructor and assignment operator, because the default versions created by the compiler could lead to multiple deletes of the same `Element` due to inadvertent sharing of elements between copies of a `Stack`.

### Maintain Linked List Tail Pointer

This problem seems relatively straightforward. Deletion and insertion are common operations on a linked list, and you should be accustomed to using a head pointer for the list. The requirement to maintain a tail pointer is the only unusual aspect of this problem. This requirement doesn’t seem to fundamentally change anything about the list or the way you operate on it, so it doesn’t look as if you need to design any new algorithms. Just be sure to update the head and tail pointers when necessary.

When do you need to update these pointers? Obviously, operations in the middle of a long list do not affect either the head or tail. You need to update the pointers only when you change the list such that a different element appears at the beginning or end. More specifically, when you insert a new element at either end of the list, that element becomes the new beginning or end of the list. When you delete an element at the beginning or end of the list, the next-to-first or next-to-last element becomes the new first or last element.

For each operation you have a general case for operations in the middle of the list and special cases for operations at either end. When you deal with many special cases, it can be easy to miss some of them, especially if some of the special cases have more specific special cases of their own. One technique to identify special cases is to consider what circumstances are likely to lead to special cases being invoked. Then, you can check whether your proposed implementation works in each of these circumstances. If you discover a circumstance that creates a problem, you have discovered a new special case.

The circumstance where you are instructed to operate on the ends of the list has already been discussed. Another error-prone circumstance is a null pointer argument. The only other thing that can change is the list on which you are operating—specifically, its length.

What lengths of lists may be problematic? You can expect somewhat different cases for the beginning, middle, and end of the list. Any list that doesn’t have these three distinct classes of elements could lead to additional special cases. An empty list has no elements, so it obviously has no beginning, middle, or end elements. A one-element list has no middle elements and one element that is both the beginning and end element. A two-element list has distinct beginning and end elements, but no middle element. Any list longer than this has all three classes of elements and is effectively the general case of lists—unlikely to lead to additional special cases. Based on this reasoning, you should explicitly confirm that your implementation works correctly for lists of length 0, 1, and 2.

At this point in the problem, you can begin writing `delete`. If you use the common implementation that traverses the list using a single pointer to the current element, then as mentioned earlier, you need a special case for deleting the first element of the list. You can compare the element to be deleted to `head` to determine whether you need to invoke this case:

```
bool delete( Element *elem ){
```

Now write the general middle case. You need an element pointer to keep track of your position in the list. (Call the pointer `curPos`.) Recall that to delete an element from a linked list, you need a pointer to the preceding element so that you can change its next pointer. The easiest way to find the preceding element is to compare `curPos->next` to `elem`, so `curPos` points to the preceding element when you find `elem`.

You also need to construct your loop so you don’t miss any elements. If you initialize `curPos` to `head`, then `curPos->next` starts as the second element of the list. Starting at the second item is fine because you treat the first element as a special case, but make your first check before advancing `curPos` or you’ll miss the second element. If `curPos` becomes `NULL`, you have reached the end of the list without finding the element you were supposed to delete, so you should return failure. The middle case yields the following (added code is bolded):

```
bool delete( Element *elem ){
```

Next, consider the last element case. The last element’s next pointer is `NULL`. To remove it from the list, you need to make the next-to-last element’s next pointer `NULL` and free the last element. If you examine the loop constructed for middle elements, you see that it can delete the last element as well as middle elements. The only difference is that you need to update the tail pointer when you delete the last element. If you set `curPos->next` to `NULL`, you know you changed the end of the list and must update the tail pointer. Adding this to complete the function, you get the following:

```
bool delete( Element *elem ){
```

This solution covers the three discussed special cases. Before you present the interviewer with this solution, you should check behavior for null pointer arguments and the three potentially problematic list length circumstances.

What happens if `elem` is `NULL`? The `while` loop traverses the list until `curPos->next` is `NULL` (when `curPos` is the last element). Then, on the next line, evaluating `elem->next` dereferences a null pointer. Because it’s never possible to delete `NULL` from the list, the easiest way to fix this problem is to return `false` if `elem` is `NULL`.

If the list has zero elements, then `head` and `tail` are both `NULL`. Because you’ll check that `elem` isn’t `NULL`, `elem == head` will always be false. Further, because `head` is `NULL`, `curPos` will be `NULL`, and the body of the `while` loop won’t be executed. There doesn’t seem to be any problem with zero-element lists. The function simply returns `false` because nothing can be deleted from an empty list.

Now try a one-element list. In this case, `head` and `tail` both point to the one element, which is the only element you can delete. Again, `elem == head` is true. `elem->next` is `NULL`, so you correctly set `head` to `NULL` and free the element; however, `tail` still points to the element you just freed. As you can see, you need another special case to set `tail` to `NULL` for one-element lists.

What about two-element lists? Deleting the first element causes `head` to point to the remaining element, as it should. Similarly, deleting the last element causes `tail` to be correctly updated. The lack of middle elements doesn’t seem to be a problem. You can add the two additional special cases and then move on to `insertAfter`:

```
bool delete( Element *elem ){
```

You can apply similar reasoning to writing `insertAfter`. Because you allocate a new element in this function, you must take care to check that the allocation is successful and that you don’t leak any memory. Many of the special cases encountered in `delete` are relevant in `insertAfter`, however, and the code is structurally similar:

```
bool insertAfter( Element *elem, int data ){
```

These are adequate solutions, but not particularly elegant ones. Each of them has multiple special cases, and one special case that’s nested inside another. Enumerating special cases as you design the algorithm is good practice. Many interview problems have special cases, so you should expect to encounter them frequently. In the real world of programming, unhandled special cases represent bugs that may be difficult to find, reproduce, and fix. Programmers who identify special cases as they are coding are likely to be more productive than those who find special cases through debugging.

An alternative to writing special-case code is to try to further generalize your general-case algorithm so it can handle the special cases as the general case. When possible, this may produce code that is more concise, elegant, better performing, and easier to maintain.

The introduction to this chapter demonstrated a technique to eliminate the special-case code for updating the head pointer when the first element is deleted. How might you use that approach to eliminate special-case code for this problem?

Again, start by considering `delete`. If you try to apply the technique from the introduction directly, you’ll encounter a problem. The `deleteElement` function in the introduction didn’t need to do anything with the element preceding the element to be deleted other than changing `next`, so a pointer to `next` was sufficient. In the current problem, you may need to set `tail` to point at the element preceding the deleted element; there’s no good, portable way to get the address of the element if all you have is a pointer to its `next` field. One solution is to traverse the list with two pointers in tandem: `curPos` pointing to the current element and `ppNext` pointing to the pointer to the next element. Try writing out this implementation.

Think carefully about what the initial values for these pointers should be. The reason for using `ppNext` is that it generalizes updating the `head` pointer and the `next` pointers; to accomplish this `ppNext` must be initialized to `&head`. If `ppNext` points to `head`, the current position in the traversal is effectively before the first element of the list. Because there is no element before the first element, you can initialize `curPos` to `NULL`. This helps to generalize updating `tail`, but also introduces the complication that `curPos` is `NULL` at the beginning as well as the end of the list traversal. You need to make sure that you advance `curPos` before testing its value; otherwise you’ll never traverse the list. Reimplementing `delete` with these considerations yields a more concise, elegant function:

```
bool delete( Element *elem ){
```

You can reimplement `insertAfter` using a similar approach with similar improvement in length and elegance:

```
bool insertAfter( Element *elem, int data ){
```

Recognizing problematic special cases and writing code to specifically address them is important; recoding to make special cases cease to exist is even better.

### Bugs in removeHead

Bug-finding problems occur with some frequency, so it’s worthwhile to discuss a general strategy that you can apply to this and other problems.

Because you will generally be given only a small amount of code to analyze, your bug-finding strategy will be a little different from real-world programming. You don’t need to worry about interactions with other modules or other parts of the program. Instead, you must do a systematic analysis of every line of the function without the help of a debugger. Consider four common problem areas for any function you are given:

1.  **Check that the data comes into the function properly.** Make sure you aren’t accessing a variable that you don’t have, you aren’t reading something as an `int` that should be a `long`, and you have all the values you need to perform the task.
2.  **Check that each line of the function works correctly.** The function is intended to perform a task. Verify that the task is executed correctly at each line and that the desired result is produced at the end.
3.  **Check that the data comes out of the function correctly.** The return value should be what you expect. In addition, if the function is expected to update any caller variables, make sure this occurs.
4.  **Check for common error conditions.** Error conditions vary depending on the specifics of a problem. They tend to involve unusual argument values. For instance, functions that operate on data structures may have trouble with empty or nearly empty data structures; functions that take a pointer as an argument may fail if passed a null pointer. Make sure that error conditions from operations that can fail, such as memory allocation and I/O, are handled appropriately.

Starting with the first step, verify that data comes into the function properly. In a linked list, you can access every element given only the head. Because you are passed the list head, you have access to all the data you require—no bugs so far.

Now do a line-by-line analysis of the function. The first line frees `head`—okay so far. Line 2 then assigns a new value to `head` but uses the old value of `head` to do this. That’s a problem. You have already freed `head`, and you are now dereferencing freed memory. You could try reversing the lines, but this would cause the element after `head` to be freed. You need to free `head`, but you also need its `next` value after it has been freed. You can solve this problem by using a temporary variable to store `head`’s `next` value. Then you can free `head` and use the temporary variable to update `head`. These steps make the function look like the following:

```
void removeHead( ListElement *head ){
```

Now, move to step 3 of the strategy to make sure the function returns values properly. Though there is no explicit return value, there is an implicit one. This function is supposed to update the caller’s `head` value. In C, all function parameters are passed by value, so functions get a local copy of each argument, and any changes made to that local copy are not reflected outside the function. Any new value you assign to `head` on line 3 has no effect—another bug. To correct this, you need a way to change the value of `head` in the calling code. Variables cannot be passed by reference in C, so the solution is to pass a pointer to the variable you want to change—in this case, a pointer to the head pointer. After the change, the function should look like this:

```
void removeHead( ListElement **head ){
```

Now you can move on to the fourth step and check error conditions. Check a one-element and a zero-element list. In a one-element list, this function works properly. It removes the one element and sets the `head` to `NULL`, indicating that the head was removed. Now take a look at the zero-element case. A zero-element list is simply a null pointer. If `head` is a null pointer, you would dereference a null pointer on line 1. To correct this, check whether `head` is a null pointer and be sure not to dereference it in this case. This check makes the function look like the following:

```
void removeHead( ListElement **head ){
```

You have checked that the body of the function works properly, that the function is called correctly and returns values correctly, and that you have dealt with the error cases. You can declare your debugging effort complete and present this version of `removeHead` to the interviewer as your solution.

### _M_th-to-Last Element of a Linked List

Why is this a difficult problem? Finding the _m_th element from the beginning of a linked list would be an extremely trivial task. Singly linked lists can be traversed only in the forward direction. For this problem you are asked to find a given element based on its position relative to the _end_ of the list. While you traverse the list you don’t know where the end is, and when you find the end, there is no easy way to backtrack the required number of elements.

You may want to tell your interviewer that a singly linked list is a particularly poor choice for a data structure when you frequently need to find the _m_th-to-last element. If you were to encounter such a problem while implementing a real program, the correct and most efficient solution would probably be to substitute a more suitable data structure (such as a doubly linked list or dynamic array) to replace the singly linked list. Although this comment shows that you understand good design, the interviewer still wants you to solve the problem as it was originally phrased.

How, then, can you get around the problem that there is no way to traverse backward through this data structure? You know that the element you want is _m_ elements from the end of the list. Therefore, if you traverse _m_ elements forward from an element and that places you exactly at the end of the list, you have found the element you were searching for. One approach is to simply test each element in this manner until you find the one you’re searching for. Intuitively, this feels like an inefficient solution because you will traverse over the same elements many times. If you analyze this potential solution more closely, you can see that you would be traversing _m_ elements for most of the elements in the list. If the length of the list is _n_, the algorithm would be _O_(_mn_). You need to find a solution more efficient than _O_(_mn_).

What if you store some of the elements (or, more likely, pointers or references to the elements) as you traverse the list? Then, when you reach the end of the list, you can look back _m_ elements in your storage data structure to find the appropriate element. If you use an appropriate temporary storage data structure, this algorithm is _O_(_n_) because it requires only one traversal through the list. Yet this approach is far from perfect. As _m_ becomes large, the temporary data structure would become large as well. In the worst-case scenario, this approach might require almost as much storage space as the list itself—not a particularly space-efficient algorithm.

Perhaps working back from the end of the list is not the best approach. Because counting from the beginning of the list is trivial, is there any way to count from the beginning to find the desired element? The desired element is _m_ from the end of the list, and you know the value of _m_. It must also be _l_ elements from the beginning of the list, although you don’t know _l_. However, _l + m = n_, the length of the list. It’s easy to count all the elements in the list. Then you can calculate _l = n – m_, and traverse _l_ elements from the beginning of the list.

Although this process involves two passes through the list, it’s still _O_(_n_). It requires only a few variables’ worth of storage, so this method is a significant improvement over the previous attempt. If you could change the functions that modify the list such that they would increment a count variable for every element added and decrement it for every element removed, you could eliminate the count pass, making this a relatively efficient algorithm. Again, though this point is worth mentioning, the interviewer is probably looking for a solution that doesn’t modify the data structure or place any restrictions on the methods used to access it.

Assuming you must explicitly count the elements in the current algorithm, you must make almost two complete traversals of the linked list. A large list on a memory-constrained system might exist mostly in paged-out virtual memory (on disk). In such a case, each complete traversal of the list would require a large amount of disk access to swap the relevant portions of the list in and out of memory. Under these conditions, an algorithm that made only one complete traversal of the list might be significantly faster than an algorithm that made two traversals, even though they would both be _O_(_n_). Is there a way to find the target element with a single traversal?

The counting-from-the-beginning algorithm obviously demands that you know the length of the list. If you can’t track the length so that you know it ahead of time, you can determine the length only by a full-list traversal. There doesn’t seem to be much hope for getting this algorithm down to a single traversal.

Try reconsidering the previous linear time algorithm, which required only one traversal but was rejected for requiring too much storage. Can you reduce the storage requirements of this approach?

When you reach the end of the list, you are actually interested in only one of the _m_ elements you’ve been tracking—the element that is _m_ elements behind your current position. You are tracking the rest of the _m_ elements merely because the element _m_ behind your current position changes every time your position advances. Keeping a queue _m_ elements long, where you add the current element to the head and remove an element from the end every time you advance your current position, ensures that the last element in the queue is always _m_ elements behind your current position.

In effect, you are using this _m_ element data structure to make it easy to implicitly advance an _m_\-behind pointer in lock step with your current position pointer. However, this data structure is unnecessary—you can explicitly advance the _m_\-behind pointer by following each element’s next pointer just as you do for your current position pointer. This is as easy as (or perhaps easier than) implicitly advancing by shifting through a queue, and it eliminates the need to track all the elements between your current position pointer and your _m_\-behind pointer. This algorithm seems to be the one you’ve been looking for: linear time, a single traversal, and negligible storage requirements. Now you just need to work out the details.

You need to use two pointers: a current position pointer and an _m_\-behind pointer. You must ensure that the two pointers are actually spaced _m_ elements apart; then you can advance them at the same rate. When your current position is the end of the list, _m_\-behind points to the _m_th-to-last element. How can you get the pointers spaced properly? If you count elements as you traverse the list, you can move the current position pointer to the _m_th element of the list. If you then start the _m_\-behind pointer at the beginning of the list, they will be spaced _m_ elements apart.

Are there any error conditions you need to watch for? If the list is less than _m_ elements long, then there is no _m_th-to-last element. In such a case, you would run off the end of the list as you tried to advance the current position pointer to the _m_th element, possibly dereferencing a null pointer in the process. Therefore, check that you don’t hit the end of the list while doing this initial advance.

With this caveat in mind, you can implement the algorithm. It’s easy to introduce off-by-one errors in any code that spaces any two things _m_ items apart or counts _m_ items from a given point. You may want to refer to the exact definition of “_m_th to last” given in the problem and try a simple example on paper to make sure you get your counts right, particularly in the initial advancement of `current`.

```
ListElement *findMToLastElement( ListElement *head, int m ){
```

### List Flattening

This list-flattening problem gives you plenty of freedom. You have simply been asked to flatten the list. You can accomplish this task in many ways. Each way results in a one-level list with a different node ordering. Start by considering several options for algorithms and the node orders they would yield. Then implement the algorithm that looks easiest and most efficient.

Begin by looking at the data structure. This data structure is a little unusual for a list. It has levels and children—somewhat like a tree. A tree also has levels and children, as you’ll see in the next chapter, but trees don’t have links between nodes on the same level. You might try to use a common tree traversal algorithm and copy each node into a new list as you visit it as a simple way to flatten the structure.

The data structure is not exactly a normal tree, so any traversal algorithm you use must be modified. From the perspective of a tree, each separate child list in the data structure forms a single extended tree node. This may not seem too bad: where a standard traversal algorithm checks the child pointers of each tree node directly, you just need to do a linked list traversal to check all the child pointers. Every time you check a node, you can copy it to a duplicate list. This duplicate list will be your flattened list.

Before you work out the details of this solution, consider its efficiency. Every node is examined once, so this is an _O_(_n_) solution. There is likely to be some overhead for the recursion or data structure required for the traversal. In addition, you make a duplicate copy of each node to create the new list. This copying is inefficient, especially if the structure is large. See if you can identify a more efficient solution that doesn’t require so much copying.

So far, the proposed solution has concentrated on an algorithm, letting the ordering follow. Instead, try focusing on an ordering and then try to deduce an algorithm. You can focus on the data structure’s levels as a source of ordering. It helps to define the parts of a level as _child lists_. Just as rooms in a hotel are ordered by level, you can order nodes by the level in which they occur. Every node is in a level and appears in an ordering within that level (arranging the child lists from left to right). Therefore, you have a logical ordering just like hotel rooms. You can order by starting with all the first-level nodes, followed by all the second-level nodes, followed by all the third-level nodes, and so on. Applying these rules to the example data structure, you should get the ordering shown in [Figure 5-4](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c05-fig-0004).

![[attachments/c05f004.jpg]]

[**FIGURE 5-4**](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c05-fig-0004)

Now try to discover an algorithm that yields this ordering. One property of this ordering is that you never rearrange the order of the nodes in their respective levels, so you could connect all the nodes on each level into a list and then join all the connected levels. However, to find all the nodes on a given level so that you can join them, you would need to do a breadth-first search of that level. Breadth-first searching is inefficient, so you should continue to look for a better solution.

In [Figure 5-3](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c05-fig-0003), the second level is composed of two child lists. Each child list starts with a different child of a first-level node. You could try to append the child lists one at a time to the end of the first level instead of combining the child lists.

To append the child lists one at a time, traverse the first level from the start, following the next pointers. Every time you encounter a node with a child, append the child (and thus the child list) to the end of the first level and update the tail pointer. Eventually, you append the entire second level to the end of the first level. You can continue traversing the first level and arrive at the start of the old second level. If you continue this process of appending children to the end of the first level, you eventually append every child list to the end and have a flattened list in the required order. More formally, this algorithm is as follows:

```
Start at the beginning of the first level
```

This algorithm is easy to implement because it’s so simple. In terms of efficiency, every node after the first level is examined twice. Each node is examined once when you update the tail pointer for each child list and once when you examine the node to see if it has a child. The nodes in the first level are examined only once when you examine them for children because you had a first-level tail pointer when you began. Therefore, there are no more than 2_n_ comparisons in this algorithm, and it is an _O_(_n_) solution. This is the best time order you can achieve because every node must be examined. Although this solution has the same time order as the tree traversal approach considered earlier, it is more efficient because it requires no recursion or additional memory. (Other equally efficient solutions to this problem exist. One such solution involves inserting child lists after their parents, rather than at the end of the list.)

The code for this algorithm is as follows. Note that the function takes a pointer to the tail pointer so that changes to the tail pointer are retained when the function returns:

```
void flattenList( Node *head, Node **tail ){
```

### List Unflattening

This problem is the reverse of the previous problem, so you already know a lot about this data structure. One important insight is that you created the flattened list by combining all the child lists into one long level. To get back the original list, you must separate the long flattened list back into its original child lists.

Try doing the exact opposite of what you did to create the list. When flattening the list, you traversed down the list from the start and added child lists to the end. To reverse this, you go backward from the tail and break off parts of the first level. You could break off a part when you encounter a node that was the beginning of a child list in the unflattened list. Unfortunately, this is more difficult than it might seem because you can’t easily determine whether a particular node is a child (indicating that it started a child list) in the original data structure. The only way to determine whether a node is a child is to scan through the child pointers of all the previous nodes. All this scanning would be inefficient, so you should examine some additional possibilities to find a better solution.

One way to get around the child node problem is to go through the list from start to end, storing pointers to all the child nodes in a separate data structure. Then you could go backward through the list and separate every child node. Looking up nodes in this way frees you from repeated scans to determine whether a node is a child. This is a good solution, but it still requires an extra data structure. Try looking for a solution without an extra data structure.

It seems you have exhausted all the possibilities for going backward through the list, so try an algorithm that traverses the list from the start to the end. You still can’t immediately determine whether a node is a child. One advantage of going forward, however, is that you can find all the child nodes in the same order that you appended them to the first level. You also know that every child node began a child list in the original list. If you separate each child node from the node before it, you get the unflattened list back.

You can’t simply traverse the list from the start, find each node with a child, and separate the child from its previous node. You would get to the end of your list at the break between the first and second level, leaving the rest of the data structure untraversed. This approach seems promising, though. You can traverse every child list, starting with the first level (which is a child list itself). When you find a child, continue traversing the original child list and also traverse the newly found child list. You can’t traverse both at the same time, however. You could save one of these locations in a data structure and traverse it later. However, rather than design and implement this data structure, you can use recursion. Specifically, every time you find a node with a child, separate the child from its previous node, start traversing the new child list, and then continue traversing the original child list.

This is an efficient algorithm because each node is checked at most twice, resulting in an _O_(_n_) running time. Again, an _O_(_n_) running time is the best you can do because you must check each node at least once to see if it is a child. In the average case, the number of function calls is small in relation to the number of nodes, so the recursive overhead is not too bad. In the worst case, the number of function calls is no more than the number of nodes. This solution is approximately as efficient as the earlier proposal that required an extra data structure, but somewhat simpler and easier to code. Therefore, this recursive solution would probably be a better choice in an interview. In outline form, the algorithm looks like the following:

```
Explore path:
```

It can be implemented in C as:

```
/* unflattenList wraps the recursive function and updates the tail pointer. */
```

The preceding solution was derived by reversing the list-flattening algorithm. The function call overhead in a recursive implementation can be substantial. Just because a problem can be solved with recursion doesn’t mean it should be: consider whether there’s a simple iterative solution.

You might iterate through the list from `start`, looking for `child` pointers that are not `NULL`. When you find one, you could break the remainder of the list starting at the child off so that it becomes part of a lower level. However, this has the downside that the entire remainder of the list immediately becomes part of the next lower level. Instead of reconstructing the original data structure this would produce a data structure that looks like a set of stairs.

However, if you look at how you constructed the flattened list, each child brought up to a higher level is placed before each child encountered later in the search. So, if you start from the end of the list and work backward, you can unflatten the list by breaking off each child list as soon as you encounter its parent, while avoiding the previously described problem you would have if you used this strategy while iterating forward through the list. If you’re careful about how you break out the child list and track `tail` carefully, you can reconstruct the original list.

The C code to do this is:

```

```

### Null or Cycle

Start by looking at the pictures to see if you can determine an intuitive way to differentiate a cyclic list from an acyclic list.

The difference between the two lists appears at their ends. In the cyclic list, there is an end node that points back to one of the earlier nodes. In the acyclic list, there is an end node that is null terminated. Thus, if you can find this end node, you can test whether the list is cyclic or acyclic.

In the acyclic list, it is easy to find the end node. You traverse the list until you reach a null terminated node.

In the cyclic list, though, it is more difficult. If you just traverse the list, you go in a circle and won’t know whether you’re in a cyclic list or just a long acyclic list. You need a more sophisticated approach.

Consider the end node a bit more. The end node points to a node that has another node pointing at it. This means that two pointers are pointing at the same node. This node is the only node with two elements pointing at it. You can design an algorithm around this property. You can traverse the list and check every node to determine whether two other nodes are pointing at it. If you find such a node, the list must be cyclic. Otherwise, the list is acyclic, and you will eventually encounter a null pointer.

Unfortunately, it is difficult to check the number of nodes pointing at each element. Look for another special property of the end node in a cyclic list. When you traverse the list, the end node’s next node is a node that you have previously encountered. Instead of checking for a node with two pointers pointing at it, you can check whether you have already encountered a node. If you find a previously encountered node, you have a cyclic list. If you encounter a null pointer, you have an acyclic list. This is only part of the algorithm. You still need to figure out how to determine whether you have previously encountered a node.

The easiest way to do this would be to mark each element as you visit it, but you’ve been told you’re not allowed to modify the list. You could keep track of the nodes you’ve encountered by putting them in a separate list. Then you would compare the current node to all the nodes in the already-encountered list. If the current node ever points to a node in the already-encountered list, you have a cycle. Otherwise, you’ll get to the end of the list and see that it’s null terminated and thus acyclic. This would work, but in the worst case the already-encountered list would require as much memory as the original list. Try to reduce this memory requirement.

What are you storing in the already-encountered list? The already-encountered list’s first node points to the original list’s first node, its second node points to the original list’s second node, its third node points to the original list’s third node, and so on. You’re creating a list that mirrors the original list. This is unnecessary—you can just use the original list.

Try this approach: Because you know your current node in the list and the start of the list, you can compare your current node’s next pointer to all its previous nodes directly. For the _i_th node, compare its next pointer to see if it points to any of nodes 1 to _i_ – 1. If any are equal, you have a cycle.

What’s the time order of this algorithm? For the first node, 0 previous nodes are examined; for the second node, one previous node is examined; for the third node, two previous nodes are examined, and so on. Thus, the algorithm examines 0 + 1 + 2 + 3 +. . .+ _n_ nodes. As discussed in [Chapter 4](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c04.xhtml), such an algorithm is _O_(_n_<sup>2</sup>).

That’s about as far as you can go with this approach. Although it’s difficult to discover without some sort of hint, there is a better solution involving two pointers. What can you do with two pointers that you couldn’t do with one? You can advance them on top of each other, but then you might as well have one pointer. You could advance them with a fixed interval between them, but this doesn’t seem to gain anything. What happens if you advance the pointers at different speeds?

In the acyclic list, the faster pointer reaches the end. In the cyclic list, they both loop endlessly. The faster pointer eventually catches up with and passes the slower pointer. If the fast pointer is ever behind or equal to the slower pointer, you have a cyclic list. If it encounters a null pointer, you have an acyclic list. You’ll need to start the fast pointer one node ahead of the slow pointer so they’re not equal to begin with. In outline form, this algorithm looks like this:

```
Start slow pointer at the head of the list
```

You can now implement this solution:

```
/* Takes a pointer to the head of a linked list and determines if
```

Is this algorithm faster than the earlier solution? If this list is acyclic, the faster pointer comes to the end after examining _n_ nodes, while the slower pointer traverses 1⁄2 _n_ nodes. Thus, you examine 3_⁄_2_n_ nodes, which is an _O_(_n_) algorithm.

What about a cyclic list? The slower pointer never goes around any loop more than once. When the slower pointer has examined _n_ nodes, the faster pointer will have examined 2_n_ nodes and “passed” the slower pointer, regardless of the loop’s size. Therefore, in the worst case you examine 3_n_ nodes, which is still _O_(_n_). Regardless of whether the list is cyclic or acyclic, this two-pointer approach is better than the single-pointer approach to the problem.

## SUMMARY

Although they are simple data structures, problems with linked lists often arise in interviews focusing on C or C++ experience as a way to determine whether a candidate understands basic pointer manipulation. Each element in a singly linked list contains a pointer to the next element in the list, whereas each element in a doubly linked list points to both the previous and the next elements. The first element in both list types is referred to as the _head_, whereas the last element is referred to as the _tail_. Circular linked lists have no head or tail; instead, the elements are linked together to form a cycle.

List operations are much simpler to perform on doubly linked lists, so most interview problems use singly linked lists. Typical operations include updating the head of the list, traversing the list to find a specific element from the end of the list, and inserting or removing list elements.
