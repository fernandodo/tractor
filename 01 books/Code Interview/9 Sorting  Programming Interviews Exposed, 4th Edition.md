---
created: 2025-09-23T20:39:39 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 9 Sorting | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> 9Sorting

 Sorting algorithms have several applications. The most obvious is to order data for presentation to the user, such as sorting a list of employees alphabetically by employee ID number...

---
## 9  
Sorting

Sorting algorithms have several applications. The most obvious is to order data for presentation to the user, such as sorting a list of employees alphabetically by employee ID number or last name. Another important use is as a building block for other algorithms, where having ordered data enables optimizations that wouldn’t otherwise be possible.

You rarely need to code a sorting algorithm. Most languages include at least one sorting algorithm (typically quicksort) in their standard libraries. These built-in algorithms are suitable for general use. In situations in which a general-purpose sorting algorithm doesn’t meet your needs, implementations of specialized sorting algorithms can usually be adapted with minimal effort.

Although you’re unlikely to implement sorting algorithms, it’s important to understand the differences and trade-offs between them. Each algorithm has benefits and drawbacks, and there’s no single best way to sort in all cases. Interviewers like sorting problems because they provide a simple way to address a wide range of issues from algorithmic complexity to memory usage.

## SORTING ALGORITHMS

Choosing the right sorting algorithm can have a huge impact on application performance. What’s right in one situation isn’t necessarily right for another. Here are some criteria to consider when selecting a sorting algorithm:

-   **How much data is to be sorted?** For small data sets it doesn’t matter which algorithm you choose because there is little difference in the execution times, but for large data sets, the worst-case bounds become radically different. Beware of data sets that are typically small but may occasionally be much larger—you need to select an algorithm that performs acceptably on the largest data sets your code may encounter.
-   **Does the data fit in memory?** Most sorting algorithms are efficient only when the data they operate on resides in memory. If the data set is too large for memory, you may need to split it into smaller chunks for sorting and then combine those sorted chunks to create the final sorted data set.
-   **Is the data already mostly sorted?** You can add new data to a sorted list efficiently with certain algorithms, but those same algorithms have poor performance on randomly ordered data.
-   **How much additional memory does the algorithm require?** An _in-place_ sorting algorithm sorts the data without using any additional memory, such as by swapping elements in an array. When memory is at a premium, an in-place algorithm may be a better choice than one with otherwise superior efficiency.
-   **Is relative order preserved?** A _stable_ sorting algorithm preserves the relative order of data elements that are otherwise identical for sorting purposes. (In other words, if elements A and B have identical key values and A precedes B in the original data set, A will still precede B after a stable sorting.) Stability is generally a desirable feature, but in many cases it may be worth sacrificing stability for improved performance.

In an interview situation, it’s not unusual for the interviewer to vary the criteria as the interview progresses to see how well you understand the differences between sorting algorithms.

For simplicity, the sorting problems used in interviews often deal with simple integer values stored in arrays. In the real world, sorting usually involves more complex data structures with only one or a few of the values in those data structures affecting the sorting order. The value (or values) that determines the sorting order is referred to as the _key_. Most sorting algorithms in standard libraries are _comparison_ algorithms, which require only that there is a way to determine whether one key is less than, equal to, or greater than another key. No comparison algorithm can have a more optimal worst-case running time than _O_(_n_ log(_n_)).

### Selection Sort

_Selection sort_ is one of the simplest sorting algorithms. It starts with the first element in the array (or list) and scans through the array to find the element with the smallest key, which it swaps with the first element. The process is then repeated with each subsequent element until the last element is reached.

The description of this algorithm suggests a recursive approach, as shown here with the `selectionSortRecursive` method:

```
// Sort an array using a recursive selection sort.
```

This implementation depends on the two helper routines `findMinimumIndex` and `swap`:

```
// Find the position of the minimum value starting at the given index.
```

This implementation could be optimized by transforming this tail-recursive procedure into an iterative implementation and in-lining the two helper functions.

How efficient is selection sort? The first swap requires _n_ – 1 comparisons, the second _n_ – 2, the third _n_ – 3, and so on. This is the series (_n_ – 1) + (_n_ – 2) + ... + 1, which simplifies to _n_(_n_ – 1)/2. This means that the algorithm is _O_(_n_<sup>2</sup>) in the best, average, _and_ worst cases—the initial order of the data has no effect on the number of comparisons. As you’ll see later in this chapter, other sorting algorithms have more efficient running times than this.

Selection sort does have the advantage that it requires at most _n_ – 1 swaps. In situations in which moving data elements is more expensive than comparing them, selection sort may perform better than other algorithms. The efficiency of an algorithm depends on what you’re optimizing for.

Selection sort is an in-place algorithm. Typical implementations of selection sort, such as the one shown here, are not stable.

### Insertion Sort

_Insertion sort_ is another simple sorting algorithm. It builds a sorted array (or list) one element at a time by comparing each new element to the already-sorted elements and _inserting_ the new element into the correct location, similar to the way you sort a hand of playing cards.

A simple implementation of insertion sort is as follows:

```
// Sort an array using a simple insertion sort.
```

Unlike selection sort, the best-case running time for insertion sort is _O_(_n_), which occurs when the list is already sorted. This means insertion sort is an efficient way to add a few new elements to a presorted list. The average and worst cases are both _O_(_n_<sup>2</sup>), however, so it’s not the best algorithm to use for large amounts of randomly ordered data.

In the preceding implementation, insertion sort is a stable, in-place sorting algorithm especially suitable for sorting small data sets and is often used as a building block for other, more complicated sorting algorithms.

### Quicksort

_Quicksort_ is a divide-and-conquer algorithm that involves choosing a _pivot value_ from a data set and then splitting the set into two subsets: a set that contains all values less than the pivot and a set that contains all values greater than or equal to the pivot. The pivot/split process is recursively applied to each subset until there are no more subsets to split. The results are combined to form the final sorted set.

A naïve implementation of this algorithm looks like:

```
// Sort an array using a simple but inefficient quicksort.
```

The preceding code illustrates the principles of quicksort, but it’s not a particularly efficient implementation due to scanning the starting array twice, allocating new arrays, and copying results from the new arrays to the original.

Quicksort’s performance is dependent on the choice of pivot value. The ideal pivot value is one that splits the original data set into two subsets of identical (or nearly identical) size. Every time you do a pivot-and-split, you perform constant-time operations on each of the elements involved. How many times do you do this for each element? In the best case, the size of a sublist is halved on each successive recursive call, and the recursion terminates when the sublist size is 1. This means the number of times you operate on an element is equal to the number of times you can divide _n_ by 2 before reaching 1: log(_n_). Performing log(_n_) operations on each of _n_ elements yields a combined _best case_ complexity of _O_(_n_ log(_n_)).

On the other hand, what if your pivot choice is poor? In the worst case, the pivot is the minimum (or maximum) value in the data set, which means that one subset is empty and the other subset contains _n_ – 1 items (all the items except for the pivot). The number of recursive calls is then _O_(_n_) (analogous to a completely unbalanced tree degenerating to a linked list), which gives a combined worst-case complexity of _O_(_n_<sup>2</sup>). This is the same as selection sort or insertion sort.

On average almost any pivot value will split a data set into two nonempty subsets, making the number of recursive calls fall somewhere between _O_(log(_n_)) and _O_(_n_). A bit of mathematical work (omitted here) is enough to show that in _most_ cases the number of times you operate on an element is still _O_(log(_n_)), so the _average case_ complexity of quicksort is also _O_(_n_ log(_n_)).

For truly randomly ordered data, the value of the pivot is unrelated to its location, so you can choose a pivot from any location because they’re all equally likely to be good choices. But if the data is already sorted (or mostly sorted), choosing the value located in the middle of the data set ensures that each subset contains approximately half the data, which gives guaranteed _O_(_n_ log(_n_)) complexity for sorted data. Because the value in the middle location is the best choice for ordered data and no worse than any other for unordered data, most quicksort implementations use it as the pivot.

Like the preceding implementation, most implementations of quicksort are not stable.

### Merge Sort

_Merge sort_ is another divide-and-conquer algorithm that works by splitting a data set into two or more subsets, sorting the subsets, and then merging them together into the final sorted set.

The algorithm can be implemented recursively as follows:

```
// Sort an array using a simple but inefficient merge sort.
```

Most of the work is done in the `merge` method, which combines two sorted arrays into a larger sorted array.

A _hybrid_ merge sort occurs when a different sorting algorithm is used to sort subsets below a specified minimum size. For example, you can transform the `mergeSortSimple` method into a hybrid algorithm by replacing the termination condition:

```
if ( data.length < 2 ){
```

with an insertion sort:

```
if ( data.length < 10 ){ // some small empirically determined value
```

This is a common optimization because insertion sort has lower overhead than merge sort and typically has better performance on very small data sets.

Unlike most other sorting algorithms, merge sort is a good choice for data sets that are too large to fit into memory. In a typical scenario, the contents of a large file are split into multiple smaller files. Each of the smaller files is read into memory, sorted using an appropriate algorithm, and written back out. A merge operation is then performed using the sorted files as input and the sorted data is written directly to the final output file.

The best, average, and worst-case running times for merge sort are all _O_(_n_ log(_n_)), which is great when you need a guaranteed upper bound on the sorting time. However, merge sort requires _O_(_n_) additional storage—substantially more than many other algorithms.

Typical (maximally efficient) merge sort implementations are stable but not in-place.

## SORTING PROBLEMS

Sorting problems often involve selecting the most appropriate algorithm for a particular situation, or modifying a standard sorting algorithm to give it a new property.

### The Best Sorting Algorithm

This is a bit of a trick question. The key is _not_ to just respond with “quicksort” (or any other specific sorting algorithm). If you do, your interviewer will likely describe a scenario in which the algorithm you just named is particularly poorly suited and then ask you if you still think that algorithm is the best choice. Don’t get drawn into that trap!

Each sorting algorithm has its strengths and weaknesses, so you need to fully understand the context before you can select the best algorithm for a particular situation. Start by asking the interviewer some questions about the data you are sorting, the requirements for the sort, and the system that will perform the sort. Specifically, you might ask some of these questions:

-   **What do we know about the data?** Is the data already sorted or mostly sorted? How large are the data sets likely to be? Can there be duplicate key values?
-   **What are the requirements for the sort?** Do you want to optimize for best-case, worst-case, or average-case performance? Does the sort need to be stable?
-   **What do we know about the system?** Is the largest data set to be sorted smaller than, the same size as, or larger than available memory?

Sometimes, just asking these questions is enough to illustrate your knowledge of sorting algorithms. (One of the authors started this problem in an interview by asking the question “What can you tell me about the data?” The interviewer responded “Yes, that’s the right answer,” and moved on to a different problem.) More commonly, the interviewer will answer your questions and describe a scenario that points toward one algorithm as a better choice than the others.

The naïve approach to this is to concatenate all the sublists and apply a general-purpose sorting algorithm such as quicksort to the combined list, yielding _O_(_n_ log(_n_)) running time (where _n_ is the combined size of all the departmental lists).

What do you know about the data that might help you find a more efficient solution? In this case, you know that the sublists are sorted. Can you use this to your advantage? You have several sorted sublists and you need to combine them. This sounds very much like part of a merge sort. In fact, the situation here is like the final stage of a merge sort, after the recursive calls have already sorted the sublists. All that’s left to do is merge the lists. In merge sort, the merge operation is only _O_(_n_), so it might seem like this strategy would yield an _O_(_n_) sort. Will it?

Consider the differences between the merge operation in merge sort and the merge you need to do here. In merge sort, you’re always merging two lists, so you make a single comparison between the next element of each list. For this problem, you’re merging one list for each server. With multiple lists to merge, it’s a little less straightforward to compare the next elements of all the lists to identify the next smallest key. If you take the naïve approach of scanning through the next element of each list, then if there are _k_ lists, this scanning operation will be _O_(_k_). You’ll need to do this for every element you sort, so overall this algorithm would be _O_(_kn_). If the number of servers producing lists (_k_) is small, this may outperform quicksort of the concatenated lists, but as the number of servers grows this could easily become less efficient than _O_(_n_ log(_n_)). Is there a way to salvage this approach?

You know you have to examine every element, so there’s no way to avoid the factor of _n_. Focus instead on the scan of the next element from each server that yields the factor of _k_. You have a set of elements and you want to be able to efficiently find the smallest element in the set. This is the problem that a heap is designed to address. If you construct a min-heap consisting of the next element from each list, you can efficiently find the next element to merge. Each time you find the next element to merge, you’ll need to remove it from the heap and add a new element to the heap. These are both logarithmic time operations. But because you only need to keep the next element from each list on the heap (rather than all the elements) this is _O_(log(_k_)) rather than _O_(log(_n_)), or even worse the _O_(_k_) of checking the next element of each list. This yields an overall running time of _O_(_n_ log(_k_)), which is an improvement over the quicksort approach (making the fairly safe assumption that the number of servers is less than the number of accounts). Note that there’s a minor but important implementation detail regarding where you get the element that you add to the heap: it needs to come from the same list as the element you removed from the heap so the heap always has the next element from each list. To accomplish this, you’ll need to track which list each element in the heap comes from.

What are the limitations of this strategy? The running time is improved, but it also requires _O_(_n_) auxiliary temporary space (in addition to the space required for storing the records in memory) while performing the merge. If that space is available, then this is an excellent solution.

How would you respond if the interviewer told you that memory on the server is tight and it’s not acceptable to use _O_(_n_) auxiliary space during the sort? In-place sorting algorithms have minimal requirements for auxiliary storage. If you assume you can get the sublists concatenated without using _O_(_n_) auxiliary storage (for example, you might receive them into one large buffer to begin with), then one option is to revert to the original method and use an in-place sorting algorithm such as in-place quicksort; you’ll sacrifice some performance, but _O_(_n_ log(_n_)) is not that much worse than _O_(_n_ log(_k_)).

Before you settle on this solution, consider why the merge approach requires additional space. You have each of the sublists in memory, requiring _n_ records of storage. Then you need to allocate a temporary buffer of size _n_ to store the merged result. There doesn’t seem to be any way around the output buffer requirement, but do you actually need to have each of the sublists in memory? The sublists are already sorted, so at each point in the merge you just need the next item from each sublist. Obviously you still need storage for all _n_ account records, but if you merge the sublists as you receive them, you no longer have a requirement for an additional size _n_ buffer. (You probably need a small constant-size buffer for each of the servers sending information, so if there are _k_ departmental servers, additional memory required is _O_(_k_)_._)) This is an example of an _online algorithm_: an algorithm that processes data as it becomes available, rather than requiring all data to be available before starting processing.

The online approach has limitations, too. It requires the merge to be integrated with the communications with the departmental servers to avoid overflowing the buffers, increasing complexity, and decreasing modularity. Also, if one of the departmental servers has problems during the process and stops sending data, it stalls the entire operation. Everything has trade-offs, but in an appropriately controlled environment, this could be the best option.

Each night, only the newly added serial numbers can be out of order because the rest were sorted the previous night. Even the newly added serial numbers are likely partially sorted because serial numbers are usually assigned in order, and the items are likely tested roughly in order. After the plant has been running for more than a few weeks, the number of items added to the list each day will probably be much smaller than the total size of the list.

To summarize, you have a few unsorted items to add to a large sorted list. This sounds like a job for insertion sort! The situation described is close to that for which insertion sort has its best-case _O_(_n_) performance. But stop to consider the other properties of insertion sort to see if there are any problems with this choice. Insertion sort is stable and in-place, so no problems there. Worst and average case performance are _O_(_n_<sup>2</sup>)—that could be a problem. In this scenario the number of unsorted items is usually small, in which case you can expect nearly _O_(_n_) performance, but if the factory has a bad day and a large number of items fail, you may see closer to _O_(_n_<sup>2</sup>). Ask the interviewer if an occasional sort that runs long can be tolerated in this environment: if so, then insertion sort is your answer; if not, you need to keep looking.

Suppose that worst-case _O_(_n_<sup>2</sup>) is not acceptable. What other options do you have? Instead of looking at your data as a sorted list and some unsorted items to insert, try thinking of it as two lists: a large sorted list and a (usually) small, possibly partially sorted list. Sorted lists can be efficiently merged, so you just need to sort the small (new serial numbers) list and then merge the two of them. Because you’ll do at least some merging, you might choose to sort the small list with a merge sort. What’s the worst-case efficiency of this approach? If the length of the old, sorted list is _l_ and the new, unsorted list is _m_, then the sort of the new list is _O_(_m_ log(_m_)) and the merge is _O_(_l_ + _m_). Combined, this is _O_(_l + m_ log(_m_)). This approach does have the drawback that _O_(_l_ + _m_) additional memory is needed for the merge. There’s no free lunch.

If you immediately jumped to something like quicksort for the first problem in this series, the current problem is probably what you had in mind. This general case of sorting in which you don’t know much about what you’re sorting is common, so you must be able to solve it efficiently. Just make sure that your problem is actually a general-purpose sorting problem and you’re not missing an opportunity to select a more appropriate special-purpose sorting algorithm.

Optimizing sorting performance across a wide range of potential inputs is the problem faced by programmers who write frameworks and standard libraries, so typically these sort routines are appropriate choices, such as `Arrays.sort()` in Java. These routines typically employ merge sort (if stability is important) or quicksort (if it isn’t) for most data sets, often switching to insertion sort for very small data sets (typically _n_ less than approximately 10).

For all these problems involving selecting a sorting algorithm, the interviewer’s objective is not actually for you to arrive at any particular solution. Instead, the interviewer wants to see that you recognize that there’s no single sorting algorithm that’s optimal in all situations, that you have some knowledge of what sorting algorithms are available, and that you can apply this knowledge to select appropriate algorithms and intelligently discuss the running time and memory trade-offs between different options.

### Stable Selection Sort

This problem requires that you know what a selection sort is. If you don’t remember, ask the interviewer. Briefly, a selection sort works by repeatedly scanning the not-yet-sorted values to find the lowest key, and then swapping the lowest key into sorted position at the end of the already-sorted values, as described in more detail earlier in this chapter. A typical implementation is:

```
  // Sort an array using an iterative selection sort.
```

You’re asked to make this sort stable. Recall the definition of a stable sort: it is a sort that preserves the input ordering of elements with equal keys. If _a_<sub>1</sub> and _a_<sub>2</sub> are two elements with equal keys, and _a_<sub>1</sub> comes before _a_<sub>2</sub> in the original data set, _a_<sub>1</sub> will always be ahead of _a_<sub>2</sub> after a stable sort.

You may remember that the standard implementation of a selection sort is not stable; even if you don’t, the wording of the problem strongly implies it. It’s easier to create a stable version of the sort if you understand exactly why the preceding implementation is unstable. Try working through a simple example that produces an unstable result: \[5<sub>1</sub>, 3, 5<sub>2</sub>, 2\]. After the first iteration of the sort, this becomes \[2, 3, 5<sub>2</sub>, 5<sub>1</sub>\]—already the original ordering of the two equal keys has been lost. It seems that the sort is unstable because of the swapping of keys: when an unsorted key is swapped into the location that the key being sorted came from, information about the position of that unsorted key relative to the other unsorted keys is lost. The net effect of the swapping is that the unsorted keys are shuffled as the sort progresses. If you can eliminate the swapping, you might make the sort stable.

The standard unstable selection sort swaps keys because it’s the easiest, most efficient way to create space for the key being sorted. How might you create space for this key without swapping? If you _insert_ the key being sorted, then the ordering of the unsorted keys remains unchanged. You’ll also need to delete this key from its original location. Remember that you can’t arbitrarily insert or delete elements from an array—you must move the adjacent elements to open or close the space. In this case, you can accomplish the deletion and insertion as part of the same process by moving all the keys between the original location of the key being sorted and its destination one element to the right.

For simplicity, you can continue to implement the algorithm to sort an array of `int`, understanding (and telling your interviewer) that if you were actually just sorting `int`s, you couldn’t distinguish between the results of a stable and an unstable sort. Stable and unstable sorts produce different results only when the key is part of a larger record or object, so objects with the same key value are not necessarily identical. An implementation of stable selection sort for an array of `int` might look like:

```
// Sort an array using a stable selection sort.
```

This stable version of selection sort replaces a fast _O_(1) swap operation with a much slower _O_(_n_) array insertion/deletion operation implemented by the `System.arraycopy` call. You were already performing an _O_(_n_) operation (`findMinimumIndex`) for each key, so adding another _O_(_n_) operation doesn’t change the overall runtime complexity—it’s still _O_(_n_<sup>2</sup>)—but because you’ve replaced a fast operation with a much slower one, the actual performance will be worse.

Is there any situation in which it makes sense to use this kind of implementation of stable selection sort? Other stable sort algorithms are more efficient than _O_(_n_<sup>2</sup>). One advantage that the original unstable selection sort has over many other sort algorithms is that the total number of moves (swaps) is _O_(_n_). In the preceding stable implementation, the array insertion/deletion makes _O_(_n_) moves, and this happens once for each of the _n_ keys to be sorted: the total number of moves for this stable selection sort is _O_(_n_<sup>2</sup>). This implementation gains stability at the price of sacrificing the only significant benefit of selection sort, so it’s difficult to imagine a scenario in which it would be useful. How might you maintain _O_(_n_) total key moves?

The current implementation executes _O_(_n_<sup>2</sup>) moves because it uses an array, where insertion and deletion are inefficient operations requiring moving _O_(_n_) elements. If you used a different data structure where insertion and deletion affect only _O_(1) elements, then you would regain _O_(_n_) total moves. A linked list meets these requirements. The following is an implementation of a stable selection sort using a linked list with _O_(_n_) total moves. This implementation also operates on any object implementing `Comparable` rather than being limited to `int`:

```
    public static void selectionSortStable( CursorableLinkedList data ){
```

This implementation uses the Apache Commons Collections `CursorableLinkedList` class rather than `LinkedList` from the Java Collections Framework because `CursorableLinkedList` can maintain the validity of an iterator (cursor) even as the list is modified through other iterators. This capability enables a more efficient implementation of the sort. The implementation could be further optimized if you implemented a custom linked list class that supported copying iterators and moving (rather than just deleting and inserting) elements.

### Multi-Key Sort

To sort the data using a routine from the standard library, you need a _comparator_: a function that compares two objects. A comparator returns a negative value if the first object is “less than” the second object; zero if the two objects have equal keys; or a positive value if the first object is “greater than” the second.

For this problem, the key has two components—the surname and the given name—so the comparator needs to use both of these values. You must order first by surname and then by given name, so the comparator should start by comparing the surnames and then resolve ties by comparing the given names.

In Java, comparators implement the `java.util.Comparator` interface:

```
import java.util.Comparator;
```

Now it’s just a matter of invoking the `Arrays.sort` method with the array and the comparator:

```
public static void sortEmployees( Employee[] employees ){
```

The approach shown here of using a comparator that considers both parts of the key in a single sort is the most efficient approach, but there is another alternative. If the sort routine you use is stable (the modified merge sort used by `Arrays.sort` is), you can achieve the same result by calling the sort routine twice and sorting on one part of the key at a time. For this problem, you would first sort by given name and then make a second call to sort by surname. During the second sort, by the definition of a stable sort, employees with the same surname would retain their relative ordering based on given name, established by the first sort.

### Make a Sort Stable

Stability is all about preserving the relative order of elements with equal keys. When the data set being sorted has keys that are equal, an unstable sort is not guaranteed to yield the same result as a stable sort. But what if there are _no_ equal keys? Stability is meaningless in this case, and all sorting algorithms produce the same result. If you can transform the input data to ensure that there are no equal keys in the data set, then it won’t matter that `shakySort()` isn’t stable.

One approach you might consider is to scan through the data, identify keys with equal values, and then modify the values based on their positions in the input data set so that keys with earlier positions have lower values. Then when you do an unstable sort, the formerly equal keys retain their original relative ordering. Think about how this might be implemented. If the keys have discrete values, then you might have a situation in which there aren’t enough intermediate values available to easily modify the keys. For instance, if you had the integer keys \[5, 4, 6, 5\] you must modify 4 or 6 in addition to at least one of the 5s. Furthermore, the keys likely represent data that may be needed for other purposes. This seems like an overly complicated and undesirable approach.

Because modifying the keys seems undesirable, you need another way to represent information about their original order. What if you added another value and used that as part of the key? You could have a field that represented the relative ordering of each otherwise identical key and compare these values when the main part of the key has the same value. After processing this way, the previous example becomes \[5<sub>1</sub>, 4, 6, 5<sub>2</sub>\], where subscripts represent the new field. This is a definite improvement, but it’s still somewhat complex: you need to scan the data, using some additional data structure to track what the next number in sequence is for each main key value.

Try to simplify this further. Is it necessary for each repeated key to be consecutively numbered (that is, 1, 2, 3…)? No, you just need earlier occurrences of the key to have lower sequence numbers than later ones. Based on this observation, you can just assign the value for the sequence field based on the element’s starting position: \[5<sub>1</sub>, 4<sub>2</sub>, 6<sub>3</sub>, 5<sub>4</sub>\]. For repeated keys, this meets the requirement of establishing the relative ordering; for nonrepeated keys you can ignore the sequence number.

With the sequence number as a secondary part of the key, each key is now unique, and the result of an unstable sort using the new expanded key is the same as that of a stable sort on the original key.

Implementation is simpler if you have something concrete to sort: add a `sequence` field to the `Employee` class in the previous problem and sort objects of that class.

You must reinitialize the sequence fields before each sort:

```
  public static void sortEmployeesStable( Employee[] employees ){
```

You also must create a comparator that uses the sequence number as a _tie breaker_ for otherwise identical keys. For instance, to perform a stable sort by surname:

```
// A comparator for Employee instances.
```

What’s the complexity of making `shakySort()` stable? Assigning the sequence numbers takes _O_(_n_) time, but because no comparison sort can be more efficient than _O_(_n_ log(_n_)), the asymptotic running time is not increased ( _O_(_n_ + _n_ log(_n_)) = _O_(_n_ log(_n_)) ). There’s one sequence number for each element, so this approach requires _O_(_n_) additional memory.

### Optimized Quicksort

Before you can start on any implementation, you must understand the quicksort algorithm. Briefly, quicksort begins by selecting a _pivot value_ from the elements to be sorted. The remaining elements are then divided into two new lists: one list _L_ containing all the values less than the pivot and another list _G_ containing all the values greater than or equal to the pivot. Then quicksort is recursively called to sort _L_ and _G_. After these calls return, _L_, the pivot, and _G_ are concatenated (in that order) to yield the sorted data set. If you didn’t remember at least that much about quicksort, you’d probably have to ask the interviewer to help you get started.

The simplest implementations of quicksort (such as the one earlier in this chapter) allocate new lists (or arrays) for _L_ and _G_ and copy results back from them after the recursive calls return, which is inefficient and requires additional memory. For this problem, you’re asked to write an implementation that avoids this.

The memory allocations that you need to eliminate happen during the partitioning step: when the values are rearranged into _L_ and _G_. Considering the partitioning, there’s no change in the number of elements, just their position, so it should be possible to store _L_, the pivot, and _G_ all in the original array. How might you do this?

You need to move elements to one end of the array or the other depending on the list to which they belong. Assume that _L_ is on the left side of the array and _G_ is on the right side of the array. Initially you don’t know what the sizes of _L_ and _G_ are, just that the sum of their sizes is equal to the array. You know the pivot value, so you can determine whether an individual element belongs to _L_ or _G_. If you scan through the elements left to right, each time you find a value greater than or equal to the pivot, you need to move it to the right, into _G_. Because, again, you don’t know what the final size of _G_ will be, it makes sense to have _G_ start at the end of the array and grow toward the left. You don’t have any extra space available, so when you move an element to the right into _G_, you also must move an element to the left to open space. The easiest way to do this is to swap the positions of the element going into _G_ with the element at its destination.

After you swap, the element moving to the left as part of the swap hasn’t been checked yet, so be sure to check it before advancing. In addition to tracking your position as you scan through the array, you also need to track the location of the leftmost element of _G_ as it grows to the left, so you know where to put elements when you swap them into _G_. When your scan position reaches the leftmost element of _G_, all the elements greater than or equal to the pivot have been moved into _G_, so the remaining elements in the left portion of the array constitute _L_. The array is now partitioned into _L_ and _G_ without using any additional memory. This algorithm can then be recursively applied to both lists.

In summary, this algorithm is:

```
Select a pivot
```

As with any complex procedure that you design, you should test this with a few potentially problematic cases before you code it. Some cases to check include a two-element array and an array with several identical values. When you work through the latter case, you can identify a bug: if all the values in an array are equal, the algorithm never terminates because all the elements are greater than or equal to the pivot, so they all end up in _G_ on each recursive call!

How can you fix this bug? It occurs because _G_ is exactly the same on each successive recursive call. With the current algorithm, _G_ contains all the elements including the pivot (because the pivot is equal to the pivot value). What if you separate the pivot from the rest of _G_? Then _G_ can never equal the initial array because it’s always at least one element smaller. You need somewhere to store the pivot while you do the partition. One convenient location to keep it out of the way is the end of the array. When you start the procedure, swap the pivot element to the end of the array and then partition the remainder of the array. After partitioning, swap the first element of _G_ with the pivot you had previously stored at the end of the array. Now the pivot is in its correct location with all the smaller elements (in _L_) on its left; _G_ is everything to the right of the pivot. When you make recursive calls on _L_ and _G_, the pivot is now excluded, so _G_ decreases in size by at least one on each cycle.

An implementation of this algorithm is as follows:

```
public static void quicksortSwapping( int[] data ){
```

The version of quicksort you just developed keeps track of two indexes, one on the left and one on the right. The partitions are determined by where the indexes meet. But you’re only comparing values on the left side of the array. Can you compare values on the right as well? Instead of blindly swapping values between left and right, wouldn’t it make sense to swap mismatched pairs of values? In other words, on the left you would swap a value greater than or equal to the pivot for one on the right that is less than or equal to the pivot. This could considerably reduce the total number of swaps.

While you’re at it, you can also make the math a bit simpler by using indexes to mark partition boundaries instead of a starting index and a length. The result is this optimized version of quicksort:

```
public static void quicksortOptimized( int[] data ){
```

Note that this implementation doesn’t need to explicitly move the pivot as the previous implementation did. Because it compares values at both ends, and values equal to the pivot are swapped into the partition at the other end, there is no case in which all the values end up in one partition. This means that values equal to the pivot may end up in either partition, but the sort is still correct.

This is about as good as quicksort can get! The only other optimization that might be worth considering is to replace the recursive call to quicksort with another sorting algorithm like insertion sort after the partition size falls below a certain threshold.

### Pancake Sorting

At first this seems like a simple sorting problem: you have a set of items to sort and you’d like to optimize the worst-case running time of the sort. A merge sort has worst-case _O_(_n_ log(_n_)); this seems like an easy solution.

Any time there’s a solution that seems this simple, it probably isn’t correct. Compare the situation in this problem to the usual problem of sorting. In most sorting problems, you can arbitrarily rearrange or exchange the items to be sorted; here, you’re limited to using flips of a substack.

There’s one other important difference: in analysis of the running time of sort algorithms, you must include the time required to examine each item. In this problem you must optimize the number of flips—in a sense you get to examine the pancakes to determine their locations and plan your strategy for free. After you recognize these differences, it becomes clear that this problem involves more than applying a standard sorting algorithm.

It’s hard to calculate the worst-case number of flips that a sorting algorithm requires without knowing what the algorithm is, so start by trying to devise an algorithm for sorting pancakes. You’re allowed to use only one operation for changing the order of pancakes: the flip. Think about what happens every time you perform a flip. The order of the pancakes above the point you inserted your flipper is reversed, but the order of the pancakes below the flipper remains unchanged. It seems like it may be difficult to maintain pancakes in sorted order near the top of the stack because they keep getting flipped over, so try sorting the stack starting at the bottom.

The largest pancake should end up on the bottom. How can you get it there? Consider three cases for where the largest pancake could start out: on the bottom, somewhere in the middle, or on the top. If the largest pancake starts out on the bottom, then you don’t need to move it. If it’s in the middle, things seem a little complicated—certainly there’s no way to get it to the bottom with a single flip. If you don’t see how to deal with this case right away, put it aside, and come back to it later. What if the largest pancake starts out on the top? Then you could flip the entire stack, moving the pancake from the top to where you want it on the bottom. This also gives you a method for solving the middle case: you just need to first move the largest pancake to the top and then flip it to the bottom. It’s quite simple to move a pancake from somewhere in the middle to the top: insert the flipper immediately underneath the pancake and do a flip. Combining all this, you see that in the worst case it takes two flips to move the largest pancake to the bottom of the stack.

Because the pancakes at the bottom of the stack are unaffected by flips above them, you can continue sorting from the bottom up using the same procedure. On each cycle, identify the next largest not-yet-sorted pancake, flip it to the top, and then flip the stack above the largest already-sorted pancake to move the current pancake from the top into its sorted position. This would be a worst case of 2_n_ flips.

Can you do better than this? You’ve already worked through sorting the first few pancakes; now think about what happens when you sort the last pancakes. After you’ve sorted the next-to-smallest pancake, all the other pancakes larger than it are in sorted order beneath it. There’s only one position left that the smallest pancake can be in: its sorted location at the top of the stack. If you apply the sorting procedure to the smallest pancake at this point, you just flip it over twice. This wastes two flips without changing anything, so you can skip these flips. The worst case is no more than 2_n_ – 2 flips.

There seems to be room for optimization at the end of the sort, so try backing up one more step to see if you can do any better (assuming that _n_ > 1). After you’ve sorted all but the last two pancakes, you’ve (worst case) performed 2_n_ – 4 flips. The final two pancakes can be arranged only two ways at this point. Either they’re already in sorted order and you’re done, or the larger one is above the smaller. In the latter case, you just have to flip the two pancakes. This gives a worst-case total of 2_n_ – 4 + 1 = 2_n_ – 3 flips.

Yet more optimal solutions can be derived, but this is probably as far as anyone would expect you to go in an interview. This problem has an interesting history. Although commonly known as _the pancake problem_, it’s more formally classified as _sorting by prefix reversal_ and has applications in routing algorithms. Before he disappointed his family and friends by dropping out of Harvard, Bill Gates published a journal article on the problem (Gates, WH and Papadimitriou, CH, “Bounds for Sorting by Prefix Reversal,” _Discrete Mathematics_: 27(1) 47–57, 1979). Gates’ algorithm, which is substantially more complex than what we’ve discussed, stood as the most efficient known solution to the problem for almost 30 years.

## SUMMARY

Sorting algorithms are selected using criteria such as memory use and stability as well as best, average, and worst-case performance. No comparison sort can have better worst-case performance than _O_(_n_ log(_n_)).

Selection sort is one of the simplest sorting algorithms, but it is _O_(_n_<sup>2</sup>) in all cases. It requires only _O_(_n_) swaps, however, so it may be suitable for data sets where copying is very expensive. Insertion sort is efficient when dealing with mostly sorted data sets, where it can have _O_(_n_) performance, but average and worst cases are _O_(_n_<sup>2</sup>). Quicksort is a divide-and-conquer algorithm that offers _O_(_n_ log(_n_)) performance in the best and average cases and _O_(_n_<sup>2</sup>) in the worst case. Merge sort is another divide-and-conquer algorithm that offers _O_(_n_ log(_n_)) performance in all cases. It is especially useful for sorting data sets that cannot fit into memory. You can make any sorting algorithm stable by assigning a sequence number to each element and using the sequence number as the tie-breaker in a multi-key sort.
