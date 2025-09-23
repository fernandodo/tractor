---
created: 2025-09-23T20:39:28 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 7 Arrays and Strings | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> 7Arrays and Strings

 Arrays and strings are closely related. In the abstract sense, a string is just a (possibly read-only) array of characters. Most of the string-manipulation problems you...

---
## 7  
Arrays and Strings

Arrays and strings are closely related. In the abstract sense, a string is just a (possibly read-only) array of characters. Most of the string-manipulation problems you encounter are based on your understanding of array data types, particularly in C where strings and character arrays are essentially identical. Other languages consider strings and character arrays as distinct data types, but there’s always a way to convert a string to an array and vice versa. When the two are different, it’s important to understand how and why they diverge. In addition, not all array problems involve strings, so understanding how arrays work in the abstract and how they’re implemented by the language you use is crucial to answering array-focused problems.

An _array_ is a sequence of variables of the same type arranged contiguously in a block of memory. Because arrays play an important role in every major language used in commercial development, we assume you’re at least somewhat familiar with their syntax and usage. With that in mind, this discussion focuses on the theory and application of arrays.

Like a linked list, an array provides an essentially linear form of storage, but its properties are significantly different. (Multidimensional arrays are not exactly linear, but they are implemented as linear arrays of linear arrays.) In a linked list, lookup is always an _O_(_n_) operation, but array lookup is _O_(1) as long as you know the index of the element you want. The provision regarding the index is important—if you know only the value, lookup is still _O_(_n_) in the average case. For example, suppose you have an array of characters. Locating the sixth character is _O_(1), but locating the character with value `'w'` is _O_(_n_).

The price for this improved lookup is significantly decreased efficiency for insertion and deletion of data in the middle of the array. Because an array is essentially a block of contiguous memory, it’s not possible to create or eliminate storage between any two elements as it is with a linked list. Instead, you must physically _move_ data within the array to make room for an insertion or to close the gap left by a deletion; this is an _O_(_n_) operation.

Arrays are not dynamic data structures: they have a finite, fixed number of elements. Memory must be allocated for every element in an array, even if only part of the array is used. Arrays are best used when you know how many elements you need to store before the program executes. When the program needs a variable amount of storage, the size of the array imposes an arbitrary limit on the amount of data that can be stored. Making the array large enough so that the program always operates below the limit doesn’t solve the problem: either you waste memory or you won’t have enough memory to handle the largest data sizes possible.

Most modern languages also have library support for _dynamic arrays_: arrays that can change size to store as much or as little data as necessary. (Some languages, typically scripting languages, use dynamic arrays as their fundamental array type and have no static array type.) This discussion won’t go into the details of implementing a dynamic array, but you should know that most dynamic array implementations use static arrays internally. A _static array_ cannot be resized, so dynamic arrays are resized by allocating a new array of the appropriate size, copying every element from the old array into the new array, and freeing the old array. This is an expensive operation that should be done as infrequently as possible.

Each language handles arrays somewhat differently, giving each language a different set of array programming pitfalls.

### C and C++

Despite the differences between C and C++, they are similar in their treatment of arrays. In most cases, an array name is equivalent to a pointer constant to the first element of the array. This means that you can’t initialize the elements of one array with another array using a simple assignment.

For example,

```
arrayA = arrayB;  /* Compile error: arrayA is not an lvalue */
```

is interpreted as an attempt to make `arrayA` refer to the same area of memory as `arrayB`. If `arrayA` has been declared as an array, this causes a compile error because you can’t change the memory location to which `arrayA` refers. To copy `arrayB` into `arrayA`, you must write a loop that does an element-by-element assignment or use a library function such as `memcpy` that does the copying for you (usually much more efficiently).

In C and C++, the compiler tracks only the location of arrays, not their size. The programmer is responsible for tracking array sizes, and there is no bounds checking on array accesses—the language won’t complain if you store something in the 20th element of a 10-element array. As you can imagine, writing outside the bounds of an array usually overwrites some other data structure, leading to all manner of curious and difficult-to-find bugs. Development tools are available to help programmers identify out-of-bounds array accesses and other memory-related problems in their C and C++ programs.

### Java

Unlike a C array, a Java array is an object in and of itself, separate from the data type it holds. A reference to an array is therefore not interchangeable with a reference to an element of the array. Java arrays are static, and the language tracks the size of each array, which you can access via the implicit `length` data member. As in C, you cannot copy arrays with a simple assignment. If two array references have the same type, assignment of one to the other is allowed, but it results in both symbols referring to the same array, as shown in the following example:

```
byte[] arrayA = new byte[10];
```

If you want to copy the contents of one array to another, you must do it element by element in a loop or call a system function:

```
if ( arrayB.length <= arrayA.length ){
```

Each access to an array index is checked against the current size of the array, and an exception is thrown if the index is out of bounds. This can make array access a relatively expensive operation when compared to C or C++ arrays; although, in cases in which the JVM can prove that the bounds check is unnecessary, it is skipped to improve performance.

When arrays are allocated, the elements are initialized to their default values. Because the default value for object types is `null`, no objects are constructed when you create an array of objects. You must construct the objects and assign them to the elements of the array:

```
Button myButtons[] = new Button[3]; // Buttons not yet constructed
```

Alternatively, you can use array initialization syntax (which is allowed only where the array is declared):

```
Button myButtons[] = { new Button(), new Button(), new Button() };
```

Two dimensional arrays are implemented in Java by creating an array of array objects. Since each of the nested arrays is a separate object, they can have different lengths. Arrays can be nested more deeply to create multidimensional arrays.

### C#

C# supports Java-style arrays of array objects accessed using syntax of the form `foo[2][3]`. C# also supports single-object multidimensional arrays using a different syntax: `foo[2,3]`. The Java-style arrays are referred to as _jagged arrays_ because each inner array can have a different length (so a diagram of the data structure would have a jagged edge). In contrast, the single-object multidimensional arrays must be rectangular (each inner array has the same length); these types of arrays are referred to as _multidimensional arrays_. C# arrays can be declared to be read-only. All arrays derive from the `System.Array` abstract base class, which defines methods for array manipulation.

### JavaScript

Arrays in JavaScript are instances of the `Array` object. JavaScript arrays are dynamic and resize themselves automatically:

```
Array cities = new Array(); // zero length array
```

You can change the size of an array simply by modifying its `length` property:

```
cities.length = 1; // drop Los Angeles...
```

You can use methods on the `Array` object to split, combine, and sort arrays.

Array values in JavaScript are usually but not always stored in a single contiguous memory block. They have the expected array performance characteristics only when they are stored contiguously.

## STRINGS

_Strings_ are sequences of characters. However, what constitutes a _character_ depends on the language used and the settings of the operating system on which the application runs. Gone are the days when you could assume each character in a string is represented by a single byte. Multibyte encodings (either fixed-length or variable-length) of Unicode are needed to accurately store text in today’s global economy.

More recently designed languages, such as Java and C#, have a multibyte fundamental character type, whereas a `char` in C and C++ is always a single byte. (Recent versions of C and C++ also define a character type `wchar_t`, which is usually multibyte.) Even with built-in multibyte character types, properly handling all cases of Unicode can be tricky: more than 100,000 _code points_ (representation-independent character definitions) are defined in Unicode, so they can’t all be represented with a single, 2-byte Java or C# `char`. This problem is typically solved using variable-length encodings, which use sequences of more than one fundamental character type to represent some code points.

One such encoding is UTF-16, used to encode strings in Java and C#. UTF-16 represents most of the commonly used Unicode code points in a single 16-bit `char` and uses two 16-bit `char`s to represent the remainder.

UTF-8, another common encoding, is frequently used for text stored in files or transmitted across networks. UTF-8 uses one to four bytes to encode all Unicode code points. Each code point is encoded using one of these four bit patterns:

```
0xxxxxxx
```

The high bits of the _leading byte_ (the first byte) indicate how many bytes are used to represent the character. One advantage of UTF-8 is that all ASCII characters (which are values in the range 0 to 127) are represented as single bytes, which means that ASCII encoded text is a subset of UTF-8 encoded text.

Variable-length encodings make string manipulation considerably more complicated. There may be fewer characters in a string than the number of `char`s required to store it, and you must take care to avoid interpreting a part of a multi-`char` encoded code point as a complete character. For simplicity, most programming problems involving strings focus on string manipulation algorithms using the language’s natural character type and neglect issues of variable-length encoding.

If you have specific expertise in internationalization and localization, string problems give you a great opportunity to highlight this valuable experience. Although your interviewer may tell you to assume that your input string has a fixed-length character encoding such as ASCII, you can explain what you would do differently to handle a variable-length character encoding, even as you code the requested fixed-length encoded solution.

No matter how they’re encoded, most languages store strings internally as arrays, even if they differ greatly in how they treat arrays and strings. Many string problems involve operations that require accessing the string as an array. In languages where strings and arrays are distinct types, it may be helpful to convert the string to an array and then back to a string after processing.

### C

A C string is contained in a `char` array. Because C doesn’t track the size of arrays, it can’t track the size of strings either. Instead, the end of the string is marked with a null character, represented in the language as `'\0'`. The null character is sometimes referred to as `NUL`. (Don’t confuse `NUL`, which is a `char` type with value `0`, to `NULL`, which is a pointer to memory address `0`.) The character array must have room for the terminator: a 10-character string requires an 11-character array. This scheme makes finding the length of the string an _O_(_n_) operation instead of _O_(1) as you might expect: `strlen()` (the library function that returns the length of a string) must scan through the string until it finds the end.

For the same reason that you can’t assign one C array to another, you cannot copy C strings using the `=` operator. Instead, you generally use the `strlcpy()` function. (Use of the older `strcpy()` is deprecated in most cases because it’s a common source of buffer overrun security holes.)

It is often convenient to read or alter a string by addressing individual characters of the array. If you change the length of a string in this manner, make sure you write a null character after the new last character in the string, and that the character array you work in is large enough to accommodate the new string and terminator. It’s easy to truncate a C string (although the array that contains the string remains the same size): just place a null character immediately after the new end of the string.

Modern C compilers also define a wide character type (`wchar_t`) and extend the standard library functions to operate on strings represented as `wchar_t` arrays. (C doesn’t support overloading, so these functions have similar names to their `char` counterparts, replacing `str` with `wcs`.) One caveat to using `wchar_t` is that its size is implementation-dependent and in unusual cases may even be the same as `char`. This makes C code that uses `wchar_t` even less portable than usual.

### C++

C-style strings can be used with C++, but the preferred approach is to use the `string` or `wstring` class (when you need multibyte characters) from the C++ Standard Template Library (STL) whenever possible. Both of these classes are specializations of the same `basic_string` template class using the `char` and `wchar_t` data types, respectively.

The string classes are well integrated with the STL. You can use them with streams and iterators. In addition, C++ strings are not null-terminated, so they can store null bytes, unlike C strings. Multiple copies of the same string share the same underlying buffer whenever possible, but because a string is mutable (the string can be changed), new buffers are created as necessary. For compatibility with older code, it is possible to derive a C-style string from a C++ string, and vice versa.

The `string_view` class, introduced to the STL in C++17, defines a _view_ on all or part of an existing string. Views don’t allocate additional memory and as such are very inexpensive to create and pass between functions. They are trivial to construct from strings and new strings can be created from views as necessary. As long as the underlying memory doesn’t change or move, consider using views to optimize your string operations.

### Java

Java strings are objects of the `String` class, a special system class. Although strings can be readily converted to and from character and byte arrays—internally, the class holds the string using a `char` array—they are a distinct type. Java’s `char` type has a size of two bytes. The individual characters of a string cannot be accessed directly, but only through methods on the `String` class. `String` literals in program source code are automatically converted into `String` instances by the Java compiler. As in C++, the underlying array is shared between instances whenever possible. The length of a string can be retrieved via the `length()` method. Various methods are available to search and return substrings, extract individual characters, trim whitespace characters, and so on.

Java strings are immutable; they cannot be changed after the string has been constructed. Methods that appear to modify a string actually return a new string instance. The `StringBuffer` and `StringBuilder` classes (the former is in all versions of Java and is thread-safe; the latter is newer and higher performance, but not thread-safe) create mutable strings that can be converted to a `String` instance as necessary. The compiler implicitly uses `StringBuilder` instances when two `String` instances are concatenated using the `+` operator, which is convenient but can lead to inefficient code if you’re not careful. For example, the code

```
String s = "";
```

is equivalent to

```
String s = "";
```

which would be more efficiently coded as

```
StringBuilder b = new StringBuilder();
```

Watch for this case whenever you manipulate strings within a loop.

### C#

C# strings are almost identical to Java strings. They are instances of the `String` class (the alternative form `string` is an alias), which is similar to Java’s `String` class. C# strings are also immutable just like Java strings. You create mutable strings with the `StringBuilder` class, and similar caveats apply when strings are concatenated.

### JavaScript

Although JavaScript defines a `String` object, many developers are unaware of its existence due to JavaScript’s implicit typing. However, the usual string operations are there, as well as more advanced capabilities, such as using regular expressions for string matching and replacement.

## ARRAY AND STRING PROBLEMS

Many array and string problems require the use of additional temporary data structures to achieve the most efficient solution. In some cases, in languages where strings are objects, it may be more efficient to convert the string to an array than to process it directly as a string.

### Find the First Nonrepeated Character

At first, this task seems almost trivial. If a character is repeated, it must appear in at least two places in the string. Therefore, you can determine whether a particular character is repeated by comparing it with all other characters in the string. It’s a simple matter to perform this search for each character in the string, starting with the first. When you find a character that has no match elsewhere in the string, you’ve found the first nonrepeated character.

What’s the time order of this solution? If the string is _n_ characters long, then in the worst case, you’ll make almost _n_ comparisons for each of the _n_ characters. That gives worst case _O_(_n_<sup>2</sup>) for this algorithm. (You can improve this algorithm somewhat by comparing each character with only the characters following it, because it has already been compared with the characters preceding it. This is still _O_(_n_<sup>2</sup>).) You are unlikely to encounter the worst case for single-word strings, but for longer strings, such as a paragraph of text, it’s likely that most characters will repeat, and the most common case might be close to the worst case. The ease with which you arrived at this solution suggests that there are better alternatives—if the answer were truly this trivial, the interviewer wouldn’t bother you with the problem. There must be an algorithm with a worst case better than _O_(_n_<sup>2</sup>).

Why was the previous algorithm _O_(_n_<sup>2</sup>)? One factor of _n_ came from checking each character in the string to determine whether it was nonrepeated. Because the nonrepeated character could be anywhere in the string, it seems unlikely that you can improve efficiency here. The other factor of _n_ was due to searching the entire string when trying to look up matches for each character. If you improve the efficiency of this search, you improve the efficiency of the overall algorithm. The easiest way to improve search efficiency on a set of data is to put it in a data structure that allows more efficient searching. What data structures can be searched more efficiently than _O_(_n_)? Binary trees can be searched in _O_(log(_n_)). Arrays and hash tables both have constant time element lookup. (Hash tables have worst-case lookup of _O_(_n_) but the average case is _O_(1).) Begin by trying to take advantage of an array or hash table because these data structures offer the greatest potential for improvement.

You want to quickly determine whether a character is repeated, so you need to be able to search the data structure by character. This means you must use the character as the index (in an array) or key (in a hash table). (You can convert a character to an integer to use it as an index.) What values would you store in these data structures? A nonrepeated character appears only once in the string, so if you store the number of times each character appears, it would help you identify nonrepeating characters. You must scan the entire string before you have the final counts for each character.

When you complete this, you could scan through all the count values in the array or hash table looking for a 1. That would find a nonrepeated character, but it wouldn’t necessarily be the first one in the original string.

Therefore, you need to search your count values in the order of the characters in the original string. This isn’t difficult—you just look up the count value for each character until you find a 1. When you find a 1, you’ve located the first nonrepeated character.

Consider whether this new algorithm is actually an improvement. You always have to go through the entire string to build the count data structure. In the worst case, you might have to look up the count value for each character in the string to find the first nonrepeated character. Because the operations on the array or hash you use to hold the counts are constant time, the worst case would be two operations for each character in the string, giving 2_n_, which is _O_(_n_)—a major improvement over the previous attempt.

Both hash tables and arrays provide constant-time lookup; you need to decide which one to use. On the one hand, hash tables have a higher lookup overhead than arrays. On the other hand, an array would initially contain random values that you would have to take time to set to zero, whereas a hash table initially has no values. Perhaps the greatest difference is in memory requirements. An array would need an element for every possible value of a character. This would amount to a relatively reasonable 128 elements if you process ASCII strings, but if you have to process strings that could potentially contain any Unicode character, you would need more than 100,000 elements. In contrast, a hash table would require storage for only the characters that actually exist in the input string. Therefore, arrays are a better choice for long strings with a limited set of possible character values; hash tables are more efficient for shorter strings or when many possible character values exist.

You could implement the solution either way. We’ll assume the code may need to process Unicode strings (a safe bet these days) and choose the hash table implementation. In outline form, the function you write looks like this:

```
First, build the character count hash table:
```

Now implement the function. You might choose to write the function in Java or C#, both of which have built-in support for both hash tables and Unicode. Because you don’t know what class your function would be part of, implement it as a `public static` function:

```
public static Character firstNonRepeated( String str ){
```

The preceding implementation would probably be sufficient in most interview situations, but it has at least two major flaws. The first is that it assumes that every Unicode character can be represented in a single 16-bit Java `char`. With the UTF-16 encoding that Java uses internally for strings, only about the first 2<sup>16</sup> Unicode characters or code points (the Basic Multilingual Plane or BMP) can be represented in a single `char`; the remaining code points require two `char`s. Because the preceding implementation iterates through the string one `char` at a time, it won’t interpret anything outside the BMP correctly.

In addition, there’s room to improve performance. Although autoboxing makes it less obvious, recall that Java Collections classes work only on reference types. That means that every time you increment the value associated with a key, the `Integer` object that held the value is thrown away, and a new `Integer` with the incremented value is constructed. Is there a way you could avoid having to construct so many `Integer`s? Consider what information you actually need about the number of times a character appears in the string. There are only three relevant quantities: you need to know whether it occurs _zero_ times, _one_ time, or _more than one_ time. Instead of storing integers in the hash table, why not just construct two `Object` values for use as your “one time” and “more than one time” flags (with not present in the hash table meaning “zero times”) and store those in the hash table? Here’s a reimplementation that addresses these problems:

```
public static String firstNonRepeated( String str ){
```

As this implementation demonstrates, handling Unicode code points encoded as two `char`s requires several changes. The Unicode code points are represented as 32-bit `int`s because they can’t always fit in a `char`. Because a code point may take one or two `char`s in the string, you must check the number of `char`s in each code point and advance the string index by this quantity to find the next code point. Finally, the first nonrepeated character could be one that can’t be represented in a single `char`, so the function now returns a `String`.

### Remove Specified Characters

This problem breaks down into two separate tasks. For each character in `str`, you must determine whether it should be deleted. Then, if appropriate, you must delete the character. The second task, deletion, is discussed first.

Your initial task is to delete a character from a string, which is algorithmically equivalent to removing an element from an array. An array is a contiguous block of memory, so you can’t simply remove an element from the middle as you might with a linked list. Instead, you must rearrange the data in the array so that it remains a contiguous sequence of characters after the deletion. For example, if you want to delete `'c'` from the string `"abcd"` you could either shift `'a'` and `'b'` forward one position (toward the end) or shift `'d'` back one position (toward the beginning). Either approach leaves you with the characters `"abd"` in contiguous elements of the array.

In addition to shifting the data, you need to decrease the size of the string by one character. If you shift characters before the deletion forward, you need to eliminate the first element; if you shift the characters after the deletion backward, you need to eliminate the last element. In most languages, it’s easier to shorten strings at the end (by either decrementing the string length or writing a `NUL` character, depending on the language) than at the beginning, so shifting characters backward is probably the best choice.

How would the proposed algorithm fare in the worst-case scenario in which you need to delete all the characters in `str`? For each deletion, you would shift all the remaining characters back one position. If `str` were _n_ characters long, you would move the last character _n_ – 1 times, the next to last _n_ – 2 times, and so on, giving worst-case _O_(_n_<sup>2</sup>) for the deletion. (If you start at the end of the string and work back toward the beginning, it’s somewhat more efficient but still _O_(_n_<sup>2</sup>) in the worst case.) Moving the same characters many times seems extremely inefficient. How might you avoid this?

What if you allocated a temporary string buffer and built your modified string there instead of in place? Then you could simply copy the characters you need to keep into the temporary string, skipping the characters you want to delete. When you finish building the modified string, you can copy it from the temporary buffer back into `str`. This way, you move each character at most twice, yielding _O_(_n_) deletion. However, you’ve incurred the memory overhead of a temporary buffer the same size as the original string, and the time overhead of copying the modified string back over the original string. Is there any way you can avoid these penalties while retaining your _O_(_n_) algorithm?

To implement the _O_(_n_) algorithm just described, you need to track a source position for the read location in the original string and a destination position for the write position in the temporary buffer. These positions both start at zero. The source position is incremented every time you read, and the destination position is incremented every time you write. In other words, when you copy a character, you increment both positions, but when you delete a character, you increment only the source position. This means the source position is always the same as or ahead of the destination position. After you read a character from the original string (that is, the source position has advanced past it), you no longer need that character—because you’re just going to copy the modified string over it. Because the destination position in the original string is always a character you don’t need anymore, you can write directly into the original string, eliminating the temporary buffer entirely. This is still an _O_(_n_) algorithm but without the memory and time overhead of the earlier version.

Now that you know how to delete characters, consider the task of deciding whether to delete a particular character. The easiest way to do this is to compare the character to each character in `remove` and delete it if it matches any of them. How efficient is this? If `str` is _n_ characters long and `remove` is _m_ characters long, then in the worst case you make _m_ comparisons for each of _n_ characters, so the algorithm is _O_(_nm_). You can’t avoid checking each of the _n_ characters in `str`, but perhaps you can make the lookup that determines whether a given character is in `remove` better than _O_(_m_).

If you’ve already worked through “Find the First Nonrepeated Character,” this should sound familiar. Just as you did in that problem, you can use `remove` to build an array or hash table that has constant time lookup, thus giving an _O_(_n_) solution. The trade-offs between hash tables and arrays are the same as previously discussed. In this case, an array is most appropriate when `str` and `remove` are long and characters have relatively few possible values (for example, ASCII strings). A hash table may be a better choice when `str` and `remove` are short or characters have many possible values (for example, Unicode strings). Either choice could be acceptable as long as you justify it appropriately. This time, you’re told that the inputs are ASCII strings, so the array wouldn’t be too big. For variety, since the previous implementation used a hash table, try using a lookup array for this one.

Your function has three parts:

1.  Iterate through each character in `remove`, setting the corresponding value in the lookup array to `true`.
2.  Iterate through `str` with a source and destination index, copying each character only if its corresponding value in the lookup array is `false`.
3.  Set the length of `str` to account for the characters that have been removed

Now that you’ve combined both subtasks into a single algorithm, analyze the overall efficiency for `str` of length _n_ and `remove` of length _m_. You perform a constant time assignment for each character in `remove`, so building the lookup array is _O_(_m_). Finally, you do at most one constant time lookup and one constant time copy for each character in `str`, giving _O_(_n_) for this stage. Summing these parts yields _O_(_n_ + _m_), so the algorithm has linear running time.

Having justified and analyzed your solution, you’re ready to code it. You can write this function in Java. (The C# implementation would be nearly identical.) You’re told that the `str` argument is mutable, so it will be a `StringBuilder` rather than an immutable `String`.

```
public static void removeChars( StringBuilder str, String remove ) {
```

### Reverse Words

You probably already have a good idea how to start this problem. Because you need to operate on words, you must be able to recognize where words start and end. You can do this with a simple token scanner that iterates through each character of the string. Based on the definition given in the problem statement, your scanner can differentiate between _nonword characters_—namely, the space character—and _word characters_, which for this problem are all characters except space. A word begins, not surprisingly, with a word character and ends at the next nonword character or the end of the string.

The most obvious approach is to use your scanner to identify words, write these words into a temporary buffer, and then copy the buffer back over the original string. To reverse the order of the words, you must either scan the string backward to identify the words in reverse order or write the words into the buffer in reverse order (starting at the end of the buffer). It doesn’t matter which method you choose; the following discussion identifies the words in reverse order.

As always, consider the mechanics of how this works before you begin coding. First, you need to allocate a temporary buffer of the appropriate size. Next, enter the scanning loop, starting with the last character of the string. When you find a nonword character, you can write it directly to the buffer. When you find a word character, however, you can’t write it immediately to the temporary buffer. Because you scan the string in reverse, the first word character you encounter is the last character of the word, so if you were to copy the characters in the order you find them, you’d write the characters within each word backward. Instead, you need to keep scanning until you find the first character of the word and then copy each character of the word in the correct, nonreversed order. (You may think you could avoid this complication by scanning the string forward and writing the words in reverse. However, you then must solve a similar, related problem of calculating the start position of each word when writing to the temporary buffer.) When you copy the characters of a word, you need to identify the end of the word so that you know when to stop. You could do this by checking whether each character is a word character, but because you already know the position of the last character in the word, a better solution is to continue copying until you reach that position.

An example may help to clarify this. Suppose you are given the string `"piglet quantum"`. The first word character you encounter is `'m'`. If you copy the characters as you found them, you end up with the string `"mutnauq telgip"`, which is not nearly as good a name for a techno group as the string you were supposed to produce, `"quantum piglet"`. To get `"quantum piglet"` from `"piglet quantum"` you need to scan until you get to `'q'` and then copy the letters in the word in the forward direction until you get back to `'m'` at position 13. Next, copy the space character immediately because it’s a nonword character. Then, just as for `"quantum"`, you would recognize the character `'t'` as a word character, store position 5 as the end of the word, scan backward to `'p'`, and finally write the characters of `"piglet"` until you got to position 5.

After you scan and copy the whole string, copy the buffer back over the original string. Then you can deallocate the temporary buffer and return from the function. This process is illustrated graphically in [Figure 7-1](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c07-fig-0001).

![[attachments/c07f001.jpg]]

[**FIGURE 7-1**](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c07-fig-0001)

It’s obviously important that your scanner stop when it gets to the first character of the string. Although this sounds simple, it can be easy to forget to check that the read position is still in the string, especially when the read position is changed at more than one place in your code. In this function, you move the read position in the main token scanning loop to get to the next token and in the word scanning loop to get to the next character of the word. Make sure neither loop runs past the beginning of the string.

Just for variety, implement this problem in C, and assume that you’re dealing with ASCII characters that can be safely stored in byte arrays:

```
bool reverseWords( char str[] ){
```

The preceding token scanner-based implementation is the general-case solution for this type of problem. It is reasonably efficient, and its functionality could easily be extended. It is important that you are able to implement this type of solution, but the solution is not perfect. All the scanning backward, storing positions, and copying forward is somewhat lacking in algorithmic elegance. The need for a temporary buffer is also less than desirable.

Often, interview problems have obvious general solutions and less-obvious special-case solutions. The special-case solution may be less extensible than a general solution but more efficient or elegant. Reversing the words of a string is such a problem. You have seen the general solution, but a special-case solution also exists. In an interview, you might have been steered away from the general solution before you got to coding it. (The general solution is followed through to code here because token and string scanning are important techniques.)

One way to improve an algorithm is to focus on a particular, concrete deficiency and try to remedy that. Because elegance, or lack thereof, is hard to quantify, you might try to eliminate the need for a temporary buffer from your algorithm. You can probably see that this is going to require a significantly different algorithm. You can’t simply alter the preceding approach to write to the same string it reads from—by the time you get halfway through, you will have overwritten the rest of the data you need to read.

Rather than focus on what you can’t do without a buffer, you should turn your attention to what you can do. You can reverse an entire string in place by exchanging characters. Try an example to see whether this might be helpful: “in search of algorithmic elegance” would become “ecnagele cimhtirogla fo hcraes ni”. Look at that! The words are in exactly the order you need them, but the characters in the words are backward. All you have to do is reverse each word in the reversed string. You can do that by locating the beginning and end of each word using a scanner similar to the one used in the preceding implementation and calling a reverse function on each word substring.

Now you just have to design an in-place reverse string function. The only trick is to remember that there’s no one-statement method of exchanging two values in C—you have to use a temporary variable and three assignments. Your reverse string function should take a string, a start index, and an end index as arguments. Begin by exchanging the character at the start index with the character at the end index, and then increment the start index and decrement the end index. Continue like this until the start and end index meet in the middle (in a string with odd length) or end is less than start (in a string with even length)—put more succinctly, continue while end is greater than start.

You can continue to implement in C, but to keep things interesting this time, use wide character strings. (Wide character string and character literals are prepended with `L` to distinguish them from regular byte-sized literals.) These functions look like the following:

```
void wcReverseString( wchar_t str[], int start, int end ){
```

This solution does not need a temporary buffer and is considerably more elegant than the previous solution. It’s also more efficient, mostly because it doesn’t suffer from dynamic memory overhead and doesn’t need to copy a result back from a temporary buffer.

### Integer/String Conversions

Every language has library routines to do these conversions. For example, in C# the `Convert.ToInt32()` and `Convert.ToString()` methods are available. Java uses the `Integer.parseInt()` and `Integer.toString()` methods. You should mention to the interviewer that under normal circumstances, you know better than to duplicate functionality provided by standard libraries. This doesn’t get you off the hook—you still need to implement the functions called for by the problem.

### From String to Integer

You can start with the string-to-integer routine, which is passed a valid string representation of an integer. Think about what that gives you to work with. Suppose you were given `"137"`. You would have a three-character string with the character encoding for `'1'` at position 0, `'3'` at position 1, and `'7'` at position 2. Recall from grade school that the 1 represents 100 because it is in the hundred’s place, the 3 represents 30 because it is in the ten’s place, and the 7 is just 7 because it is in the one’s place. Summing these values gives the complete number: 100 + 30 + 7 = 137.

This gives you a framework for dissecting the string representation and building it back into a single integer value. You need to determine the numeric (integer) value of the digit represented by each character, multiply that value by the appropriate place value, and then sum these products.

Consider the character-to-numeric-value conversion first. What do you know about the values of digit characters? In all common character encodings, the values are sequential: `'0'` has a value one less than `'1'`, which in turn is followed by `'2'`, `'3'`, and so on. (Of course, if you didn’t know this, you’d have to ask the interviewer.) Therefore, the value of a digit character is equal to the digit plus the value of `'0'`. (The value of `'0'` is the nonzero code number representing the character `'0'`.) This means you subtract the value of `'0'` from a digit character to find the numeric value of the digit. You don’t even need to know what the value of `'0'` is; just write `-'0'`, which the compiler interprets as “subtract the value of `'0'`.”

Next, you need to know what place value each digit must be multiplied by. Working through the digits left to right seems problematic because you don’t know what the place value of the first digit is until you know how long the number is. For example, the first character of `"367"` is identical to that of `"31"`; although it represents 300 in the first case and 30 in the second case. The most obvious solution is to scan the digits from right to left because the rightmost position is always the one’s place, the next to rightmost is always the ten’s, and so on. This enables you to start at the right end of the string with a place value of 1 and work backward through the string, multiplying the place value by 10 each time you move to a new place. This method, however, requires two multiplications per iteration, one for multiplying the digit by the place value and another for increasing the place value. That seems a little inefficient.

Perhaps the alternative of working through the characters left to right was too hastily dismissed. Is there a way you could get around the problem of not knowing the place value for a digit until you’ve scanned the whole string? Returning to the example of `"367"`, when you encounter the first character, `'3'`, you register a value of 3. If the next character were the end of the string, the number’s value would be 3. However, you encounter `'6'` as the next character of the string. Now the `'3'` represents 30 and the 6 represents `'6'`. On the next iteration, you read the last character, `'7'`, so the `'3'` represents 300, the `'6'` represents 60, and the `'7'` represents 7. In summary, the value of the number you’ve scanned so far increases by a factor of 10 every time you encounter a new character. It doesn’t matter that you don’t initially know whether the `'3'` represents 3, 30, or 30,000—every time you find a new digit you just multiply the value you’ve already read by 10 and add the value of the new digit. You’re no longer tracking a place value, so this algorithm saves you a multiplication on each iteration. The optimization described in this algorithm is frequently useful in computing checksums and is considered clever enough to merit a name: _Horner’s Rule._

Up to this point, the discussion has touched on only positive numbers. How can you expand your strategy to include negative numbers? A negative number has a `'-'` character in the first position. You want to skip over the `'-'` character so that you don’t interpret it as a digit. After you scan all the digits and build the number, you need to change the number’s sign so that it’s negative. You can change the sign with the negation operator: `-` . You have to check for the `'-'` character before you scan the digits so that you know whether to skip the first character, but you can’t negate the value until after you’ve scanned the digits. One way around this problem is to set a flag if you find the `'-'` character and then apply the negation operator only if the flag is set.

In summary, the algorithm is as follows:

```
Start number at 0
```

Coding this in Java results in the following:

```
public static int strToInt( String str ){
```

Before you declare this function finished, check it for cases that may be problematic. At minimum, you should check –1, 0, and 1, so you’ve checked a positive value, a negative value, and a value that’s neither positive nor negative. You should also check a multidigit value like 324 to ensure that the loop has no problems. The function appears to work properly for these cases, so you can move on to the opposite conversion in `intToStr`.

### From Integer to String

In `intToStr`, you perform the inverse of the conversion you did in `strToInt`. Given this, much of what you discovered in writing `strToInt` should be of use to you here. For example, just as you converted digits to integer values by subtracting `'0'` from each digit, you can convert integer values back to digits by adding `'0'` to each digit.

Before you can convert values to characters, you need to know what those values are. Consider how you might do this. Suppose you have the number 732. Looking at this number’s decimal representation on paper, it seems a simple matter to identify the digit values 7, 3, and 2. However, you must remember that the computer isn’t using a decimal representation, but rather the binary representation 1011011100. Because you can’t select decimal digits directly from a binary number, you must calculate the value of each digit. It seems logical to try to find the digit values either left to right or right to left.

Try left to right first. Integer-dividing 732 by the place value (100) gives the first digit, 7. However, now if you integer-divide by the next place value (10), you get 73, not 3. It looks as if you need to subtract the hundreds value you found before moving on. Starting over with this new process gives you the following:

```
732 / 100 = 7 (first digit); 732 – 7 * 100 = 32
```

To implement this algorithm, you must find the place value of the first digit and divide the place value by 10 for each new digit. This algorithm seems workable but complicated. What about working right to left?

Starting again with 732, what arithmetic operation can you perform to yield 2, the rightmost digit? Modulo gives the remainder of an integer division. (In languages with C-influenced syntax the modulo operator is `%`.) 732 modulo 10 gives you 2. Now how can you get the next digit? 732 modulo 100 gives you 32. You could integer-divide this by 10 to get the second digit, 3, but now you have to track two separate place values.

What if you did the integer divide before the modulo? Then you’d have 732 integer divide by 10 is 73; 73 modulo 10 is 3. Repeating this for the third digit you have 73 / 10 = 7; 7 % 10 = 7. This seems like an easier solution—you don’t even have to track place values; you just divide and modulo until there’s nothing left.

The major downside of this approach is that you find the digits in reverse order. Because you don’t know how many there will be until you’ve found them all, you don’t know where in the string to begin writing. You could run through the calculations twice—once to find the number of digits so that you know where to start writing them and again to actually write the digits—but this seems wasteful. Perhaps a better solution is to write the digits out backward as you discover them and then reverse them into the proper order when you’re done. Because the largest possible value of an integer yields a relatively short string, you could write the digits into a temporary buffer and then reverse them into the final string.

Again, negative numbers have been ignored so far. Unfortunately, the modulo of a negative number is not handled consistently across different languages, so writing code that calculates the modulo of a negative number is likely to be error prone and may confuse others reading your code. One way around this problem is to avoid it entirely. In `strToInt`, you treated the number as if it were positive and then made an adjustment at the end if it were negative. How might you employ this type of strategy here? You could start by negating the number if it were negative. Then it would be positive, so treating it as a positive number wouldn’t be a problem. The only wrinkle would be that you’d need to write a `'-'` if the number had originally been negative, but that isn’t difficult—just set a flag indicating that the number is negative when you negate it.

You’ve solved all the important subproblems in `intToStr`—now assemble these solutions into an outline you can use to write your code:

```
If number less than zero:
```

Rendering this in Java might give the following:

```
public static final int MAX_DIGITS = 10;
```

Again, check the same potentially problematic cases you tried for `strToInt` (multidigit, –1, 0, and 1). Multidigit numbers, –1, and 1 cause no problems, but if `num` is 0 you never go through the body of the `while` loop. This causes the function to write an empty string instead of `"0"`. How can you fix this bug? You need to go through the body of the `while` loop at least once so that you write a `'0'` even if `num` starts at 0. You can ensure that the body of the loop is executed at least once by changing it from a `while` loop to a `do...while`.

Another class of errors that’s particularly relevant for functions like this that do extensive numeric computation is arithmetic overflow. Try to identify overflow errors by considering each arithmetic operation you perform, and whether it could overflow. In particular consider what happens with minimum and maximum value inputs. In this function, neither the modulo, division, nor addition operations can result in an overflow. However, there is a very subtle overflow that can occur when negating a negative input to make it positive (`num = -num;`). Consider the result when `num` is `Integer.MIN_VALUE.` Because of the way that two’s complement representation works (see [Chapter 14](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c14.xhtml)), the minimum value of a signed integer has a larger magnitude than the maximum value. As a result, if you try to negate `Integer.MIN_VALUE` it overflows and wraps back around to yield `Integer.MIN_VALUE` again. There are several ways you could deal with this overflow. Since it occurs for only one input value, probably the most straightforward solution is to special case that input value.

These fixes yield the following code, which can handle converting 0 as well as positive and negative values to strings:

```
public static final int MAX_DIGITS = 10;
```

### UTF-8 String Validation

Start by looking for organizing principles in these patterns. A few things are apparent:

-   Trailing bytes start with a 10.
-   A leading byte that starts with 0 indicates a single-byte pattern (an ASCII character).
-   All other leading bytes start with 11: a leading byte that starts with 110 is followed by a single trailing byte; 1110 is followed by two bytes; and 11110 is followed by three bytes.

You’ll need to evaluate the high bits of each byte to determine which category it falls into. You can use bit operators to do this. Specifically, you construct a value called a _mask_ where the bits you are interested in have a value of 1 and all other bits have values of 0. Combining the mask with the byte to be interrogated using the `&` operator zeros out everything except your bits of interest. (See [Chapter 14](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c14.xhtml) if this technique is unfamiliar.) It may be useful to write helper functions that implement these operations for each category of byte. In C, they might look like:

```
// Byte is 10xxxxxx
```

Using these helper functions you can implement a basic algorithm that checks that each character starts with the correct bit pattern and skips over the correct number of bytes:

```
bool ValidateUTF8( const unsigned char* buffer, size_t len ) {
```

This code is incomplete, though, because it doesn’t check that the buffer ends with a complete UTF-8 character, or that only trailing bytes are in between the leading bytes. You can easily confirm that problems exist using the following test cases:

```
// Bad buffer -- 4-byte character chopped off.
```

Remember to check the edge conditions after coding an algorithm!

Checking that the buffer ends with a complete UTF-8 character is simple enough; all you need to do is to make sure that the buffer index is exactly equal to the buffer length:

```
bool ValidateUTF8( const unsigned char* buffer, size_t len ) {
```

But that doesn’t really fix the underlying problem with this algorithm, which is that it skips over entire sequences of bytes without ever checking their validity. To be valid, a leading byte must be followed by the correct number of trailing bytes, which means tracking how many trailing bytes are expected and confirming that they are all trailing bytes. Implementing this yields:

```
bool ValidateUTF8( const unsigned char* buffer, size_t len ) {
```

## SUMMARY

Arrays are an essential part of nearly every programming language, so you should expect that they will appear in some of your interview problems. In most languages, accessing an array is constant time if you have the index of the element you need, but linear time if you have only the value of the element but not the index. If you insert or delete in the middle of an array, you must move all the elements that follow to open or close the space. Static arrays are created with a fixed size; dynamic arrays grow as needed. Most languages support both types to a greater or lesser extent.

Strings are one of the most common applications of arrays. In C, a string is little more than an array of characters. In object-oriented languages, the array is typically hidden within a string object. String objects can be converted to and from character arrays; make sure you know how to do this in the languages you’ll be using because the operations required by programming problems are often more convenient with arrays. Basic string objects are immutable (read-only) in C# and Java; other classes provide writeable string functionality. Careless concatenation of immutable strings can lead to inefficient code that creates and throws away many string objects.

Most modern applications support multiple languages using Unicode. Multiple encodings exist for representing Unicode, all of which require multiple bytes for at least some characters, and many of which are variable-length. (Some characters require more bytes than others.) These encodings can considerably complicate string problems, but most of the time you probably won’t need to worry about this for interview problems.
