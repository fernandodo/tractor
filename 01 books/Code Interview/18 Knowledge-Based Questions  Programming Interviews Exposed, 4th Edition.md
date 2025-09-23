---
created: 2025-09-23T20:47:11 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 18 Knowledge-Based Questions | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> 18Knowledge-Based Questions

 Knowledge-based questions vary greatly in frequency from interview to interview. Some interviewers do not ask knowledge-based questions, whereas others focus solely...

---
## PROBLEMS

It would be impossible to cover every conceivable area of computer knowledge that could appear on a résumé or in an interview. Instead, this chapter provides a sample of knowledge-based questions. These questions focus on system-level issues, trade-offs between various methods of programming, and advanced features of languages. All these topic areas make sense from the interviewer’s perspective. A candidate who claims to know a lot about computers but who isn’t aware of basic performance issues of data structures, networks, and architecture is likely to make poor design decisions that may be expensive to fix later. Furthermore, many job assignments are not as specific as “Implement this algorithm in this language,” but may be more along the lines of “We have this problem that we need solved.” A strong candidate understands the trade-offs between various solutions and knows when to use each one.

Interviewers prefer specific, detailed descriptions to general answers. For example, suppose you are asked, “What is AJAX?” One general answer is, “It stands for _asynchronous JavaScript and XML_.” Although this answer is technically correct, it doesn’t demonstrate that you actually understand what AJAX programming is about and why it has become so popular. A better answer would be “AJAX, which is short for asynchronous JavaScript and XML, is an architectural style for building interactive web applications in which code to perform tasks such as interface updates and input validation is implemented on the client in JavaScript and data exchanges with the server occur in the background over HTTP. XML was originally the preferred format for returning data to the client for processing, but many applications have shifted to other formats, like JSON. Applications built using AJAX don’t suffer the frustrating delays in user interface response that are common in conventional web applications.” It seems clear which answer is better.

One final note: the answers presented here have been researched and polished by several people over an extended period of time. In many cases they also include detailed explanations and examples. As a candidate answering a question in an interview, you would not be expected to provide such a detailed response. Any well-organized answer that hits most of the points in these solutions would probably be considered excellent.

### C++ versus Java

C++ and Java are syntactically similar. Java’s designers intended this to make it easy for C++ developers to learn Java. Apart from this similarity, Java and C++ differ in a variety of ways, largely because of their different design goals. Security, portability, and simplicity were of paramount importance in the design of Java, whereas C++ is more concerned with performance, backward compatibility with C, and programmer control. Java is compiled to virtual machine byte-code and requires a virtual machine to run; C++ is compiled to native machine code. This gives Java greater potential for portability and security. Historically, this has also made Java slower than C++, but with just-in-time compiler techniques in modern virtual machines, performance is often comparable.

C++ is an approximate superset of C and maintains features such as programmer-controlled memory management, pointers, and a preprocessor for backward compatibility with C. In contrast, Java eliminates these and other error-prone features. Java replaces programmer memory deallocations with garbage collection. Java further dispenses with C++ features such as operator overloading and multiple inheritance. (A limited form of multiple inheritance can be simulated in Java using interfaces.) These differences are seen by some to make Java a better choice for rapid development and for projects where portability and security are important.

In Java, all objects are passed by reference, whereas in C++, objects may be passed by reference or pointer, but the default behavior is to pass by value (invoking a copy constructor). Java does not perform automatic type casting like C++; though Java features such as generics and autoboxing handle many common cases of type casting. In Java, by default, a method is virtual, meaning the implementation for a method is selected according to the type of the object as opposed to the type of the reference; a method is nonvirtual when declared `final`. In C++, methods are nonvirtual unless they are explicitly declared as virtual. In either language, the overhead of virtual function calls can be avoided where they are not needed. Java has defined sizes for primitive data types, whereas type sizes are implementation-dependent in C++.

In situations in which there is legacy C code or a great need for performance, C++ has certain benefits, especially when low-level system access is required. In situations in which portability, security, and speed of development are emphasized, Java (or a similar language such as C#) may be a better choice.

### Friend Classes

The `friend` keyword is applied to either a function or a class. It gives the `friend` function or `friend` class access to the private members of the class in which the declaration occurs. Some programmers feel this feature violates the principles of object-oriented programming because it allows a class to operate on another class’s private members. This violation can, in turn, lead to unexpected bugs when a change in the internal implementation of a class causes problems with the friend class that accesses it.

In some cases, however, the benefits of a `friend` class outweigh its drawbacks. For example, suppose you implemented a dynamic array class. Imagine that you want a separate class to iterate through your array. The iterator class would probably need access to the dynamic array class’s private members to function correctly. It would make sense to declare the iterator as a `friend` to the array class. The workings of the two classes are inextricably tied together already, so it probably doesn’t make sense to enforce a meaningless separation between the two.

Java and C# do not support the concept of `friend` classes. The closest match these languages have to `friend`s is to omit the access modifiers, thereby specifying “default” access (in Java) or use the “internal” access modifier (in C#) for member data. However, this makes every class in the package (Java) or assembly (C#) equivalent to a `friend`. In some cases, it may be possible to use a nested class to accomplish a similar design to that achieved with `friend` classes in C++.

### Argument Passing

In the first prototype, the object argument is passed by value. This means that `Fruit`’s copy constructor would be called to duplicate the object on the stack. The compiler will create a default member-by-member copy constructor if `Fruit` doesn’t have an explicit one defined; this may lead to bugs if `Fruit` contains pointers to resources it owns, such as dynamically allocated memory or file handles. Within the function, `bar` is an object of class `Fruit`. Because `bar` is a copy of the object that was passed to the function, any changes made to `bar` will not be reflected in the original object. This is the least efficient way to pass an object because every data member of the object must be copied into a new copy of the object.

For the second prototype, `bar` is a pointer to a `Fruit` object, and the pointer value is passed to `foo`. This is more efficient than passing the object by value because only the address of the object is copied onto the stack (or possibly into a register), not the object itself. Because `bar` points at the object that was passed to `foo`, any changes made through `bar` are reflected in the original object.

The third prototype shows `bar` being passed by reference. This case is similar to the second: it involves no copying of the object and allows `foo` to operate directly on the calling function’s object. The most obvious difference between a function using a reference and one using a pointer is syntactic. A pointer must be explicitly dereferenced before member variables and functions can be accessed, but members can be accessed directly using a reference. Therefore, the arrow operator (`->`) is usually used to access members when working with pointers, whereas the dot operator (`.`) is used for references. A subtler but more important difference is that the pointer may not point at a `Fruit`; the pointer version of `foo` could be passed a null pointer. In the implementation using references, however, `bar` is guaranteed to be a reference to a `Fruit` (although it’s possible for the reference to be invalid).

In the fourth prototype, `bar` is passed as a constant pointer to the object. This has the performance advantages of passing pointers, but `foo` is prevented from modifying the object to which `bar` `points`. Only methods declared as `const` can be called on `bar` from within `foo`, which prevents `foo` from indirectly modifying the object to which `bar` points.

In the fifth prototype, `bar` is a reference to a pointer to a `Fruit` object. As in the second case, this means that changes made to the object are seen by the calling function. In addition, because `bar` is a reference to a pointer, not merely a pointer, if `bar` is modified to point to a different `Fruit` object, the pointer in the calling function is modified as well.

The final prototype is an example of an _rvalue reference_, a new feature introduced in C++11. Here `bar` is being passed by reference, as in the third prototype, but `bar` is an rvalue. The complete definition of an rvalue is somewhat complex, but you can think of it as an expression that doesn’t have a defined memory location (you can’t take its address using the `&` operator). Rvalues commonly occur as the values returned by functions or operators. Because the rvalue object can’t be referred to anywhere else and would be destroyed at the end of the statement, it’s safe for the function using this prototype to do anything it wants to with the contents of `bar`, including taking ownership of encapsulated data. This has a limited but important use case: it allows for implementation of constructors and assignment operators that take ownership of the member data of the object they are passed rather than copying it. A _move constructor_ implemented using an rvalue reference argument accomplishes the same purpose as a _copy constructor_ but is generally more efficient because it avoids copying data.

### Macros and Inline Functions

Macros are implemented with simple text replacement in the preprocessor. For example, if you define the macro:

```
#define TRIPLE(x) 3 * x
```

the preprocessor replaces any occurrences of `TRIPLE(foo)` in your code with `3 * foo`. You commonly use macros in places where the thing that you’re substituting is ugly and used often enough that it warrants abstraction behind a pretty name, but is too simple to be worth the overhead of a function call.

Inline functions are declared and defined much like regular functions. Unlike macros, they are handled by the compiler directly. An inline function implementation of the `TRIPLE` macro would look like:

```
inline int Triple(int x)
```

From the programmer’s perspective, calling an inline function is like calling a regular function. Just as for a regular function, you must specify the argument and return types for an inline function, which is not necessary (or possible) for the macro. This can be both an advantage and a disadvantage: the inline function has better type safety, but you can use a single definition of the macro for any type that has addition and division operators defined. A templated inline function would avoid the need to write a separate definition for each argument type, at the expense of increased complexity. From the compiler’s perspective, when it encounters a call to an inline function, it writes a copy of the compiled function definition instead of generating a function call. (Technically, when a programmer specifies a function as inline the compiler interprets this as a suggestion—it may or may not actually inline the function depending on its calculations of what will yield the best performance.)

Both inline functions and macros provide a way to eliminate function call overhead at the expense of program size. Although inline functions have the semantics of a function call, macros have the semantics of text replacement. Macros can create bugs due to the unexpected behavior of text replacement semantics.

For example, suppose you had the following macro and code:

```
#define CUBE(x) x * x * x
```

You would probably expect this code to set `bar` to 3 and `foo` to 27, but look at how it expands:

```
foo = ++bar * ++bar * ++bar;
```

Because of this, `bar` is set to 5 and `foo` is set to a compiler-dependent value larger than 27 (for example, 80 with one version of the GNU C++ compiler). If `CUBE` were implemented as an inline function, this problem wouldn’t occur. Inline functions (like normal functions) evaluate their arguments only once, so any side effects of evaluation happen only once.

Here’s another problem that stems from using macros. Suppose you have a macro with two statements in it like this:

```
#define INCREMENT_BOTH(x, y) x++; y++
```

If you favor leaving off the curly brackets when there’s only one statement in the body of an `if` statement, you might write something like this:

```
if (flag)
```

You would probably expect this to be equivalent to:

```
if (flag) {
```

Instead, when the macro is expanded, the `if` binds to just the first statement in the macro definition, leaving you with code equivalent to:

```
if (flag) {
```

An inline function call is a single statement, regardless of how many statements there are in the body of the function, so this problem would not occur.

A final reason to avoid macros is that when you use them, the code that is compiled is not visible in the source. This makes debugging macro-related problems particularly difficult. Macros are included in C++ and C99 largely for compatibility with older versions of C; in general it’s a good idea to avoid macros and opt for inline functions.

### Inheritance

Clearly, you can pass B because that’s exactly what the method takes. You can’t possibly pass D because it may have totally different characteristics than B. A is the parent class of B. Consider that a child class is required to implement all the methods of the parent, but the parent does not necessarily have all the methods of a child. Thus, the parent class, A, cannot be passed to the method. C is the child class of B and is guaranteed to have all the methods of B, so you can pass C to the method.

### Garbage Collection

_Garbage collection_ is the process by which memory that is no longer in use is identified and reclaimed. This reclamation occurs without programmer assistance. C#, Java, Lisp, and Python are examples of languages with garbage-collection facilities.

Garbage collection provides several advantages over having a programmer explicitly deallocate memory. It eliminates bugs caused by dangling pointers, multiple deallocation, and memory leaks. It also promotes greater simplicity in program and interface design because the complicated mechanisms traditionally used to ensure that memory is properly freed are unnecessary. In addition, because programmers don’t have to worry about memory deallocation, program development proceeds at a more rapid pace.

Garbage collection is not without its disadvantages. Garbage-collected programs often run more slowly because of the overhead needed for the system to determine when to deallocate and reclaim memory that is no longer needed. In addition, the system will occasionally over-allocate memory and may not free memory at the ideal time.

One method of garbage collection is _reference counting_. This involves tracking how many variables reference an object. Initially, there will be one reference to a piece of memory. The reference count increases if the variable referencing it is copied. When a variable referencing an object changes value or goes out of scope, the object’s reference count is decremented. If a reference count ever goes to 0, the memory associated with the object is freed: If there are no references to the object, then the object (and hence its memory) is no longer needed.

Reference counting is simple and relatively fast. Memory is freed and becomes available for reuse as soon as it is no longer referenced, which is usually an advantage. However, simple implementations have difficulty with circular references. Consider what happens in the case of a circular linked list with nothing external pointing to it. Every element in the list has a nonzero reference count, yet the memory isn’t referenced by any object outside the list itself. Thus, the memory could safely be deallocated, but simple reference-based garbage collection won’t free it.

_Weak references_—references that are not included in an object’s reference count—provide one means to deal with this problem. If every cycle of references in a data structure contains a weak reference, then you can reclaim the structure when you lose the last external reference. For example, consider a doubly linked list: in a simple reference counting system, every pair of adjacent elements form a cycle, so the list isn’t reclaimed even when it’s no longer externally referenced. If all the “previous” references are defined as weak references, then when there are no external references to the list, the head element’s reference count becomes 0, and it is deallocated. This causes a cascading deallocation along the list as deallocation of each element sets the reference count of the next element to 0. This style of garbage collection is available in C++ as `std::shared_ptr` and `std::weak_ptr`.

A second method of garbage collection is known as a _tracing garbage collector_. Under this scheme, memory that is no longer referenced remains allocated until it is identified and deallocated during a garbage collection cycle. This has the advantages of handling cyclical data structures and avoiding the overhead of incrementing and decrementing reference counts. The simplest implementation of a tracing garbage collector is called _mark and sweep_. Each cycle involves two passes. In the first pass, the memory manager marks all objects that can be accessed by any thread in the program. In the second pass, all unmarked objects are deallocated, or swept away. Mark and sweep requires that all execution threads are suspended during garbage collection, which results in unpredictable pauses during program execution. Most modern tracing garbage collectors, including those in the Java Virtual Machine and the .NET Common Language Runtime that C# uses, employ a more complex scheme called _tri-color marking_, which doesn’t require suspending execution (although it doesn’t eliminate the computational overhead of garbage collection cycles).

### 32-Bit versus 64-Bit Applications

These terms refer to the size of the memory addresses and general-purpose registers that an application uses. A 64-bit application requires a 64-bit processor and a 64-bit operating system to run. Most 64-bit systems are also capable of running 32-bit applications in a compatibility mode.

Memory address size is the most important difference between 32- and 64-bit applications. Use of 64-bit memory addresses allows a process to address a theoretical maximum of 2<sup>64</sup> = 16 exabytes of memory, a dramatic increase from the 2<sup>32</sup> = 4 gigabytes of memory to which a 32-bit process is limited. Many modern computers have more than 4 gigabytes of physical memory, so a 64-bit application may be faster because it can keep more data in memory, reducing slow disk access. The expanded 64-bit address size also makes memory-mapped files more practical, which may allow for more efficient file access than traditional APIs. In addition, 64-bit arithmetic may be faster because of the larger register size (although many “32-bit” processors have extensions that allow for 64-bit arithmetic).

On the other hand, 64-bit memory addresses mean that all pointers require twice as much memory to store. For data structures that employ pointers (or references, which use pointers behind the scenes), this means that the same structure requires more memory in a 64-bit application than a 32-bit application. More important, any given system has the same fixed-size processor cache whether running 32-bit or 64-bit applications. Because the 64-bit data structures are larger, less of them fit in cache, so there are likely to be more cache misses in which the processor must wait for values to be accessed from main memory (or higher cache levels).

Because some aspects of a 64-bit application lead to higher performance and others lead to lower performance, some codes may run faster as 32-bit and others run faster as 64-bit.

### Network Performance

Any network can be measured by two major characteristics: latency and bandwidth. _Latency_ refers to the time it takes a given bit of information to get from one point to another on the network. _Bandwidth_ refers to the rate at which data moves through the network once communication is established. The perfect network would have infinite bandwidth and no latency.

A pipe is a useful analog for a network. The time it takes for a molecule of water to go through the whole pipe is related to the length; this is analogous to the latency. The width of the pipe determines the bandwidth: how much water can pass in a given time.

Informally, people often talk about the “speed” of a network as if it’s a single quantity, but a network may have good performance by one measure and poor performance by the other. For example, satellite-based data services frequently have high bandwidth but also high latency.

Depending on the application the network is used for, either bandwidth or latency may be the most important factor. For example, telephone calls over a network (such as Voice over IP) are sensitive to latency, which causes irritating delays that lead to people accidentally talking over each other, but telephony requires relatively little bandwidth. On the other hand, streaming HD video requires a network with fairly high bandwidth, but the latency affects only the time between requesting the stream and the start of the video, which is usually of little concern.

### Web Application Security

This code constructs a SQL query by concatenating strings provided by the user. If the username and password match a row stored in the database, then the user ID is returned to allow access to that account. Because these strings come from an untrusted source, they open this application to attack by _SQL injection._ Consider how this application would behave if a malicious user entered a username of `admin' OR 'A' = 'B` and a random password string (for example, `xyz`). After concatenation, the query string becomes

```
SELECT uid FROM Users WHERE user = 'admin' OR 'A' = 'B' AND pass = 'xyz';
```

which returns the user ID for the administrative account regardless of whether the password matches, allowing the malicious user to log in as the administrator. This attack has many variations, depending on the goal of the attacker and the form of the query being attacked, but they all stem from the same issue: data from an untrusted source compiled or interpreted in an executable context. It’s easy to forget that SQL is a programming language (a limited, domain-specific language, but a programming language nonetheless). Concatenating user data directly into a query essentially gives the user some ability to modify part of the source code of your application—clearly not a good security practice.

You can fix this problem in one of two ways: filter the data so that it can be trusted, or avoid putting the data in an executable context.

Filtering the data involves searching the string returned from the user for potentially problematic patterns and either escaping or deleting them. For example, the preceding example would fail if, prior to constructing the query, the application either removed all instances of `'` or escaped them by changing them to `''`.

This type of approach to security is called _blacklisting_. The problem with blacklisting is that you can block only forms of attacks that you know about. A large number of ways to construct a SQL injection exist, and new forms are frequently invented. Many of the more complex forms are specifically designed to evade filters by using unusual encodings for strings that appear benign when filtered but are later translated into malicious form by other layers of the application stack. To maintain security with a filtering approach, the filter must detect all forms of attack, known and yet to be invented, and must be applied to every piece of untrusted data the application receives; the odds of this are poor.

A better approach is to avoid putting the data in executable context. You can achieve this through the use of prepared statements. A _prepared statement_ is a SQL query that has placeholders to identify the locations that are filled with data when the query executes. The statement is compiled before data is bound to the placeholders. When the prepared statement executes, compilation has already taken place, so any potentially executable SQL strings in the data can’t affect the structure or intent of the query. Prepared statements also improve performance when queries execute more than once because instead of parsing, compiling, and optimizing the query for each execution, this process is performed only once. A reimplementation of this code with prepared statements might look like:

```
sql = db.prepareStatement("SELECT uid FROM Users WHERE user = ? AND " +
```

There’s one more problem with this application. The password string that the user enters is compared directly to the `pass` column. This suggests that the passwords are stored as _cleartext_: the same string that the user enters. Passwords stored this way are a major security risk. If an attacker obtains the contents of the `Users` table, it’s trivial to use the data to log in as any user. Worse yet, because many users have the same passwords across multiple sites, the attacker may be able to impersonate your users elsewhere.

The solution to this problem is to use a _cryptographic hash_. A cryptographic hash is a function that takes an arbitrary input string and produces a fixed-length _fingerprint_ or _digest_ string. The function has the property that, given the digest, it is computationally infeasible to compute either the original input or another input that would produce the same digest. The definition of computationally infeasible changes as processing power becomes less expensive and new attacks are developed, so cryptographic hashes that were once secure often become obsolete and insecure as time goes on. Some commonly used cryptographic hash functions are _MD5_ (now obsolete due to security flaws), _SHA-1_ (also obsolete), _SHA-256_ (considered secure, but no longer recommended for password hashing because it can be computed so quickly), PBKDF2, and bcrypt. Instead of storing the cleartext password, the cryptographic hash function is applied to the password and the resulting digest value is stored in the database. On subsequent login attempts, the password is again hashed, and if the digest values are the same, it’s safe to assume that the password is correct.

With hashed passwords, an attacker who obtains the contents of the `Users` table won’t be able to use the data to log in directly because it doesn’t contain the passwords. The attacker can still use the data to try to determine the passwords by brute-force guessing. Well-designed applications take several steps to make this more difficult. If the hash function is applied to the passwords directly, then a given password will have the same digest value for any user in the system. The attacker can compute the digest for each guess once and compare it against every account. Historically attackers could compare the stored password digests to a large set of precomputed digests of common passwords—called a _rainbow table_—to rapidly test a large number of password guesses; with modern GPU based password cracking tools it’s typically faster to compute hashes than to read precomputed digest values from a large table on disk, so rainbow tables are generally considered obsolete.

To prevent this, you should always salt cryptographic hashes. A _salt_ is a random string of characters selected for each user that is concatenated with the password before hashing. You store the salt in cleartext, so its value would be known to the attacker, but because the salt is different for each user, the attacker must crack each user’s password separately, rather than cracking the whole list of passwords in parallel. This increases the time required to crack a list of salted passwords by a factor of _n_, where _n_ is the number of passwords in the list.

Another technique commonly used to make cracking passwords more difficult is iteration of the hash: repeated application of the hash function with the output of one round becoming the input of the next. This increases the cost (time) of computing the hash. Algorithms specifically designed for hashing passwords, such as PBKDF2 and bcrypt, typically have an iteration process built into the algorithm, where the number of iterations is a user-supplied parameter. With an appropriately chosen number of iterations, computation of the iterated hash once for each login has a negligible performance impact on the web application, but the cost of computing it millions or billions of times to crack passwords is infeasible.

### Cryptography

_Symmetric key cryptography,_ also called _shared key cryptography_, uses the same key to encrypt and decrypt information. _Public key cryptography_ makes use of two different keys: typically a public key for encryption and a private key for decryption. Symmetric key cryptography has the advantage that it’s much faster than public key cryptography. It is also generally easier to implement and usually requires less processing power. On the downside, the two parties sending messages must agree on the same private key before securely transmitting information. This is often inconvenient or even impossible. If the two parties are geographically separated, then a secure means of communication is needed for one to tell the other what the key will be. In a pure symmetric key scenario, secure communication is generally not available. (If it were, there would be little need for encryption to create another secure channel.)

Public key cryptography has the advantage that the public key, used for encryption, does not need to be kept secret for encrypted messages to remain secure. This means public keys can be transmitted over insecure channels. Often, applications use public key cryptography to establish a shared session key and then communicate via symmetric key cryptography using the shared session key. This solution provides the convenience of public key cryptography with the performance of shared key cryptography.

Both public key and symmetric key cryptography are used to get secure information from the web. First your browser establishes a shared session key with the website using public key cryptography. Then you communicate with the website using symmetric key cryptography to actually obtain the private information.

### Hash Tables versus Binary Search Trees

A hash table does one thing well. It stores and retrieves data quickly (in _O_(1) or constant time in the average case). However, its uses beyond this are limited.

A binary search tree can insert and retrieve in _O_(log(_n)_). This is fast, though not as fast as a hash table’s _O_(1). However, a binary search tree also maintains its data in sorted order.

In a mobile device, you want to keep as much memory as possible available for data storage. If you use an unordered data structure such as a hash table, you need additional memory to sort the values, as you undoubtedly want to display the values in alphabetical order. Therefore, if you use a hash table, you must set aside memory for sorting that could otherwise be used as storage space.

If you use a binary search tree, you won’t have to waste memory or processing time on sorting records for display. Although binary tree operations are slower than hash table operations, a device like this is unlikely to have more than a few thousand entries, so a binary search tree’s _O_(log(_n_)) lookup will be fast enough. For these reasons, a binary search tree is better suited for this kind of task than a hash table.

### MapReduce

The term _MapReduce_ refers to a generalized technique for processing large data sets in parallel using a distributed infrastructure. The MapReduce framework takes care of the details of distributing the work across machines, leaving the programmer to focus solely on the logic for processing and analyzing the data.

A MapReduce system has three phases. In the _map_ phase, the system transforms the data (usually by filtering and/or sorting it) and associates each chunk of transformed data with a specific key. The transformations occur in parallel, often on separate machines. The transformed data is written to temporary storage, typically disk.

The _shuffle_ phase moves the transformed data across machines so that all data chunks with the same key are available on the same machine.

Finally, the _reduce_ phase reads all the chunks with the same key (the keys are processed in parallel across the different machines) and does some analysis or further transformation of the data. The output of the reduce phase is combined to create the final output of the MapReduce.

MapReduce is used when the data is too large to fit into memory but can be split into smaller pieces for processing. The technique can process very large data sets in a short amount of time, although it may require significant storage for the temporary data it creates and a nontrivial amount of communication overhead to coordinate the different machines and to move data between them.

## SUMMARY

Knowledge-based questions are an easy way for interviewers to assess your familiarity and experience with the programming languages and techniques they expect you to know based on the requirements for the job and what’s in your résumé. Be sure you have a good grasp of the fundamental knowledge you’ll need for the job for which you’re applying.
