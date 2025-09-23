---
created: 2025-09-23T20:39:52 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 12 Design Patterns | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> 12Design Patterns

 No two programming projects are the same, but the problems that need to be solved often recur across many otherwise dissimilar projects. Much of this book concerns data...

---
## WHAT ARE DESIGN PATTERNS?

_Design patterns_ are guidelines for identifying and solving common design problems in object-oriented programming. Unlike frameworks or class libraries, design patterns are abstract, providing recommendations on how to solve specific kinds of programming problems without providing fully fleshed-out code to implement those recommendations. They distill years of software programming experience into a set of organizational principles for object-oriented application architecture.

Design patterns were popularized and formalized in the 1990s by the publication of _Design Patterns: Elements of Reusable Object Oriented Software_,[<sup>1</sup>](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c12-note-0001) but the ideas predate the book. Many of the core design patterns, like Iterator and Singleton, are widely used and familiar to most Java and C++ programmers. Other patterns, like Builder, are less frequently used but are highly useful in the appropriate situation.

### Why Use Design Patterns?

Design patterns are useful for two reasons. The obvious reason is that they provide bestpractices solutions to common software design problems based on the collected wisdom of many programmers. This makes them invaluable as an educational tool and as a programming resource.

The second—and perhaps more important—reason is that design patterns provide a concise vocabulary for discussing design problems and their solutions. This vocabulary is a valuable aid to communicating design decisions to other programmers in informal discussions, design documents, or program comments.

Despite their general usefulness, design patterns are not a “miracle cure” for programming problems. The wrong design pattern can add unnecessary complexity to an application, and an incorrect or inefficient implementation of a pattern can introduce bugs or compromise performance.

Some programmers argue that design patterns are only necessary because of the flaws inherent in the structure of popular object-oriented languages like C++ and Java. Whether or not this is true, design patterns remain useful for programmers using these languages on a day-to-day basis.

### Design Patterns in Interviews

It’s more common that you use patterns as a way to communicate design concepts with the interviewer than that you’re asked to implement a specific design pattern. For example, as you code you might say something like _“I would define an iterator for this class to make it easy to use”_ or _“Let’s assume this data is made available through a singleton.”_ This can speed up your coding by letting you omit (if the interviewer agrees) sections of code that are not directly related to the problem at hand.

If you mention a design pattern, however, the interviewer might ask you questions to see how well you understand the design pattern. Don’t use patterns unless you can implement them and explain how they work!

## COMMON DESIGN PATTERNS

The _Design Patterns_ book by Gamma _et al_. (often referred to as “The Gang of Four”), takes a formal and detailed approach to describing 23 fundamental design patterns. It groups these patterns into three basic categories: creational, behavioral, and structural. In the following pages we’ll look at a variety of patterns from these categories to understand what patterns are and how they’re used.

### Singleton

The Singleton pattern ensures that at most one instance of a class exists at any given time. This instance acts as a gatekeeper to shared resources or as a central communications hub. An application cannot create new instances—all methods are accessed through the singleton. The application obtains the singleton by invoking a static method exposed by the class.

Core system functions are often accessed using singletons. In Java, for example, the `java.lang.Runtime` class is a singleton used to interact with the application’s execution environment. Singletons are also sometimes used as a substitute for global variables, but this doesn’t avoid any of the state problems that plague global variables, so many people consider using singletons to store global data as an anti-pattern.

Why is a singleton better than a set of static methods?

-   **Inheritance and interfaces.** Singletons are objects. They can inherit from base classes and implement interfaces.
-   **Possible multiplicity.** You can change your mind and create multiple objects (for example, one per thread) without changing a lot of code. (Of course, if you do this, it’s no longer a singleton.)
-   **Dynamic binding.** The actual class used to create the singleton can be determined at run time, not at compile time.

Singletons are not without their disadvantages. Methods must be synchronized in multithreaded environments, slowing access to the singleton’s state. A singleton may also slow the application’s startup time as it initializes (unless it uses lazy initialization), and it may hold onto resources longer than necessary, because normally a singleton isn’t destroyed until the application ends.

### Builder

The Builder pattern creates objects in a stepwise manner without knowing or caring how those objects are constructed. Instead of constructing an object directly, you instantiate a builder and let it create the object on your behalf.

Builders are particularly useful for initializing objects that require multiple constructor parameters, especially parameters of the same or similar types. Consider this simple class:

```
public class Window {
```

The constructor for `Window` takes three boolean parameters in no obvious order. Rather than frequently referring to the class documentation to remember which parameter does what, create a builder to gather all the required data and create the object for you:

```
public class WindowBuilder {
```

Then instead of directly constructing a `Window` object:

```
Window w = new Window( false, true, true ); // ??? confusing parameters!
```

use a `WindowBuilder` instance to define the new object’s initial state:

```
Window w = new WindowBuilder().setVisible( false )
```

Not only is the object initialization much clearer and easier to understand, but new initialization parameters can be easily added and removed. Specific parameters can be made mandatory, in which case an error or exception would occur if they were missing and others can be made optional with default values.

Simpler initialization is one use for builders. Sometimes it’s also useful to create a hierarchy of builders. At the top of the hierarchy is an abstract builder class that defines the methods for initializing the different parts of an object. Concrete subclasses override these methods to build the object in different ways. For example, a generic document builder would expose abstract methods like `addHeading` and `addParagraph`, which would be implemented by different subclasses to create HTML documents, PDF documents, and so on.

Use builders when objects are complex to construct and/or are constructed in several steps.

### Iterator

The Iterator pattern enables you to traverse through all the elements in a data structure without knowing or caring how those elements are stored or represented. Built-in support for iterators is common in most modern languages.

Many kinds of iterators exist, with different trade-offs to using them. The simplest iterators provide for unidirectional traversal of elements with no changes allowed to the underlying data structure. More complex iterators allow for bidirectional traversal and/or permit elements to be added to or removed from the underlying data structure.

### Observer

The Observer pattern lets objects broadcast changes in state to interested observers without needing to know much about the observers. This loose coupling is also called the _Publish-Subscribe pattern_. Observers register themselves with the subject (the object observed) using a common interface for update notifications. The subject notifies each registered observer whenever its state changes.

The _model-view-controller_ (_MVC_) separation of responsibilities found within many user-interface toolkits is a classic example of the Observer pattern in action, where changes to the model (the underlying data) automatically cause the views (the user interface) to redraw themselves.

Note that the Observer pattern does not specify what kind of information is passed to the observers, the order in which they’re updated, or how quickly and how often changes are propagated. These implementation details can have quite an impact on the performance and utility of the overall system.

### Decorator

The Decorator pattern modifies the behavior of an object by “wrapping” it with another object that is derived from the same base class and thus has the same set of methods as the original object. The Decorator pattern is therefore sometimes referred to as the _Wrapper pattern_.

A decorator forwards method calls to the underlying object. The decorator modifies the behavior of the underlying object by performing some additional processing before and/or after calling some of the methods of the underlying object.

The prototypical implementation of the Decorator pattern involves four kinds of classes: Component, Concrete Component, Decorator, and Concrete Decorator. Component is an abstract class or interface that defines all the public methods needed for the underlying object and the decorators that wrap it. It serves as the base class for both Concrete Components (the classes of the underlying objects) and Decorators. Decorator is a (typically abstract) class that provides the functionality shared by all decorators: it wraps a Concrete Component and forwards all method calls to the Component. Concrete Decorators, of which there are typically several, modify the behavior of the wrapped Concrete Component by overriding one or more methods of their parent Decorator class.

The Java IO classes (in `java.io`) provide an example of the Decorator pattern. `InputStream` is an abstract class that serves as a parent class for all input streams; this is the Component class. Several derived classes such as `FileInputStream` provide implementations for stream input from different sources; these are the Concrete Components. The Decorator is called `FilterInputStream`; it wraps an object of class `InputStream` and forwards all method calls to the wrapped object. This is not particularly useful by itself, but it serves as the base class for several Concrete Decorators that modify the behavior of input streams; these include `DeflaterInputStream`, `BufferedInputStream`, and `CipherInputStream`.

Decorators provide an alternative to subclassing. Multiple different Concrete Decorators can be applied to a given instance of Concrete Component, with each successive decoration forming another layer of wrapping around the object. The behavior of the underlying Concrete Component is modified by all of the decorators that wrap it.

## DESIGN PATTERN PROBLEMS

Because design patterns are so abstract, you can expect a lot of variation in the types of questions that are asked.

### Singleton Implementation

The Singleton pattern ensures that at most one instance of the logging class exists at any given time. The easiest way to do this is to make the constructor private and initialize the single instance within the class. Here’s a Java implementation of the logger:

```
// Implements a simple logging class using a singleton.
```

If you’ve claimed deep expertise in Java, an interviewer might ask you how an application could create multiple instances of the `Logger` class despite the existence of the private constructor and how to prevent that from happening. (Hint: think about cloning and object serialization.)

The Singleton pattern doesn’t specify _when_ an instance is created, just that there can be _at most_ one instance of the class created. It’s not necessary for the instance to be created when the class is loaded, it just needs to be created before it’s needed. Following this approach, `getInstance` should initialize the instance before returning it, if it hasn’t yet been initialized. This technique is known as _deferred initialization—_also called _lazy initialization_ or _lazy loading_.

Deferred initialization has both advantages and disadvantages, and it is not the best choice in every situation:

-   Deferred initialization yields faster startup times, at the cost of a delay caused by initialization the first time the instance is accessed.
-   If a deferred initialization singleton is never accessed, it is never initialized, saving the initialization time and resources it would otherwise require.
-   Deferred initialization allows selection of the class of the singleton object to be deferred until run time rather than being specified at compile time. Because the instance is created only once, this selection must be made before the instance is accessed for the first time, but there might still be utility in making this selection at run time. For example, this would allow selection of the class based on settings in a configuration file.
-   In a resource-limited environment, deferred initialization of the instance could fail due to inadequate resources. This could be particularly problematic for something like an error logging class that must be available when needed.
-   Deferred initialization increases the complexity of the singleton class, especially in a multithreaded system.

Now modify the `Logger` class you just wrote to use deferred initialization:

```
// Deferred initialization of Logger.
```

This simple change accomplishes deferred initialization, but introduces a new problem—it’s no longer thread-safe. In the original version of your class, the instance was initialized when the class was loaded, before any methods could be called. In the revised, deferred initialization version, the instance is created in `getInstance`. What happens if two threads call `getInstance` simultaneously? They might both see `instance` as uninitialized, and both try to create the instance—clearly not what you want for a singleton. You can prevent this from happening by making `getInstance` a synchronized method:

```
    // Return the singleton instance.
```

There is a significant performance penalty to pay for this change, but if `getInstance` is called infrequently, it may not be important. It is possible to avoid this penalty in cases where the performance of `getInstance` is relevant. Consider that you are synchronizing _every_ call to `getInstance` for the lifetime of the program, but once `instance` has been fully initialized, all you’re doing is returning `instance`, which doesn’t require synchronization to be thread-safe. Ideally, you’d like the method to be synchronized before the instance is initialized, and then stop being synchronized after the deferred initialization to avoid the overhead that synchronization entails.

Several language-specific idioms achieve this goal. One such Java idiom combines deferred and static initialization by employing deferred loading of an inner class that performs static initialization of the instance. This is thread-safe because the classloader is guaranteed to be serialized, so the inner class is loaded and initialized only once, no matter how many threads call `getInstance` simultaneously. It also avoids the overhead of synchronization, because serialization is provided by the classloader—after the class has been loaded, the classloader is no longer involved, so there’s no residual overhead. You can implement this for `Logger` by replacing the preceding implementation of `getInstance` with:

```
    // Inner class initializes instance on load, won’t be loaded
```

### Decorator versus Inheritance

Recall that the Decorator pattern _wraps_ one object with another object to change the original object’s behavior. The wrapper object can take the place of the original object because they share the same abstract base class or implement the same interface.

Both the Decorator pattern and inheritance provide means of modifying the behavior of an object of the underlying class, but in different ways. Inheritance typically allows modification of the parent class only at compile time, while decorations are applied dynamically at run time.

Suppose you have an object that needs to dynamically change behavior. Accomplishing this with inheritance may be cumbersome and inefficient: every time you need to change behavior, you’ll probably need to construct a new object of a different child class with the desired behavior, copy the state from the existing object to the new one, and throw the old one away. In contrast, modifying the behavior of the existing object using the Decorator pattern is much simpler—just add the appropriate decoration (that is, wrap the existing object with another wrapper that implements the modified behavior).

The dynamic nature of the Decorator pattern has another advantage. Suppose you have several behavior modifications that you’d like to implement for a class. Assume that none of these modifications interfere with any of the others, so you can apply them in any combination. A classic example of this is a GUI toolkit with a `Window` class that may be modified by multiple different behaviors, such as `Bordered`, `Scrollable`, `Disabled`, and so on. You could implement this with inheritance: deriving `BorderedWindow` from `Window`, `ScrollableBorderedWindow` and `DisabledBorderedWindow` from `BorderedWindow`, and so on. This is reasonable for a small number of behaviors, but as the number of behaviors increases, your class hierarchy rapidly gets out of hand. The number of classes doubles each time you add a new behavior. You can avoid this explosion of largely redundant classes with the Decorator pattern. Each behavior is completely described by a single Decorator class, and you can generate whatever combination of behaviors you need by applying the appropriate set of decorations.

The Decorator pattern simplifies object-oriented design when applied correctly, but may have the opposite effect when used indiscriminately. If you _don’t_ need to dynamically modify the behavior of an object, then it’s probably best to use simple inheritance and avoid the complexity of this pattern. Also, Concrete Decorator classes generally shouldn’t expose new public methods; so if you need to do this, using Decorators probably isn’t the best approach (Concrete Decorator classes shouldn’t expose new public methods because these methods are forwarded in the parent Decorator class, so they become inaccessible unless they are the last decoration applied). Finally, you should make sure that your Concrete Decorator classes are truly mutually noninterfering. There’s no good way to forbid combinations of decorations that are conflicting or don’t make sense, so using the Decorator pattern in these circumstances may invite bugs later on.

### Efficient Observer Updates

A naïve implementation of the Observer pattern can yield poor performance if many objects are observing other objects.

The most obvious problem is that a subject updates its state too often, causing it to spend most of its time updating its observers. This can happen when multiple properties are changed many times in rapid succession in a single code sequence. In such situations it may make more sense to briefly turn updates off, make the changes, then turn updates on and send a single update notification to all interested objects.

Another potential problem relates to how observers determine what has changed. In many windowing systems, for example, it’s much more efficient to redraw just the part of the screen that has changed, rather than the entire display. To do this properly, the view (the observer) needs to know which part of the model (the subject) has changed. Rather than have the observer query the subject to determine what changed, why not have the subject pass the information along as part of the update notification?

Some interesting problems also occur when dealing with updates across multiple threads, such as how to avoid deadlock conditions. We leave these as an exercise for you!

## SUMMARY

Design patterns are useful tools for communicating software design concepts to interviewers. Interviewers may use your level of familiarity with design patterns to try to assess how much experience you have with object-oriented design. Make sure you understand and have experience with common design patterns.
