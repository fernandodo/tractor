---
created: 2025-09-23T20:39:47 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 11 Object-Oriented Programming | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> 11Object-Oriented Programming

 Most professional development is done using an object-oriented programming (OOP) language such as Java, C#, or C++. Even JavaScript, though not an OOP language,...

---
## 11  
Object-Oriented Programming

Most professional development is done using an object-oriented programming (OOP) language such as Java, C#, or C++. Even JavaScript, though not an OOP language, supports some features of OOP through prototype objects and the clever use of function definitions. As such, you need to have a good grasp of fundamental OOP principles.

## FUNDAMENTALS

Object-oriented programming’s roots date back several decades to languages such as Simula and Smalltalk. OOP has been the subject of much academic research and debate, especially since the widespread adoption of OOP languages by practicing developers.

### Classes and Objects

No clear consensus exists on the many different ways to describe and define object orientation as a programming technique, but all of them revolve around the notions of classes and objects. A _class_ is an abstract definition of something that has _attributes_ (sometimes called _properties_ or _states_) and _actions_ (_capabilities_ or _methods_). An _object_ is a specific instance of a class that has its own state separate from any other object instance. Here’s a class definition for `Point`, which is a pair of integers that represents the _x_ and _y_ values of a point in a Cartesian coordinate plane:

```
public class Point {
```

To represent a specific point, simply create an instance of the `Point` class with the appropriate values:

```
Point p1 = new Point( 5, 10 );
```

This simple example shows one of the principles of OOP, that of _encapsulation_—the hiding of implementation details. By declaring the x and y variables to be private, the preceding implementation of the Point class “hides” these variables. They can be directly accessed only by code in the Point class. This allows for tight control of how and when properties of an object can change. In the preceding implementation of the Point class, objects are immutable because the class has no methods that change the values of the hidden variables after the object is constructed.

Encapsulation can also make code easier to maintain. Historically, non-object oriented code has often been _tightly coupled_: data structures are accessed directly from wherever they are needed. This makes changing implementations of data structures challenging, as all of the code that makes use of the data structure also needs to be changed. This may be a large amount of code, and in a large, complex application it may be hard to be sure that all the affected code has been identified. In contrast, encapsulation encourages code that is _loosely coupled_: the public methods of a class provide a well-defined interface that is the only access to the data structures contained in the class. As long as the method names, their arguments, and their conceptual purpose remain unchanged, the internal implementation of the class can be changed without affecting other code.

### Construction and Destruction

Objects are instances of classes. Creating an object is called _constructing the object_. Part of the process involves invoking a _constructor_ in the class. The constructor initializes the state of the object, which usually involves calling (either explicitly or implicitly) the constructors of its parent classes so that they can initialize their part of the object’s state.

Destroying objects is not as straightforward as constructing them. In C++ a method called the _destructor_ is invoked to clean up an object’s state. Destructors are invoked automatically when an object goes out of scope or when the `delete` operator is used to destroy a dynamically created object—keeping track of objects is important to avoid leaking memory. In languages such as C# and Java, however, the garbage collector is responsible for finding and destroying unused objects, in which case the time and place of the destruction (it usually happens on a separate, system-defined thread) is out of the application’s control. An optional _finalizer_ method is invoked by the system prior to the object’s destruction to give it the opportunity to clean itself up before its “final” destruction. (In C# and Java it’s possible—though generally inadvisable—for objects to “resurrect” themselves from destruction in their finalizers.)

### Inheritance and Polymorphism

Two other important principles are inheritance and polymorphism, which are closely related. _Inheritance_ allows a class to be defined as a modified or more specialized version of another class. When class B _inherits from_ class A (Java uses the term _extends_), class A is B’s _parent_ or _base_ class, and class B is A’s _subclass_. All the behaviors defined by class A are also part of class B, though possibly in a modified form. The same method might be defined both in a parent class and a subclass, the latter _overriding_ the former for instances of the subclass. Because a subclass has, at minimum, all the methods that its parent does, an instance of class B can be used wherever an instance of class A is required.

A core concept of OOP, enabled by overriding, is runtime selection of which definition of a method should be executed based on the class of the object. This is called _polymorphism_. Polymorphism allows class-specific code to be invoked without having to directly specify which definition to invoke in the calling code.

The classic example of inheritance and polymorphism is a shapes library representing the different shapes in a vector-based drawing application. At the top of the hierarchy is the `Shape` class, which defines the things that all shapes have in common:

```
public abstract class Shape {
```

You can then specialize the shapes into `Rectangle` and `Ellipse` subclasses:

```
public class Rectangle extends Shape {
```

The `Rectangle` and `Ellipse` classes could be further specialized into `Square` and `Circle` subclasses.

Even though many shapes may be defined in the library, the part of the application that draws them on the screen doesn’t need to do much work because polymorphism is used to select the specific, appropriate draw method-body to run:

```
void paintShapes( Graphics g, List<Shape> shapes ){
```

Adding a new shape to the library is just a matter of subclassing one of the existing classes and implementing the things that are different.

Problems you are presented with relating to object-oriented programming are likely to focus on the concepts of object orientation, particularly on issues relevant to the languages the company is using in its coding.

### Interfaces and Abstract Classes

The specific answer to this depends on the language, but some general definitions are:

-   An _interface_ declares a set of related methods, outside of any class.
-   An _abstract class_ is an incomplete class definition that declares but does not define all its methods.

Conceptually, then, an interface defines an _application programming interface_ (_API_) that is independent of any class hierarchy. Interfaces are particularly important in languages that support only single inheritance, in which classes can inherit only from one base class. A class that defines—either directly or via inheritance—all the methods described in an interface is said to _implement_ the interface.

Unlike an interface, an abstract class is a proper class: it can have data members and method definitions and can be a subclass of other classes. Unlike a concrete (nonabstract) class, some of its behaviors are deliberately left to be defined by its own subclasses. Abstract classes cannot be instantiated because of this—only instances of concrete subclasses can be created.

An interface is equivalent to an abstract class with no data members and no method definitions. In C++ this is how you define an interface: by declaring a class with no data members and only pure virtual functions. For example:

```
class StatusCallback {
```

A class implements the interface by deriving from it and providing a definition for the methods:

```
class MyClass : SomeOtherClass, StatusCallback {
```

In Java, an interface is defined using the `interface` keyword:

```
public interface StatusCallback {
```

The interface is then implemented by a class:

```
public class MyClass implements StatusCallback {
```

A common pattern you see with languages that support both interfaces and abstract classes is the provision of a _default implementation_ of an interface via an abstract class. For example, the following interface:

```
public interface XMLReader {
```

might have a default implementation that provides a definition for only some of its inherited methods:

```
public abstract class XMLReaderImpl implements XMLReader {
```

A programmer who wants to implement `XMLReader` would then have the option to create a class that subclasses `XMLReaderImpl` and implement only one method instead of two.

In general, abstract classes are useful when the classes derived from them are more specific types of the base class (they have an _is-a_ relationship), particularly when there’s some shared functionality (for example, data members or method definitions) in the abstract base class that derived classes can use. Interfaces are useful when unrelated classes need to provide a common way to invoke conceptually related functionality, but the implementation of this functionality can vary widely from class to class.

### Virtual Methods

In OOP, child classes can override (redefine) methods defined by ancestor classes. If the method is virtual, the method definition to invoke is determined at run time based on the actual type (class) of the object on which it is invoked. Nonstatic, nonprivate Java methods are virtual unless declared final. Methods declared final cannot be overridden, so in Java there is no need to select which definition of a nonvirtual method to invoke, since there can only be one. In C# and C++, methods are only virtual when declared with the `virtual` keyword—nonvirtual methods are the default. If the method is not virtual, the method definition invoked is determined at _compile time_ based on the type of the reference (or pointer).

Some examples may be helpful to illustrate this. Consider the following three C++ classes:

```
class A {
```

Because `print` is not virtual, the method invoked depends on the type used at compile time:

```
A *a = new A();
```

If `print` is declared virtual instead:

```
class A {
```

The _runtime type_ of the object determines which method definition is invoked:

```
A *a = new A();
```

Virtual methods are used for polymorphism. They allow a single method call to invoke different method definitions based on the class of the object. A C++ version of the `Shape` class defined at the beginning of the chapter would need to declare the `draw` method as virtual for the `paintShapes` method—which accesses the objects as `Shape` references—to work.

One special type of virtual method in C++ is a _pure virtual method_: a method declared but explicitly not defined. (It is actually possible for a C++ class to declare a pure virtual method and also define it, but the definition can be called only from a derived class. When it comes to complexity, C++ never disappoints.) Any class that contains a pure virtual method or inherits one without redefining it is an abstract class. (In Java or C#, the equivalent to a pure virtual method is an abstract method.)

Virtual methods aren’t free. It (almost always) takes longer to invoke a virtual method because the address of the appropriate method definition must be looked up in a table before it is invoked. This table also requires a small amount of extra memory. In most applications, the overhead associated with virtual methods is so small as to be negligible.

### Multiple Inheritance

In C++ a class can inherit (directly or indirectly) from more than one class, which is referred to as _multiple inheritance_. C# and Java, however, limit classes to _single inheritance_—each class inherits from a single parent class.

Multiple inheritance is a useful way to create classes that combine aspects of two disparate class hierarchies, something that often happens when using different class frameworks within a single application. If two frameworks define their own base classes for exceptions, for example, you can use multiple inheritance to create exception classes that can be used with either framework.

The problem with multiple inheritance is that it can lead to ambiguity. The classic example is when a class inherits from two other classes, each of which inherits from the same class:

```
class A {
```

In this example, the `flag` data member is defined by class `A`. But class `D` descends from class `B` and class `C`, which both derive from `A`, so in essence _two copies_ of `flag` are available because two instances of `A` are in `D`’s class hierarchy. Which one do you want to set? The compiler will complain that the reference to `flag` in `D` is ambiguous. One fix is to explicitly disambiguate the reference:

```
B::flag = nflag;
```

Another fix is to declare `B` and `C` as _virtual base classes_, which means that only one copy of `A` can exist in the hierarchy, eliminating any ambiguity.

Other complexities exist with multiple inheritance, such as the order in which the base classes are initialized when a derived object is constructed, or the way members can be inadvertently hidden from derived classes. To avoid these complexities, some languages restrict themselves to the simpler single inheritance model. Although this does simplify inheritance considerably, it also limits its usefulness because only classes with a common ancestor can share behaviors. Interfaces mitigate this restriction somewhat by allowing classes in different hierarchies to expose common interfaces even if they’re not implemented by sharing code.

### Resource Management

At first the solution to this may seem simple: all you have to do is call `closeResource` at the end of the function before you return. But what if the function has more than one return statement? You can still make this approach work by adding a call to `closeResource` before every return statement, but this starts to look like a less desirable solution. You’re now duplicating code at every exit point. This makes maintenance more difficult and error prone, and creates the potential for someone to add a new return statement to the function at a later date and forget to include the call to `closeResource`.

This solution is inelegant but workable in code that doesn’t employ exceptions, but if exceptions are used, every statement is potentially an exit point from the routine and a different approach is required.

The nature of the different approach depends on the language you’re using. In languages like Java that have `finally` blocks and do not deterministically destroy objects, the best solution is to put the call to `closeResource` in a `finally` block. This ensures that `closeResource` will always be called and the resource will not be leaked, regardless of how or where the routine exits. Your first inclination may be to wrap the entire body of the function in the `try` block that corresponds to the `finally`; consider whether cases exist where that might be problematic. What if an exception is thrown by `openResource` (for example, no resources are available)? If the call to `openResource` is within the `try` block, flow will transfer to the `finally` block, which will call `closeResource` on a null reference because the resource was never successfully opened. Depending on the API, this may cause errors or unpredictable behavior. To avoid closing a resource you never opened, call `openResource` immediately before opening the `try` block, and wrap the rest of the routine in the `try` block. An implementation using this strategy may look like:

```
public static void useResource () {
```

A different strategy is necessary in C++, which doesn’t have (or doesn’t need) `finally` blocks. What guarantees does C++ make when you exit a function? Whenever you exit a function, whether by a return statement or because of an exception, all of the automatic (local) objects that pass out of scope are destroyed. How might you use this to ensure that you avoid leaking resources?

You can create a class to wrap the resource. If you call `openResource` in the constructor and `closeResource` in the destructor, you can use the life cycle of the object to manage the resource. All you have to do is remember to declare the object as a local object on the stack so that it will be automatically destroyed. As discussed previously, there should be only one call to `closeResource` for each call to `openResource`. Consider the cases where the constraint might be violated. If an instance of the wrapper class were copied, then both objects would wrap the same resource handle, and each object would try to release the handle when the object was destroyed. One way to guard against this is to declare private copy constructor and assignment operators to prevent the object from being duplicated. One implementation of this strategy is as follows:

```
class Resource {
```

Whether this is more or less complex than the preceding Java implementation depends on your perspective and how many places in your code you need to use the resource. This approach requires declaration of a wrapper class; if you’re only going to use the resource in one place, this is probably more complex and difficult than the `try`/`finally` approach in Java. On the other hand, particularly in large codebases, you may need to use the resource in multiple places. With the Java approach, you’d have to duplicate the logic of the `try`/`finally` blocks in every function that uses the resource. This is repetitive and introduces potential for error in each function that uses the resource. In contrast, the C++ approach represents all of the necessary logic in one place in the wrapper class, so there’s no duplication and the code that uses the resource is simple and clean.

The pattern embodied by this C++ approach is commonly called _Resource Acquisition Is Initialization_, or _RAII_, and is the preferred way to manage resources in C++. Wrapper classes for commonly used resources are available in the standard library. For instance, a very common resource that needs management is a block of dynamically allocated memory. `std::unique_ptr` wraps a pointer to dynamically allocated memory to ensure that the memory is deallocated when the pointer is destroyed.

Just as the `try/finally` approach can’t be implemented in C++, the RAII approach isn’t really possible in Java. You might be tempted to use a Java finalizer in place of the C++ destructor, but this doesn’t work reliably. RAII relies on deterministic, immediate destruction of automatic objects when they go out of scope, which ensures that resources are released as soon as they’re no longer accessible through their wrapper objects. Java makes no guarantees about when garbage collection and finalization will occur, so relying on a Java finalizer to release resources risks running out of resources because they’re all being held by objects awaiting finalization. In partial recognition of the usefulness of RAII, Java 1.7 added `try`\-with-resources, a language feature that allows resources to be acquired as part of a `try` statement and ensures that they are closed when the `try` block exits. `try`\-with-resources is more limited than RAII in C++: closing the resource is accomplished by calling a `close` method rather than destroying the object. Because of this, resource-wrapping objects must implement the `AutoCloseable` interface to ensure that the `close` method is available, and there’s no protection against the resource being released more than once.

C# is very similar to Java in terms of resource management. Like Java, it provides `try/finally` functionality, but lacks the ability to do true RAII because it does not have deterministic destruction of automatic objects. `using` is the C# equivalent of Java’s `try`\-with-resources; C# wrapper classes must implement the `IDisposable` interface to use this language feature.

## SUMMARY

Object-oriented programming languages are in widespread use today, so a firm understanding of basic OOP principles is necessary for most jobs.

Be sure you understand how each programming language you use handles the different aspects of OOP.
