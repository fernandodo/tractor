---
created: 2025-09-23T20:39:43 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 10 Concurrency | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> Not that long ago, it was common for programs to have a single thread of execution, even if they were running on a multithreading system. Even today, code you write for an application or web server is often single-threaded, even if the server itself is multithreaded. Why? Because multithreaded programming (often referred to as concurrency) is hard to do correctly, even when the programming language directly supports it. Incorrect use of threads can easily halt your program’s execution or corrupt its data; worse yet, it can lead to intermittent, difficult to reproduce bugs.

---
## 10  
Concurrency

Not that long ago, it was common for programs to have a single thread of execution, even if they were running on a multithreading system. Even today, code you write for an application or web server is often single-threaded, even if the server itself is multithreaded. Why? Because multithreaded programming (often referred to as _concurrency_) is hard to do correctly, even when the programming language directly supports it. Incorrect use of threads can easily halt your program’s execution or corrupt its data; worse yet, it can lead to intermittent, difficult to reproduce bugs.

However, if you write an application that has a graphical user interface that may perform lengthy operations, you probably need to use threads to maintain a responsive interface. Even noninteractive applications use threads: increases in processing power these days come mostly in the form of additional cores, which a single-threaded application can’t take advantage of. Thread-related issues can appear even in environments that don’t explicitly support threads, such as JavaScript programs doing AJAX-style operations, because the web server responses are processed asynchronously, and hence the JavaScript that runs to process the response may access data that is simultaneously used by other parts of the application. That’s why good programmers take the time to learn how to write multithreaded programs correctly.

## BASIC THREAD CONCEPTS

Multithreaded programming is substantially more complex than programming with a single thread. Creation and destruction of threads must be managed; most of the complexity stems from coordinating access to resources shared between the threads. When access is not appropriately controlled, multithreaded programs can exhibit several classes of bugs not encountered in single-threaded applications.

### Threads

A _thread_ is the fundamental unit of execution within an application: a running application consists of at least one thread. Each thread has its own stack and runs independently from the application’s other threads. By default, threads share their resources, such as file handles or memory. Problems can occur when access to shared resources is not properly controlled. Data corruption is a common side effect of having two threads simultaneously write data to the same block of memory, for example.

Threads can be implemented in different ways. On most systems, threads are created and managed by the operating system; these are called _native threads_ or _kernel-level threads_. Sometimes the threads are implemented by a software layer above the operating system, such as a virtual machine; these are called _green threads_. Both types of threads have essentially the same behavior. Some thread operations are faster on green threads, but they typically cannot take advantage of multiple processor cores, and implementation of blocking I/O is difficult. As multicore systems have become prevalent, most virtual machines have shifted away from green threads. The remainder of this chapter assumes that the threads are native threads.

Because the number of threads that can be executed at any given instant is limited by the number of cores in the computer, the operating system rapidly switches from thread to thread, giving each thread a small window of time to run. This is known as _preemptive threading_, because the operating system can suspend a thread’s execution at any point to let another thread run. (A _cooperative model_ requires a thread to explicitly take some action to suspend its own execution and let other threads run.) Suspending one thread so another can start to run is referred to as a _context switch_.

### System Threads versus User Threads

A system thread is created and managed by the system. The first (main) thread of an application is a system thread, and the application often exits when the first thread terminates. User threads are explicitly created by the application to do tasks that cannot or should not be done by the main thread.

Applications that display user interfaces must be particularly careful with how they use threads. The main thread in such an application is usually called the _event thread_ because it waits for and delivers events (such as mouse clicks and key presses) to the application for processing. Generally speaking, making the event thread unavailable to process events for any length of time (for instance, by performing lengthy processing in this thread or making it wait for something) is considered bad programming practice because it leads to (at best) an unresponsive application or (at worst) a frozen computer. One way to avoid these issues is to create threads to handle potentially time-consuming operations, especially those involving network access. These user threads often communicate data back to the event (main) thread by queueing events for it to process; this allows the event thread to receive data without stopping and waiting or wasting resources by repeatedly polling.

### Monitors and Semaphores

Applications must use _thread synchronization_ mechanisms to control threads’ interactions with shared resources. Two fundamental thread synchronization constructs are monitors and semaphores. Which you use depends on what your system or language supports.

A _monitor_ is a set of routines protected by a mutual exclusion lock. A thread cannot execute any of the routines in the monitor until it acquires the lock, which means that only one thread at a time can execute within the monitor; all other threads must wait for the currently executing thread to give up control of the lock. A thread can suspend itself in the monitor and wait for an event to occur, in which case another thread is given the chance to enter the monitor. At some point the suspended thread is notified that the event has occurred, allowing it to awake and reacquire the lock at the earliest possible opportunity.

A _semaphore_ is a simpler construct: just a lock that protects a shared resource. Before using a shared resource, the thread is supposed to acquire the lock. Any other thread that tries to acquire the lock to use the resource is blocked until the lock is released by the thread that owns it, at which point one of the waiting threads (if any) acquires the lock and is unblocked. This is the most basic kind of semaphore—a mutual exclusion, or _mutex,_ semaphore. Other semaphore types include _counting semaphores_ (which let a maximum of _n_ threads access a resource at any given time) and _event semaphores_ (which notify one or all waiting threads that an event has occurred).

Monitors and semaphores can be used to achieve similar goals, but monitors are simpler to use because they handle all details of lock acquisition and release. When using semaphores, each thread must be careful to release every lock it acquires, including under conditions in which it exits the code that needs the lock unexpectedly (for example, due to an exception); otherwise, no other thread that needs the shared resource can proceed. In addition, you need to ensure that every routine that accesses the shared resource explicitly acquires a lock before using the resource, something that can be easy to miss because it’s not typically enforced by the compiler. Monitors automatically acquire and release the necessary locks.

Most systems provide a way for the thread to time out if it can’t acquire a resource within a certain amount of time, allowing the thread to report an error and/or try again later.

Thread synchronization doesn’t come for free: it takes time to acquire and release locks whenever a shared resource is accessed. This is why some libraries include both thread-safe and non–thread-safe classes, for instance `StringBuffer` (thread-safe) and `StringBuilder` (non–thread-safe) in Java. In general, prefer the non–thread-safe versions for their improved performance and use the thread-safe versions only when necessary.

### Deadlocks

Consider the situation in which two threads block each other because each is waiting for a lock that the other holds. This is called a _deadlock_: each thread is permanently stalled because neither can continue running to the point of releasing the lock that the other needs.

One typical scenario in which this occurs is when two processes each need to acquire two locks (A and B) before proceeding but attempt to acquire them in different orders. If process 1 acquires A, but process 2 acquires B before process 1 does, then process 1 blocks on acquiring B (which process 2 holds) and process 2 blocks on acquiring A (which process 1 holds). A variety of complicated mechanisms exist for detecting and breaking deadlocks, none of which are entirely satisfactory. In theory the best solution is to write code that cannot deadlock—for instance, whenever it’s necessary to acquire more than one lock, the locks should always be acquired in the same order and released in reverse order. In practice, it becomes difficult to enforce this across a large application with many locks, each of which may be acquired by code in many different places.

### A Threading Example

A banking system provides an illustration of basic threading concepts and the necessity of thread synchronization. The system consists of a program running on a single central computer that controls multiple automated teller machines (ATMs) in different locations. Each ATM has its own thread so that the machines can be used simultaneously and easily share the bank’s account data.

The banking system has an `Account` class with a method to deposit and withdraw money from a user’s account. The following code is written as a Java class but the code is almost identical to what you’d write in C#:

```
public class Account {
```

Suppose a husband and wife, Ron and Sue, walk up to different ATMs to withdraw $100 each from their joint account. The thread for the first ATM deducts $100 from the couple’s account, but the thread is switched out after executing this line:

```
newBalance = userBalance – amount;
```

Processor control then switches to the thread for Sue’s ATM, which is also deducting $100. When that thread deducts $100, the account balance is still $500 because the variable, `userBalance`, has not yet been updated. Sue’s thread executes until completing this function and updates the value of `userBalance` to $400. Then, control switches back to Ron’s transaction. Ron’s thread has the value $400 in `newBalance`. Therefore, it simply assigns this value to `userBalance` and returns. Thus, Ron and Sue have deducted $200 total from their account, but their balance still indicates $400, or a net $100 withdrawal. This is a great feature for Ron and Sue, but a big problem for the bank.

Fixing this problem is trivial in Java. Just use the `synchronized` keyword to create a monitor:

```
public class Account {
```

The first thread that enters either `deposit` or `withdraw` blocks all other threads from entering either method. This protects the `userBalance` class data from being changed simultaneously by different threads. The preceding code can be made marginally more efficient by having the monitor synchronize only the code that uses or alters the value of `userBalance` instead of the entire method:

```
public class Account {
```

In fact, in Java a synchronized method such as:

```
synchronized void someMethod(){
```

is exactly equivalent to:

```
void someMethod(){
```

The `lock` statement in C# can be used in a similar manner, but only within a method:

```
void someMethod(){
```

In either case, the parameter passed to `synchronize` or `lock` is the object to use as the lock.

Note that the C# `lock` isn’t as flexible as the Java `synchronized` because the latter allows threads to suspend themselves while waiting for another thread to signal them that an event has occurred. In C# this must be done using event semaphores.

## CONCURRENCY PROBLEMS

Issues that you encounter with threads in professional development can be Byzantine in their complexity, and concise thread problems appropriate for an interview are difficult to compose. Therefore, the questions you get are likely to come from a fairly small set of classic thread problems, several of which are presented here.

### Busy Waiting

This is a simple problem, but one with important performance implications for any multithreaded application.

Consider a thread that spawns another thread to complete a task. Assume that the first thread needs to wait for the second thread to finish its work, and that the second thread terminates as soon as its work is done. The simplest approach is to have the first thread keep checking whether the second thread is alive and proceed as soon as it is dead:

```
Thread task = new TheTask();
```

This is called _busy waiting_ because the waiting thread is still active, but it’s not actually accomplishing anything. It’s “busy” in the sense that the thread is still executed by the processor, even though the thread is doing nothing but waiting for the second thread to finish. Typically there are more active threads than cores, so this actually “steals” processor cycles away from the second thread (and any other active threads in the system), cycles that could be better spent doing real work.

Busy waiting is avoided by using a monitor or a semaphore, depending on what’s available to the programmer. The waiting thread simply sleeps (suspends itself temporarily) until the other thread notifies it that it’s done. In Java, any shared object can be used as a notification mechanism:

```
Object theLock = new Object();
```

In this case, because `TheTask` terminates after it completes its task, the first thread could also sleep until it completes using `join()`, but `wait()` and `notify()` provide a more general approach that isn’t dependent on thread termination. The preceding code can be simplified somewhat by using the thread object itself for the signaling:

```
Thread task = new TheTask();
```

Very few circumstances exist where _spinlocks,_ a form of busy waiting, are actually desirable. If you can guarantee that the resource you’re waiting for will be released in less time than it would take to acquire a conventional lock (a situation often encountered in kernel programming), it may be more efficient to use a spinlock that busy waits for this short period of time.

Another case where spinlocks are useful is _high-performance computing_ (_HPC_), where the entire system is dedicated to a single application and exactly one compute thread is created per core. In this scenario, if one thread is waiting on data from a second thread running on a different core, there’s no useful work that can be performed on the first thread’s core until the data arrives, so there’s no downside to wasting compute cycles by busy waiting. The time between data arrival and the process proceeding past the lock is often less for a spinlock than a semaphore, so under these specific circumstances an application using spinlocks may have better performance than one using semaphores. In any case, appropriate use of spinlocks requires careful assembly coding (to ensure that the attempts at lock acquisition are atomic); busy waiting should always be avoided in high-level languages.

### Producer/Consumer

This is one of the canonical concurrency problems. The first step is to answer the problem without using any concurrency control, and then comment on what the problems are. The algorithm isn’t difficult when concurrency isn’t an issue. The data buffer looks like this:

```
public class IntBuffer {
```

The producer and consumer are almost trivial:

```
public class Producer extends Thread {
```

Then, somewhere in the code you start the threads:

```
IntBuffer b = new IntBuffer();
```

This approach has two problems, however. First, it uses busy waiting, which wastes a lot of CPU time. Second, there is no access control for the shared resource, the buffer. If a context switch occurs as the index is being updated, the next thread may read from or write to the wrong element of the buffer.

You may think at first that making the `add` and `remove` methods synchronized fixes the problem:

```
public class IntBuffer {
```

This actually creates an even worse problem. `add` and `remove` still busy wait when the buffer is full or empty (respectively). When a thread is busy waiting in `add`, no thread can enter `remove` (because the methods are now synchronized), so the buffer remains full forever. A similar problem is encountered if `remove` is called when the buffer is empty; the first time either of these situations is encountered, the application locks up in an infinite busy wait loop. The code inside the methods needs to be changed so that the producer suspends itself when the buffer is full and waits for a slot to open up, and the consumer suspends itself if the buffer is empty and waits for a new value to arrive:

```
public class IntBuffer {
```

This code actually allows multiple producers and consumers to use the same buffer simultaneously, so it’s even more general-purpose than the two-thread-only solution you’d be expected to come up with.

### The Dining Philosophers

This is another concurrency classic, and although it may seem quite contrived—in the real world no one would starve because each philosopher would simply ask the adjacent philosophers for their forks—it accurately reflects real-world concurrency issues involving multiple shared resources. The point of the problem is to see whether you understand the concept of deadlock and know how to avoid it.

A naïve approach would be to have each philosopher wait until the left fork is available, pick it up, wait until the right fork is available, pick that fork up, eat, and then put down both forks. The following code implements this in Java using a separate thread for each philosopher:

```
public class DiningPhilosophers {
```

What will happen when you run this code? It’s not entirely deterministic because you don’t know exactly when the scheduler will have each thread running. (This is one of the challenges of debugging multithreaded code.) You do know that each philosopher will try to grab his or her left fork and will always hold it until he or she can pick up the right fork and eat. Any time there’s a fork on the table to the right of a philosopher who holds a left fork, you have a _race condition_ that determines whether that philosopher gets the fork or the philosopher to his or her right picks it up. In the latter case, you have two philosophers with only left forks, and the first philosopher will have to wait until after the second eats before getting another shot at the fork. This will tend to lead to a lot of philosophers hungrily sitting around the table holding forks in their left hands.

At some point you would expect to reach a situation where four of the five philosophers have forks in their left hands and only one fork remains on the table. (In practice, this is reached fairly quickly.) If this last fork is picked up as a right-handed fork, that philosopher eats, puts down both forks, and life goes on. If instead it’s picked up as a left-handed fork, then each philosopher has one fork that cannot be released until the philosopher to the right gets a second fork and eats. Because the philosophers are seated around a circular table, this will never happen, so you have five soon-to-be-dead philosophers in a deadlock. (Somewhat more formally: when each philosopher has a left fork, by induction, a given philosopher can’t get the right fork until after putting down the left fork but is required to get the right fork before putting down the left fork, so nothing happens.)

How can you avoid this deadlock? One solution is to add a timeout to the waiting: if a philosopher is not able to eat within a predetermined amount of time after acquiring the first fork, then the philosopher drops the fork and tries again. This doesn’t actually solve the problem, though: it may get some philosophers eating, but it doesn’t stop them from reaching a deadlock. Worse, there’s no way to know exactly which philosophers will get to eat—you could have a situation in which the interactions of the timeouts and the scheduler is such that some philosophers starve because they _never_ acquire both forks. This is referred to as _livelock_.

Perhaps there’s a better solution that can avoid reaching deadlock in the first place. Deadlock occurs when each of the philosophers holds one fork in his or her left hand. What if one of the philosophers went for the right fork first? Then that philosopher would never hold just a left-hand fork (because he or she has to pick up a right fork first), so there’s no way to reach the all-left-forks deadlock condition. Another way to look at this is in terms of the order in which the forks are acquired. You know that deadlocks often result from locks (forks) being acquired in different orders. If you number each of the philosophers and forks counterclockwise around the table, then under the left-fork-first strategy, each philosopher tries to pick up first a lower numbered fork and then a higher numbered fork. This is true of every philosopher except for the last, who has fork _n_ – 1 on the left and fork 0 on the right. Reversing the left-right order of acquisition for this philosopher means that all the philosophers acquire forks in the same order from a global perspective: lower number first. You can implement this with a change to the constructor that changes the order in which one of the philosophers picks up forks:

```
    // Prepare the forks and philosophers
```

This solution avoids deadlock and would likely be adequate for most interviews, but it can be improved. Under the current implementation, each philosopher will eat, but will they all get an equal opportunity? Consider the philosopher sitting to the left of the right-hand-first philosopher (index number 3 in the preceding implementation, which represents the fourth philosopher). This philosopher is in the unique position that neither neighbor takes one of his or her forks as a first fork; as a result it’s much easier for him or her to get forks, and he eats like a philosopher king (or queen). On the other hand (literally), the right-hand-first philosopher pays the price, frequently waiting for the series of left-fork wielding philosophers to the right to eat and put down their forks. The exact ratio of number of times that the lucky philosopher eats to the number of times the unlucky philosopher beside him or her does will vary by system, but in informal tests of five philosophers on our machines, philosopher 3 eats about a hundred times more frequently than philosopher 4.

How can you make mealtimes fairer? You’ll want to preserve an ordering of the forks to avoid deadlocks. Consider whether you need to have all the forks ordered. A philosopher holds at most two forks, so you just need a rule that defines the order for each philosopher for acquisition of a maximum of two forks. One such rule would be that each philosopher must pick up an odd numbered fork before an even numbered fork. (If—as in this problem—you have an odd number of philosophers, then philosopher _n_ sits between two even numbered forks: _n_ – 1 and 0. It doesn’t matter in which order this philosopher picks up forks because he or she is the only one picking up two even forks.) A constructor that configures the philosophers under this scheme looks like:

```
    // Prepare the forks and philosophers
```

This approach is completely fair for any even number of philosophers. For an odd number of philosophers, there’s still a “lucky” philosopher. Although it’s not completely fair in this case, it’s a marked improvement for five philosophers: the lucky philosopher eats only about ten times more often than the least lucky one. In addition, as the number of philosophers at the table increases, this approach becomes increasingly fair while the single right-hand-first philosopher algorithm becomes increasingly unfair.

## SUMMARY

Using multiple threads of execution within an application can make it more responsive and allow it to take full advantage of a multicore system but also makes programming more complicated. Synchronization is required to avoid data corruption whenever multiple threads access shared resources.

Synchronization is often achieved using monitors or semaphores. These facilities enable applications to control access to shared resources and to signal other threads when the resources are available for processing. Misuse of these constructs can halt threads through deadlock. Writing quality multithreaded code that avoids both data corruption and deadlock is challenging, requiring care and discipline.
