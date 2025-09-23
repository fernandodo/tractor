---
created: 2025-09-23T20:38:44 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 3 The Phone Screen | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> 3The Phone Screen

 The first set of programming problems you’ll encounter will likely be in a technical phone interview, also known as a phone screen. Phone screens are designed to weed out...

---
The first set of programming problems you’ll encounter will likely be in a technical phone interview, also known as a _phone screen_. Phone screens are designed to weed out unqualified candidates who aren’t worth the time investment of in-person technical interviews. If you don’t pass the phone screens (there may be multiple), you won’t be invited to the on-site interviews.

## UNDERSTANDING PHONE SCREENS

Phone screens are all about determining if a candidate has the basic technical knowledge and experience required to work at the company. It’s easy for candidates to inflate their skills and experience to try to make it past the initial résumé screenings. For example, it’s not uncommon for candidates to claim proficiency in every programming language they’ve ever used, even those used only once for a school assignment. Some candidates go beyond exaggeration: they just lie. Inviting someone to on-site technical interviews only to discover that they really don’t have the basic skills needed for the position is a waste of everyone’s time—especially the software engineers conducting the interviews. Phone screens are used to avoid these scenarios.

### Phone Screens by Software Engineers

In general, the highest quality phone screens are those given by software engineers. These screens are typically a mix of knowledge-based questions and some basic coding and design tasks. (Don’t be surprised or offended by the apparent simplicity of some of the questions. Remember, they’re designed to quickly weed out the exaggerators and liars.)

Knowledge-based questions are common during on-site interviews as well as phone screens. These types of questions are covered in-depth in [Chapter 18](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c18.xhtml). As discussed in detail in that chapter, knowledge-based questions are frequently based on two sources: items on your résumé and concepts you reference in answers to other questions. Make sure you’re prepared to discuss everything on your résumé, and don’t use terms or concepts you don’t really understand in answers to problems.

Along with testing your knowledge, a good interviewer will also give you programming tasks to solve. As a rule, these questions will be straightforward coding questions that can be solved in a short amount of time. At this stage, the interviewer wants to see whether or not you can actually code; detailed evaluation of the quality of your code is usually saved for the on-site interview. They may ask you additional knowledge-based questions based on the code you write. Particularly if you’re an experienced software engineer, they may also ask you open-ended design questions.

Programming problems should be approached as discussed in [Chapter 4](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c04.xhtml), but because the problems encountered in phone screens are typically simpler than those found in on-site interviews, you’ll generally solve them faster and may not need so many problem solving techniques.

### Phone Screens by Nontechnical People

A software engineer’s time is valuable, so some companies use nontechnical interviewers (typically recruiters) or automated testing systems to do their screening. This unfortunately puts you at a disadvantage when compared to interviewing with a software engineer, because the interview generally follows a rigid format with little or no chance for you to explain your answers or to resolve ambiguities in the questions asked. If you _are_ dealing with a human, be sure to point out problems and trade-offs with questions if you encounter any ambiguities or things that just don’t make sense. If possible, ask the person to take notes beyond just recording whether or not the answers you gave match the possibilities on their checklist of acceptable answers. If you’re lucky, the interviewer will raise these questions with someone more knowledgeable and score you accordingly.

When you are being screened by someone nontechnical, you should adjust your strategy accordingly. You will usually realize that you’re in this situation when it sounds like your screeners are reading you a quiz, and they don’t seem to have much understanding of what they’re reading. Each question has a correct answer, and the person you’re speaking with has no ability to interpret your answers beyond determining whether they’re an exact match to the “correct” answer on the answer sheet. In these circumstances, it’s important to answer questions so that you maximize your chances of covering whatever answer they have been provided. For example, if you’re asked what value the C function `malloc` returns if it’s unable to allocate memory and you answer, “0x0”—the interviewer may say, “no, it’s the null pointer” (or worse, silently mark you wrong). Any attempt to explain after the fact that these answers are equivalent will likely fall on deaf ears. When you’re interviewing with a nontechnical recruiter, it’s important to answer questions with all possible equivalent or synonymous answers, for instance, “It’s zero, which is 0x0 in hexadecimal, usually referred to as a null pointer.”

If you see several possible solutions to a problem and the person can’t help you narrow down which answer is “correct,” list the solutions with an explanation of why each one is correct for a given scenario: if one of your answers is in fact the “correct” one, the interviewer will usually take it and move on to the next question. For example, you may be asked, “What is the fastest sorting algorithm?” As part of an interview with a knowledgeable engineer, the best answer is generally a fairly lengthy and nuanced discussion of the relative merits of several different algorithms depending on what is being sorted, and the specifics of the performance requirements for the sort. However, a nontechnical person just wants to hear a single, succinct answer, like “quicksort.” You may be frustrated if the person answers, “I just need a single word, the name of the fastest algorithm.” However, always be polite and don’t express disbelief or frustration at the questions being asked. You’ll never get a job by trying to convince the screener that their process is wrong.

Automated screenings are in some ways simpler. When you’re asked a question, you’ll normally get to choose between several answers. If two or more of the answers could be correct, choose the simplest one, but write down the question and why it’s ambiguous for later discussion with a recruiter. If you’re asked to write some code, make sure it’s syntactically correct and does what the problem requires, including covering edge cases and error conditions. If the code doesn’t run and pass some basic testing, chances are the system will reject it outright. Be sure to add comments to the code explaining what you’re doing so that anyone reviewing the code later understands what you were trying to accomplish.

## HOW TO TAKE A PHONE SCREEN

In a typical phone screen, you don’t see the interviewer, you just hear them, so the interview process has a different feel than an on-site interview. Here are some tips on how to take a phone screen:

-   **Prepare the environment.**
    -   You’ll want to take the interview in a quiet space with a computer and no distractions.
    -   Be sure to have pen and paper available for note taking.
    -   If the interview is conducted via a videoconferencing or remote interviewing system, make sure you’ve installed and tested the required software ahead of time.
    -   A phone with a headset or speakerphone capabilities (any mobile phone will work) is a must—your hands need to be free for typing and note taking. (The phone can also be a backup method of communication if the videoconferencing audio fails.)
-   **Block off time for the interview.**
    -   Most phone screens are 15 to 45 minutes long.
    -   Clear your schedule immediately before and after the interview.
    -   Be ready 10 to 15 minutes before the scheduled start time.
    -   By booking extra time, you won’t feel pressured if the interview is running long.
-   **Speak loudly and clearly.**
    -   At the beginning of the interview make sure the interviewer can hear you clearly and that you also hear them clearly.
    -   Tell the interviewer what you’re thinking as you work through the questions, and let them know if you need a few minutes of quiet to think about a problem.
    -   Ask clarifying questions as necessary.
-   **Be polite.**
    -   Keep a positive, pleasant, and respectful tone throughout the interview, even if the questions seem too simple or poorly written.
    -   At the end of the interview be sure to thank the interviewer.
    -   If there’s time, ask them specific questions about what it’s like to work for the company and (if the interviewer is a software engineer) the projects they work on.

One final tip: if you’re unwell or your schedule changes, ask the recruiter to move the interview to another date. Rescheduling a phone screen is a lot easier than rescheduling on-site interviews, so make sure you’re at your best before taking the phone screen.

## PHONE SCREEN PROBLEMS

The screening problems that follow are representative of the simple knowledge-based and coding questions you’re likely to encounter in a phone screen, but are certainly not an exhaustive list. If you do well with this type of question—by showing you can do basic coding—the interviewer may move on to more complex problems.

### Memory Allocation in C

This is an example of a question where you’ll want to adjust your response based on the technical understanding of the person asking you. The most common way to allocate memory in C is by calling `malloc`. If you’re speaking with a nontechnical person doing the phone screen, that’s probably as far as you go. If the screener has a technical background (or you’re not sure yet whether they do), you might go on from there to discuss trade-offs between dynamic and static allocation, less common calls into the standard C memory manager like `calloc` and `realloc`, and possibly special circumstances under which you might want to use a custom memory manager.

### Recursion Trade-Offs

This is a poorly formed question that you would probably only get from a nontechnical screener. Recursion is a technique. It has trade-offs. A better wording would have been, “What are the drawbacks of using recursion?” It doesn’t matter if you dislike the question; you still need to make your best effort to answer it. The answer written on the key that the nontechnical screener is using likely has some of the drawbacks of recursion. Make sure that you hit as many drawbacks as you can to cover all the possibilities. You might say something like: “Recursion involves repeated function calls, each of which has overhead in both time and stack space. Many people find recursion confusing, which may make a recursive function harder to document, debug, and maintain.”

### Mobile Programming

This is a question that involves a lot of unstated assumptions, including what constitutes a “normal” computer. If you are dealing with a nontechnical screener, you’re not likely to get much clarification on these assumptions. Again, it’s important to list all of the possible differences, so that you hopefully manage to hit the answer on the sheet.

Mobile programming has several differences from nonmobile programming. Mobile devices generally use mobile-specific operating systems like Android or iOS. These operating systems have different paradigms for filesystem access, memory access, and inter-application communication than desktop or server operating systems. Mobile programming requires more careful attention to power consumption, and storage and network bandwidth are often more limited. Network connectivity may be intermittent and available bandwidth may vary over a wide range. Most mobile devices use a touch screen and a microphone as their primary input devices, so user interface design centers on optimizing usage of small screens, finger-friendly widgets, gestures, voice recognition, and minimizing text input, which is inconvenient on a soft (screen-based) keyboard. Mobile devices have more consistent availability of resources like accelerometers, GPS location services, notifications, and contacts databases than nonmobile devices; access to these may be limited by a permissions model. Application deployment typically occurs exclusively or almost exclusively through an online application store.

### FizzBuzz

This simple task is based on a children’s game for teaching division in the United Kingdom. It was first proposed as a screening problem by Imran Ghory and popularized by coders like Jeff Atwood and Joel Spolsky. All that’s really needed to complete the task is an understanding of `for` loops and modulo arithmetic. The one potential pitfall of a problem like this is overthinking it. Any competent coder should be able to come up with a working solution to this fairly quickly, but you can get tripped up if you focus on designing a perfectly elegant solution. Particularly in a screening interview, don’t let perfect be the enemy of good. You’re much better off banging out a solid, accurate working solution quickly than spending an inordinate amount of time trying to optimize and perfect your approach before you code. If you feel like your solution is inelegant, you can mention that and see whether your interviewer directs you to work on improving it or simply moves on. A working, but not beautifully elegant solution in Java looks like:

```
for ( int i = 1; i <= 100; ++i ) {
```

### Reversing a String

This simple question reveals how well you understand basic string operations. A naive answer in Java might be:

```
public static String reverse( String in ) {
```

It works, but this is better:

```
public static String reverse( String in ) {
```

This version is more efficient and demonstrates a clear understanding of the immutability of strings in Java by avoiding construction of a new String object on each iteration of the loop (see [Chapter 7](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c07.xhtml) for more detailed discussion of this). In a screening interview, either of these solutions would probably be sufficient. If you initially offered the first solution in a screen led by a coder, you might get follow-up questions about inefficiencies in your implementation trying to see if you could come up with something more like the second solution.

### Removing Duplicates

A simple but clumsy approach is to store values in a second list, searching that list to determine if a value has already been seen. For example, in C++:

```
#include <list>
```

You can accomplish the same goal much more concisely and efficiently by converting the list to a set, which doesn’t allow duplicated values. Sets don’t preserve the original ordering of their data. That’s probably OK here, as the problem statement doesn’t require that ordering be preserved, but you should clarify that with the interviewer. If the goal is to avoid duplicates, a set may be a better choice for storing this data than a list (which is something you could discuss with a technically knowledgeable screener). However, the problem calls for a list to be returned, so you should convert the set back to a list rather than returning a set. In C++, you can convert a list to a set and vice versa by passing iterators from one data structure to the constructor of the other data structure. An implementation would look like:

```
#include <list>
```

Again, because the intent in screening interviews is generally to see whether or not you can code, the first solution might be considered sufficient, but the second is clearly superior if the list ordering doesn’t matter, and once you come up with the idea of using a set, it’s easier to write, too.

### Nested Parentheses

You know that in a correctly nested string, there will be an equal number of left and right parentheses, so you might begin by approaching this as a counting problem. Because you don’t care about the total number of sets of parentheses, you can do this with a single variable that tracks the relative number of left and right parentheses. Increment a counter when you see a left parenthesis, and decrement it when you see a right parenthesis. If at the end the counter is nonzero, you know you don’t have a properly nested string.

Check your solution before you start coding. Is what you’ve come up with so far sufficient? At minimum you should check the four example cases given in the problem. The counting approach would give the appropriate result on the first two correctly nested cases, as well as the first incorrectly nested case, but for the final case the final counter value would be zero, so you would erroneously conclude that the nesting was correct. How can you extend your solution to detect incorrect nesting when the number of right and left parentheses is equal?

The example of `")("` is not correctly nested because the right parenthesis comes before the left parenthesis with which it is paired. It’s not sufficient for there to be merely the same number of right and left parentheses; every right parenthesis has to come after a left parenthesis that it’s paired with. Put in terms of the counter that you’re using, it’s not sufficient for it to end at zero; it also has to never be negative. When do you need to check this? The counter can only become negative after it’s decremented, so you can just check immediately following the decrement. Implementing this in Java yields:

```
public static boolean checkNesting( String s ) {
```

## SUMMARY

Phone screens are a way for employers to screen out candidates who don’t have the required skills and experience to proceed to on-site interviews. Passing a phone screen is necessary to be invited to on-site interviews, making it a key step toward obtaining a job offer. Be sure you have a good grasp of the fundamental knowledge you’ll need for the job for which you’re applying and everything you’ve mentioned in your résumé. Prepare yourself, your schedule, and your environment before taking a phone screen.
