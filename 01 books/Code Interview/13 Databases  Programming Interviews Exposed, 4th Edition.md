---
created: 2025-09-23T20:39:55 (UTC +12:00)
tags: []
source: https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#head-2-6
author: 
---

# 13 Databases | Programming Interviews Exposed, 4th Edition

> ## Excerpt
> Tools are available to help you create and manage databases, many of which hide the complexities of the underlying data structures. Ruby on Rails, for example, abstracts all database access and makes most direct access unnecessary, as do component technologies such as Enterprise JavaBeans and many object-oriented frameworks. Still, you need an understanding of how databases work to make good design decisions.

---
## DATABASE FUNDAMENTALS

Tools are available to help you create and manage databases, many of which hide the complexities of the underlying data structures. Ruby on Rails, for example, abstracts all database access and makes most direct access unnecessary, as do component technologies such as Enterprise JavaBeans and many object-oriented frameworks. Still, you need an understanding of how databases work to make good design decisions.

### Relational Databases

Data in a relational database is stored in _tables_, which consist of _rows_ and _columns_ (also known as _tuples_ and _attributes_). A set of table definitions is referred to as a _schema_. Each column has a name and data type associated with it. The column data type limits the range of data that can be stored in the column; the column may also have additional constraints beyond those imposed by the type. Typically, the columns of a table are defined when the database is created; columns are modified infrequently (or never). Data is added and removed from a table by inserting and deleting rows. Although the columns are typically ordered, the rows aren’t. If ordering of rows is required, it is done when the data is fetched (via a _query_) from the database.

Most tables have keys. A _key_ is a column or set of columns used to identify rows in the table. One of the keys is usually designated the _primary key_. Each row in the table must have a value for the primary key, and each of these values must be unique. For example, in a table of employees, you might use the employee identification number—guaranteed to be unique for each employee—as the primary key. When the data being stored does not naturally contain guaranteed unique values that can be used as primary keys, the database is often configured to automatically assign a unique serial numbered value as the primary key for each row inserted in the table.

A table can be linked to another table using a foreign key. A _foreign key_ is a column where the values match values from a key column in the other table (usually the primary key). When every foreign key value exists as a key in the table it references, the database has _referential integrity_. This can be enforced through the use of _foreign key constraints_. Depending on how the constraints are configured, an attempt to delete a row with a key value that exists in another table as a foreign key is either prevented or causes deletion or modification of the rows that reference it.

The most common way to manipulate and query databases is through the use of _Structured Query Language_ (_SQL_). Some variations in syntax exist across different database management systems (DBMS), particularly for advanced features, but the basic syntax is fairly consistent.

### SQL

SQL is the lingua franca of relational database manipulation. It provides mechanisms for most kinds of database manipulations. Understandably, SQL is a big topic, and numerous books are devoted just to SQL and relational databases. Nevertheless, the basic tasks of storing and retrieving We begin with the following schema:

```
Player (
```

[Table 13-1](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c13-tbl-0001) shows some sample data for `Player`, and [Table 13-2](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c13-tbl-0002) shows sample `Stats` data.

**[TABLE 13-1:](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c13-tbl-0001)** Player Sample Data

| **NAME** | **NUMBER** |
| --- | --- |
| Larry Smith | 23 |
| David Gonzalez | 12 |
| George Rogers | 7 |
| Mike Lee | 14 |
| Rajiv Williams | 55 |

**[TABLE 13-2:](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c13-tbl-0002)** Stats Sample Data

| **NUMBER** | **TOTALPOINTS** | **YEAR** |
| --- | --- | --- |
| 7 | 59 | Freshman |
| 55 | 90 | Senior |
| 23 | 150 | Senior |
| 23 | 221 | Junior |
| 55 | 84 | Junior |

In this schema, neither table has a primary key defined. The `number` column in `Player` is a good candidate for a primary key because every player has a number, and the player number uniquely identifies each player. (This wouldn’t work so well if the database was used long enough that some players graduated and had their uniforms and numbers reassigned to new players.) The `number` column in the `Stats` table is a foreign key—a reference to the number column in the `Player` table. Explicitly defining these relationships in the schema makes it easier for others to understand—and the database to maintain—the relationship between these tables:

```
Player (
```

With these changes, the database takes an active role in ensuring the referential integrity of these tables. For example, you can’t add a row to the `Stats` table that references a player not listed in the `Player` table; the foreign key relationship between `Stats.number` and `Player.number` forbids this.

One fundamental SQL statement is `INSERT`, which is used to add values to a table. For example, to insert a player named Bill Henry with the number 50 into the `Player` table, you would use the following statement:

```
INSERT INTO Player VALUES('Bill Henry', 50);
```

`SELECT` is the SQL statement most commonly seen in interviews. A `SELECT` statement retrieves data from a table. For example, the statement:

```
SELECT * FROM Player;
```

returns all the values in the table `Player`:

```
+----------------+--------+
```

You can specify which columns you want to return like this:

```
SELECT name FROM Player;
```

which yields:

```
+----------------+
```

You may want to be more restrictive about which values you return. For example, if you want to return only the names of the players with numbers less than 10 or greater than 40, you would use the statement:

```
SELECT name FROM Player WHERE number < 10 OR number > 40;
```

which would return:

```
+----------------+
```

Much of the power of a relational database comes from the relationships between data in different tables, so you frequently want to use data from more than one table. For example, you may want to print out the names of all players along with the number of points that each player has scored. To do this, you have to _join_ the two tables on the `number` field. The `number` field is called a _common key_ because it represents the same value in both tables. The query is as follows:

```
SELECT name, totalPoints, year FROM Player, Stats
```

It returns this:

```
+----------------+-------------+----------+
```

Some players have played on the team for more than one year, so their names appear multiple times; others have no rows in `Stats` (apparently they’ve been warming the bench) so they don’t appear at all in the results of this query. Conceptually, when you include two tables in the `FROM` clause, the query constructs a _Cartesian product_ of the tables: a single table containing all possible combinations of rows from the first table with rows from the second table. Then the `WHERE` limits the results returned by the query to rows where the two keys are equal. This is the most common type of join, called an _inner join_. An alternative syntax that accomplishes exactly the same query is:

```
SELECT name, totalPoints, year FROM Player INNER JOIN Stats
```

This syntax provides a cleaner separation between the logic of joining tables and the logic of choosing rows. Inner joins are the default type of join, so the `INNER` keyword is optional for an inner join. When the key columns in the tables being joined have the same name, a more succinct syntax can be used:

```
SELECT name, totalPoints, year FROM Player JOIN Stats
```

A query that performs a join with `USING` is not exactly the same as one that performs the join with `ON`. With `USING`, the key column appears in the result of the join only once, labeled with an unqualified name (in this example `number`). With `ON`, the key columns from both tables appear in the result and must be referenced with qualified names to avoid ambiguity (in this example `Player.number` and `Stats.number`).

A less commonly used type of join is the _outer join_. Unlike inner joins, which exclude rows with key values that don’t match the corresponding key in the joined table, outer joins include these rows. Because included rows with no match in the other table have no values for the columns from the other table, these values are returned as `NULL`. The three kinds of outer joins are left, right, and full. A _left outer join_ retains all rows from the first table, but only matching rows from the second; a _right outer join_ retains all rows from the second table but only matching rows from the first, and a _full outer join_ retains all rows from both tables. For this database, a left outer join of the two tables would include the names of the benchwarmers:

```
SELECT name, totalPoints, year FROM Player LEFT OUTER JOIN Stats
```

It returns:

```
+----------------+-------------+----------+
```

The _aggregates_, `COUNT`, `MAX`, `MIN`, `SUM`, and `AVG`, are another commonly used SQL feature. These aggregates enable you to retrieve the count, maximum, minimum, sum, and average, respectively, for a particular column. For example, you may want to print the average number of points each player has scored. To do this, use the following query:

```
SELECT AVG(totalPoints) FROM Stats;
```

producing:

```
+------------------+
```

Other times, you may want to apply aggregates separately over subsets of the data. For example, you may want to calculate each player’s average total points per year. You accomplish this with the `GROUP` `BY` clause, as in the following query:

```
SELECT name, AVG(totalPoints) FROM Player INNER JOIN Stats
```

which produces:

```
+----------------+------------------+
```

Most interview problems focus on using these sorts of `INSERT` and `SELECT` statements. You’re less likely to encounter SQL problems related to other features, such as `UPDATE` statements, `DELETE` statements, permissions, or security.

### NoSQL

While SQL relational databases have long been the standard for data storage, other types of databases have become increasingly common and popular. A _NoSQL_ database is, as its name suggests, any database that does not conform to the relational model embodied by SQL. Many such databases could be fair game in an interview. You’re most likely to be asked about NoSQL if you mention experience with it or you’re interviewing for a job where one of these databases is used extensively. We’ll focus on two common types: object databases (like Firebase) and denormalized key-value/column hybrids (like Cassandra). You should also expect questions on other types of databases which you list on your resume.

### Object Databases

An object database is a database that stores data in an object model as used in object-oriented programming, instead of in tables as in a relational database. It typically has a hierarchical structure and relies on function calls through an API to store and retrieve data rather than a domain-specific language like SQL. A major advantage of an object database is that you can maintain consistency between your object model and your object database schema. Depending on your application and class hierarchy, it can be trivial to store and retrieve objects. Also, depending on the usage, object databases can be faster as their hierarchical structure can allow them to quickly access certain data elements by following nodes.

For example, in a messenger application, individual messages can be stored as objects, with all of their associated data (content, sender ID, recipient ID, time, read receipt, and so on). Because these messages would always be associated with a conversation between a sender and a recipient, you could define a structure that enabled quick and easy retrieval of all of a user’s conversations, and then all of the messages within a conversation. Additionally, the structure of conversations would closely follow an object model where each conversation contains messages, as opposed to a SQL database where relationships between objects might require separate tables for conversations and messages.

Most object databases, as in this messenger example, are hierarchical, where each instance represents an instance of an object within the application’s data model. They are generally optimized for storing and retrieving objects, but can be less flexible for queries based on the properties of the data. For example, querying the previously described messenger object database to determine each user’s most frequent message recipient could require an inefficient exhaustive search of each user’s conversations and messages.

### Hybrid Key-Value/Column Databases

These databases evolved out of a common problem with SQL databases: the flexibility that enables arbitrary joins inherent in the relational model limits performance. This can result in poor scaling for use cases with high volumes of reads and writes, even when this flexibility is unneeded. For example, status updates for social networks have a very high read/write volume, but rarely need to associate with more than one user.

Consequently, a class of databases like Cassandra was developed to enable high scalability (and some other properties like reliability) at the expense of SQL’s flexibility. Often this scaling is horizontal, meaning that adding additional servers allows linear scaling with increasing load.

When using these databases there’s an element of denormalization and duplication that has generally been actively avoided in SQL. This may require more storage space and place more of the burden of maintaining data consistency and integrity on the programmer, but in cases where performance, especially reads, needs to be optimized this can be a positive trade-off, particularly given how inexpensive storage has become. For the example of social network status updates, it might be necessary to write the status update to two tables, one where the key is the user and the other where the key is a group the user is in. Though this denormalization duplicates the data, this design would allow for looking up all updates in a group very quickly and efficiently because you can create two tables with the same data, and place a primary key on different elements in each table.

One final advantage of this type database is that it can use a query language that is similar to (but more restrictive than) SQL, allowing users who are familiar with relational databases to come up to speed quickly. For example, in simple cases the insert and select statements are identical to SQL:

```
INSERT INTO student (student_id, first_name, last_name)
```

### Database Transactions

The integrity of the data stored in a database is paramount. If the data is corrupted, every application that depends on the database may fail or encounter errors. Although referential integrity helps keep the data consistent, other forms of inconsistency can occur, even in a database that has referential integrity. An additional mechanism to maintain data integrity is the database transaction.

A _transaction_ groups a set of related database manipulations together into a single unit. If any operation within the transaction fails, the entire transaction fails, and any changes made by the transaction are abandoned (_rolled back_). Conversely, if all the operations succeed, all the changes are _committed_ together as a group.

[Chapter 10](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c10.xhtml) includes a simple example involving the addition and removal of money from a bank account. If you expand the example to involve the transfer of money between two accounts with account balances maintained in a database, you can see why transactions are so important. A transfer is actually two operations: removing money from the first account and then adding it to the second account. If an error occurs immediately after the money is removed from the first account, you want the system to detect the problem and redeposit the withdrawn money into the original account. As long as both operations are contained in a transaction, there won’t be any problems with this: Either both of them are successfully committed and the transfer succeeds, or neither one is committed and the transfer fails. In either case, no money is lost or created.

The four properties of a transaction are as follows:

-   **Atomicity.** The database system guarantees that either all operations within the transaction succeed or else they all fail.
-   **Consistency**. The transaction must ensure that the database is in a correct, consistent state at the start and the end of the transaction. No referential integrity constraints can be broken, for example.
-   **Isolation**. All changes to the database within a transaction are isolated from all other queries and transactions until the transaction is committed.
-   **Durability**. When committed, changes made in a transaction are permanent. The database system must have some way to recover from crashes and other problems so that the current state of the database is never lost.

These four properties are generally referred to as _ACID_. As you might imagine, there is a significant performance penalty to be paid if all four properties are to be guaranteed on each transaction. The isolation requirement can be particularly onerous on a system with many simultaneous transactions, so most systems allow the isolation requirements to be relaxed in different ways to provide improved performance.

Note that ACID compliance is _not_ a relational database requirement, but most modern databases support it.

### Distributed Databases

As databases and datasets grow, they almost inevitably become distributed—data is stored at multiple locations, across a network. This has the advantages of low latency, redundancy, and often lower cost. As such, many real-world databases consist of multiple nodes, frequently in different data centers.

The CAP theorem is one of the core distributed network database concepts. At its core, it recognizes that all distributed networks have delays and the connections will sometimes fail. The theorem states that a database can only possess two of the following three properties:

-   **Consistency.** Every read returns the most recent write. For example, if you have a distributed banking application and you have recently made a deposit into your account on one node, a read on any other node would reflect the most recent account balance.
-   **Accessibility.** Every request receives a response, though not necessarily reflecting the most recent writes. For example, in a distributed banking application, if you queried any node at any time for your account information, you would always get a response, though it may not be the most recent value for the account balance.
-   **Partitionability.** The system can be partitioned into nodes and the system continues to function even if data is dropped between nodes on the network. For example, in a distributed banking application, even if several nodes go down, the system as a whole still functions.

It would be great to have all three of these properties—a database that is always accessible, can be split up across nodes on an unreliable real-world network, and only returns the most recent information. But, unfortunately, all three properties are not simultaneously achievable. (Another way of looking at the CAP theorem is that it states that you can’t have both consistency and accessibility in a distributed system, because if you have both of these it can’t be partitioned, so it’s not distributed.)

Given the CAP theorem, many databases (for example, most distributed banking systems) choose availability over consistency. The system then applies limits, such as how much money can be withdrawn at once or in a day, so that the magnitude of potential inconsistency in account data can be limited, while maintaining availability in a distributed system. Because it is not possible for a high availability distributed system to be truly consistent, such systems are instead often designed to achieve _eventual consistency_. The properties of eventual consistency are summarized by the acronym _BASE_: Basically Available, Soft state, Eventual consistency. (Note that BASE is the opposite of ACID in a chemical sense.) In this model, eventual consistency means that every piece of data entered into the system will eventually propagate to all nodes and (if the system stops receiving input) this propagation will eventually result in systemwide consistency.

A database that sacrifices accessibility in favor of consistency is not available or provides no response when it is not able to provide the most recent data. This may manifest as “shutting down” while it is updating, or locking out all users for periods of time. Systems that are not 24x7 (such as a stock market), often take advantage of downtimes to allow for updates.

## DATABASE PROBLEMS

Database operations are so common in most programming roles, that you should expect a few questions if you indicate on your résumé that you have even the slightest experience.

### Simple SQL

This is an extremely easy problem that an interviewer might use to determine whether you have ever used SQL before or whether you were padding your résumé when you mentioned it. If you know SQL, you’re all set. It’s a straightforward SQL `INSERT` statement; no tricks. If you don’t know SQL, you’re in trouble. The correct answer is:

```
INSERT INTO Olympics VALUES( 'Montreal', 1976 );
```

### Company and Employee Database

**[TABLE 13-3:](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c13-tbl-0003)** Company Sample Data

| **COMPANYNAME** | **ID** |
| --- | --- |
| Hillary Plumbing | 6 |
| John Lawn Company | 9 |
| Dave Cookie Company | 19 |
| Jane Electricity | 3 |

**[TABLE 13-4:](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c13-tbl-0004)** Employees Hired Sample Data

| **ID** | **NUMHIRED** | **FISCALQUARTER** |
| --- | --- | --- |
| 3 | 3 | 3 |
| 3 | 2 | 4 |
| 19 | 4 | 1 |
| 6 | 2 | 1 |

This problem involves retrieving data from two tables. You must join the two tables to get all the needed information; `id` is the only key common to both tables, so you want to join on the column `id`. After you join the two tables, you can select the company name where the fiscal quarter is 4. This SQL statement looks like:

```
SELECT companyName FROM Company, EmployeesHired
```

There is a small problem with this SQL statement. Consider what might happen if a company did not hire anyone in the fourth quarter. There could still be a tuple (a row of data) such as `EmployeesHired(6, 0, 4)`. The company with `id` 6 would be returned by the preceding query even though it didn’t hire anyone during fiscal quarter 4. To fix this bug, you need to ensure that `numHired` is greater than 0. The revised SQL statement looks like this:

```
SELECT companyName FROM Company, EmployeesHired
```

The best way to start this problem is by looking at the previous answer. You know how to get the names of all the companies that hired an employee in quarter 4. If you remove the `WHERE` condition that `fiscalQuarter = 4`, you have a query that returns the names of all companies that hired employees during all fiscal quarters. If you use this query as a _subquery_ and select all the companies that are not in the result, you get all the companies that did not hire anyone in fiscal quarters 1 through 4. As a slight optimization, you can select just the `id` from the `EmployeesHired` table and select the `companyName` for company `id` values not in the subquery. The query looks like this:

```
SELECT companyName FROM Company WHERE id NOT IN
```

You’re asked to retrieve the totals of some sets of values, which indicates that you must use the `SUM` aggregate. In this problem, you don’t want the sum of the entire column, you want only a sum of the values that have the same `id`. To accomplish this task, you need to use the `GROUP BY` feature. This feature enables you to apply `SUM` over grouped values of data. Other than the `GROUP BY` feature, this query is similar to the first query except you omit `fiscalQuarter = 4` in the `WHERE` clause. The query looks like this:

```
SELECT companyName, SUM(numHired)
```

This query is almost, but not quite, correct. The problem asks for the names of _all_ companies, but the preceding query performs an inner join, so only companies that have rows in `EmployeesHired` appear in the results. For instance, with the provided sample data, John Lawn Company would not appear in the results. As the query is currently written, you want to retain unmatched rows from the first table, `Company`, so you must perform a left outer join. (Due to the foreign key constraint, there can’t be any unmatched rows in `EmployeesHired`.) A query performing a left outer join looks like:

```
SELECT companyName, SUM(numHired)
```

There’s one final wrinkle: you’re instructed to return the total number of employees each company hired, but by the definition of an outer join, `numHired` will be `NULL` for companies with no rows in `EmployeesHired`. `SUM(NULL)` is `NULL`, so for these companies the preceding query returns `NULL` as the number hired instead of `0`. You can fix this by applying a SQL function to the result that replaces any `NULL` values with `0` (if you know the name of this function without having to look it up, you’re a real SQL wizard):

```
SELECT companyName, COALESCE(SUM(numHired), 0)
```

### Max, No Aggregates

In this problem, your hands are tied behind your back; you must find a maximum without using the feature designed for finding the maximum. A good way to start is by drawing a table with some sample data, as shown in [Table 13-5](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c13-tbl-0005).

**[TABLE 13-5:](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c13-tbl-0005)** Sample Values for num

| **NUM** |
| --- |
| 5 |
| 23 |
| − 6 |
| 7 |

For this sample data, you want the query to return 23. 23 has the property that all other numbers are less than it. Though true, this way of looking at things doesn’t offer much help with constructing the SQL statement. A similar but more useful way to say the same thing is that 23 is the _only_ number that does not have a number that is greater than it. If you could return every value that does not have a number greater than it, you would return only 23, and you would have solved the problem. Try designing a SQL statement to print out every number that does not have a number greater than it.

First, figure out which numbers do have numbers greater than themselves. This is a more manageable query. Begin by joining the table with itself to create all possible pairs for which each value in one column is greater than the corresponding value in the other column, as in the following query (`AS` gives the table a temporary alias for use within the query, allowing you to use the same table twice in a query):

```
SELECT Lesser.num, Greater.num
```

Using the sample data, this yields the results shown in [Table 13-6](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c13-tbl-0006).

**[TABLE 13-6:](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c13-tbl-0006)** Temporary Table Formed after Join

| **LESSER.NUM** | **GREATER.NUM** |
| --- | --- |
| −6 | 23 |
| 5 | 23 |
| 7 | 23 |
| −6 | 7 |
| 5 | 7 |
| −6 | 5 |

As desired, every value is in the lesser column except the maximum value of 23. Thus, if you use the previous query as a subquery and select every value not in it, you get the maximum value. This query would look like the following:

```
SELECT num FROM Test WHERE num NOT IN
```

There is one minor bug in this query. If the maximum value is repeated in the `Test` table, it will be returned twice. To prevent this, use the `DISTINCT` keyword. This changes the query to the following:

```
SELECT DISTINCT num FROM Test WHERE num NOT IN
```

### Three-Valued Logic

This problem seems simple. The immediately obvious solution is this query:

```
SELECT * FROM Address WHERE apartment = NULL;
```

This won’t return any addresses, however, because of SQL’s use of _ternary,_ or _three-valued,_ logic. Unlike the two-valued Boolean logic used in most programming languages, three possible logical values exist in SQL: `TRUE`, `FALSE` and `UNKNOWN`. As you might expect, `UNKNOWN` means that the truth is uncertain because it involves a value that is unknown, missing, or not representable.

The familiar `AND`, `OR`, and `NOT` operations return different values in the presence of an `UNKNOWN` value, as shown in [Tables 13-7](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c13-tbl-0007), [13-8](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c13-tbl-0008), and [13-9](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#c13-tbl-0009).

**[TABLE 13-7:](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c13-tbl-0007)** Ternary AND Operations

| **AND** | **TRUE** | **FALSE** | **UNKNOWN** |
| --- | --- | --- | --- |
| **TRUE** | TRUE | FALSE | UNKNOWN |
| **FALSE** | FALSE | FALSE | FALSE |
| **UNKNOWN** | UNKNOWN | FALSE | UNKNOWN |

**[TABLE 13-8:](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c13-tbl-0008)** Ternary OR Operations

| **OR** | **TRUE** | **FALSE** | **UNKNOWN** |
| --- | --- | --- | --- |
| **TRUE** | TRUE | TRUE | TRUE |
| **FALSE** | TRUE | FALSE | UNKNOWN |
| **UNKNOWN** | TRUE | UNKNOWN | UNKNOWN |

**[TABLE 13-9:](https://learning-oreilly-com.ezproxy.christchurchcitylibraries.com/library/view/programming-interviews-exposed/9781119418474/c01.xhtml#R_c13-tbl-0009)** Ternary NOT Operations

| **_NOT_** |   |
| --- | --- |
| **TRUE** | FALSE |
| **FALSE** | TRUE |
| **UNKNOWN** | UNKNOWN |

The preceding query fails because it uses the equality operator ( = ) to test for a `NULL` column value. In most databases, a comparison to `NULL` returns `UNKNOWN`—even when comparing `NULL` to `NULL`. The rationale for this is that `NULL` represents missing or unknown data, so it’s unknown whether two `NULL` values represent the same value or two unequal pieces of missing data. Queries return rows only where the `WHERE` clause is `TRUE`; if the `WHERE` clause contains `= NULL`, then all the rows have `UNKNOWN` value and none are returned. The proper way to check for a `NULL` or non-`NULL` column is to use the `IS NULL` or `IS NOT NULL` syntax. Thus, the original query should be restated as follows:

```
SELECT * FROM Address WHERE apartment IS NULL;
```

Not accounting for `UNKNOWN` values in `WHERE` clause conditions is a common error, especially when the appearance of `NULL` values is less obvious. For instance, the following query doesn’t return every row except those where `apartment = 1`; it returns only rows that have a non-`NULL apartment` not equal to `1`:

```
SELECT * FROM Address WHERE apartment <> 1;
```

### School Schemata

Start with an example. Assume a student, Eric, takes five classes. Each class (for example, “Advanced Programming”) has 30–90 students in it. If you take an approach that started with students, you could keep track of all of the students and the classes that they take, using a schema like:

```
{
```

In this example, you are able to quickly look up a student, and then find out all of the classes that the student is taking. However, to find all of the students in a given class, you would have to retrieve every single student, and go through every class of every student. The current schema is clearly not optimized for this use case.

Alternatively, you could start with classes as the root, as in:

```
{
```

This approach has the opposite problem. It’s very fast to find the students within a given class, but very time consuming to find all of the classes a certain student is taking.

Since you want two properties—fast lookup of a student’s classes and fast lookup of a class’s students—you can use both of the preceding representations in your schema. If you use both, you have data redundancy, and would have to maintain consistency with operations for adding and deleting students and classes. However, you would be able to achieve the properties that you want for this dataset.

This is just a trivial translation of the requirements:

```
CREATE TABLE student (
```

This is a straightforward SQL `SELECT` statement, where you can join the class table and enrollment tables and use the `COUNT` function:

```
SELECT COUNT (*)
```

In the preceding SQL schema, every time you want to use an email to look up the number of classes that a student is taking, you need to perform a join across tables. One potential way to denormalize and avoid the join would be to add email as a column inside the enrollment table. This would allow you to then count the number of classes without having to join tables to identify the student by email. Doing this results in an enrollment table that looks like:

```
CREATE TABLE enrollment {
```

Then, your query is a straightforward count of rows in the `enrollment` table:

```
SELECT COUNT (*)
```

This produces the answer without a join. However, it is still not as optimized for speed as it could be—because you need to search for the rows where the email matches the desired value every time you call the query. While an index on `email` would speed this up considerably, you would still have to count at least the number of rows that had the desired value for `email`. If you wanted to further speed up this query, (particularly if you imagine a similar schema where the number of class-type entities associated with a student-type entity became very high) you could denormalize the database by creating a column in the `student` table that directly tracks the number of classes a student is taking. Then, every time you inserted or deleted from the class function, you would update this column in the `student` table.

In this approach, the `student` table becomes:

```
CREATE TABLE student (
```

Now, the query to look up the number of classes is trivial:

```
SELECT num_classes from student where email = 'john@pie.com';
```

This query becomes very efficient as it only has to read a single value from a single row identified by a unique key.

Trade-offs exist for everything, and this change to the schema does come with some disadvantages. You’ve introduced a new column to represent information that was already present in the original schema. This requires additional storage space, and also creates a potential for inconsistency in the database (if the value of `num_classes` for a student is not equal to the number of rows in `enrollment` associated with that student).

Now, when you add or remove a row from `enrollment`, you also need to update `num_classes` in the `student` table. You would need to make both of these changes inside a single transaction to avoid inconsistency in case one of the changes failed. This makes inserts and deletes slower, but that could be a worthwhile trade-off in a system where updates are rare and reads need to be efficient.

As previously discussed, you need to ensure that your statements are inside a single transaction to avoid inconsistency. The syntax and mechanism for this varies by database system, so we don’t represent it explicitly in the following solution.

Assuming the transaction has been taken care of, for enrollment you have to both insert into the `enrollment` table, and then increment `num_classes` in the `student` table:

```
INSERT INTO enrollment VALUES (334, 887);
```

Unenrollment looks very much like the reverse of the insert and the update—again making sure this occurs within a transaction:

```
DELETE FROM enrollment WHERE student_id = 334 AND class_id = 887;
```

You should work through a couple of examples to make sure that both sets of statements are correct. When you do, you may find a few special cases that require your attention. When inserting a new row into `student`, you would need to ensure that you start `num_classes` at 0. Then you have some edge cases. For example, if you try to delete a row from `enrollment` that does not exist, you would not want to decrement the number of classes. Additionally, you would want to make sure that you do not insert a duplicate into rows in the `enrollment` table.

This is a real-world issue: a distributed database is not guaranteed to be always consistent. In general, the guarantee is for eventual consistent consistency.

One way to think about this is by considering a common distributed system that already meets this criterion—the banking system. In banking, limits exist on updates; for instance, how much money you can withdraw at any one time, or how long before a deposited check is credited to your account. These limits allow time for a potentially inconsistent network to update, and minimize the size of inconsistency that can occur in an account while still providing the usability and convenience of high availability.

Similarly, in a student system, you could create rules such as having an enrollment period for students, a time delay between the end of class enrollment and final class roster announcement, and a time delay between when grades are due and when they are announced. Not coincidentally, these rules bear a striking resemblance to those employed in most universities’ class registration systems.

## SUMMARY

Databases are common building blocks of applications, especially web-based applications. Most database systems are based on the concepts of relational database theory, so you can expect most problems you encounter to involve accessing and manipulating relational data. To do this, you need to understand basic SQL commands such as `SELECT` and `INSERT`. Transactions and foreign key constraints are among the mechanisms that databases provide to maintain consistency. NoSQL concepts and questions also come up occasionally, as do properties of transactions and distributed databases.
