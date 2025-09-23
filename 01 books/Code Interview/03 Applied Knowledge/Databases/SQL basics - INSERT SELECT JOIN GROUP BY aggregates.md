# SQL Basics: INSERT, SELECT, JOIN, GROUP BY, Aggregates

## Overview
Fundamental SQL operations for database interaction.

## Basic Operations

### INSERT
```sql
-- Insert single row
INSERT INTO users (name, email) VALUES ('John', 'john@email.com');

-- Insert multiple rows
INSERT INTO users (name, email) VALUES
    ('Alice', 'alice@email.com'),
    ('Bob', 'bob@email.com');
```

### SELECT
```sql
-- Basic selection
SELECT name, email FROM users;

-- With conditions
SELECT * FROM users WHERE age > 25;

-- With ordering
SELECT * FROM users ORDER BY name ASC;
```

### JOIN
```sql
-- Inner join
SELECT u.name, o.total
FROM users u
JOIN orders o ON u.id = o.user_id;

-- Left join (includes users without orders)
SELECT u.name, o.total
FROM users u
LEFT JOIN orders o ON u.id = o.user_id;
```

### GROUP BY
```sql
-- Group and count
SELECT department, COUNT(*) as employee_count
FROM employees
GROUP BY department;

-- Group with conditions
SELECT department, AVG(salary)
FROM employees
GROUP BY department
HAVING AVG(salary) > 50000;
```

### Aggregates
- **COUNT()**: Number of rows
- **SUM()**: Total of numeric values
- **AVG()**: Average value
- **MIN()/MAX()**: Minimum/Maximum values

---
*Related: [[Inner vs outer joins]] | [[Primary keys foreign keys referential integrity]]*
*Part of: [[Databases MOC]]*