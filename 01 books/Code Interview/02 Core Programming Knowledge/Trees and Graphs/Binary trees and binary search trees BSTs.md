# Binary Trees and Binary Search Trees (BSTs)

## Overview
Fundamental tree data structures where each node has at most two children, with BSTs maintaining ordered properties for efficient searching, insertion, and deletion.

## Binary Tree Structure

### Basic Node Definition
```cpp
template<typename T>
struct TreeNode {
    T data;
    TreeNode* left;
    TreeNode* right;

    TreeNode(const T& value) : data(value), left(nullptr), right(nullptr) {}
};

template<typename T>
class BinaryTree {
protected:
    TreeNode<T>* root;

public:
    BinaryTree() : root(nullptr) {}

    virtual ~BinaryTree() {
        clear();
    }

    void clear() {
        clearHelper(root);
        root = nullptr;
    }

private:
    void clearHelper(TreeNode<T>* node) {
        if (node) {
            clearHelper(node->left);
            clearHelper(node->right);
            delete node;
        }
    }
};
```

### Tree Properties and Types
```cpp
// Calculate basic tree properties
template<typename T>
class BinaryTree {
public:
    // Height of tree (longest path from root to leaf)
    int height() const {
        return heightHelper(root);
    }

    // Number of nodes in tree
    int size() const {
        return sizeHelper(root);
    }

    // Check if tree is balanced (height difference ≤ 1)
    bool isBalanced() const {
        return isBalancedHelper(root) != -1;
    }

    // Check if tree is complete
    bool isComplete() const {
        if (!root) return true;

        queue<TreeNode<T>*> q;
        q.push(root);
        bool nullFound = false;

        while (!q.empty()) {
            TreeNode<T>* current = q.front();
            q.pop();

            if (!current) {
                nullFound = true;
            } else {
                if (nullFound) return false;  // Non-null after null found

                q.push(current->left);
                q.push(current->right);
            }
        }

        return true;
    }

private:
    int heightHelper(TreeNode<T>* node) const {
        if (!node) return -1;
        return 1 + max(heightHelper(node->left), heightHelper(node->right));
    }

    int sizeHelper(TreeNode<T>* node) const {
        if (!node) return 0;
        return 1 + sizeHelper(node->left) + sizeHelper(node->right);
    }

    int isBalancedHelper(TreeNode<T>* node) const {
        if (!node) return 0;

        int leftHeight = isBalancedHelper(node->left);
        if (leftHeight == -1) return -1;

        int rightHeight = isBalancedHelper(node->right);
        if (rightHeight == -1) return -1;

        if (abs(leftHeight - rightHeight) > 1) return -1;

        return 1 + max(leftHeight, rightHeight);
    }
};
```

## Binary Search Tree (BST)

### BST Properties and Implementation
```cpp
template<typename T>
class BinarySearchTree : public BinaryTree<T> {
public:
    // Insert value maintaining BST property
    void insert(const T& value) {
        this->root = insertHelper(this->root, value);
    }

    // Search for value
    bool search(const T& value) const {
        return searchHelper(this->root, value);
    }

    // Remove value
    void remove(const T& value) {
        this->root = removeHelper(this->root, value);
    }

    // Find minimum value
    T findMin() const {
        if (!this->root) throw runtime_error("Empty tree");
        TreeNode<T>* node = findMinNode(this->root);
        return node->data;
    }

    // Find maximum value
    T findMax() const {
        if (!this->root) throw runtime_error("Empty tree");
        TreeNode<T>* node = findMaxNode(this->root);
        return node->data;
    }

    // Check if tree maintains BST property
    bool isValidBST() const {
        return isValidBSTHelper(this->root, nullptr, nullptr);
    }

private:
    TreeNode<T>* insertHelper(TreeNode<T>* node, const T& value) {
        if (!node) {
            return new TreeNode<T>(value);
        }

        if (value < node->data) {
            node->left = insertHelper(node->left, value);
        } else if (value > node->data) {
            node->right = insertHelper(node->right, value);
        }
        // If value equals node->data, don't insert (no duplicates)

        return node;
    }

    bool searchHelper(TreeNode<T>* node, const T& value) const {
        if (!node) return false;

        if (value == node->data) return true;
        if (value < node->data) return searchHelper(node->left, value);
        return searchHelper(node->right, value);
    }

    TreeNode<T>* removeHelper(TreeNode<T>* node, const T& value) {
        if (!node) return nullptr;

        if (value < node->data) {
            node->left = removeHelper(node->left, value);
        } else if (value > node->data) {
            node->right = removeHelper(node->right, value);
        } else {
            // Node to be deleted found
            if (!node->left) {
                TreeNode<T>* temp = node->right;
                delete node;
                return temp;
            } else if (!node->right) {
                TreeNode<T>* temp = node->left;
                delete node;
                return temp;
            } else {
                // Node with two children
                TreeNode<T>* successor = findMinNode(node->right);
                node->data = successor->data;
                node->right = removeHelper(node->right, successor->data);
            }
        }

        return node;
    }

    TreeNode<T>* findMinNode(TreeNode<T>* node) const {
        while (node->left) {
            node = node->left;
        }
        return node;
    }

    TreeNode<T>* findMaxNode(TreeNode<T>* node) const {
        while (node->right) {
            node = node->right;
        }
        return node;
    }

    bool isValidBSTHelper(TreeNode<T>* node, TreeNode<T>* minNode, TreeNode<T>* maxNode) const {
        if (!node) return true;

        if ((minNode && node->data <= minNode->data) ||
            (maxNode && node->data >= maxNode->data)) {
            return false;
        }

        return isValidBSTHelper(node->left, minNode, node) &&
               isValidBSTHelper(node->right, node, maxNode);
    }
};
```

### Advanced BST Operations
```cpp
template<typename T>
class BinarySearchTree : public BinaryTree<T> {
public:
    // Find kth smallest element
    T kthSmallest(int k) const {
        int count = 0;
        TreeNode<T>* result = kthSmallestHelper(this->root, k, count);
        if (!result) throw runtime_error("k is out of range");
        return result->data;
    }

    // Find lowest common ancestor
    TreeNode<T>* lowestCommonAncestor(const T& p, const T& q) const {
        return lcaHelper(this->root, p, q);
    }

    // Convert BST to sorted array
    vector<T> toSortedArray() const {
        vector<T> result;
        inorderHelper(this->root, result);
        return result;
    }

    // Convert sorted array to balanced BST
    static BinarySearchTree<T> fromSortedArray(const vector<T>& arr) {
        BinarySearchTree<T> bst;
        bst.root = sortedArrayToBSTHelper(arr, 0, arr.size() - 1);
        return bst;
    }

    // Range query: find all values in [low, high]
    vector<T> rangeQuery(const T& low, const T& high) const {
        vector<T> result;
        rangeQueryHelper(this->root, low, high, result);
        return result;
    }

private:
    TreeNode<T>* kthSmallestHelper(TreeNode<T>* node, int k, int& count) const {
        if (!node) return nullptr;

        TreeNode<T>* left = kthSmallestHelper(node->left, k, count);
        if (left) return left;

        count++;
        if (count == k) return node;

        return kthSmallestHelper(node->right, k, count);
    }

    TreeNode<T>* lcaHelper(TreeNode<T>* node, const T& p, const T& q) const {
        if (!node) return nullptr;

        if (p < node->data && q < node->data) {
            return lcaHelper(node->left, p, q);
        }
        if (p > node->data && q > node->data) {
            return lcaHelper(node->right, p, q);
        }

        return node;  // Found LCA
    }

    void inorderHelper(TreeNode<T>* node, vector<T>& result) const {
        if (node) {
            inorderHelper(node->left, result);
            result.push_back(node->data);
            inorderHelper(node->right, result);
        }
    }

    static TreeNode<T>* sortedArrayToBSTHelper(const vector<T>& arr, int left, int right) {
        if (left > right) return nullptr;

        int mid = left + (right - left) / 2;
        TreeNode<T>* root = new TreeNode<T>(arr[mid]);

        root->left = sortedArrayToBSTHelper(arr, left, mid - 1);
        root->right = sortedArrayToBSTHelper(arr, mid + 1, right);

        return root;
    }

    void rangeQueryHelper(TreeNode<T>* node, const T& low, const T& high, vector<T>& result) const {
        if (!node) return;

        if (node->data >= low && node->data <= high) {
            result.push_back(node->data);
        }

        if (node->data > low) {
            rangeQueryHelper(node->left, low, high, result);
        }

        if (node->data < high) {
            rangeQueryHelper(node->right, low, high, result);
        }
    }
};
```

## Tree Traversals

### Recursive Traversals
```cpp
template<typename T>
class BinaryTree {
public:
    // Inorder traversal (Left, Root, Right)
    void inorderTraversal() const {
        inorderHelper(root);
        cout << endl;
    }

    // Preorder traversal (Root, Left, Right)
    void preorderTraversal() const {
        preorderHelper(root);
        cout << endl;
    }

    // Postorder traversal (Left, Right, Root)
    void postorderTraversal() const {
        postorderHelper(root);
        cout << endl;
    }

private:
    void inorderHelper(TreeNode<T>* node) const {
        if (node) {
            inorderHelper(node->left);
            cout << node->data << " ";
            inorderHelper(node->right);
        }
    }

    void preorderHelper(TreeNode<T>* node) const {
        if (node) {
            cout << node->data << " ";
            preorderHelper(node->left);
            preorderHelper(node->right);
        }
    }

    void postorderHelper(TreeNode<T>* node) const {
        if (node) {
            postorderHelper(node->left);
            postorderHelper(node->right);
            cout << node->data << " ";
        }
    }
};
```

### Iterative Traversals
```cpp
template<typename T>
class BinaryTree {
public:
    // Iterative inorder traversal
    void inorderIterative() const {
        stack<TreeNode<T>*> stk;
        TreeNode<T>* current = root;

        while (current || !stk.empty()) {
            while (current) {
                stk.push(current);
                current = current->left;
            }

            current = stk.top();
            stk.pop();
            cout << current->data << " ";
            current = current->right;
        }
        cout << endl;
    }

    // Iterative preorder traversal
    void preorderIterative() const {
        if (!root) return;

        stack<TreeNode<T>*> stk;
        stk.push(root);

        while (!stk.empty()) {
            TreeNode<T>* current = stk.top();
            stk.pop();

            cout << current->data << " ";

            if (current->right) stk.push(current->right);
            if (current->left) stk.push(current->left);
        }
        cout << endl;
    }

    // Level order traversal (BFS)
    void levelOrderTraversal() const {
        if (!root) return;

        queue<TreeNode<T>*> q;
        q.push(root);

        while (!q.empty()) {
            TreeNode<T>* current = q.front();
            q.pop();

            cout << current->data << " ";

            if (current->left) q.push(current->left);
            if (current->right) q.push(current->right);
        }
        cout << endl;
    }

    // Level order with level separation
    void levelOrderWithLevels() const {
        if (!root) return;

        queue<TreeNode<T>*> q;
        q.push(root);

        while (!q.empty()) {
            int levelSize = q.size();

            for (int i = 0; i < levelSize; ++i) {
                TreeNode<T>* current = q.front();
                q.pop();

                cout << current->data;
                if (i < levelSize - 1) cout << " ";

                if (current->left) q.push(current->left);
                if (current->right) q.push(current->right);
            }
            cout << endl;
        }
    }
};
```

## Common Interview Problems

### Problem 1: Validate Binary Search Tree
```cpp
bool isValidBST(TreeNode<int>* root) {
    return validate(root, LONG_MIN, LONG_MAX);
}

bool validate(TreeNode<int>* node, long minVal, long maxVal) {
    if (!node) return true;

    if (node->data <= minVal || node->data >= maxVal) {
        return false;
    }

    return validate(node->left, minVal, node->data) &&
           validate(node->right, node->data, maxVal);
}
```

### Problem 2: Serialize and Deserialize Binary Tree
```cpp
class Codec {
public:
    // Serialize tree to string
    string serialize(TreeNode<int>* root) {
        ostringstream oss;
        serializeHelper(root, oss);
        return oss.str();
    }

    // Deserialize string to tree
    TreeNode<int>* deserialize(string data) {
        istringstream iss(data);
        return deserializeHelper(iss);
    }

private:
    void serializeHelper(TreeNode<int>* node, ostringstream& oss) {
        if (!node) {
            oss << "null,";
        } else {
            oss << node->data << ",";
            serializeHelper(node->left, oss);
            serializeHelper(node->right, oss);
        }
    }

    TreeNode<int>* deserializeHelper(istringstream& iss) {
        string val;
        getline(iss, val, ',');

        if (val == "null") {
            return nullptr;
        }

        TreeNode<int>* node = new TreeNode<int>(stoi(val));
        node->left = deserializeHelper(iss);
        node->right = deserializeHelper(iss);

        return node;
    }
};
```

### Problem 3: Binary Tree Maximum Path Sum
```cpp
int maxPathSum(TreeNode<int>* root) {
    int maxSum = INT_MIN;
    maxPathHelper(root, maxSum);
    return maxSum;
}

int maxPathHelper(TreeNode<int>* node, int& maxSum) {
    if (!node) return 0;

    // Get maximum sum from left and right subtrees
    int leftMax = max(0, maxPathHelper(node->left, maxSum));
    int rightMax = max(0, maxPathHelper(node->right, maxSum));

    // Current path sum including this node
    int currentPathSum = node->data + leftMax + rightMax;

    // Update global maximum
    maxSum = max(maxSum, currentPathSum);

    // Return maximum sum including this node and one subtree
    return node->data + max(leftMax, rightMax);
}
```

## Performance Analysis

### Time Complexity Summary
```cpp
void performanceAnalysis() {
    cout << "Binary Search Tree Time Complexities:" << endl;
    cout << "Average Case (Balanced Tree):" << endl;
    cout << "  Search: O(log n)" << endl;
    cout << "  Insert: O(log n)" << endl;
    cout << "  Delete: O(log n)" << endl;
    cout << "  Min/Max: O(log n)" << endl;

    cout << "\nWorst Case (Skewed Tree):" << endl;
    cout << "  Search: O(n)" << endl;
    cout << "  Insert: O(n)" << endl;
    cout << "  Delete: O(n)" << endl;
    cout << "  Min/Max: O(n)" << endl;

    cout << "\nTraversal Operations:" << endl;
    cout << "  All traversals: O(n)" << endl;
    cout << "  Space complexity: O(h) where h is height" << endl;
}
```

### Self-Balancing Trees Comparison
```cpp
void compareSelfBalancingTrees() {
    cout << "Self-Balancing Tree Comparison:" << endl;

    cout << "\nAVL Trees:" << endl;
    cout << "  Guarantee: |height(left) - height(right)| ≤ 1" << endl;
    cout << "  Search: O(log n), Insert: O(log n), Delete: O(log n)" << endl;
    cout << "  Pros: Strictly balanced, good for search-heavy applications" << endl;

    cout << "\nRed-Black Trees:" << endl;
    cout << "  Guarantee: Longest path ≤ 2 × shortest path" << endl;
    cout << "  Search: O(log n), Insert: O(log n), Delete: O(log n)" << endl;
    cout << "  Pros: Less rigid balancing, used in STL map/set" << endl;

    cout << "\nSplay Trees:" << endl;
    cout << "  Guarantee: Amortized O(log n) operations" << endl;
    cout << "  Pros: Self-optimizing, good for skewed access patterns" << endl;
}
```

---
*Related: [[Tree basics - root parent child ancestor descendant leaves]] | [[Lookup complexity - O(log n) vs O(n) in skewed trees]]*
*Part of: [[Trees and Graphs MOC]]*