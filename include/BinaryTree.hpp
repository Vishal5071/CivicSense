#pragma once

#include <iostream>
#include <algorithm>
#include "Queue.hpp"

struct BTNode {
    int data;
    BTNode* left;
    BTNode* right;

    // Constructor
    BTNode(int val) : data(val), left(nullptr), right(nullptr) {}
};

// --- Class Declaration & Implementation ---
class BinaryTree {
private:
    BTNode* root;

    // --- Private Helper Functions Implementation ---
    void _destroyTree(BTNode* node) {
        if (node) {
            _destroyTree(node->left);
            _destroyTree(node->right);
            delete node;
        }
    }

    void _inorder(BTNode* node) {
        if (!node) return;
        _inorder(node->left);
        std::cout << node->data << " ";
        _inorder(node->right);
    }

    void _preorder(BTNode* node) {
        if (!node) return;
        std::cout << node->data << " ";
        _preorder(node->left);
        _preorder(node->right);
    }

    void _postorder(BTNode* node) {
        if (!node) return;
        _postorder(node->left);
        _postorder(node->right);
        std::cout << node->data << " ";
    }

    int _height(BTNode* node) {
        if (!node) {
            return -1;
        }
        return 1 + std::max(_height(node->left), _height(node->right));
    }

    bool _search(BTNode* node, int key) {
        if (!node) {
            return false;
        }
        if (node->data == key) {
            return true;
        }
        return _search(node->left, key) || _search(node->right, key);
    }

public:
    // --- Constructor and Destructor ---
    BinaryTree() : root(nullptr) {}

    ~BinaryTree() {
        _destroyTree(root);
    }

    // --- Public Interface Methods ---
    void setRoot(BTNode* node) {
        root = node;
    }

    BTNode* getRoot() {
        return root;
    }

    // --- Traversal Methods ---
    void inorder() {
        std::cout << "In-order: ";
        _inorder(root);
        std::cout << std::endl;
    }

    void preorder() {
        std::cout << "Pre-order: ";
        _preorder(root);
        std::cout << std::endl;
    }

    void postorder() {
        std::cout << "Post-order: ";
        _postorder(root);
        std::cout << std::endl;
    }

    void levelOrder() {
        if (!root) return;
        std::cout << "Level-order: ";
        Queue<BTNode*> q;
        q.enqueue(root);
        while (!q.isEmpty()) {
            BTNode* current = q.peek();
            q.dequeue();
            std::cout << current->data << " ";
            if (current->left) {
                q.enqueue(current->left);
            }
            if (current->right) {
                q.enqueue(current->right);
            }
        }
        std::cout << std::endl;
    }

    // --- Utility Methods ---
    int height() {
        return _height(root);
    }

    bool search(int key) {
        return _search(root, key);
    }

    void insert(int key) {
        BTNode* newNode = new BTNode(key);
        if (!root) {
            root = newNode;
            return;
        }
        Queue<BTNode*> q;
        q.enqueue(root);
        while (!q.isEmpty()) {
            BTNode* current = q.peek();
            q.dequeue();
            if (!current->left) {
                current->left = newNode;
                return;
            } else {
                q.enqueue(current->left);
            }
            if (!current->right) {
                current->right = newNode;
                return;
            } else {
                q.enqueue(current->right);
            }
        }
    }

    void deleteNode(int key) {
        if (!root) {
            return;
        }
        if (!root->left && !root->right) {
            if (root->data == key) {
                delete root;
                root = nullptr;
            }
            return;
        }
        Queue<BTNode*> q;
        q.enqueue(root);
        BTNode* key_node = nullptr;
        BTNode* deepest_node = nullptr;
        BTNode* parent_of_deepest = nullptr;
        while (!q.isEmpty()) {
            deepest_node = q.peek();
            q.dequeue();
            if (deepest_node->data == key) {
                key_node = deepest_node;
            }
            if (deepest_node->left) {
                parent_of_deepest = deepest_node;
                q.enqueue(deepest_node->left);
            }
            if (deepest_node->right) {
                parent_of_deepest = deepest_node;
                q.enqueue(deepest_node->right);
            }
        }
        if (key_node) {
            key_node->data = deepest_node->data;
            if (parent_of_deepest->right == deepest_node) {
                delete parent_of_deepest->right;
                parent_of_deepest->right = nullptr;
            } else {
                delete parent_of_deepest->left;
                parent_of_deepest->left = nullptr;
            }
        }
    }
};
