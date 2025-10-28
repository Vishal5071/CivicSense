#pragma once

#include <iostream>
#include <stdexcept>

// --- Templated Node Structure for the Linked List ---
template <typename T>
struct StackNode {
    T data;
    StackNode<T>* next;

    StackNode(T val) : data(val), next(nullptr) {}
};

// --- Templated Stack Class Declaration and Implementation ---
template <typename T>
class Stack {
private:
    StackNode<T>* top;
    int elements;

public:
    // Constructor to initialize an empty stack
    Stack() : top(nullptr), elements(0) {}

    // Destructor to free all memory
    ~Stack() {
        while (!isEmpty()) {
            pop();
        }
    }

    // Pushed a new value onto the stack
    void push(T data) {
        StackNode<T>* newNode = new StackNode<T>(data);
        newNode->next = top;
        top = newNode;
        elements++;
    }

    // Pops top value from the stack
    T pop() {
        if (isEmpty()) {
            throw std::out_of_range("Stack underflow: cannot pop from an empty stack.");
        }
        StackNode<T>* temp = top;
        T value=top->data;
        top = top->next;
        delete temp;
        elements--;
        return value;
    }

    // Gives referance to the top value
    const T& peek() const {
        if (isEmpty()) {
            throw std::out_of_range("Stack is empty: cannot peek.");
        }
        return top->data;
    }

    // Checks if the stack is empty
    bool isEmpty() const {
        return top == nullptr;
    }

    // Returns the size of the stack
    int size() const {
        return elements;
    }
};