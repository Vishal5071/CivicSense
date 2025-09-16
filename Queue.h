#pragma once

#include <iostream>
#include<stdexcept>

// --- Templated Node Structure for the Linked List ---
template <typename T>
struct QueueNode {
    T data;
    QueueNode<T>* next;

    // Constructor
    QueueNode(T val) : data(val), next(nullptr) {}
};

// --- Templated Queue Class Declaration and Implementation ---
template <typename T>
class Queue {
private:
    QueueNode<T>* front;
    QueueNode<T>* rear;
    int elements;

public:
    // Constructor: Initializes an empty queue
    Queue() : front(nullptr), rear(nullptr), elements(0) {}

    // Destructor: Frees all allocated memory
    ~Queue() {
        while (!isEmpty()) {
            dequeue();
        }
    }

    // Checks if queue is empty
    bool isEmpty() {
        return front == nullptr;
    }

    // Gives the size of the queue
    int size() {
        return elements;
    }

    // Adds element to the rear of the queue
    void enqueue(T data) {
        QueueNode<T>* newNode = new QueueNode<T>(data);
        if (isEmpty()) {
            front = rear = newNode;
        } else {
            rear->next = newNode;
            rear = newNode;
        }
        elements++;
    }

    // Removes element from front of the queue
    T dequeue() {
        if (isEmpty()) {
            std::cerr << "Error: Cannot dequeue from an empty queue." << std::endl;
            throw std::out_of_range("Queue is empty.");
        }
        QueueNode<T>* temp = front;
        T value = temp->data;
        front = front->next;
        
        delete temp;
        
        if (front == nullptr) {
            rear = nullptr;
        }
        elements--;
        return value;
    }

    // Gives value of the front element
    T peek() {
        if (isEmpty()) {
            std::cerr << "Warning: Peeking from an empty queue." << std::endl;
            return T(); 
        }
        return front->data;
    }
};
