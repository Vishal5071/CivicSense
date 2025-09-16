#pragma once

#include <vector>
#include <stdexcept>

template <typename T>
class PriorityQueue {
private:
    std::vector<T> heap;

    // Helper function to get the parent index
    int parent(int i) { return (i - 1) / 2; }

    // Helper function to get the left child index
    int leftChild(int i) { return (2 * i) + 1; }

    // Helper function to get the right child index
    int rightChild(int i) { return (2 * i) + 2; }

    // Helper function to swap two elements
    void swap(T& a, T& b) {
        T temp = a;
        a = b;
        b = temp;
    }

    // Moves element up the heap to maintain heap property
    void heapifyUp(int i) {
        while (i > 0 && heap[i] < heap[parent(i)]) {
            swap(heap[i], heap[parent(i)]);
            i = parent(i);
        }
    }

    // Moves element down the heap to maintain heap property
    void heapifyDown(int i) {
        int left = leftChild(i);
        int right = rightChild(i);
        int smallest = i;

        if (left < heap.size() && heap[left] < heap[smallest]) {
            smallest = left;
        }
        if (right < heap.size() && heap[right] < heap[smallest]) {
            smallest = right;
        }

        if (smallest != i) {
            swap(heap[i], heap[smallest]);
            heapifyDown(smallest);
        }
    }

public:
    PriorityQueue() {}

    // Pushes new element
    void push(const T& value) {
        heap.push_back(value);
        heapifyUp(heap.size() - 1);
    }

    // Removes element with highest priority (smallest value)
    void pop() {
        if (isEmpty()) {
            throw std::out_of_range("PriorityQueue is empty.");
        }
        heap[0] = heap[heap.size() - 1];
        heap.pop_back();
        
        if (!isEmpty()) {
            heapifyDown(0);
        }
    }

    // Gives reference to top element (smallest value)
    const T& top() const {
        if (isEmpty()) {
            throw std::out_of_range("PriorityQueue is empty.");
        }
        return heap.at(0);
    }

    // Checks if empty
    bool isEmpty() const {
        return heap.empty();
    }

    // Gives number of elements
    int size() const {
        return heap.size();
    }
};
