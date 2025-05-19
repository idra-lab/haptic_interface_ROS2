#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP
#include <iostream>
#include <stdexcept>
#include <vector>

// Template class for Circular Buffer
template <typename T>
class CircularBuffer {
 public:
  // Default constructor
  CircularBuffer() : capacity_(0), head_(0), tail_(0), size_(0) {}

  // Constructor with capacity
  CircularBuffer(size_t capacity)
      : capacity_(capacity), buffer_(capacity), head_(0), tail_(0), size_(0) {
    if (capacity_ == 0) {
      throw std::invalid_argument("Capacity must be greater than zero.");
    }
  }

  // Initialize buffer with capacity
  void initialize(size_t capacity) {
    if (capacity == 0) {
      throw std::invalid_argument("Capacity must be greater than zero.");
    }
    capacity_ = capacity;
    buffer_.resize(capacity);
    head_ = tail_ = size_ = 0;
  }

  // Add an element to the buffer
  void push(const T& value) {
    ensureInitialized();
    buffer_[head_] = value;
    head_ = (head_ + 1) % capacity_;
    if (size_ == capacity_) {
      tail_ = (tail_ + 1) % capacity_;  // Overwrite the oldest element
    } else {
      ++size_;
    }
  }

  // Remove and return the oldest element
  T pop() {
    ensureInitialized();
    if (isEmpty()) {
      throw std::runtime_error("Buffer is empty.");
    }
    T value = buffer_[tail_];
    tail_ = (tail_ + 1) % capacity_;
    --size_;
    return value;
  }

  // Get the oldest element without removing it
  T peek() const {
    ensureInitialized();
    if (isEmpty()) {
      throw std::runtime_error("Buffer is empty.");
    }
    return buffer_[tail_];
  }

  // Check if the buffer is empty
  bool isEmpty() const { return size_ == 0; }

  // Check if the buffer is full
  bool isFull() const { return size_ == capacity_; }

  // Get the current size of the buffer
  size_t size() const { return size_; }

  // Get the capacity of the buffer
  size_t capacity() const { return capacity_; }

  // Access an element in the buffer
  T& operator[](size_t i) {
    ensureInitialized();
    if (i >= size_) {
      throw std::out_of_range("Index out of range.");
    }
    return buffer_[(tail_ + i) % capacity_];
  }
  // Access an element in the buffer
  const T& operator[](size_t i) const {
    ensureInitialized();
    if (i >= size_) {
      throw std::out_of_range("Index out of range.");
    }
    return buffer_[(tail_ + i) % capacity_];
  }

 private:
  size_t capacity_;
  std::vector<T> buffer_;
  size_t head_;
  size_t tail_;
  size_t size_;

  void ensureInitialized() const {
    if (capacity_ == 0) {
      throw std::runtime_error(
          "Buffer not initialized. Call initialize() first.");
    }
  }
};
#endif  // CIRCULAR_BUFFER_HPP