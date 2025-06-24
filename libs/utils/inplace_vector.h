#ifndef INPLACE_VECTOR_H
#define INPLACE_VECTOR_H

#include <cstring>
#include <stdexcept>

namespace StereoVisionApp {

/*!
 * \brief The InPlaceVector class is a simplified vector which is garanteed to not move memory around between reserve operations
 *
 * The usage of the class is that before using the class, the reserve method needs to be called.
 * Reserve allocate a block of memory. The vector can then be resized within the limits of the allocated memory.
 * The addresses of the objects in the vector are guaranteed to not change between reserve operations.
 *
 * Trying to resize beyond the limit of the allocated block is undefined behavior.
 * For debuggind purposes, a bool template parameter can be set to true to catch out of bound errors for any undefined behavior.
 */
template <typename T, bool debugOn = false>
class InPlaceVector {
public:
    InPlaceVector() :
        _data(nullptr),
        _currentSize(0),
        _currentAllocated(0)
    {

    }
    template<bool debug>
    InPlaceVector(InPlaceVector<T, debug> const& other) :
        _data(nullptr),
        _currentSize(other.size()),
        _currentAllocated(other._currentAllocated)
    {
        _data = new T[_currentSize];
        std::memcpy(_data, other._data, _currentSize);
    }
    template<bool debug>
    InPlaceVector(InPlaceVector<T, debug> && other) :
        _data(other._data),
        _currentSize(other._currentSize),
        _currentAllocated(other._currentAllocated)
    {
        other._data = nullptr;
        other._currentSize = 0;
        other._currentAllocated = 0;
    }
    template<bool debug>
    InPlaceVector<T,debugOn>& operator=(InPlaceVector<T, debug> const& other)
    {
        if (_data != nullptr) {
            delete [] _data;
        }

        _currentSize = other._currentSize;
        _currentAllocated = other._currentAllocated;

        _data = new T[_currentSize];
        std::memcpy(_data, other._data, _currentSize);

        return *this;
    }

    template<bool debug>
    InPlaceVector<T,debugOn>& operator=(InPlaceVector<T, debug> && other)
    {
        if (_data != nullptr) {
            delete [] _data;
        }

        _data = other._data;
        _currentSize = other._currentSize;
        _currentAllocated = other._currentAllocated;

        other._data = nullptr;
        other._currentSize = 0;
        other._currentAllocated = 0;

        return *this;
    }

    ~InPlaceVector() {
        if (_data != nullptr) {
            delete [] _data;
        }
    }

    void reserve(int size) {

        if constexpr (debugOn) {
            if (size < 0) {
                throw std::out_of_range("Trying to reserve negative memory!");
            }
        }

        if (_currentAllocated < size) {
            T* newBuffer = new T[size];
            _currentAllocated = size;
            if (_data != nullptr) {
                std::memcpy(newBuffer, _data, _currentSize);
                delete [] _data;
            }
            _data = newBuffer;
        }
    }

    void clear() {
        resize(0);
    }

    void resize(int size) {

        if constexpr (debugOn) {
            if (size < 0) {
                throw std::out_of_range("Trying to resize to negative size!");
            }
        }

        if (size < _currentAllocated) {
            _currentSize = size;
        }
    }

    void push_back(T const& val) {

        if constexpr (debugOn) {
            if (_currentSize >= _currentAllocated) {
                throw std::out_of_range("Trying to push back beyond allocated memory!");
            }
        }

        if (_currentSize < _currentAllocated) {
            _data[_currentSize] = val;
            _currentSize++;
        }
    }

    T& operator[](int idx) {

        if constexpr (debugOn) {
            if (idx < 0 or idx >= _currentSize) {
                throw std::out_of_range("Index out of range!");
            }
        }
        return _data[idx];
    }

    T const& operator[](int idx) const {

        if constexpr (debugOn) {
            if (idx < 0 or idx >= _currentSize) {
                throw std::out_of_range("Index out of range!");
            }
        }

        return _data[idx];
    }

    inline int size() const {
        return _currentSize;
    }

    inline int allocated() const {
        return _currentAllocated;
    }

protected:

    T* _data;

    int _currentSize;
    int _currentAllocated;
};

}

#endif // INPLACE_VECTOR_H
