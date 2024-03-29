#pragma once
#include <memory>
#include <vector>

//----------------------------------------------------------------------------------------------------------------------
// circular_buffer
//----------------------------------------------------------------------------------------------------------------------

/// Circular buffer class.
/// Ported from RAVE VST's circular_buffer -- see https://github.com/acids-ircam/rave_vst/blob/main/source/CircularBuffer.h

template <class in_type, class out_type> class circular_buffer {
public:
  circular_buffer();
  void initialize(size_t size);
  bool empty();
  bool full();
  void put(in_type *input_array, int N);
  void get(out_type *output_array, int N);
  void push(in_type input);
  out_type pop();
  out_type peek(int i);
  int len();
  void reset();

protected:
  std::vector<out_type> _buffer;
  size_t _max_size;
  size_t _head = 0;
  size_t _tail = 0;
  int _count = 0;
  bool _full = false;
};

template <class in_type, class out_type>
circular_buffer<in_type, out_type>::circular_buffer() {}

template <class in_type, class out_type>
void circular_buffer<in_type, out_type>::initialize(size_t size) {
  _buffer.resize(size);
  _max_size = size;
}

template <class in_type, class out_type>
int circular_buffer<in_type, out_type>::len() {
  return _count;
}

template <class in_type, class out_type>
bool circular_buffer<in_type, out_type>::empty() {
  return (!_full && _head == _tail);
}

template <class in_type, class out_type>
bool circular_buffer<in_type, out_type>::full() {
  return _full;
}

template <class in_type, class out_type>
void circular_buffer<in_type, out_type>::put(in_type *input_array, int N) {
  if (!_max_size)
    return;

  while (N--) {
    _buffer[_head] = out_type(*(input_array++));
    _head = (_head + 1) % _max_size;
    if (_full)
      _tail = (_tail + 1) % _max_size;
    _full = _head == _tail;
    _count++;
  }
}

template <class in_type, class out_type>
void circular_buffer<in_type, out_type>::get(out_type *output_array, int N) {
  if (!_max_size)
    return;

  while (N--) {
    if (empty()) {
      *(output_array++) = out_type();
    } else {
      *(output_array++) = _buffer[_tail];
      _tail = (_tail + 1) % _max_size;
      _full = false;
    }
    _count--;
  }
}

template <class in_type, class out_type>
void circular_buffer<in_type, out_type>::push(in_type input) {
  if (!_max_size)
    return;

  _buffer[_head] = out_type(input);
  _head = (_head + 1) % _max_size;
  if (_full)
    _tail = (_tail + 1) % _max_size;
  _full = _head == _tail;
  _count++;
}

template <class in_type, class out_type>
out_type circular_buffer<in_type, out_type>::pop() {
    out_type value = out_type();

    if (!_max_size) {
        return value;
    }

    if (!empty()) {
        value = _buffer[_tail];
        _tail = (_tail + 1) % _max_size;
        _full = false;
    }

    _count--;

    return value;
}

template <class in_type, class out_type>
out_type circular_buffer<in_type, out_type>::peek(int i) {
  out_type value = out_type();

  if (!_max_size) {
    return value;
  }

  if (!empty()) {
    value = _buffer[(_tail + i) % _max_size];
  }

  return value;
}

template <class in_type, class out_type>
void circular_buffer<in_type, out_type>::reset() {
  _head = _tail;
  _count = 0;
  _full = false;
}
