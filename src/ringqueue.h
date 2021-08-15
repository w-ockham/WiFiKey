/*
* Simple Ring Queue buffer
*/
#ifndef ARDUINO_RING_QUEUE
#define ARDUINO_RING_QUEUE

template <class T>
class RingQueue
{
private:
    int _head, _tail, _size, _max_length;
    T *_queue;

public:
    RingQueue(int max = 128)
    {
        _head = 0;
        _tail = 0;
        _max_length = 0;
        _size = max;
        _queue = new T[max];
    }
    ~RingQueue()
    {
        delete[] _queue;
    }
    inline boolean isEmpty();
    inline boolean isFull();
    int length();
    int maxLength();
    int enqueue(const T &d);
    void dequeue(T &d);
};

template<class T>
inline boolean RingQueue<T>::isEmpty()
{
    return (_head == _tail);
}

template<class T>
inline boolean RingQueue<T>::isFull()
{
    return (_head == ((_tail + 1) % _size));
}

template<class T>
int RingQueue<T>::length()
{
    int c = _tail - _head;
    if (c < 0)
        c += _size;
    if (c > _max_length)
        _max_length = c;
    return c;
}

template<class T>
int RingQueue<T>::maxLength()
{
    int c = _max_length;
    _max_length = 0;
    return c;
}

template<class T>
int RingQueue<T>::enqueue(const T &d)
{
    if (isFull())
        return 0;
    _queue[_tail] = d;
    _tail = (_tail + 1) % _size;

    return length();
}

template<class T>
void RingQueue<T>::dequeue(T &d)
{
    if (isEmpty())
        return;
    d = _queue[_head];
    _head = (_head + 1) % _size;
}
#endif