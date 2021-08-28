/*
* Simple Ring Queue buffer
*/
#ifndef ARDUINO_WIFIKEY_RING_QUEUE
#define ARDUINO_WIFIKEY_RING_QUEUE

template <class T>
class RingQueue
{
private:
    int _head, _tail, _size, _max_length;
    int _phead, _ptail;
    T *_queue, *_pqueue;
    unsigned long *_seqnum, *_pseqnum;

public:
    RingQueue(int max = 64)
    {
        _head = 0;
        _tail = 0;
        _phead = 0;
        _ptail = 0;
        _max_length = 0;
        _size = max;
        _queue = new T[max];
        _pqueue = new T[max];
        _seqnum = new unsigned long[max];
        _pseqnum = new unsigned long[max];
    }
    ~RingQueue()
    {
        delete[] _queue;
        delete[] _pqueue;
        delete[] _seqnum;
        delete[] _pseqnum;
    }

    inline boolean isEmpty()
    {
        return (_head == _tail);
    };

    inline boolean isFull()
    {
        return (_head == ((_tail + 1) % _size));
    };

    inline boolean ispEmpty()
    {
        return (_phead == _ptail);
    };

    inline boolean ispFull()
    {
        return (_phead == ((_ptail + 1) % _size));
    };

    int length();
    int maxLength();
    int enqueue(unsigned long, const T &d);
    inline int enqueue(const T &d)
    {
        enqueue(0, d);
        return length();
    }
    void pri_enqueue(unsigned long, const T &d);

    inline unsigned long pri_peek()
    {
        return _pseqnum[_phead];
    }

    void pri_dequeue(T &d);
    void dequeue(T &d);
};

template <class T>
int RingQueue<T>::length()
{
    int c = _tail - _head;
    if (c < 0)
        c += _size;
    if (c > _max_length)
        _max_length = c;
    return c;
}

template <class T>
int RingQueue<T>::maxLength()
{
    int c = _max_length;
    _max_length = 0;
    return c;
}

template <class T>
int RingQueue<T>::enqueue(unsigned long n, const T &d)
{
    if (isFull())
        return 0;

    _queue[_tail] = d;
    _seqnum[_tail] = n;
    _tail = (_tail + 1) % _size;

    return length();
}

template <class T>
void RingQueue<T>::pri_enqueue(unsigned long n, const T &d)
{
    if (ispFull())
        return;

    _pqueue[_ptail] = d;
    _pseqnum[_ptail] = n;
    _ptail = (_ptail + 1) % _size;
}

template <class T>
void RingQueue<T>::pri_dequeue(T &d)
{
    if (ispEmpty())
        return;

    d = _pqueue[_phead];
    _pseqnum[_phead] = 0;
    _phead = (_phead + 1) % _size;
}

template <class T>
void RingQueue<T>::dequeue(T &d)
{
    if (isEmpty())
        return;

    if (ispEmpty())
    {
        d = _queue[_head];
        _seqnum[_head] = 0;
        _head = (_head + 1) % _size;
    }
    else
    {
        unsigned long pseq = pri_peek();
        if (_seqnum[_head] >= pseq)
        {
            if (_seqnum[_head] == pseq)
                dequeue(d);

            pri_dequeue(d);
            return;
        }
        else
        {
            d = _queue[_head];
            _seqnum[_head] = 0;
            _head = (_head + 1) % _size;
        }
    }
}
#endif