using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.IO;

namespace mapf;

/// <summary>
/// Supporting O(1) insertion and removal of items that compare equal to the top of the heap.
/// TODO: Compare against a bucketed implementation where all equal costs are bucketed in a queue,
///       even though it might not respect all tie-breaking (but still prioritizes goal nodes).
/// </summary>
[DebuggerDisplay("Count = {Count}")]
public class OpenList<Item> : IAccumulatingStatisticsCsvWriter
{
    private Queue<Item> _queue;
    protected BinaryHeap<Item> _heap;
        
    protected ISolver _user;  // For updating its stats
    private int _quickInsertionCount;
    private int _accQuickInsertionCount;

    private int _quickInsertionsCancelled;
    private int _accQuickInsertionsCancelled;

    private readonly IComparer<Item> _comparer;

    public OpenList(ISolver user, IComparer<Item> comparer)
    {
        _heap = new BinaryHeap<Item>(_comparer);
        _comparer = comparer;
        _queue = new Queue<Item>();

        _user = user;
        ClearPrivateStatistics();
        ClearPrivateAccumulatedStatistics();
        _comparer = comparer;
    }

    public int Count
    {
        get { return _heap.Count + _queue.Count; }
    }

    public Item Peek()
    {
        if (_queue.Count != 0)
            return _queue.Peek();
        return _heap.Peek();
    }

    public void Clear()
    {
        _queue.Clear();
        _heap.Clear();
    }

    public void Add(Item item)
    {
        if (_queue.Count == 0)
        {
            if (_heap.Count == 0)
                _heap.Add(item); // It's very cheap.
            else
            {
                int compareRes = _comparer.Compare(item, _heap.Peek());
                if (compareRes != -1) // Even if equal, respect the stable order, don't "cut the line".
                    _heap.Add(item);
                else
                {
                    _queue.Enqueue(item);
                    _quickInsertionCount++;
                }
            }
        }
        else
        {
            int compareRes = _comparer.Compare(item, _queue.Peek());
            if (compareRes == 1) // item is larger than the queue
                _heap.Add(item);
            else // 
            {
                if (compareRes == -1) // Item is smaller than the queue
                {
                    while (_queue.Count != 0)
                    {
                        Item fromQueue = _queue.Dequeue();
                        _heap.Add(fromQueue);
                        _quickInsertionCount--;
                        _quickInsertionsCancelled++;
                    }
                }
                _queue.Enqueue(item);
                _quickInsertionCount++;
            }
        }

        //// The last removed item is the parent of all items added until another item is removed,
        //// or the same node that was last removed, partially expanded or deferred with increased cost.
        //// Otherwise the inserted item is one that was already in the open list, and its cost was
        //// was increased by one of the children of the last removed item. In this case, since the item
        //// wasn't the min of the open list and the last removed item was, now, with its increased cost,
        //// it certainly won't be smaller than the last removed item.
        //// If a partially expanded or otherwise deferred node is re-inserted with an updated cost,
        //// that must be done after all its children generated so far are inserted. Otherwise the
        //// cost comparison with the last removed item, which would still be the same node, would have
        //// incorrect results.
    }

    public virtual Item Remove()
    {
        Item item;
        if (_queue.Count != 0)
        {
            item = _queue.Dequeue();
        }
        else
            item = _heap.Remove();
        return item;
    }

    /// <summary>
    /// Uses Equality check only for removing from the queue.
    /// Might cost O(n) if all items are in the queue and not the heap.
    /// </summary>
    /// <param name="item"></param>
    /// <returns></returns>
    public bool Remove(Item item)
    {
        bool removedFromQueue = false;
        // Remove from the queue if it's there, keeping the order in the queue.
        for (int i = 0; i < _queue.Count; ++i )
        {
            Item temp = _queue.Dequeue();
            if (temp.Equals(item))
            {
                removedFromQueue = true;
            }
            else
                _queue.Enqueue(temp);
        }
        if (removedFromQueue == true)
            return true;

        return _heap.Remove(item);
    }

    public virtual void OutputStatisticsHeader(TextWriter output)
    {
        output.Write(_user.ToString() + " Quick Insertions");
        output.Write(Run.RESULTS_DELIMITER);
        output.Write(_user.ToString() + " Quick Insertions Cancelled");
        output.Write(Run.RESULTS_DELIMITER);
    }

    public virtual void OutputStatistics(TextWriter output)
    {
        Console.WriteLine(_user.ToString() + " Quick insertions: {0}", _quickInsertionCount);
        Console.WriteLine(_user.ToString() + " Quick insertions cancelled: {0}", _quickInsertionsCancelled);

        output.Write(_quickInsertionCount + Run.RESULTS_DELIMITER);
        output.Write(_quickInsertionsCancelled + Run.RESULTS_DELIMITER);
    }

    public virtual int NumStatsColumns
    {
        get
        {
            return 2;
        }
    }

    protected void ClearPrivateStatistics()
    {
        _quickInsertionCount = 0;
        _quickInsertionsCancelled = 0;
    }

    protected void ClearPrivateAccumulatedStatistics()
    {
        _accQuickInsertionCount = 0;
        _accQuickInsertionsCancelled = 0;
    }

    public virtual void ClearStatistics()
    {
        ClearPrivateStatistics();
    }

    public virtual void ClearAccumulatedStatistics()
    {
        ClearPrivateAccumulatedStatistics();
    }

    public virtual void AccumulateStatistics()
    {
        _accQuickInsertionCount += _quickInsertionCount;
        _accQuickInsertionsCancelled += _quickInsertionsCancelled;
    }

    public virtual void OutputAccumulatedStatistics(TextWriter output)
    {
        Console.WriteLine(_user.ToString() + " Accumulated Quick insertions: {0}", _accQuickInsertionCount);
        Console.WriteLine(_user.ToString() + " Accumulated Quick insertions cancelled: {0}", _accQuickInsertionsCancelled);

        output.Write(_accQuickInsertionCount + Run.RESULTS_DELIMITER);
        output.Write(_accQuickInsertionsCancelled + Run.RESULTS_DELIMITER);
    }

    public override string ToString()
    {
        return GetName();
    }

    public virtual string GetName()
    {
        return "OpenList";
    }
}
