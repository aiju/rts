using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RTS
{
    public class MultiMap<TKey, TValue>
    {
        Dictionary<TKey, List<TValue>> map;
        public MultiMap() { map = new Dictionary<TKey, List<TValue>>(); }
        public void Add(TKey key, TValue value)
        {
            List<TValue> list;
            if (map.TryGetValue(key, out list))
                list.Add(value);
            else
                map.Add(key, new List<TValue>() { value });
        }
        public void Remove(TKey key, TValue value)
        {
            if (map.TryGetValue(key, out var list))
                list.Remove(value);
        }
        public IEnumerable<TValue> this[TKey key]
        {
            get
            {
                if (map.TryGetValue(key, out var list))
                    return list;
                return Enumerable.Empty<TValue>();
            }
        }
        public bool Contains(TKey key, TValue value)
        {
            if (map.TryGetValue(key, out var list))
                return list.Contains(value);
            return false;
        }
    }
    internal class BinaryHeap<TElement, TPriority> where TPriority : IComparable
    {
        List<(TElement, TPriority)> heap;
        Dictionary<TElement, int> map;
        public BinaryHeap()
        {
            heap = new List<(TElement, TPriority)>();
            map = new Dictionary<TElement, int>();
        }
        private int parent(int i) { Debug.Assert(i != 0); return (i - 1) / 2; }
        private int left(int i) { return 2 * i + 1; }
        private int right(int i) { return 2 * i + 2; }
        private TPriority priority(int i) { var (_, p) = heap[i]; return p; }
        private int compare(int i, int j) { return priority(i).CompareTo(priority(j)); }
        private void swap(int i, int j)
        {
            var (ei, pi) = heap[i];
            var (ej, pj) = heap[j];
            (heap[j], heap[i]) = ((ei, pi), (ej, pj));
            map[ei] = j;
            map[ej] = i;
        }
        private void bubbleUp(int i)
        {
            while (i != 0 && compare(i, parent(i)) < 0)
            {
                swap(i, parent(i));
                i = parent(i);
            }
        }
        private void bubbleDown(int i)
        {
            for (; ; )
            {
                var smallest = i;
                if (left(i) < heap.Count && compare(left(i), smallest) < 0)
                    smallest = left(i);
                if (right(i) < heap.Count && compare(right(i), smallest) < 0)
                    smallest = right(i);
                if (smallest == i) break;
                swap(i, smallest);
                i = smallest;
            }
        }
        public void Enqueue(TElement elem, TPriority prio)
        {
            if (map.TryGetValue(elem, out int index))
            {
                var oldp = priority(index);
                heap[index] = (elem, prio);
                if (prio.CompareTo(oldp) < 0) bubbleUp(index);
                else if (prio.CompareTo(oldp) > 0) bubbleDown(index);
            }
            else
            {
                int n = heap.Count;
                heap.Add((elem, prio));
                map.Add(elem, n);
                bubbleUp(n);
            }
        }
        public bool TryDequeue(out TElement element, out TPriority priority)
        {
            if (heap.Count == 0)
            {
                element = default(TElement);
                priority = default(TPriority);
                return false;
            }
            (element, priority) = heap[0];
            swap(0, heap.Count - 1);
            heap.RemoveAt(heap.Count - 1);
            map.Remove(element);
            bubbleDown(0);
            return true;
        }
    }
}
