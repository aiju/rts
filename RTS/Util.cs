using System;
using System.Collections.Generic;
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
}
