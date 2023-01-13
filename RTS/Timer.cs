using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RTS
{
    internal class Timer
    {
        List<(TimeSpan, Action)> queue;
        public Timer()
        {
            queue = new List<(TimeSpan, Action)>();
        }
        public void Add(TimeSpan delay, Action action)
        {
            TimeSpan t = delay;
            for(int i = 0; i < queue.Count; i++)
            {
                var (s, act) = queue[i];
                if (t < s)
                {
                    queue.Insert(i, (t, action));
                    queue[i + 1] = (s - t, act);
                    return;
                }
                else
                    t -= s;
            }
            queue.Add((t, action));
        }
        public void Update(GameTime gameTime)
        {
            TimeSpan t = gameTime.ElapsedGameTime;
            while(queue.Count > 0)
            {
                var (s, act) = queue[0];
                if(s <= t)
                {
                    t -= s;
                    queue.RemoveAt(0);
                    act();
                }
                else
                {
                    queue[0] = (s - t, act);
                    break;
                }
            }
        }
    }
}
