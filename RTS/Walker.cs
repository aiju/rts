using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RTS
{
    interface IWalker
    {
        public void GoTo(Vector2 target, Action onDone, Action onCancel);
        public void Cancel();
    }
    internal class Walker : IWalker
    {
        public Vector2 Position { get; private set; }
        public Vector2 Target { get; private set; }
        public bool Walking { get; private set; }
        public float Velocity { get; set; }
        public event Action OnPositionUpdate;
        Action _onDone;
        Action _onCancel;
        public Walker(Vector2 Position, float Velocity)
        {
            this.Position = Position;
            this.Target = Position;
            this.Walking = false;
            this.Velocity = Velocity;
        }
        public void Update(GameTime gameTime)
        {
            if (this.Walking)
            {
                float step = Velocity * (float)gameTime.ElapsedGameTime.TotalSeconds;
                if (Vector2.Distance(Position, Target) < step)
                    Done();
                else
                {
                    Vector2 v = Target - Position;
                    v.Normalize();
                    Position += v * step;
                    OnPositionUpdate?.Invoke();
                }
            }
        }
        public void GoTo(Vector2 target, Action onDone, Action onCancel)
        {
            if (Walking) Cancel();
            Target = target;
            Walking = true;
            _onDone = onDone;
            _onCancel = onCancel;
            if (Position == Target) Done();
        }
        public void Cancel()
        {
            if (Walking)
            {
                var f = _onCancel;
                _onDone = null;
                _onCancel = null;
                Walking = false;
                f?.Invoke();
            }
        }
        void Done()
        {
            Position = Target;
            var f = _onDone;
            _onDone = null;
            _onCancel = null;
            Walking = false;
            OnPositionUpdate?.Invoke();
            f?.Invoke();
        }
    }
}
