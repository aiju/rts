using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading.Channels;
using System.Drawing;
using System.Reflection.Metadata.Ecma335;
using System.Reflection.Metadata;

namespace RTS
{
    internal class Shape
    {
        VertexPositionColor[] vertices;
        int[] indices;
        GraphicsDevice device;
        BasicEffect effect;
        enum Type { Lines, ClosedLines, Filled };
        Type type;
        private Shape(GraphicsDevice device)
        {
            this.device = device;
            type = Type.Lines;
            effect = new BasicEffect(device);
            effect.VertexColorEnabled = true;
            effect.Projection = Matrix.CreateOrthographicOffCenter(0, device.Viewport.Width, device.Viewport.Height, 0, 0, 1);
        }
        public static Shape RegularPolygon(GraphicsDevice device, int n, float size, float angle = 0)
        {
            Shape s = new Shape(device);
            s.vertices = new VertexPositionColor[n];
            for (int i = 0; i < n; i++)
            {
                s.vertices[i].Position = new Vector3(
                    size * MathF.Cos(2 * MathF.PI * i / n + angle * MathF.PI / 180),
                    size * MathF.Sin(2 * MathF.PI * i / n + angle * MathF.PI / 180),
                    0);
                s.vertices[i].Color = Microsoft.Xna.Framework.Color.Black;
            }
            s.type = Type.ClosedLines;
            return s;
        }
        public static Shape FromVertices(GraphicsDevice device, Vector2[] vertices)
        {
            Shape s = new Shape(device);
            s.vertices = new VertexPositionColor[vertices.Length];
            for (int i = 0; i < vertices.Length; i++)
            {
                s.vertices[i].Position = new Vector3(vertices[i].X, vertices[i].Y, 0);
                s.vertices[i].Color = Microsoft.Xna.Framework.Color.Black;
            }
            s.type = Type.Lines;
            return s;
        }
        public Shape Color(Microsoft.Xna.Framework.Color c)
        {
            for (int i = 0; i < vertices.Length; i++)
                vertices[i].Color = c;
            return this;
        }
        public Shape Close()
        {
            type = Type.ClosedLines;
            return this;
        }
        public Shape Fill()
        {
            type = Type.Filled;
            return this;
        }
        public Shape Translate(Vector2 pos)
        {
            effect.World = Matrix.CreateTranslation(pos.X, pos.Y, 0);
            return this;
        }
        public Shape Transform(Func<Vector2, Vector2> f)
        {
            for (int i = 0; i < vertices.Length; i++)
            {
                var v = f(new Vector2(vertices[i].Position.X, vertices[i].Position.Y));
                vertices[i].Position = new Vector3(v.X, v.Y, 0);
            }
            return this;
        }
        private void build() {
            if (type == Type.Filled)
            {
                int n = vertices.Length;
                indices = new int[3 * (n - 2)];
                for (int i = 0; i < n - 2; i++)
                {
                    indices[3 * i] = 0;
                    indices[3 * i + 1] = i + 1;
                    indices[3 * i + 2] = i + 2;
                }
            }
            else
            {
                int n = vertices.Length;
                int m = type == Type.ClosedLines ? n + 1 : n;
                indices = new int[m]; ;
                for (int i = 0; i < m; i++)
                    indices[i] = i % n;
            }
        }
        public void Draw(GraphicsDevice device)
        {
            build();
            foreach (EffectPass effectPass in effect.CurrentTechnique.Passes)
            {
                effectPass.Apply();
                if (type != Type.Filled)
                    device.DrawUserIndexedPrimitives<VertexPositionColor>(PrimitiveType.LineStrip, vertices, 0, vertices.Count(), indices, 0, indices.Count() - 1);
                else
                    device.DrawUserIndexedPrimitives<VertexPositionColor>(PrimitiveType.TriangleList, vertices, 0, vertices.Count(), indices, 0, indices.Count() / 3);
            }
        }
    }

    internal class Polygon : IDrawAt
    {
        VertexPositionColor[] _vertices;
        int[] _indices;
        BasicEffect _effect;
        GraphicsDevice _device;
        bool _filled;
        public Polygon(GraphicsDevice device, int n, float size, float angle, Microsoft.Xna.Framework.Color color, bool filled = false)
        {
            _filled = filled;
            _device = device;
            _effect = new BasicEffect(device);
            _effect.VertexColorEnabled = true;
            _effect.Projection = Matrix.CreateOrthographicOffCenter(0, device.Viewport.Width, device.Viewport.Height, 0, 0, 1);
            _vertices = new VertexPositionColor[n];
            for (int i = 0; i < n; i++)
            {
                _vertices[i].Position = new Vector3(
                    size * MathF.Cos(2 * MathF.PI * i / n + angle * MathF.PI / 180),
                    size * MathF.Sin(2 * MathF.PI * i / n + angle * MathF.PI / 180),
                    0);
                _vertices[i].Color = color;
            }
            if (_filled)
            {
                _indices = new int[3 * (n - 2)];
                for (int i = 0; i < n - 2; i++)
                {
                    _indices[3 * i] = 0;
                    _indices[3 * i + 1] = i + 1;
                    _indices[3 * i + 2] = i + 2;
                }
            }
            else
            {
                _indices = new int[n + 1];
                for (int i = 0; i <= n; i++)
                    _indices[i] = i % n;
            }
        }
        public Polygon(GraphicsDevice device, Vector2[] vertices, Microsoft.Xna.Framework.Color color, bool filled = false)
        {
            _filled = filled;
            _device = device;
            _effect = new BasicEffect(device);
            _effect.VertexColorEnabled = true;
            _effect.Projection = Matrix.CreateOrthographicOffCenter(0, device.Viewport.Width, device.Viewport.Height, 0, 0, 1);
            _vertices = new VertexPositionColor[vertices.Count()];
            var n = vertices.Count();
            for (int i = 0; i < n; i++)
            {
                _vertices[i].Position = new Vector3(vertices[i].X, vertices[i].Y, 0);
                _vertices[i].Color = color;
            }
            if (_filled)
            {
                _indices = new int[3 * (n - 2)];
                for (int i = 0; i < n - 2; i++) {
                    _indices[3 * i] = 0;
                    _indices[3 * i + 1] = i + 1;
                    _indices[3 * i + 2] = i + 2;
                }
            }
            else
            {
                _indices = new int[n + 1];
                for (int i = 0; i <= n; i++)
                    _indices[i] = i % n;
            }
        }
        public void DrawAt(GraphicsDevice device, Vector2 pos)
        {
            _effect.World = Matrix.CreateTranslation(new Vector3(32 * pos.X, 32 * pos.Y, 0));
            foreach (EffectPass effectPass in _effect.CurrentTechnique.Passes)
            {
                effectPass.Apply();
                if(!_filled)
                    _device.DrawUserIndexedPrimitives<VertexPositionColor>(PrimitiveType.LineStrip, _vertices, 0, _vertices.Count(), _indices, 0, _indices.Count() - 1);
                else
                    _device.DrawUserIndexedPrimitives<VertexPositionColor>(PrimitiveType.TriangleList, _vertices, 0, _vertices.Count(), _indices, 0, _indices.Count() / 3);
            }
        }
    }
}
