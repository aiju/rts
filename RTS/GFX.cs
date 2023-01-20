using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;

namespace RTS
{
    internal class DrawContext
    {
        GraphicsDevice device;
        BasicEffect effect;
        public DrawContext(GraphicsDevice device)
        {
            this.device = device;
            effect = new BasicEffect(device);
            effect.VertexColorEnabled = true;
            effect.Projection = Matrix.CreateOrthographicOffCenter(0, device.Viewport.Width, device.Viewport.Height, 0, 0, 1);
        }
        public void Translate(Vector2 pos)
        {
            effect.World *= Matrix.CreateTranslation(pos.X, pos.Y, 0);
        }
        public void Line(Vector2 a, Vector2 b, Color color)
        {
            Path(new Vector2[2] { a, b }, color);
        }
        public void Path(Vector2[] vertices, Color color)
        {
            int n = vertices.Count();
            if (n < 2) return;
            var vert = new VertexPositionColor[n];
            for(int i = 0; i < n; i++)
            {
                vert[i].Position = new Vector3(vertices[i].X, vertices[i].Y, 0);
                vert[i].Color = color;
            }
            foreach (EffectPass effectPass in effect.CurrentTechnique.Passes)
            {
                effectPass.Apply();
                device.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineStrip, vert, 0, n - 1);
            }
        }
        public void Polygon(Vector2[] vertices, Color color, bool filled)
        {
            int n = vertices.Count();
            var vert = new VertexPositionColor[n];
            for (int i = 0; i < n; i++)
            {
                vert[i].Position = new Vector3(vertices[i].X, vertices[i].Y, 0);
                vert[i].Color = color;
            }
            if (filled)
            {
                var ind = new int[3 * (n - 2)];
                for (int i = 0; i < n - 2; i++)
                {
                    ind[3 * i] = 0;
                    ind[3 * i + 1] = i + 1;
                    ind[3 * i + 2] = i + 2;
                }
                foreach (EffectPass effectPass in effect.CurrentTechnique.Passes)
                {
                    effectPass.Apply();
                    device.DrawUserIndexedPrimitives<VertexPositionColor>(PrimitiveType.TriangleList, vert, 0, n, ind, 0, n - 2);
                }
            }
            else
            {
                var ind = new int[n + 1];
                for (int i = 0; i < n; i++)
                    ind[i] = i;
                ind[n] = 0;
                foreach (EffectPass effectPass in effect.CurrentTechnique.Passes)
                {
                    effectPass.Apply();
                    device.DrawUserIndexedPrimitives<VertexPositionColor>(PrimitiveType.LineStrip, vert, 0, n, ind, 0, n);
                }
            }
        }
        public void RegularPolygon(Vector2 centre, int n, float size, float angle, Color color, bool filled)
        {
            var vertices = new Vector2[n];
            for (int i = 0; i < n; i++)
                vertices[i] = new Vector2(
                        centre.X + size * MathF.Cos(2 * MathF.PI * i / n + angle * MathF.PI / 180),
                        centre.Y + size * MathF.Sin(2 * MathF.PI * i / n + angle * MathF.PI / 180)
                    );
            Polygon(vertices, color, filled);
        }
    }
}
