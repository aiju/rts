using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RTS
{
    internal static class MazeGenerator
    {
        public static void Generate(Mesh mesh, HashSet<Mesh.Edge> walls, Mesh.Face start)
        {
            HashSet<Mesh.Face> visited = new HashSet<Mesh.Face>();
            List<Mesh.Edge> wallList = new List<Mesh.Edge>();
            void visitCell(Mesh.Face cell)
            {
                var queue = new Queue<Mesh.Face>();
                queue.Enqueue(cell);
                while(queue.TryDequeue(out var face))
                {
                    if (visited.Contains(face)) continue;
                    visited.Add(face);
                    foreach (var e in face.EnumerateEdges())
                        if (walls.Contains(e))
                            wallList.Add(e);
                        else
                        {
                            var f = e.OtherFace(face);
                            if (f != null)
                                queue.Enqueue(f);
                        }
                }
            }
            visitCell(start);
            Random random = new Random();
            while(wallList.Count > 0)
            {
                int i = random.Next() % wallList.Count;
                var edge = wallList[i];
                wallList.RemoveAt(i);
                var (f0, f1) = edge.Faces;
                if (f0 == null || f1 == null) continue;
                if(visited.Contains(f0) && !visited.Contains(f1))
                {
                    visitCell(f1);
                    walls.Remove(edge);
                }
                if(visited.Contains(f1) && !visited.Contains(f0))
                {
                    visitCell(f0);
                    walls.Remove(edge);
                }
            }
        }
        public static Mesh RoundCorners(Mesh mesh, HashSet<Mesh.Edge> walls, float radius)
        {
            Mesh output = new Mesh();
            void addArc(Mesh.Vertex a, Mesh.Vertex b, Vector2 centre)
            {
                Vector2 av = a.Pos - centre;
                Vector2 bv = b.Pos - centre;
                float phi1 = MathF.Atan2(av.Y, av.X);
                float phi2 = MathF.Atan2(bv.Y, bv.X);
                float alpha = phi2 - phi1;
                if (alpha < 0) alpha += 2 * MathF.PI;
                const int n = 10;
                alpha /= n;
                var vertices = new Mesh.Vertex[n + 1];
                vertices[0] = a;
                vertices[n] = b;
                for (int i = 1; i < n; i++)
                {
                    float aa = phi1 + alpha * i;
                    vertices[i] = output.NewVertex(new Vector2(centre.X + radius * MathF.Cos(aa), centre.Y + radius * MathF.Sin(aa)));
                }
                for (int i = 0; i < n; i++)
                    output.NewEdge(vertices[i], vertices[i + 1]);
            }
            Dictionary<Mesh.Edge, Mesh.Vertex[]> vertexMap = new Dictionary<Mesh.Edge, Mesh.Vertex[]>();
            foreach(var v in mesh.Vertices())
            {
                List<(float, Mesh.Edge)> edges = new List<(float, Mesh.Edge)>();
                foreach(var e in v.Edges())
                {
                    if (!walls.Contains(e)) continue;
                    var w = e.OtherVertex(v).Pos - v.Pos;
                    float phi = MathF.Atan2(w.Y, w.X);
                    edges.Add((phi, e));
                }
                edges.Sort();
                for(int i = 0; i < edges.Count; i++)
                {
                    var e1 = edges[i].Item2;
                    var e2 = edges[(i + 1) % edges.Count].Item2;
                    var a = v.Pos;
                    var b = e1.OtherVertex(v).Pos;
                    var c = e2.OtherVertex(v).Pos;
                    Mesh.Vertex p1, p2;
                    if(b == c)
                    {
                        var n = Geometry.UnitLeftNormal(b - a) * radius;
                        p1 = output.NewVertex(a + n);
                        p2 = output.NewVertex(a - n);
                        addArc(p1, p2, a);
                    }
                    else if (Geometry.IsCollinear(a, b, c))
                    {
                        var n = Geometry.UnitLeftNormal(b - a) * radius;
                        p1 = output.NewVertex(a + n);
                        p2 = p1;
                    }
                    else if(Geometry.IsClockwise(a, c, b))
                    {
                        var n1 = Geometry.UnitLeftNormal(b - a) * radius;
                        var n2 = Geometry.UnitLeftNormal(a - c) * radius;
                        bool ok = Geometry.IntersectLines(a + n1 * 2, b + n1 * 2, a + n2 * 2, c + n2 * 2, out var centre);
                        Debug.Assert(ok);
                        p1 = output.NewVertex(centre - n1);
                        p2 = output.NewVertex(centre - n2);
                        addArc(p2, p1, centre);
                    }
                    else
                    {
                        var n1 = Geometry.UnitLeftNormal(b - a) * radius;
                        var n2 = Geometry.UnitLeftNormal(a - c) * radius;
                        p1 = output.NewVertex(a + n1);
                        p2 = output.NewVertex(a + n2);
                        addArc(p1, p2, a);
                    }
                    if (!vertexMap.ContainsKey(e1))
                        vertexMap.Add(e1, new Mesh.Vertex[4]);
                    if (!vertexMap.ContainsKey(e2))
                        vertexMap.Add(e2, new Mesh.Vertex[4]);
                    if (v == e1.Vertex1)
                        vertexMap[e1][0] = p1;
                    else
                        vertexMap[e1][3] = p1;
                    if (v == e2.Vertex1)
                        vertexMap[e2][2] = p2;
                    else
                        vertexMap[e2][1] = p2;
                }
            }
            foreach(var (edge, vertices) in vertexMap)
            {
                if (vertices[0] != null && vertices[1] != null)
                    output.NewEdge(vertices[0], vertices[1]);
                if (vertices[2] != null && vertices[3] != null)
                    output.NewEdge(vertices[2], vertices[3]);
            }
            return output;
        }
    }
    internal class MazeManipulator : IEntity
    {
        public const float Scale = 400;
        public const float Offset = 50;
        Mesh mesh;
        HashSet<Mesh.Edge> walls;
        Vector2 cursor;
        bool lastDown;
        KeyboardState lastState;
        public MazeManipulator()
        {
            mesh = new Mesh();
            walls = new HashSet<Mesh.Edge>();

            const int width = 10;
            const int height = 10;
            const float cellWidth = 1.0f / width;
            const float cellHeight = 1.0f / height;
            var vertices = new Mesh.Vertex[width + 1, height + 1];
            for (int i = 0; i <= width; i++)
                for (int j = 0; j <= height; j++)
                    vertices[i, j] = mesh.NewVertex(new Vector2(i * cellWidth, j * cellHeight));
            for(int i = 0; i < width; i++)
                for(int j = 0; j < height; j++)
                {
                    var f0 = mesh.NewFace(vertices[i, j], vertices[i + 1, j], vertices[i + 1, j + 1]);
                    var f1 = mesh.NewFace(vertices[i, j], vertices[i, j + 1], vertices[i + 1, j + 1]);
                    walls.Add(mesh.GetEdge(vertices[i, j], vertices[i + 1, j]));
                    walls.Add(mesh.GetEdge(vertices[i, j], vertices[i, j + 1]));
                    if(j == height - 1) walls.Add(mesh.GetEdge(vertices[i, j + 1], vertices[i + 1, j + 1]));
                    if(i == width - 1) walls.Add(mesh.GetEdge(vertices[i + 1, j], vertices[i + 1, j + 1]));
                }
            mesh.TryGetFace(vertices[0, 0], vertices[0, 1], vertices[1, 1], out var start);
            MazeGenerator.Generate(mesh, walls, start);
            mesh = MazeGenerator.RoundCorners(mesh, walls, 0.01f);
        }
        public void Update(GameTime gameTime)
        {
            var m = Mouse.GetState();
            var k = Keyboard.GetState();
            cursor = new Vector2(((float)m.X - Offset) / Scale, ((float)m.Y - Offset) / Scale);
            bool inRange = cursor.X > 0 && cursor.X < 1 && cursor.Y > 0 && cursor.Y < 1;
            lastDown = m.LeftButton == ButtonState.Pressed;
            lastState = k;
        }
        Vector2 toScreen(Vector2 p)
        {
            return p * Scale + new Vector2(Offset, Offset);
        }
        public void Draw(GraphicsDevice device)
        {
            var ctx = new DrawContext(device);
            foreach (Mesh.Edge v in mesh.Edges())
            {
                //if(walls.Contains(v))
                ctx.Line(toScreen(v.Vertex1.Pos), toScreen(v.Vertex2.Pos), Color.Blue);
            }
            ctx.RegularPolygon(toScreen(cursor), 6, 2, 0, Color.Purple, true);
        }
    }
}
