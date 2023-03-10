using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Reflection.Metadata.Ecma335;
using System.Threading;

namespace RTS
{
    internal class Funnel
    {
        CDT cdt;
        public Vector2 Start { get; }
        public Vector2 Goal { get; }
        public float Radius { get; }
        public const float ArcTolerance = 0.1f;
        List<Mesh.Face> Faces;
        enum PointType { End, Left, Right };
        LinkedList<(Vector2, PointType)> Deque;
        public List<Vector2> Route { get; private set; }
        Vector2 apex;
        public Funnel(CDT cdt, Vector2 start, Vector2 goal, float radius, List<Mesh.Face> faces)
        {
            this.cdt = cdt;
            Start = start;
            Goal = goal;
            Radius = radius;
            Faces = faces;
        }
        private (Mesh.Vertex, Mesh.Vertex) getOrientedEdge(Mesh.Face f0, Mesh.Face f1)
        {
            var (v1, v2, v3) = f0.Vertices;
            if (!f1.Contains(v2)) (v1, v2) = (v2, v1);
            else if (!f1.Contains(v3)) (v1, v3) = (v3, v1);
            if (!Geometry.IsClockwise(v1.Pos, v2.Pos, v3.Pos))
                (v2, v3) = (v3, v2);
            return (v2, v3);
        }
        private (Vector2, Vector2) adjustVector(Vector2 from, PointType fromType, Vector2 to, PointType toType)
        {
            if (Geometry.ApproxEqual(from, to))
                return (from, to);
            bool leftUp = fromType != PointType.Left;
            bool rightUp = toType != PointType.Left;
            float r1 = fromType == PointType.End ? 0 : Radius;
            float r2 = toType == PointType.End ? 0 : Radius;
            Vector2 d = to - from;
            if (leftUp == rightUp)
            {
                float phi = MathF.Acos((r1 - r2) / d.Length());
                Vector2 n = Geometry.RotateLeft(Geometry.Normalized(d), phi * (rightUp ? 1 : -1));
                return (from + n * r1, to + n * r2);
            }
            else
            {
                float phi = MathF.Asin((r1 + r2) / d.Length());
                if (!rightUp) phi = -phi;
                Vector2 n = Geometry.RotateLeft(Geometry.UnitLeftNormal(d), phi);
                if(rightUp) n = -n;
                return (from + n * r1, to - n * r2);
            }
        }
        private void addArc(Vector2 centre, Vector2 from, Vector2 to)
        {
            Vector2 mid = (from + to) / 2;
            if(Vector2.Distance(mid, centre) < Radius * (1 - ArcTolerance))
            {
                Debug.Assert(!Geometry.ApproxEqual(mid, centre));
                mid = centre + Geometry.Normalized(mid - centre) * Radius;
                addArc(centre, from, mid);
                Route.Add(mid);
                addArc(centre, mid, to);
            }
        }
        private void addSegment(Vector2 centre, Vector2 a, Vector2 b)
        {
            var l = Route.Last();
            if(MathF.Abs(Vector2.Distance(l, centre) - Radius) < Geometry.Epsilon && MathF.Abs(Vector2.Distance(a, centre) - Radius) < Geometry.Epsilon)
                addArc(centre, l, a);
            Route.Add(a);
            Route.Add(b);
        }
        private void addLeft(Vector2 v)
        {
            for (; ; )
            {
                var (left, leftType) = Deque.First();
                var (right, rightType) = Deque.Last();
                if (Geometry.ApproxEqual(left, right))
                {
                    Deque.AddFirst((v, PointType.Left));
                    break;
                }
                var (left1, left1Type) = Deque.First.Next.Value;
                if (Geometry.ApproxEqual(left, apex))
                {
                    var (a0, a1) = adjustVector(left, leftType, left1, left1Type);
                    var (b0, b1) = adjustVector(left, leftType, v, PointType.Left);
                    var a = a1 - a0;
                    var b = b1 - b0;
                    var l1 = Vector2.Distance(left, left1);
                    var l2 = Vector2.Distance(left, v);
                    if (Geometry.IsClockwise(Vector2.Zero, b, a))
                    {
                        Deque.AddFirst((v, PointType.Left));
                        break;
                    }
                    if(l2 < l1)
                    {
                        addSegment(apex, b0, b1);
                        Deque.AddFirst((v, PointType.Left));
                        apex = v;
                    }
                    else
                    {
                        addSegment(apex, a0, a1);
                        apex = left1;
                    }
                }
                else
                {
                    var (a0, a1) = adjustVector(left1, left1Type, left, leftType);
                    var (b0, b1) = adjustVector(left, leftType, v, PointType.Left);
                    var a = a1 - a0;
                    var b = b1 - b0;
                    if (Geometry.IsClockwise(Vector2.Zero, b, a))
                    {
                        Deque.AddFirst((v, PointType.Left));
                        break;
                    }
                }
                Deque.RemoveFirst();
            }
        }
        private void addRight(Vector2 v, PointType t)
        {
            for (; ; )
            {
                var (left, leftType) = Deque.First();
                var (right, rightType) = Deque.Last();
                if (Geometry.ApproxEqual(left, right))
                {
                    Deque.AddLast((v, t));
                    break;
                }
                var (right1, right1Type) = Deque.Last.Previous.Value;
                if (Geometry.ApproxEqual(right, apex))
                {
                    var (a0, a1) = adjustVector(right, rightType, right1, right1Type);
                    var (b0, b1) = adjustVector(right, rightType, v, t);
                    var a = a1 - a0;
                    var b = b1 - b0;
                    var l1 = Vector2.Distance(right, right1);
                    var l2 = Vector2.Distance(right, v);
                    if (Geometry.IsClockwise(Vector2.Zero, a, b))
                    {
                        Deque.AddLast((v, t));
                        break;
                    }
                    if (l2 < l1)
                    {
                        addSegment(apex, b0, b1);
                        Deque.AddLast((v, t));
                        apex = v;
                    }
                    else
                    {
                        addSegment(apex, a0, a1);
                        apex = right1;
                    }
                }
                else
                {
                    var (a0, a1) = adjustVector(right1, right1Type, right, rightType);
                    var (b0, b1) = adjustVector(right, rightType, v, t);
                    var a = a1 - a0;
                    var b = b1 - b0;
                    if (Geometry.IsClockwise(Vector2.Zero, a, b))
                    {
                        Deque.AddLast((v, t));
                        break;
                    }
                }
                Deque.RemoveLast();
            }
        }
        public void Run()
        {
            Route = new List<Vector2> { Start };
            Deque = new LinkedList<(Vector2, PointType)>();
            Deque.AddFirst((Start, PointType.End));
            apex = Start;
            var (vl, vr) = getOrientedEdge(Faces[0], Faces[1]);
            addLeft(vl.Pos);
            addRight(vr.Pos, PointType.Right);
            for (int i = 1; i < Faces.Count - 1; i++)
            {
                var (vln, vrn) = getOrientedEdge(Faces[i], Faces[i + 1]);
                if (vln.Same(vl))
                    addRight(vrn.Pos, PointType.Right);
                else
                    addLeft(vln.Pos);
                (vl, vr) = (vln, vrn);
            }
            addRight(Goal, PointType.End);
            while (!Geometry.ApproxEqual(Deque.First().Item1, apex))
                Deque.RemoveFirst();
            var (v, t) = Deque.First();
            Deque.RemoveFirst();
            foreach(var (w,tt) in Deque)
            {
                var (a, b) = adjustVector(v, t, w, tt);
                addSegment(v, a, b);
                (v, t) = (w, tt);
            }
        }
    }
    internal class Pathfinding
    {
        CDT cdt;
        public CDT CDT { get { return cdt; } }
        public Pathfinding(Vector2 min, Vector2 max)
        {
            cdt = new CDT(min, max);
        }
        public Path NewPath(Vector2 start, Vector2 goal, float radius)
        {
            return new Path(this, start, goal, radius);
        }
        public class Path
        {
            Pathfinding pathfinding;
            public float Radius { get; private set; }
            public Vector2 Start { get; private set; }
            public Vector2 Goal { get; private set; }
            public List<Mesh.Face> Faces { get; private set; }
            public List<Vector2> Route { get; private set; }
            internal Path(Pathfinding pathfinding, Vector2 start, Vector2 goal, float radius)
            {
                this.pathfinding = pathfinding;
                Start = start;
                Goal = goal;
                Radius = radius;
                update();
            }
            private Mesh.Face initialFace()
            {
                Mesh.Face f;

                f = null;
                pathfinding.CDT.Mesh.LocatePoint(Start,
                    vertex =>
                    {
                        f = vertex.Edges().Where(e => !pathfinding.CDT.IsConstrained(e)).First().EnumerateFaces().First();
                    },
                    edge =>
                    {
                        if (pathfinding.CDT.IsConstrained(edge))
                            throw new ArgumentOutOfRangeException();
                        f = edge.EnumerateFaces().First();
                    },
                    face =>
                        f = face);
                Debug.Assert(f != null);
                return f;
            }
            private float heuristic(Mesh.Face f)
            {
                return f.EnumerateEdges().Select((e, _) => e.DistanceToPoint(Goal)).Min();
            }
            private float calculategScore(float gScore, Mesh.Face current, Mesh.Face next, Mesh.Edge fromEdge, Mesh.Edge toEdge)
            {
                float d1 = next.EnumerateEdges().Select(e => Geometry.DistanceSegmentToPoint(e.Vertex1.Pos, e.Vertex2.Pos, Start)).Min();
                float d2 = gScore + (heuristic(next) - heuristic(current));
                float m = Math.Max(d1, d2);
                if (fromEdge != null)
                {
                    var (a, b) = fromEdge.Vertices;
                    if (!toEdge.Contains(a)) (a, b) = (b, a);
                    var c = toEdge.OtherVertex(a);
                    float d3 = gScore + Radius * Geometry.Angle(b.Pos, a.Pos, c.Pos);
                    return Math.Max(m, d3);
                }
                else
                    return m;
            }
            private bool astar()
            {
                var queue = new BinaryHeap<Mesh.Face, float>();
                var gScore = new Dictionary<Mesh.Face, float>();
                var fScore = new Dictionary<Mesh.Face, float>();
                var cameFrom = new Dictionary<Mesh.Face, (Mesh.Face, Mesh.Edge)>();
                var start = initialFace();
                gScore.Add(start, 0);
                fScore.Add(start, heuristic(start));
                queue.Enqueue(initialFace(), 0);
                while (queue.TryDequeue(out Mesh.Face current, out _))
                {
                    if (current.Contains(Goal))
                    {
                        Faces = new List<Mesh.Face>();
                        Faces.Add(current);
                        while (cameFrom.TryGetValue(current, out var nextp))
                        {
                            current = nextp.Item1;
                            Faces.Add(current);
                        }
                        Faces.Reverse();
                        return true;
                    }
                    foreach ((Mesh.Face neighbor, Mesh.Edge toEdge) in current.Neighbours())
                    {
                        if (pathfinding.cdt.IsConstrained(toEdge))
                            continue;
                        Mesh.Edge fromEdge = null;
                        if(!current.Same(start))
                        {
                            (_, fromEdge) = cameFrom[current];
                            if (toEdge.Same(fromEdge) || pathfinding.CDT.TriangleWidth(current, fromEdge, toEdge) < 2 * Radius)
                                continue;
                        }
                        var g = calculategScore(gScore[current], current, neighbor, fromEdge, toEdge);
                        if (!gScore.ContainsKey(neighbor) || g < gScore[neighbor])
                        {
                            cameFrom[neighbor] = (current, toEdge);
                            gScore[neighbor] = g;
                            fScore[neighbor] = g + heuristic(neighbor);
                            queue.Enqueue(neighbor, fScore[neighbor]);
                        }
                    }
                }
                Faces = null;
                return false;
            }
            private void update()
            {
                if (astar())
                {
                    Debug.Assert(Faces.Count > 0);
                    if(Faces.Count == 1)
                        Route = new List<Vector2>() { Start, Goal };
                    else
                    {
                        var f = new Funnel(pathfinding.CDT, Start, Goal, Radius, Faces);
                        f.Run();
                        Route = f.Route;
                    }
                }
            }
        }
    }
    internal class PathfindingManipulator : IEntity
    {
        public const float Scale = 400;
        public const float Offset = 50;
        Mesh mesh;
        CDT cdt;
        Pathfinding pathfinding;
        Vector2 cursor;
        Pathfinding.Path path;
        bool lastDown;
        KeyboardState lastState;
        public const float radius = 0.01f;
        public PathfindingManipulator()
        {
            pathfinding = new Pathfinding(new Vector2(0, 0), new Vector2(1, 1));
            cdt = pathfinding.CDT;
            //cdt.InsertConstraint(new List<Vector2>() { new Vector2(0.26f, 0.16f), new Vector2(0.365f, 0.145f), new Vector2(0.57f, 0.155f), new Vector2(0.765f, 0.215f), new Vector2(0.87f, 0.29f), new Vector2(0.875f, 0.4f), new Vector2(0.865f, 0.555f), new Vector2(0.855f, 0.62f), new Vector2(0.83f, 0.71f), new Vector2(0.83f, 0.71f), new Vector2(0.73f, 0.825f), new Vector2(0.635f, 0.85f), new Vector2(0.485f, 0.86f), new Vector2(0.4f, 0.86f), new Vector2(0.305f, 0.835f), new Vector2(0.24f, 0.815f), new Vector2(0.175f, 0.77f), new Vector2(0.15f, 0.725f), new Vector2(0.16f, 0.67f), new Vector2(0.21f, 0.65f), new Vector2(0.28f, 0.67f), new Vector2(0.33f, 0.705f), new Vector2(0.39f, 0.735f), new Vector2(0.455f, 0.75f), new Vector2(0.455f, 0.75f), new Vector2(0.54f, 0.74f), new Vector2(0.54f, 0.74f), new Vector2(0.6f, 0.72f), new Vector2(0.6f, 0.72f), new Vector2(0.705f, 0.615f), new Vector2(0.705f, 0.615f), new Vector2(0.69f, 0.535f), new Vector2(0.69f, 0.535f), new Vector2(0.65f, 0.455f), new Vector2(0.65f, 0.455f), new Vector2(0.59f, 0.42f), new Vector2(0.535f, 0.42f), new Vector2(0.535f, 0.42f), new Vector2(0.535f, 0.42f), new Vector2(0.425f, 0.51f), new Vector2(0.425f, 0.56f), new Vector2(0.425f, 0.56f), new Vector2(0.415f, 0.61f), new Vector2(0.345f, 0.58f), new Vector2(0.345f, 0.58f), new Vector2(0.27f, 0.51f), new Vector2(0.27f, 0.51f), new Vector2(0.215f, 0.415f), new Vector2(0.215f, 0.415f), new Vector2(0.215f, 0.415f), new Vector2(0.265f, 0.39f), new Vector2(0.265f, 0.39f), new Vector2(0.325f, 0.425f), new Vector2(0.465f, 0.375f), new Vector2(0.54f, 0.29f), new Vector2(0.54f, 0.29f), new Vector2(0.48f, 0.235f), new Vector2(0.48f, 0.235f), new Vector2(0.38f, 0.235f), new Vector2(0.38f, 0.235f), new Vector2(0.33f, 0.23f), new Vector2(0.33f, 0.23f), new Vector2(0.26f, 0.16f), }, 0);
            /*

            const float size = 0.05f;
            int n = 5;
            const float offset = 0.075f;
            float spacing = (1 - 2 * offset - n * size) / (n - 1);
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++) {
                    float x0 = offset + i * (size + spacing);
                    float y0 = offset + j * (size + spacing);
                    cdt.InsertConstraint(new List<Vector2> {
                        new Vector2(x0,y0),
                        new Vector2(x0+size,y0),
                        new Vector2(x0+size,y0+size),
                        new Vector2(x0,y0+size),
                        new Vector2(x0,y0) },1);
                }
            */
            var centres = new List<(Vector2, float)>();
            Random random = new Random();
            for(int i = 0; i < 50; i++)
            {
                float r = 0.025f + 0.025f * random.NextSingle();
                Vector2 v;
            loop:
                v = new Vector2(random.NextSingle(), random.NextSingle());
                if (v.X <= r || v.Y <= r || v.X >= 1.0f - r || v.Y >= 1.0f - r)
                    goto loop;
                foreach (var (w, r2) in centres)
                    if (Vector2.Distance(w, v) <= r + r2)
                        goto loop;
                centres.Add((v, r));
            }
            foreach(var (v, r) in centres)
            {
                var polygon = new List<Vector2>();
                int n = 3 + random.Next() % 8;
                for (int i = 0; i <= n; i++)
                    polygon.Add(new Vector2(v.X + r * MathF.Cos(2 * MathF.PI * i / n), v.Y + r * MathF.Sin(2 * MathF.PI * i / n)));
                cdt.InsertConstraint(polygon, 1);
            }

            mesh = cdt.Mesh;
            path = pathfinding.NewPath(new Vector2(0.16f, 0.56f), new Vector2(0.95f, 0.95f), radius);
        }
        public void Update(GameTime gameTime)
        {
            var m = Mouse.GetState();
            var k = Keyboard.GetState();
            cursor = new Vector2(((float)m.X - Offset) / Scale, ((float)m.Y - Offset) / Scale);
            bool inRange = cursor.X > 0 && cursor.X < 1 && cursor.Y > 0 && cursor.Y < 1;
            if (inRange && k.IsKeyDown(Keys.S))//k.IsKeyUp(Keys.S) && lastState.IsKeyDown(Keys.S))
            {
                path = pathfinding.NewPath(cursor, path.Goal, radius);
            }
            if (inRange && k.IsKeyDown(Keys.E)) //(k.IsKeyUp(Keys.E) && lastState.IsKeyDown(Keys.E))
            {
                path = pathfinding.NewPath(path.Start, cursor, radius);
            }
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
            foreach (Mesh.Face f in mesh.Faces())
            {
                var fc = false && path.Faces != null && path.Faces.Contains(f) ? Color.LightGreen : Color.Green;
                ctx.Polygon(new Vector2[] { toScreen(f.Vertex1.Pos), toScreen(f.Vertex3.Pos), toScreen(f.Vertex2.Pos) }, fc, true);
            }
            foreach (Mesh.Edge v in mesh.Edges())
            {
                var ec = cdt.IsConstrained(v) ? Color.Blue : Color.Olive;
                ctx.Line(toScreen(v.Vertex1.Pos), toScreen(v.Vertex2.Pos), ec);
            }
            if (path.Route != null && path.Route.Count > 1)
            {
                ctx.Path(path.Route.Select(toScreen).ToArray(), Color.Red);
            }
            ctx.RegularPolygon(toScreen(path.Start), 6, 2, 0, Color.Pink, true);
            ctx.RegularPolygon(toScreen(path.Goal), 6, 2, 0, Color.Pink, true);
            ctx.RegularPolygon(toScreen(cursor), 6, 2, 0, Color.Purple, true);
        }
    }
}
