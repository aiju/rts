using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Security;
using System.Security.AccessControl;
using System.Security.Cryptography;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;
using Color = Microsoft.Xna.Framework.Color;

namespace RTS
{
    internal class BinaryHeap<TElement,TPriority> where TPriority : IComparable
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
            for(; ;)
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
            if(heap.Count == 0)
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
    internal class SimpleFunnel
    {
        CDT cdt;
        public Vector2 Start { get; }
        public Vector2 Goal { get; }
        public float Radius { get; }
        List<Mesh.Face> Faces;
        LinkedList<Vector2> Deque;
        public List<Vector2> Route { get; private set; }
        Vector2 apex;
        public SimpleFunnel(CDT cdt, Vector2 start, Vector2 goal, float radius, List<Mesh.Face> faces)
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
        public void addLeft(Vector2 v)
        {
            for (;;)
            {
                var left = Deque.First();
                var right = Deque.Last();
                if (Geometry.ApproxEqual(left, right))
                {
                    Deque.AddFirst(v);
                    break;
                }
                var left1 = Deque.First.Next.Value;
                if(Geometry.ApproxEqual(left, apex))
                {
                    if(Geometry.IsClockwise(left, v, left1))
                    {
                        Deque.AddFirst(v);
                        break;
                    }
                    Route.Add(apex);
                    apex = left1;
                }
                else
                {
                    if (Geometry.IsClockwise(left1, v, left))
                    {
                        Deque.AddFirst(v);
                        break;
                    }
                }
                Deque.RemoveFirst();
            }
        }
        public void addRight(Vector2 v)
        {
            for (; ; )
            {
                var left = Deque.First();
                var right = Deque.Last();
                if (Geometry.ApproxEqual(left, right))
                {
                    Deque.AddLast(v);
                    break;
                }
                var right1 = Deque.Last.Previous.Value;
                if (Geometry.ApproxEqual(right, apex))
                {
                    if (Geometry.IsClockwise(right1, v, right))
                    {
                        Deque.AddLast(v);
                        break;
                    }
                    Route.Add(apex);
                    apex = right1;
                }
                else
                {
                    if (Geometry.IsClockwise(right, v, right1))
                    {
                        Deque.AddLast(v);
                        break;
                    }
                }
                Deque.RemoveLast();
            }
        }
        public void Run()
        {
            Route = new List<Vector2> { Start };
            Deque = new LinkedList<Vector2>();
            Deque.AddFirst(Start);
            apex = Start;
            var (vl, vr) = getOrientedEdge(Faces[0], Faces[1]);
            addLeft(vl.Pos);
            addRight(vr.Pos);
            for (int i = 1; i < Faces.Count - 1; i++)
            {
                var (vln, vrn) = getOrientedEdge(Faces[i], Faces[i + 1]);
                if (vln.Id == vl.Id)
                    addRight(vrn.Pos);
                else
                    addLeft(vln.Pos);
                (vl, vr) = (vln, vrn);
            }
            addRight(Goal);
            while (!Geometry.ApproxEqual(Deque.First(), apex))
                Deque.RemoveFirst();
            while (Deque.Any())
            {
                Route.Add(Deque.First());
                Deque.RemoveFirst();
            }

        }
    }
    internal class Funnel
    {
        CDT cdt;
        public Vector2 Start { get; }
        public Vector2 Goal { get; }
        public float Radius { get; }
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
                        Route.Add(b0);
                        Route.Add(b1);
                        Deque.AddFirst((v, PointType.Left));
                        apex = v;
                    }
                    else
                    {
                        Route.Add(a0);
                        Route.Add(a1);
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
                        Route.Add(b0);
                        Route.Add(b1);
                        Deque.AddLast((v, t));
                        apex = v;
                    }
                    else
                    {
                        Route.Add(a0);
                        Route.Add(a1);
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
        public static int steps = 3;
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
                if (vln.Id == vl.Id)
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
                (v, t) = (w, tt);
                Route.Add(a);
                Route.Add(b);
            }
            /*Route.Clear();
            int n = Deque.Count();
            bool flip = false;
            for(int i = 0; i < n - 1; i++)
            {
                var (a, t) = Deque.ElementAt(i);
                var (b, s) = Deque.ElementAt(i+1);
                if (Geometry.ApproxEqual(a, apex)) flip = true;
                Vector2 x, y;
                if(!flip)
                    (y, x) = adjustVector(b, s, a, t);
                else
                {
                    (x, y) = adjustVector(a, t, b, s);
                }
                Route.Add(x);
                Route.Add(y);
            }*/
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
                        f = vertex.Edges().Where(e => !pathfinding.CDT.IsConstrained(e)).First().AdjacentFaces().First();
                    },
                    edge =>
                    {
                        if (pathfinding.CDT.IsConstrained(edge))
                            throw new ArgumentOutOfRangeException();
                        f = edge.AdjacentFaces().First();
                    },
                    face =>
                        f = face);
                Debug.Assert(f != null);
                return f;
            }
            private float heuristic(Mesh.Face f)
            {
                return f.Edges.Select((e, _) => e.DistanceToPoint(Goal)).Min();
            }
            private float calculategScore(float gScore, Mesh.Face a, Mesh.Face b)
            {
                return gScore + Vector2.Distance(a.Centroid(), b.Centroid());
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
                    foreach ((Mesh.Face neighbor, Mesh.Edge e) in current.Neighbours())
                    {
                        if (pathfinding.cdt.IsConstrained(e))
                            continue;
                        if(current.Id != start.Id)
                        {
                            var (_, f) = cameFrom[current];
                            if (e.Id != f.Id && pathfinding.CDT.TriangleWidth(current, e, f) < 2 * Radius)
                                continue;
                        }
                        var g = calculategScore(gScore[current], current, neighbor);
                        if (!gScore.ContainsKey(neighbor) || g < gScore[neighbor])
                        {
                            cameFrom[neighbor] = (current, e);
                            gScore[neighbor] = g;
                            fScore[neighbor] = g + heuristic(neighbor);
                            queue.Enqueue(neighbor, fScore[neighbor]);
                        }
                    }
                }
                Faces = null;
                return false;
            }
            private (Mesh.Vertex, Mesh.Vertex, Mesh.Vertex, Mesh.Vertex) quad(Mesh.Face f0, Mesh.Face f1)
            {
                var (v1, v2, v3) = f0.Vertices;
                if (!f1.Contains(v2)) (v1, v2) = (v2, v1);
                else if (!f1.Contains(v3)) (v1, v3) = (v3, v1);
                if (!Geometry.IsClockwise(v1.Pos, v2.Pos, v3.Pos))
                    (v2, v3) = (v3, v2);
                /* TODO maybe dont need v4 */
                var (v4, v5, v6) = f1.Vertices;
                if (v5.Id != v2.Id && v5.Id != v3.Id) (v4, v5) = (v5, v4);
                else if (v6.Id != v2.Id && v6.Id != v3.Id) (v4, v6) = (v6, v4);
                return (v1, v2, v3, v4);
            }
            private void calculatePortals(out Vector2[] left, out Vector2[] right)
            {
                left = new Vector2[Faces.Count + 1];
                right = new Vector2[Faces.Count + 1];
                var (_, vl, vr, _) = quad(Faces[0], Faces[1]);
                left[0] = Start;
                right[0] = Start;
                for (int i = 1; i < Faces.Count - 1; i++)
                {
                    left[i] = vl.Pos;
                    right[i] = vr.Pos;
                    (_, vl, vr, _) = quad(Faces[i], Faces[i + 1]);
                }
                left[Faces.Count - 1] = vl.Pos;
                right[Faces.Count - 1] = vr.Pos;
                left[Faces.Count] = Goal;
                right[Faces.Count] = Goal;
            }
            /*
            enum PointType { End, Left, Right };
            private (Vector2, Vector2) nudgeVertices(Vector2 apex, PointType apexType, Vector2 point, PointType pointType)
            {
                if (Geometry.ApproxEqual(apex, point))
                    return (apex, point);
                bool leftUp = apexType != PointType.Left;
                bool rightUp = pointType != PointType.Left;
                float r1 = apexType == PointType.End ? 0 : Radius;
                float r2 = pointType == PointType.End ? 0 : Radius;
                Vector2 d = point - apex;
                if (leftUp == rightUp)
                {
                    float phi = MathF.Atan((r1 - r2) / d.Length());
                    Vector2 n = Geometry.RotateLeft(Geometry.UnitLeftNormal(d) * (rightUp ? 1 : -1), phi);
                    return (apex + n * r1, point + n * r2);
                }
                else
                {
                    float phi = MathF.Acos((r1 + r2) / d.Length());
                    if (rightUp) phi = -phi;
                    Vector2 n = Geometry.RotateLeft(Geometry.Normalized(d), phi);
                    return (apex + n * r1, point - n * r2);
                }
            }
            */
            private void funnel(Vector2[] lefts, Vector2[] rights)
            {
                Vector2 apex = Start;
                int apexIndex = 0;
                Vector2 left = lefts[0];
                int leftIndex = 0;
                Vector2 right = rights[0];
                int rightIndex = 0;
                Route = new List<Vector2>();
                Route.Add(Start);
                for(int i = 1; i <= Faces.Count; i++)
                {
                    var (newLeft, newRight) = (lefts[i], rights[i]);
                    if(Geometry.IsClockwiseOrDegenerate(apex, newRight, right))
                    {
                        if(Geometry.ApproxEqual(apex, right) || Geometry.IsClockwise(apex, left, newRight))
                            (right, rightIndex) = (newRight, i);
                        else
                        {
                            Route.Add(left);
                            (apex, apexIndex) = (left, leftIndex);
                            (right, rightIndex) = (left, leftIndex);
                            i = apexIndex;
                            continue;
                        }
                    }
                    if(Geometry.IsClockwiseOrDegenerate(apex, left, newLeft))
                    {
                        if (Geometry.ApproxEqual(apex, left) || Geometry.IsClockwise(apex, newLeft, right))
                            (left, leftIndex) = (newLeft, i);
                        else
                        {
                            Route.Add(right);
                            (apex, apexIndex) = (right, rightIndex);
                            (left, leftIndex) = (right, rightIndex);
                            i = apexIndex;
                            continue;
                        }
                    }
                }
                if (!Geometry.ApproxEqual(Route[Route.Count - 1], Goal))
                    Route.Add(Goal);
            }
            private void antihugging()
            {
                if (Route.Count == 2) return;
                List<Vector2> newRoute = new List<Vector2>();
                for(int i = 0; i < Route.Count - 1; i++)
                {
                    bool a = i != 0 && Geometry.IsClockwise(Route[i - 1], Route[i], Route[i + 1]);
                    bool b = i != Route.Count - 2 && Geometry.IsClockwise(Route[i], Route[i + 1], Route[i + 2]);
                    float r1 = i == 0 ? 0 : Radius;
                    float r2 = i == Route.Count - 2 ? 0 : Radius;
                    Vector2 d = Route[i + 1] - Route[i];
                    if (a == b)
                    {
                        float phi = MathF.Atan((r1 - r2) / d.Length());
                        Vector2 n = Geometry.RotateLeft(Geometry.UnitLeftNormal(d) * (b ? 1 : -1), phi);
                        newRoute.Add(Route[i] + n * r1);
                        newRoute.Add(Route[i + 1] + n * r2);
                    }
                    else
                    {
                        float phi = MathF.Acos((r1 + r2) / d.Length());
                        if (b) phi = -phi;
                        Vector2 n = Geometry.RotateLeft(Geometry.Normalized(d), phi);
                        newRoute.Add(Route[i] + n * r1);
                        newRoute.Add(Route[i + 1] - n * r2);
                    }
                }
                Route = newRoute;
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
                        //calculatePortals(out var left, out var right);
                        //funnel(left, right);
                        //antihugging();
                    }
                }
            }
        }
    }
    internal class PathfindingManipulator
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
        public const float radius = 0.05f;
        public PathfindingManipulator()
        {
            pathfinding = new Pathfinding(new Vector2(0, 0), new Vector2(1, 1));
            cdt = pathfinding.CDT;
            //            cdt.InsertConstraint(new List<Vector2>() { new Vector2(0.26f, 0.16f), new Vector2(0.365f, 0.145f), new Vector2(0.57f, 0.155f), new Vector2(0.765f, 0.215f), new Vector2(0.87f, 0.29f), new Vector2(0.875f, 0.4f), new Vector2(0.865f, 0.555f), new Vector2(0.855f, 0.62f), new Vector2(0.83f, 0.71f), new Vector2(0.83f, 0.71f), new Vector2(0.73f, 0.825f), new Vector2(0.635f, 0.85f), new Vector2(0.485f, 0.86f), new Vector2(0.4f, 0.86f), new Vector2(0.305f, 0.835f), new Vector2(0.24f, 0.815f), new Vector2(0.175f, 0.77f), new Vector2(0.15f, 0.725f), new Vector2(0.16f, 0.67f), new Vector2(0.21f, 0.65f), new Vector2(0.28f, 0.67f), new Vector2(0.33f, 0.705f), new Vector2(0.39f, 0.735f), new Vector2(0.455f, 0.75f), new Vector2(0.455f, 0.75f), new Vector2(0.54f, 0.74f), new Vector2(0.54f, 0.74f), new Vector2(0.6f, 0.72f), new Vector2(0.6f, 0.72f), new Vector2(0.705f, 0.615f), new Vector2(0.705f, 0.615f), new Vector2(0.69f, 0.535f), new Vector2(0.69f, 0.535f), new Vector2(0.65f, 0.455f), new Vector2(0.65f, 0.455f), new Vector2(0.59f, 0.42f), new Vector2(0.535f, 0.42f), new Vector2(0.535f, 0.42f), new Vector2(0.535f, 0.42f), new Vector2(0.425f, 0.51f), new Vector2(0.425f, 0.56f), new Vector2(0.425f, 0.56f), new Vector2(0.415f, 0.61f), new Vector2(0.345f, 0.58f), new Vector2(0.345f, 0.58f), new Vector2(0.27f, 0.51f), new Vector2(0.27f, 0.51f), new Vector2(0.215f, 0.415f), new Vector2(0.215f, 0.415f), new Vector2(0.215f, 0.415f), new Vector2(0.265f, 0.39f), new Vector2(0.265f, 0.39f), new Vector2(0.325f, 0.425f), new Vector2(0.465f, 0.375f), new Vector2(0.54f, 0.29f), new Vector2(0.54f, 0.29f), new Vector2(0.48f, 0.235f), new Vector2(0.48f, 0.235f), new Vector2(0.38f, 0.235f), new Vector2(0.38f, 0.235f), new Vector2(0.33f, 0.23f), new Vector2(0.33f, 0.23f), new Vector2(0.26f, 0.16f), }, 0);


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

            mesh = cdt.Mesh;
            path = pathfinding.NewPath(new Vector2(0.16f, 0.56f), new Vector2(0.95f, 0.95f), radius);
        }
        public void Update()
        {
            var m = Mouse.GetState();
            var k = Keyboard.GetState();
            cursor = new Vector2(((float)m.X - Offset) / Scale, ((float)m.Y - Offset) / Scale);
            if(k.IsKeyUp(Keys.S) && lastState.IsKeyDown(Keys.S))
            {
                path = pathfinding.NewPath(cursor, path.Goal, radius);
            }
            if (k.IsKeyUp(Keys.E) && lastState.IsKeyDown(Keys.E))
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
        static int ctr;
        public void Draw(GraphicsDevice device)
        {
            if (ctr == 20)
            {
                Funnel.steps = Funnel.steps + 1;
                ctr = 0;
            }
            else
                ctr++;


            var dotRed = new Polygon(device, 6, 2, 0, Color.Red, true);
            var dotPink = new Polygon(device, 6, 2, 0, Color.Pink, true);
            var dotPurple = new Polygon(device, 6, 2, 0, Color.Purple, true);
            foreach (Mesh.Face f in mesh.Faces())
            {
                var fc = path.Faces != null && path.Faces.Contains(f) ? Color.LightGreen : Color.Green;
                new Polygon(device, new Vector2[] { toScreen(f.Vertex1.Pos), toScreen(f.Vertex3.Pos), toScreen(f.Vertex2.Pos) }, fc, true).DrawAt(device, new Vector2(0, 0));
            }
            foreach (Mesh.Vertex v in mesh.Vertices())
            {
                dotRed.DrawAt(device, toScreen(v.Pos) / 32);
                Shape.RegularPolygon(device, 32, radius * Scale).Translate(toScreen(v.Pos)).Draw(device);
            }
            foreach (Mesh.Edge v in mesh.Edges())
            {
                var ec = cdt.IsConstrained(v) ? Color.Blue : Color.LightGreen;
                new Polygon(device, new Vector2[] { toScreen(v.Vertex1.Pos), toScreen(v.Vertex2.Pos) }, ec).DrawAt(device, new Vector2(0, 0));
            }
            
            //path = pathfinding.NewPath(path.Start, path.Goal, radius);
            if (path.Route != null && path.Route.Count > 1)
            {
                Shape.FromVertices(device, path.Route.ToArray()).Transform(toScreen).Color(Color.Red).Draw(device);
            }
            dotPink.DrawAt(device, toScreen(path.Start) / 32);
            dotPink.DrawAt(device, toScreen(path.Goal) / 32);
            dotPurple.DrawAt(device, toScreen(cursor) / 32);
        }
    }
}
