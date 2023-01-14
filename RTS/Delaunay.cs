using Microsoft.VisualBasic;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.ComponentModel.DataAnnotations;
using System.Data;
using System.Diagnostics;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Text;
using System.Threading.Tasks;
using static RTS.Mesh;


/* see paper Fully Dynamic Constrained Delaunay Triangulations by Marcelo Kallmann, Hanspeter Bieri, and Daniel Thalmann */

namespace RTS
{
    internal static class Geometry
    {
        public const float Epsilon = 1e-6f;
        public static bool ApproxEqual(Vector2 a, Vector2 b)
        {
            return Vector2.Distance(a, b) < Epsilon;
        }
        public static Vector2 LeftNormal(Vector2 a)
        {
            return new Vector2(-a.Y, a.X);
        }
        public static Vector2 Normalized(Vector2 a)
        {
            a.Normalize();
            return a;
        }
        public static Vector2 UnitLeftNormal(Vector2 a)
        {
            Vector2 n = LeftNormal(a);
            n.Normalize();
            return n;
        }
        public static Vector2 RotateLeft(Vector2 v, float phi)
        {
            var (sin, cos) = MathF.SinCos(phi);
            return new Vector2(v.X * cos - v.Y * sin, v.X * sin + v.Y * cos);
        }
        public static float SignedDistanceLineToPoint(Vector2 a, Vector2 b, Vector2 p)
        {
            var s = b - a;
            return (s.Y * (p.X - a.X) - s.X * (p.Y - a.Y)) / s.Length();
        }
        public static bool IsClockwise(Vector2 a, Vector2 b, Vector2 c)
        {
            var s = b - a;
            var t = c - a;
            return s.Y * t.X > s.X * t.Y;
        }
        public static bool IsClockwiseOrDegenerate(Vector2 a, Vector2 b, Vector2 c)
        {
            var s = b - a;
            var t = c - a;
            return s.Y * t.X >= s.X * t.Y;
        }
        public static bool IsCollinear(Vector2 a, Vector2 b, Vector2 c)
        {
            var s = b - a;
            var t = c - a;
            return MathF.Abs(s.Y * t.X - s.X * t.Y) < Epsilon;
        }
        public static Vector2 ProjectPointToLine(Vector2 p, Vector2 a, Vector2 b)
        {
            var n = UnitLeftNormal(b - a);
            return p - Vector2.Dot(n, p - a) * n;
        }
        public static float DistanceAlongSegment(Vector2 a, Vector2 b, Vector2 p)
        {
            var s = b - a;
            return Vector2.Dot(p - a, s) / s.LengthSquared();
        }
        public static float DistanceSegmentToPoint(Vector2 a, Vector2 b, Vector2 p)
        {
            var s = b - a;
            float x = Vector2.Dot(p - a, s) / s.LengthSquared();
            if (x <= 0) return Vector2.Distance(a, p);
            if (x >= 1) return Vector2.Distance(b, p);
            return Math.Abs(Vector2.Dot(UnitLeftNormal(s), p - a));
        }
        public static bool IntersectSegments(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2, out Vector2 intersection)
        {
            /* TODO degeneracy */
            var s1 = a2 - a1;
            var s2 = b2 - b1;
            var n1 = LeftNormal(s1);
            var n2 = LeftNormal(s2);
            if(Math.Abs(Vector2.Dot(n1, s2)) < Epsilon)
            {
                if(Math.Abs(SignedDistanceLineToPoint(a1, a2, b1)) >= Epsilon)
                {
                    intersection = new Vector2(0, 0);
                    return false;
                }
                float x1 = DistanceAlongSegment(a1, a2, b1);
                float x2 = DistanceAlongSegment(a1, a2, b2);
                if(x1 * x2 <= 0)
                {
                    intersection = a1;
                    return true;
                }
                if((x1 - 1) * (x2 - 1) <= 0)
                {
                    intersection = a2;
                    return true;
                }
                if(x1 >= 0 && x1 <= 1 && x2 >= 0 && x1 <= 1)
                {
                    intersection = b1;
                    return true;
                }
                intersection = new Vector2(0, 0);
                return false;
            }
            var x = Vector2.Dot(a1 - b1, n1) / Vector2.Dot(n1, s2);
            var y = Vector2.Dot(b1 - a1, n2) / Vector2.Dot(s1, n2);
            if (y < 0 || y > 1 || x < 0 || x > 1)
            {
                intersection = new Vector2(0, 0);
                return false;
            }
            else
            {
                intersection = b1 + s2 * x;
                return true;
            }
        }
        internal static bool IntersectLines(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2, out Vector2 intersection)
        {
            /* TODO degeneracy */
            var s1 = a2 - a1;
            var s2 = b2 - b1;
            var n1 = LeftNormal(s1);
            if (Math.Abs(Vector2.Dot(n1, s2)) < Epsilon)
            {
                if (Math.Abs(SignedDistanceLineToPoint(a1, a2, b1)) >= Epsilon)
                {
                    intersection = new Vector2(0, 0);
                    return false;
                }
                intersection = b1;
                return true;
            }
            else
            {
                var x = Vector2.Dot(a1 - b1, n1) / Vector2.Dot(n1, s2);
                intersection = b1 + s2 * x;
                return true;
            }
        }
        public static bool FaceContainsPoint(Vector2 a, Vector2 b, Vector2 c, Vector2 p)
        {
            float x = SignedDistanceLineToPoint(a, b, p);
            float y = SignedDistanceLineToPoint(b, c, p);
            float z = SignedDistanceLineToPoint(c, a, p);
            return x > Epsilon && y > Epsilon && z > Epsilon || x < -Epsilon && y < -Epsilon && z < -Epsilon;
        }
        public static bool IsDelaunay(Vector2 A, Vector2 B, Vector2 C, Vector2 D)
        {
            if (Geometry.IsClockwise(A, B, C))
                (A, B) = (B, A);
            float a = A.X - D.X;
            float b = A.Y - D.Y;
            float c = (A.X * A.X - D.X * D.X) + (A.Y * A.Y - D.Y * D.Y);
            float d = B.X - D.X;
            float e = B.Y - D.Y;
            float f = (B.X * B.X - D.X * D.X) + (B.Y * B.Y - D.Y * D.Y);
            float g = C.X - D.X;
            float h = C.Y - D.Y;
            float i = (C.X * C.X - D.X * D.X) + (C.Y * C.Y - D.Y * D.Y);
            float Det = a * e * i + b * f * g + c * d * h - a * f * h - b * d * i - c * e * g;
            return !(Det > 0);
        }
        public static bool IsAcute(Vector2 c, Vector2 a, Vector2 b)
        {
            return Vector2.Dot(c - a, b - a) > 0;
        }
        public static bool IsTriangleFacing(Vector2 a, Vector2 b, Vector2 c, Vector2 p)
        {
            if (!IsClockwise(a, b, c))
                (b, c) = (c, b);
            return IsClockwise(a, b, p) && IsClockwise(a, p, c);
        }
        internal static bool SegmentContainsPoint(Vector2 a, Vector2 b, Vector2 p)
        {
            if (!IsCollinear(a, b, p)) return false;
            return Vector2.Dot(a - p, b - p) <= Epsilon;
        }
    }
    internal class Mesh
    {
        public const float Epsilon = Geometry.Epsilon;
        List<Vertex> vertices;
        public Mesh(Vector2 min, Vector2 max)
        {
            vertices = new List<Vertex>();
            var a = NewVertex(min);
            var b = NewVertex(new Vector2(max.X, min.Y));
            var c = NewVertex(max);
            var d = NewVertex(new Vector2(min.X, max.Y));
            NewFace(a, c, b);
            NewFace(a, d, c);
        }
        private void addFaceToEdge(Face f, Edge e)
        {
            var (a, b) = e.Faces;
            if (a == null) e.Faces = (f, b);
            else if (b == null) e.Faces = (a, f);
            else throw new ArgumentOutOfRangeException();
        }
        private void removeFaceFromEdge(Face f, Edge e)
        {
            var (a, b) = e.Faces;
            if (a == f) e.Faces = (b, null);
            else if (b == f) e.Faces = (a, null);
            else throw new ArgumentOutOfRangeException();
        }
        public Vertex NewVertex(Vector2 pos)
        {
            Vertex v = new Vertex(this, pos);
            vertices.Add(v);
            return v;
        }
        public Edge NewEdge(Vertex a, Vertex b)
        {
            Edge e;
            if (TryGetEdge(a, b, out e))
                return e;
            e = new Edge(this, a, b);
            a.edges.Add(e);
            b.edges.Add(e);
            return e;
        }
        public Face NewFace(Vertex a, Vertex b, Vertex c)
        {
            Face f;
            if (TryGetFace(a, b, c, out f))
                return f;
            var e0 = NewEdge(b, c);
            var e1 = NewEdge(a, c);
            var e2 = NewEdge(a, b);
            f = new Face(this, (a, b, c), (e0,e1,e2));
            addFaceToEdge(f, e0);
            addFaceToEdge(f, e1);
            addFaceToEdge(f, e2);
            a.faces.Add(f);
            b.faces.Add(f);
            c.faces.Add(f);
            return f;
        }
        public void RemoveVertex(Vertex v)
        {
            vertices.Remove(v);
        }
        public void RemoveEdge(Edge e)
        {
            e.Vertex1.edges.Remove(e);
            e.Vertex2.edges.Remove(e);
        }
        public void RemoveFace(Face f)
        {
            var (a, b, c) = f.Edges;
            var (A, B, C) = f.Vertices;
            removeFaceFromEdge(f, a);
            removeFaceFromEdge(f, b);
            removeFaceFromEdge(f, c);
            A.faces.Remove(f);
            B.faces.Remove(f);
            C.faces.Remove(f);
        }
        public bool TryGetFace(Vertex v1, Vertex v2, Vertex v3, out Face face)
        {
            foreach (Face f in v1.Faces())
                if (f.Contains(v2) && f.Contains(v3))
                {
                    face = f;
                    return true;
                }
            face = null;
            return false;
        }
        public bool TryGetEdge(Vertex v1, Vertex v2, out Edge edge)
        {
            foreach(Edge e in v1.Edges())
                if (e.Contains(v2))
                {
                    edge = e;
                    return true;
                }
            edge = null;
            return false;
        }
        public Edge GetEdge(Vertex v1, Vertex v2)
        {
            bool ok = TryGetEdge(v1, v2, out var edge);
            if (!ok) throw new ArgumentOutOfRangeException();
            return edge;
        }
        public IEnumerable<Vertex> Vertices() { return vertices; }
        public IEnumerable<Edge> Edges()
        {
            foreach (Vertex v in Vertices())
                foreach (Edge e in v.Edges())
                    if (e.Vertex1.Same(v))
                        yield return e;
        }
        public IEnumerable<Face> Faces()
        {
            foreach (Vertex v in Vertices())
                foreach (Face f in v.Faces())
                    if (f.Vertex1.Same(v))
                        yield return f;
        }
        internal class Vertex
        {
            Mesh mesh;
            public Vector2 Pos { get; }
            public List<Edge> edges { get; }
            public List<Face> faces { get; }
            public Vertex(Mesh mesh, Vector2 pos)
            {
                this.mesh = mesh;
                this.Pos = pos;
                edges = new List<Edge>();
                faces = new List<Face>();
            }
            public IEnumerable<Edge> Edges()
            {
                return edges;
            }
            public IEnumerable<Face> Faces()
            {
                return faces;
            }
            public bool Same(Vertex w)
            {
                return this == w;
            }
            public override string ToString()
            {
                return String.Format("{{{0}, {1}}}", Pos.X, Pos.Y);
            }
        }
        internal class Edge
        {
            Mesh mesh;
            public Edge(Mesh mesh, Vertex a, Vertex b)
            {
                this.mesh = mesh;
                Vertices = (a, b);
            }
            public Vertex Vertex1 { get { return Vertices.Item1; } }
            public Vertex Vertex2 { get { return Vertices.Item2; } }
            public (Vertex, Vertex) Vertices { get; }
            public (Face, Face) Faces { get; internal set;  }
            public object Aux { get; set; }
            public float DistanceToPoint(Vector2 p)
            {
                return Geometry.DistanceSegmentToPoint(Vertex1.Pos, Vertex2.Pos, p);
            }
            public bool Contains(Vector2 point)
            {
                return DistanceToPoint(point) < Epsilon;
            }
            public bool Contains(Vertex v)
            {
                var (a, b) = Vertices;
                return a.Same(v) || b.Same(v);
            }
            public Vector2 Project(Vector2 point)
            {
                return Geometry.ProjectPointToLine(point, Vertex1.Pos, Vertex2.Pos);
            }
            public IEnumerable<Face> EnumerateFaces()
            {
                var (a, b) = Faces;
                if (a != null) yield return a;
                if (b != null) yield return b;
            }
            public Face OtherFace(Face f)
            {
                var (a, b) = Faces;
                if (a.Same(f)) return b;
                if (b.Same(f)) return a;
                throw new ArgumentOutOfRangeException();
            }
            public bool Same(Edge e)
            {
                return this == e;
            }
            public Vertex OtherVertex(Vertex v)
            {
                var (a, b) = Vertices;
                if (a.Same(v)) return b;
                if (b.Same(v)) return a;
                throw new ArgumentOutOfRangeException();
            }
        }
        internal class Face
        {
            Mesh mesh;
            public Face(Mesh mesh, (Vertex, Vertex, Vertex) vertices, (Edge, Edge, Edge) edges)
            {
                this.mesh = mesh;
                Vertices = vertices;
                Edges = edges;
            }
            public bool Same(Face f)
            {
                return this == f;
            }
            public override string ToString()
            {
                return String.Format("{0} {1} {2}", Vertex1, Vertex2, Vertex3);
            }
            public Vertex Vertex1 { get { return Vertices.Item1; } }
            public Vertex Vertex2 { get { return Vertices.Item2; } }
            public Vertex Vertex3 { get { return Vertices.Item3;  } }
            public (Vertex, Vertex, Vertex) Vertices { get; }
            public (Edge, Edge, Edge) Edges { get; }
            public bool Contains(Edge e)
            {
                var (x, y, z) = Edges;
                return x.Same(e) || y.Same(e) || z.Same(e);
            }
            public bool Contains(Vertex v)
            {
                var (a, b, c) = Vertices;
                return a.Same(v) || b.Same(v) || c.Same(v);
            }
            public bool Contains(Vector2 point)
            {
                return Geometry.FaceContainsPoint(Vertex1.Pos, Vertex2.Pos, Vertex3.Pos, point);
            }
            public IEnumerable<(Face, Edge)> Neighbours()
            {
                var (a, b, c) = Edges;
                yield return (a.OtherFace(this), a);
                yield return (b.OtherFace(this), b);
                yield return (c.OtherFace(this), c);
            }
            internal Vertex OtherVertex(Vertex a, Vertex b)
            {
                var (x, y, z) = Vertices;
                if (!x.Same(a) && !x.Same(b)) return x;
                else if (!y.Same(a) && !y.Same(b)) return y;
                Debug.Assert(!z.Same(a) && !z.Same(b));
                return z;
            }
            public (Edge, Edge) OtherEdges(Edge e)
            {
                var (a, b, c) = Edges;
                if (a.Same(e)) return (b, c);
                if (b.Same(e)) return (a, c);
                if (c.Same(e)) return (a, b);
                throw new ArgumentOutOfRangeException();
            }
            internal (Vertex, Vertex) OtherVertices(Vertex v)
            {
                var (a, b, c) = Vertices;
                if (a.Same(v)) return (b, c);
                if (b.Same(v)) return (a, c);
                if (c.Same(v)) return (a, b);
                throw new ArgumentOutOfRangeException();
            }
            public Vector2 Centroid()
            {
                var (a, b, c) = Vertices;
                return (a.Pos + b.Pos + c.Pos) / 3;
            }
            public IEnumerable<Edge> EnumerateEdges()
            {
                var (a, b, c) = Edges;
                yield return a;
                yield return b;
                yield return c;
            }

        }
        internal class Updater
        {
            HashSet<Edge> deleteEdges;
            HashSet<Face> deleteFaces;
            Dictionary<(Vertex, Vertex), object> addEdges;
            HashSet<(Vertex, Vertex, Vertex)> addFaces;
            Mesh mesh;
            public Updater(Mesh mesh)
            {
                this.mesh = mesh;
                deleteEdges = new HashSet<Edge>();
                deleteFaces = new HashSet<Face>();
                addEdges = new Dictionary<(Vertex, Vertex), object>();
                addFaces = new HashSet<(Vertex, Vertex, Vertex)>();
            }
            public void DeleteEdge(Edge e)
            {
                deleteEdges.Add(e);
                foreach (Face f in e.EnumerateFaces())
                    deleteFaces.Add(f);
            }
            public void AddEdge(Vertex a, Vertex b, object aux)
            {
                if (addEdges.ContainsKey((b, a)))
                    addEdges[(b, a)] = aux;
                else
                    addEdges.Add((a, b), aux);
            }
            private void weakAddEdge(Vertex a, Vertex b)
            {
                if (!addEdges.ContainsKey((a, b)) && !addEdges.ContainsKey((b, a)))
                    addEdges.Add((a, b), null);
            }
            public void AddFace(Vertex a, Vertex b, Vertex c)
            {
                weakAddEdge(a, b);
                weakAddEdge(b, c);
                weakAddEdge(a, c);
                addFaces.Add((a, b, c));
            }
            public void Apply()
            {
                foreach (Face f in deleteFaces)
                    mesh.RemoveFace(f);
                foreach (Edge e in deleteEdges)
                    mesh.RemoveEdge(e);
                foreach(var (vertices, aux) in addEdges)
                {
                    var e = mesh.NewEdge(vertices.Item1, vertices.Item2);
                    e.Aux = aux;
                }
                foreach (var (a, b, c) in addFaces)
                    mesh.NewFace(a, b, c);
            }
        }
        public delegate void LocatedVertex(Vertex v);
        public delegate void LocatedEdge(Edge e);
        public delegate void LocatedFace(Face f);
        public void LocatePoint(Vector2 point, LocatedVertex ifVertex, LocatedEdge ifEdge, LocatedFace ifFace)
        {
            int n = (int)MathF.Ceiling(MathF.Pow(vertices.Count(), 1.0f / 3));
            Random random = new Random();
            Vertex minv = null;
            float mindist = float.PositiveInfinity;
            for(int i = 0; i < n; i++)
            {
                Vertex v = vertices[random.Next() % vertices.Count()];
                float d = Vector2.Distance(v.Pos, point);
                if(d < mindist)
                    (minv, mindist) = (v, d);
            }
            Debug.Assert(minv != null);
            var (type, vertex, edge, face, _) = Walk(minv, point).Last();
            switch (type)
            {
                case CrossingType.Vertex: ifVertex(vertex); break;
                case CrossingType.AlongEdge:
                case CrossingType.AcrossEdge: ifEdge(edge); break;
                case CrossingType.Face: ifFace(face); break;
            }
        }
        public enum CrossingType { Vertex, AlongEdge, AcrossEdge, Face };
        public IEnumerable<(CrossingType,Vertex,Edge,Face,Vector2)> Walk(Vertex start, Vector2 goal)
        {
            Vertex vertex;
            Edge edge;
            Face face;
            Vector2 point;
            Vertex a, b, c;

            vertex = start;
        processVertex:
            yield return (CrossingType.Vertex, vertex, null, null, vertex.Pos);
            if (Geometry.ApproxEqual(vertex.Pos, goal))
                yield break;
            foreach (var e in vertex.Edges())
            {
                var w = e.OtherVertex(vertex);
                if(Geometry.SegmentContainsPoint(vertex.Pos, goal, w.Pos))
                {
                    yield return (CrossingType.AlongEdge, null, e, null, vertex.Pos);
                    vertex = w;
                    goto processVertex;
                }
                else if(Geometry.SegmentContainsPoint(vertex.Pos, w.Pos, goal))
                {
                    yield return (CrossingType.AlongEdge, null, e, null, goal);
                    yield break;
                }
            }
            foreach(var f in vertex.Faces())
            {
                (b, c) = f.OtherVertices(vertex);
                if (Geometry.IsTriangleFacing(vertex.Pos, b.Pos, c.Pos, goal))
                {
                    if (f.Contains(goal))
                    {
                        yield return (CrossingType.Face, null, null, f, goal);
                        yield break;
                    }
                    else
                    {
                        edge = GetEdge(b, c);
                        face = f;
                        goto acrossEdge;
                    }
                }
            }
            throw new ArgumentOutOfRangeException();
        acrossEdge:
            if(Geometry.SegmentContainsPoint(edge.Vertex1.Pos, edge.Vertex2.Pos, goal))
            {
                yield return (CrossingType.AcrossEdge, null, edge, null, goal);
                yield break;
            }
            bool ok = Geometry.IntersectLines(start.Pos, goal, edge.Vertex1.Pos, edge.Vertex2.Pos, out point);
            if (!ok) throw new ArgumentOutOfRangeException();
            yield return (CrossingType.AcrossEdge, null, edge, null, point);
            face = edge.OtherFace(face);
            Debug.Assert(face != null);
            goto processFace;
        processFace:
            if (face.Contains(goal))
            {
                yield return (CrossingType.Face, null, null, face, goal);
                yield break;
            }
            yield return (CrossingType.Face, null, null, face, point);
            (a, b) = edge.Vertices;
            c = face.OtherVertex(a, b);
            if (Geometry.IsCollinear(start.Pos, goal, c.Pos))
            {
                vertex = c;
                goto processVertex;
            }
            else if (Geometry.IsClockwise(start.Pos, goal, c.Pos) == Geometry.IsClockwise(start.Pos, goal, a.Pos))
            {
                edge = GetEdge(b, c);
                goto acrossEdge;
            }
            else
            {
                Debug.Assert(Geometry.IsClockwise(start.Pos, goal, c.Pos) == Geometry.IsClockwise(start.Pos, goal, b.Pos));
                edge = GetEdge(a, c);
                goto acrossEdge;
            }
        }
    }
    internal class CDT
    {
        Mesh mesh;
        public CDT(Vector2 min, Vector2 max)
        {
            mesh = new Mesh(min, max);
            foreach (Mesh.Edge e in mesh.Edges())
                if (MathF.Abs(Vector2.Distance(e.Vertex1.Pos, e.Vertex2.Pos) - Vector2.Distance(min, max)) >= Geometry.Epsilon)
                    e.Aux = new List<int>() { 0 };
        }
        public Mesh Mesh { get { return mesh; } }
        private void edgeQuad(Edge e, out Vertex a, out Vertex b, out Vertex c, out Vertex d, out Face f0, out Face f1)
        {
            (a, b) = e.Vertices;
            (f0, f1) = e.Faces;
            c = f0.OtherVertex(a, b);
            d = f1.OtherVertex(a, b);
            Debug.Assert(!c.Same(a) && !c.Same(b) && !d.Same(a) && !d.Same(b) && !c.Same(d));
        }
        public void delaunayFlip(Edge e)
        {
            edgeQuad(e, out var a, out var b, out var c, out var d, out var f0, out var f1);
            mesh.RemoveEdge(e);
            mesh.NewEdge(c, d);
            mesh.RemoveFace(f0);
            mesh.RemoveFace(f1);
            mesh.NewFace(a, c, d);
            mesh.NewFace(b, c, d);
        }
        private bool isDelaunay(Edge e)
        {
            edgeQuad(e, out var A, out var B, out var C, out var D, out _, out _);
            return Geometry.IsDelaunay(A.Pos, B.Pos, C.Pos, D.Pos);
        }
        private Vertex faceSplit(Face f, Vector2 point, out Edge[] fp)
        {
            fp = new Edge[3];
            (fp[0], fp[1], fp[2]) = f.Edges;
            var (a, b, c) = f.Vertices;
            Vertex p = mesh.NewVertex(point);
            mesh.NewEdge(a, p);
            mesh.NewEdge(b, p);
            mesh.NewEdge(c, p);
            mesh.RemoveFace(f);
            mesh.NewFace(a, b, p);
            mesh.NewFace(b, c, p);
            mesh.NewFace(c, a, p);
            return p;
        }
        private Vertex edgeSplit(Edge e, Vector2 point, out Edge[] fp)
        {
            edgeQuad(e, out var a, out var b, out var c, out var d, out var f0, out var f1);
            fp = new Edge[4];
            (fp[0], fp[1]) = e.Faces.Item1.OtherEdges(e);
            (fp[2], fp[3]) = e.Faces.Item2.OtherEdges(e);
            Vertex p = mesh.NewVertex(point);
            var orig = e.Aux;
            mesh.RemoveEdge(e);
            var e1 = mesh.NewEdge(a, p);
            var e2 = mesh.NewEdge(b, p);
            e1.Aux = orig;
            e2.Aux = orig;
            mesh.NewEdge(c, p);
            mesh.NewEdge(d, p);
            mesh.RemoveFace(f0);
            mesh.RemoveFace(f1);
            mesh.NewFace(a, p, c);
            mesh.NewFace(a, p, d);
            mesh.NewFace(b, p, c);
            mesh.NewFace(b, p, d);
            return p;
        }
        Mesh.Vertex insertPointInEdge(Mesh.Edge edge, Vector2 point)
        {
            point = edge.Project(point);
            var vertex = edgeSplit(edge, point, out Mesh.Edge[] Fp);
            flipEdges(point, Fp);
            return vertex;
        }
        Mesh.Vertex insertPointInFace(Mesh.Face face, Vector2 point)
        {
            var vertex = faceSplit(face, point, out Mesh.Edge[] Fp);
            flipEdges(point, Fp);
            return vertex;
        }
        void flipEdges(Vector2 point, Mesh.Edge[] Fp)
        {
            var stack = new Stack<Mesh.Edge>(Fp);
            while (stack.TryPop(out Mesh.Edge edge))
            {
                if (!IsConstrained(edge) && !isDelaunay(edge))
                {
                    var face = edge.EnumerateFaces().Where(f => !f.Contains(point)).First();
                    var (a, b) = face.OtherEdges(edge);
                    stack.Push(a);
                    stack.Push(b);
                    delaunayFlip(edge);
                }
            }
        }
        public bool IsConstrained(Mesh.Edge edge)
        {
            return edge.Aux != null && ((List<int>)edge.Aux).Count > 0;
        }
        void insertSegment(Mesh.Vertex v1, Mesh.Vertex v2, int constraintId)
        {
            /* FIXME: do we need to walk twice? */
            var crossings = new List<(CrossingType, Vertex, Edge, Face, Vector2)>(mesh.Walk(v1, v2.Pos));
            foreach (var (type, _, edge, _, point) in crossings)
                if (type == CrossingType.AcrossEdge && edge != null && IsConstrained(edge))
                    insertPointInEdge(edge, point);
            var updater = new Mesh.Updater(mesh);
            var delenda = new List<Mesh.Edge>();
            Mesh.Vertex lastvertex = null;
            foreach (var (type, vertex, edge, _, point) in mesh.Walk(v1, v2.Pos))
                switch (type)
                {
                    case CrossingType.Vertex:
                        if (lastvertex != null)
                        {
                            if (mesh.TryGetEdge(lastvertex, vertex, out Mesh.Edge e))
                            {
                                if (e.Aux == null)
                                    e.Aux = new List<int>();
                                ((List<int>)e.Aux).Add(constraintId);
                            }
                            else
                            {
                                updater.AddEdge(lastvertex, vertex, new List<int>() { constraintId });
                                retriangulate(updater, lastvertex, vertex, delenda);
                            }
                        }
                        delenda.Clear();
                        lastvertex = vertex;
                        break;
                    case CrossingType.AcrossEdge:
                        if (!IsConstrained(edge))
                        {
                            delenda.Add(edge);
                            updater.DeleteEdge(edge);
                        }
                        break;
                }
            updater.Apply();
        }
        void retriangulate(Mesh.Updater updater, Mesh.Vertex a, Mesh.Vertex b, List<Mesh.Edge> delenda)
        {
            var upper = new List<Mesh.Vertex>();
            var lower = new List<Mesh.Vertex>();
            foreach (Mesh.Edge e in delenda)
            {
                var (p1, p2) = e.Vertices;
                if (!Geometry.IsClockwise(a.Pos, b.Pos, p1.Pos))
                    (p1, p2) = (p2, p1);
                if (upper.Count == 0 || !upper[upper.Count - 1].Same(p1))
                    upper.Add(p1);
                if (lower.Count == 0 || !lower[lower.Count - 1].Same(p2))
                    lower.Add(p2);
            }
            triangulatePolygon(updater, upper, 0, upper.Count, a, b);
            triangulatePolygon(updater, lower, 0, lower.Count, a, b);
        }
        void triangulatePolygon(Mesh.Updater updater, List<Mesh.Vertex> polygon, int start, int end, Mesh.Vertex a, Mesh.Vertex b)
        {
            var ci = start;
            if (end > start + 1)
            {
                for (int i = 1; i < end; i++)
                    if (Geometry.IsDelaunay(a.Pos, b.Pos, polygon[ci].Pos, polygon[i].Pos))
                        ci = i;
                triangulatePolygon(updater, polygon, start, ci, a, polygon[ci]);
                triangulatePolygon(updater, polygon, ci + 1, end, polygon[ci], b);
            }
            if (end > start)
            {
                updater.AddFace(a, b, polygon[ci]);
            }
        }
        public void InsertConstraint(List<Vector2> polygon, int constraintId)
        {
            List<Mesh.Vertex> l = new List<Mesh.Vertex>();
            foreach (Vector2 p in polygon)
            {
                mesh.LocatePoint(p,
                    vertex => l.Add(vertex),
                    edge => l.Add(insertPointInEdge(edge, p)),
                    face => l.Add(insertPointInFace(face, p)));
            }
            for (int i = 0; i < l.Count - 1; i++)
                insertSegment(l[i], l[i + 1], constraintId);
        }
        private float searchWidth(Vertex C, Face T, Edge e, float d)
        {
            var (U, V) = e.Vertices;
            if (!Geometry.IsAcute(C.Pos, U.Pos, V.Pos) || !Geometry.IsAcute(C.Pos, V.Pos, U.Pos))
                return d;
            var dd = MathF.Abs(Geometry.SignedDistanceLineToPoint(U.Pos, V.Pos, C.Pos));
            if (dd > d)
                return d;
            else if (IsConstrained(e))
                return dd;
            Face TT = e.OtherFace(T);
            var (f, g) = TT.OtherEdges(e);
            d = searchWidth(C, TT, f, d);
            return searchWidth(C, TT, g, d);
        }
        public float TriangleWidth(Face T, Edge a, Edge b)
        {
            var (A, B, C) = T.Vertices;
            if (!a.Contains(B)) (A, B) = (B, A);
            else if (!a.Contains(C)) (A, C) = (C, A);
            Debug.Assert(!a.Contains(A));
            if (b.Contains(B)) (B, C) = (C, B);
            Debug.Assert(!b.Contains(B));
            var ok = mesh.TryGetEdge(A, B, out var c);
            Debug.Assert(ok);
            var d = MathF.Min(Vector2.Distance(B.Pos, C.Pos), Vector2.Distance(A.Pos, C.Pos));
            if (!Geometry.IsAcute(C.Pos, A.Pos, B.Pos) || !Geometry.IsAcute(C.Pos, B.Pos, A.Pos))
                return d;
            else if (IsConstrained(c))
                return MathF.Abs(Geometry.SignedDistanceLineToPoint(A.Pos, B.Pos, C.Pos));
            else
                return searchWidth(C, T, c, d);
        }
    }

    internal class MeshManipulator
    {
        private const int Scale = 400;
        private const int Offset = 50;
        Mesh mesh;
        CDT cdt;
        Vector2 cursor;
        List<Vector2> polygon;
        bool lastDown;
        KeyboardState lastState;
        public MeshManipulator()
        {
            cdt = new CDT(new Vector2(0, 0), new Vector2(1, 1));
            mesh = cdt.Mesh;
            polygon = new List<Vector2>();
        }
        Vector2 snap(Vector2 p)
        {
            const float tolerance = 0.05f;
            foreach(Vector2 v in polygon)
                if (Vector2.Distance(v, p) < tolerance)
                    return v;
            /*foreach (Mesh.Vertex v in mesh.Vertices())
                if (Vector2.Distance(v.Pos, p) < tolerance)
                    return v.Pos;
            foreach (Mesh.Edge e in mesh.Edges())
                if (e.DistanceToPoint(p) < tolerance)
                    return e.Project(p);*/
            return p;
        }
        public void Update()
        {
            var m = Mouse.GetState();
            var k = Keyboard.GetState();
            cursor = new Vector2(((float)m.X - Offset) / Scale, ((float)m.Y - Offset) / Scale);
            cursor = snap(cursor);
            if(m.LeftButton == ButtonState.Released && lastDown)
            {
                if(cursor.X >= 0 && cursor.X <= 1 && cursor.Y >= 0 && cursor.Y <= 1)
                    polygon.Add(cursor);
            }
            if(k.IsKeyUp(Keys.Space) && lastState.IsKeyDown(Keys.Space))
            {
                StringBuilder sb = new StringBuilder();
                sb.Append("cdt.InsertConstraint(new List<Vector2>() {");
                foreach (Vector2 v in polygon)
                    sb.AppendFormat("new Vector2({0}f, {1}f),", v.X, v.Y);
                sb.Append("}, 0);");
                Trace.WriteLine(sb.ToString());
                cdt.InsertConstraint(polygon, 0);
                polygon.Clear();
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
            var dotRed = new Polygon(device, 6, 2, 0, Color.Red, true);
            var dotPurple = new Polygon(device, 6, 2, 0, Color.Purple, true);
            foreach (Mesh.Face f in mesh.Faces())
            {
                var fc = f.Contains(cursor) ? Color.Pink : Color.Green;
                new Polygon(device, new Vector2[] { toScreen(f.Vertex1.Pos), toScreen(f.Vertex3.Pos), toScreen(f.Vertex2.Pos) }, fc, true).DrawAt(device, new Vector2(0, 0));
            }
            foreach (Mesh.Vertex v in mesh.Vertices())
                dotRed.DrawAt(device, toScreen(v.Pos) / 32);
            foreach (Mesh.Edge v in mesh.Edges())
            {
                var ec = cdt.IsConstrained(v) ? Color.Blue : Color.Red;
                new Polygon(device, new Vector2[] { toScreen(v.Vertex1.Pos), toScreen(v.Vertex2.Pos) }, ec).DrawAt(device, new Vector2(0, 0));
            }
                
            for(int i = 0; i < polygon.Count - 1; i++)
                new Polygon(device, new Vector2[] { toScreen(polygon[i]), toScreen(polygon[i + 1]) }, Color.Purple).DrawAt(device, new Vector2(0, 0));
            foreach (Vector2 v in polygon)
                dotPurple.DrawAt(device, toScreen(v) / 32);
            dotPurple.DrawAt(device, toScreen(cursor) / 32);
        }
    }
}
