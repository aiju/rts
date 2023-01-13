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
    }
    internal class Mesh
    {
        public const float Epsilon = Geometry.Epsilon;
        List<Vector2> vertices;
        List<(int, int)> edges;
        List<object> edgeAux;
        List<(int, int, int)> faces;
        private int addVertex(Vector2 p)
        {
            int n = vertices.Count;
            vertices.Add(p);
            return n;
        }
        private int addEdge(int v1, int v2)
        {
            Debug.Assert(v1 != v2);
            if (TryGetEdge(GetVertex(v1), GetVertex(v2), out Edge edge))
                return edge.Id;
            int n = edges.Count;
            edges.Add((v1, v2));
            edgeAux.Add(null);
            return n;
        }
        private void updateEdge(int e, int a, int b)
        {
            Debug.Assert(a != b);
            edges[e] = (a, b);
        }
        private void addFace(int v1, int v2, int v3)
        {
            Debug.Assert(v1 != v2 && v1 != v3 && v2 != v3);
            addEdge(v1, v2);
            addEdge(v2, v3);
            addEdge(v1, v3);
            faces.Add((v1, v2, v3));
        }
        private void updateFace(int f, int a, int b, int c)
        {
            faces[f] = (a, b, c);
        }
        public Mesh(Vector2 min, Vector2 max)
        {
            vertices = new List<Vector2>();
            edges = new List<(int, int)>();
            edgeAux = new List<object>();
            faces = new List<(int, int, int)>();
            addVertex(min);
            addVertex(new Vector2(max.X, min.Y));
            addVertex(max);
            addVertex(new Vector2(min.X, max.Y));
            addFace(0, 1, 2);
            addFace(0, 2, 3);
        }
        public int EdgeCount { get { return edges.Count; } }
        public Vertex GetVertex(int Id) { return new Mesh.Vertex(this, Id); }
        public Edge GetEdge(int Id) { return new Mesh.Edge(this, Id); }
        public Face GetFace(int Id) { return new Mesh.Face(this, Id); }
        public IEnumerable<Vertex> Vertices() { return vertices.Select((i, j) => GetVertex(j)); }
        public IEnumerable<Edge> Edges() { return edges.Select((i, j) => GetEdge(j)); }
        public IEnumerable<Face> Faces() { return faces.Select((i, j) => GetFace(j)); }
        internal class Vertex
        {
            Mesh mesh;
            public int Id { get; }
            public Vector2 Pos { get { return mesh.vertices[Id];  } }
            public Vertex(Mesh mesh, int id)
            {
                this.mesh = mesh;
                Id = id;
            }
            internal IEnumerable<Edge> Edges()
            {
                return mesh.Edges().Where(e => e.Contains(this));
            }
            public override bool Equals(object obj)
            {
                Vertex v = obj as Vertex;
                return v != null && v.Id == Id;
            }
            public override int GetHashCode()
            {
                return Id;
            }
        }
        internal class Edge
        {
            Mesh mesh;
            public int Id { get; }
            public Edge(Mesh mesh, int id)
            {
                this.mesh = mesh;
                Id = id;
            }
            public Vertex Vertex1 { get { return mesh.GetVertex(mesh.edges[Id].Item1); } }
            public Vertex Vertex2 { get { return mesh.GetVertex(mesh.edges[Id].Item2); } }
            public (Vertex, Vertex) Vertices
            {
                get
                {
                    var (a, b) = mesh.edges[Id];
                    return (mesh.GetVertex(a), mesh.GetVertex(b));
                }
            }
            public object Aux {
                get
                {
                    return mesh.edgeAux[Id];
                }
                set
                {
                    mesh.edgeAux[Id] = value;
                }
            }
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
                var (a, b) = mesh.edges[Id];
                return v.Id == a || v.Id == b;
            }
            private void quad(out int a, out int b, out int c, out int d, out int f0, out int f1)
            {
                (a, b) = mesh.edges[Id];
                var faces = AdjacentFaces().ToArray();
                Debug.Assert(faces.Count() == 2);
                int v2, v3, w2, w3;
                (c, v2, v3) = mesh.faces[faces[0].Id];
                (d, w2, w3) = mesh.faces[faces[1].Id];
                if (v2 != a && v2 != b) c = v2;
                else if (v3 != a && v3 != b) c = v3;
                if (w2 != a && w2 != b) d = w2;
                else if (w3 != a && w3 != b) d = w3;
                Debug.Assert(c != a && c != b && d != a && d != b && c != d);
                f0 = faces[0].Id;
                f1 = faces[1].Id;
            }
            public Vertex Split(Vector2 point, out Edge splitEdge, out Edge[] fp)
            {
                quad(out int a, out int b, out int c, out int d, out int f0, out int f1);
                fp = AdjacentFaces().SelectMany((f,i) => f.Edges).Where(e => e.Id != Id).ToArray();
                Debug.Assert(fp.Count() == 4);
                int p = mesh.addVertex(point);
                mesh.updateEdge(Id, a, p);
                splitEdge = mesh.GetEdge(mesh.addEdge(p, b));
                mesh.addEdge(p, c);
                mesh.addEdge(p, d);
                mesh.updateFace(f0, a, p, c);
                mesh.updateFace(f1, a, p, d);
                mesh.addFace(b, p, c);
                mesh.addFace(b, p, d);
                return mesh.GetVertex(p);
            }
            public void DelaunayFlip()
            {
                quad(out int a, out int b, out int c, out int d, out int f0, out int f1);
                mesh.updateEdge(Id, c, d);
                mesh.updateFace(f0, a, c, d);
                mesh.updateFace(f1, b, c, d);
            }
            public bool IsDelaunay()
            {
                /* TODO degeneracy */
                quad(out int av, out int bv, out int cv, out int dv, out int _, out int _);
                var A = mesh.vertices[av];
                var B = mesh.vertices[bv];
                var C = mesh.vertices[cv];
                var D = mesh.vertices[dv];
                return Geometry.IsDelaunay(A, B, C, D);
            }
            public Vector2 Project(Vector2 point)
            {
                return Geometry.ProjectPointToLine(point, Vertex1.Pos, Vertex2.Pos);
            }
            public IEnumerable<Face> AdjacentFaces()
            {
                var (a, b) = mesh.edges[Id];
                var fs = new List<Face>();
                for(int i = 0; i < mesh.faces.Count; i++)
                {
                    var (x, y, z) = mesh.faces[i];
                    if ((a == x || a == y || a == z) && (b == x || b == y || b == z))
                        fs.Add(mesh.GetFace(i));
                }
                return fs;
            }
            public override bool Equals(object obj)
            {
                Edge e = obj as Edge;
                return e != null && e.Id == Id;
            }
            public override int GetHashCode()
            {
                return Id;
            }
        }
        internal class Face
        {
            Mesh mesh;
            public int Id { get; }
            public Face(Mesh mesh, int id)
            {
                this.mesh = mesh;
                Id = id;
            }
            public override bool Equals(object obj)
            {
                Face f = obj as Face;
                return f != null && f.Id == Id;
            }
            public override int GetHashCode()
            {
                return Id;
            }
            public Vertex Vertex1 { get { return mesh.GetVertex(mesh.faces[Id].Item1); } }
            public Vertex Vertex2 { get { return mesh.GetVertex(mesh.faces[Id].Item2); } }
            public Vertex Vertex3 { get { return mesh.GetVertex(mesh.faces[Id].Item3); } }
            public (Vertex, Vertex, Vertex) Vertices
            {
                get
                {
                    var (a, b, c) = mesh.faces[Id];
                    return (mesh.GetVertex(a), mesh.GetVertex(b), mesh.GetVertex(c));
                }
            }
            public Mesh.Edge[] Edges
            {
                get
                {
                    return mesh.Edges().Where(Contains).ToArray();
                }
            }
            public Vertex Split(Vector2 point, out Edge[] fp)
            {
                fp = Edges;
                var (a, b, c) = mesh.faces[Id];
                int p = mesh.addVertex(point);
                mesh.addEdge(a, p);
                mesh.addEdge(b, p);
                mesh.addEdge(c, p);
                mesh.updateFace(Id, a, b, p);
                mesh.addFace(b, c, p);
                mesh.addFace(c, a, p);
                return mesh.GetVertex(p);
            }
            public bool Contains(Edge e)
            {
                var (a, b, c) = mesh.faces[Id];
                var (p, q) = mesh.edges[e.Id];
                return (p == a || p == b || p == c) && (q == a || q == b || q == c);
            }
            public bool Contains(Vertex v)
            {
                var (a, b, c) = mesh.faces[Id];
                return a == v.Id || b == v.Id || c == v.Id;
            }
            public bool Contains(Vector2 point)
            {
                return Geometry.FaceContainsPoint(Vertex1.Pos, Vertex2.Pos, Vertex3.Pos, point);
            }

            public IEnumerable<(Face, Edge)> Neighbours()
            {
                foreach(Edge e in Edges)
                    foreach (Face f in e.AdjacentFaces())
                        if (f.Id != Id)
                            yield return (f, e);
            }
            public Vector2 Centroid()
            {
                var (a, b, c) = Vertices;
                return (a.Pos + b.Pos + c.Pos) / 3;
            }
            public Face FaceOppositeTo(Edge e)
            {
                var f = e.AdjacentFaces().ToArray();
                Debug.Assert(f.Count() == 2);
                if (f[0].Id == Id) return f[1];
                return f[0];
            }
        }
        internal class Updater
        {
            HashSet<int> deleteEdges;
            HashSet<int> deleteFaces;
            HashSet<(int, int)> addEdges;
            HashSet<(int, int, int)> addFaces;
            Dictionary<(int, int), object> setAux;
            Mesh mesh;
            public Updater(Mesh mesh)
            {
                this.mesh = mesh;
                deleteEdges = new HashSet<int>();
                deleteFaces = new HashSet<int>();
                addEdges = new HashSet<(int, int)>();
                addFaces = new HashSet<(int, int, int)>();
                setAux = new Dictionary<(int, int), object>();
            }
            public void DeleteEdge(Edge e)
            {
                deleteEdges.Add(e.Id);
                foreach (Face f in e.AdjacentFaces())
                    deleteFaces.Add(f.Id);
            }
            public void AddEdge(Vertex a, Vertex b, object aux)
            {
                if (a.Id > b.Id) (b, a) = (a, b);
                addEdges.Add((a.Id, b.Id));
                setAux.Add((a.Id, b.Id), aux);
            }
            public void AddFace(Vertex a, Vertex b, Vertex c)
            {
                if (a.Id > b.Id) (b, a) = (a, b);
                if (b.Id > c.Id) (c, b) = (b, c);
                if (a.Id > b.Id) (b, a) = (a, b);
                addEdges.Add((a.Id, b.Id));
                addEdges.Add((b.Id, c.Id));
                addEdges.Add((a.Id, c.Id));
                addFaces.Add((a.Id, b.Id, c.Id));
            }
            public void Apply()
            {
                if(deleteEdges.Count > 0)
                {
                    int j = 0;
                    for (int i = 0; i < mesh.edges.Count; i++)
                        if (!deleteEdges.Contains(i))
                        {
                            mesh.edges[j] = mesh.edges[i];
                            mesh.edgeAux[j] = mesh.edgeAux[i];
                            j++;
                        }
                    mesh.edges.RemoveRange(j, mesh.edges.Count - j);
                    mesh.edgeAux.RemoveRange(j, mesh.edgeAux.Count - j);
                }
                if (deleteFaces.Count > 0)
                {
                    int j = 0;
                    for (int i = 0; i < mesh.faces.Count; i++)
                        if (!deleteFaces.Contains(i))
                            mesh.faces[j++] = mesh.faces[i];
                    mesh.faces.RemoveRange(j, mesh.faces.Count - j);
                }
                foreach (var (a, b) in addEdges)
                {
                    int e = mesh.addEdge(a, b);
                    if (setAux.TryGetValue((a, b), out object aux))
                        mesh.edgeAux[e] = aux;
                }
                foreach (var (a, b, c) in addFaces)
                    mesh.addFace(a, b, c);
            }
        }
        public delegate void LocatedVertex(Vertex v);
        public delegate void LocatedEdge(Edge e);
        public delegate void LocatedFace(Face f);
        public void LocatePoint(Vector2 point, LocatedVertex ifVertex, LocatedEdge ifEdge, LocatedFace ifFace)
        {
            foreach(Vertex v in Vertices())
                if(Vector2.Distance(point, v.Pos) < Epsilon)
                {
                    ifVertex(v);
                    return;
                }
            foreach (Edge e in Edges())
                if (e.Contains(point))
                {
                    ifEdge(e);
                    return;
                }
            foreach (Face f in Faces())
                if(f.Contains(point))
                {
                    ifFace(f);
                    return;
                }
            throw new ArgumentOutOfRangeException();
        }
        public void Walk(Vertex v1, Vertex v2, Action<Edge, Vector2> crossedEdge, Action<Vertex> crossedVertex)
        {
            var crossings = new List<(float, Vertex, Edge, Vector2)>();
            foreach (Vertex v in Vertices())
                if (Geometry.DistanceSegmentToPoint(v1.Pos, v2.Pos, v.Pos) < Epsilon)
                    crossings.Add((Geometry.DistanceAlongSegment(v1.Pos, v2.Pos, v.Pos), v, null, v.Pos));
            foreach (Edge e in Edges())
                if (!(v1.Id == e.Vertex1.Id && v2.Id == e.Vertex2.Id ||
                     v2.Id == e.Vertex1.Id && v1.Id == e.Vertex1.Id) &&
                    Geometry.IntersectSegments(v1.Pos, v2.Pos, e.Vertex1.Pos, e.Vertex2.Pos, out Vector2 point))
                {
                    if (Vector2.Distance(e.Vertex1.Pos, point) >= Epsilon && Vector2.Distance(e.Vertex2.Pos, point) >= Epsilon)
                        crossings.Add((Geometry.DistanceAlongSegment(v1.Pos, v2.Pos, point), null, e, point));
                }
            crossings.Sort();
            foreach (var (f, v, e, p) in crossings)
            {
                if (v != null) crossedVertex(v);
                else crossedEdge(e, p);
            }
        }
        public bool TryGetEdge(Vertex v1, Vertex v2, out Edge edge)
        {
            foreach(Edge e in Edges())
            {
                var (w1, w2) = e.Vertices;
                if(v1.Id == w1.Id && v2.Id == w2.Id || v1.Id == w2.Id && v2.Id == w1.Id)
                {
                    edge = e;
                    return true;
                }
            }
            edge = null;
            return false;
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
        Mesh.Vertex insertPointInEdge(Mesh.Edge edge, Vector2 point)
        {
            point = edge.Project(point);
            var orig = (List<int>)edge.Aux;
            var vertex = edge.Split(point, out Mesh.Edge splitEdge, out Mesh.Edge[] Fp);
            if (orig != null)
                splitEdge.Aux = new List<int>(orig);
            flipEdges(point, Fp);
            return vertex;
        }
        Mesh.Vertex insertPointInFace(Mesh.Face face, Vector2 point)
        {
            var vertex = face.Split(point, out Mesh.Edge[] Fp);
            flipEdges(point, Fp);
            return vertex;
        }
        void flipEdges(Vector2 point, Mesh.Edge[] Fp)
        {
            var stack = new Stack<Mesh.Edge>(Fp);
            while (stack.TryPop(out Mesh.Edge edge))
            {
                if (!IsConstrained(edge) && !edge.IsDelaunay())
                {
                    var faces = edge.AdjacentFaces();
                    Mesh.Face face = faces.Where(f => !f.Contains(point)).First();
                    foreach (Mesh.Edge e in face.Edges)
                        if (e.Id != edge.Id)
                            stack.Push(e);
                    edge.DelaunayFlip();
                }
            }
        }
        public bool IsConstrained(Mesh.Edge edge)
        {
            return edge.Aux != null && ((List<int>)edge.Aux).Count > 0;
        }
        void insertSegment(Mesh.Vertex v1, Mesh.Vertex v2, int constraintId)
        {
            mesh.Walk(v1, v2,
                (edge, point) => {
                    if (IsConstrained(edge))
                        insertPointInEdge(edge, point);
                },
                _ => { });
            var updater = new Mesh.Updater(mesh);
            var delenda = new List<Mesh.Edge>();
            Mesh.Vertex lastvertex = null;
            Action<Mesh.Vertex, Mesh.Vertex> handleEdge = (v, vs) =>
                {
                    if (mesh.TryGetEdge(v, vs, out Mesh.Edge edge))
                    {
                        if (edge.Aux == null)
                            edge.Aux = new List<int>();
                        ((List<int>)edge.Aux).Add(constraintId);
                    }
                    else
                    {
                        updater.AddEdge(v, vs, new List<int>() { constraintId });
                        retriangulate(updater, v, vs, delenda);
                    }
                };
            Action<Mesh.Vertex> crossedPoint = vertex =>
                {
                    if (lastvertex != null) handleEdge(lastvertex, vertex);
                    delenda.Clear();
                    lastvertex = vertex;
                };
            mesh.Walk(v1, v2,
                (edge, point) =>
                {
                    if (!IsConstrained(edge))
                    {
                        delenda.Add(edge);
                        updater.DeleteEdge(edge);
                    }
                }, crossedPoint);
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
                if (upper.Count == 0 || upper[upper.Count - 1].Id != p1.Id)
                    upper.Add(p1);
                if (lower.Count == 0 || lower[lower.Count - 1].Id != p2.Id)
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
            Face TT = T.FaceOppositeTo(e);
            var l = TT.Edges.Where(ee => ee.Id != e.Id).ToArray();
            Debug.Assert(l.Count() == 2);
            d = searchWidth(C, TT, l[0], d);
            return searchWidth(C, TT, l[1], d);
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
