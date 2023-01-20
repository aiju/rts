using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RTS
{
    internal struct Contact
    {
        public Agent Agent { get; }
        public Mesh.Edge Edge { get; }
        public Vector2 Point { get; }
        public Contact(Agent agent, Mesh.Edge edge, Vector2 point)
        {
            Agent = agent;
            Edge = edge;
            Point = point;
        }
    }

    internal class CollisionDetection
    {
        const float Slop = 0.0001f;
        CDT cdt;
        List<Agent> agents;
        Dictionary<Agent, List<Contact>> contacts;
        public CollisionDetection(CDT cdt)
        {
            this.cdt = cdt;
            agents = new List<Agent>();
            contacts = new Dictionary<Agent, List<Contact>>();
        }
        public void Register(Agent a)
        {
            agents.Add(a);
        }
        private void checkAgentWall(Agent a)
        {
            contacts[a] = new List<Contact>();
            foreach (Mesh.Edge e in cdt.Mesh.FindCloseEdges(a.Position, a.Radius + Slop))
                if (cdt.IsConstrained(e))
                {
                    var p = Geometry.ClosestPointOnSegment(e.Vertex1.Pos, e.Vertex2.Pos, a.Position);
                    contacts[a].Add(new Contact(a, e, p));
                }

        }
        public List<Contact> GetContacts(Agent a)
        {
            return contacts[a];
        }
        public void Update()
        {
            foreach (var a in agents)
                checkAgentWall(a);
            foreach(var a in agents)
                foreach(var b in agents)
                    if(a != b && Vector2.Distance(a.Position, b.Position) <= a.Radius + b.Radius + Slop)
                    {
                        Vector2 v = Geometry.Normalized(a.Position - b.Position);
                        contacts[a].Add(new Contact(a, null, b.Position + v * b.Radius));
                    }
        }
    }
    internal class Agent
    {
        Pathfinding pathfinding;
        CollisionDetection collisionDetection;
        const float MaxAcceleration = 10.0f;
        const float MaxVelocity = 0.2f;
        const float Beta = 10.0f;
        const float Alpha = Beta * Beta / 4;
        const float ExpulsionSpeed = 1.0f;
        public Vector2 Position { get; private set; }
        public Vector2 Target { get; private set; }
        public Vector2 Velocity { get; private set; }
        public List<Vector2> Waypoints { get; private set; }
        public float Radius { get; }
        Pathfinding.Path path;
        public Agent(Pathfinding pathfinding, CollisionDetection collisionDetection, Vector2 position, float radius)
        {
            this.pathfinding = pathfinding;
            this.collisionDetection = collisionDetection;
            Position = position;
            Target = position;
            Radius = radius;
            Waypoints = new List<Vector2>();
            collisionDetection.Register(this);
        }
        public void GoTo(Vector2 target)
        {
            Target = target;
            path = new Pathfinding.Path(pathfinding, Position, target, Radius);
            if (path.Route != null)
                Waypoints = path.Route;
            else
                Waypoints.Clear();
        }
        public void Update(GameTime gameTime)
        {
            float dt = (float) gameTime.ElapsedGameTime.TotalSeconds;
            Vector2 force = Vector2.Zero;
            while (Waypoints.Any() && Vector2.Distance(Position, Waypoints.First()) <= Radius)
                Waypoints.RemoveAt(0);
            if(Waypoints.Any())
                force = Alpha * (Waypoints.First() - Position);
            //force = Alpha * (Target - Position);
            //if(Waypoints.Count() > 1 && Vector2.Distance(Waypoints.First(), Position) < 4.0f * Radius)
            //    force += Alpha * (Waypoints[1] - Position);
            if (force.Length() >= MaxAcceleration) force *= MaxAcceleration / force.Length();
            force -= Beta * Velocity;
            Velocity += force * dt;
            if (Velocity.Length() >= MaxVelocity) Velocity *= MaxVelocity / Velocity.Length();
            foreach(Contact c in collisionDetection.GetContacts(this))
            {
                Vector2 n = c.Point - Position;
                float d = n.Length();
                n /= d;
                float v = Vector2.Dot(Velocity, n);
                if(v > 0)
                    Velocity -= n * v;
                if(d < Radius)
                    Velocity -= n * ExpulsionSpeed * (Radius - d);
            }
            Debug.Assert(!float.IsNaN(Velocity.X));
            Position += Velocity * dt;
        }
        public void Draw(GraphicsDevice device)
        {

        }
    }
    internal class AgentManipulator : IEntity
    {
        public const float Scale = 400;
        public const float Offset = 50;
        Mesh mesh;
        CDT cdt;
        CollisionDetection collisionDetection;
        Pathfinding pathfinding;
        Vector2 cursor;
        bool lastDown;
        KeyboardState lastState;
        public const float radius = 0.01f;
        List<Agent> agents;
        public AgentManipulator()
        {
            pathfinding = new Pathfinding(new Vector2(0, 0), new Vector2(1, 1));
            collisionDetection = new CollisionDetection(pathfinding.CDT);
            cdt = pathfinding.CDT;
            agents = new List<Agent>();

            var centres = new List<(Vector2, float)>();
            Random random = new Random();
            for (int i = 0; i < 50; i++)
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
            foreach (var (v, r) in centres)
            {
                var polygon = new List<Vector2>();
                int n = 3 + random.Next() % 8;
                for (int i = 0; i <= n; i++)
                    polygon.Add(new Vector2(v.X + r * MathF.Cos(2 * MathF.PI * i / n), v.Y + r * MathF.Sin(2 * MathF.PI * i / n)));
                cdt.InsertConstraint(polygon, 1);
            }
            for (int i = 0; i < 10; i++)
            {
                Vector2 agentPos, agentGoal;
            again:
                agentPos = new Vector2(random.NextSingle(), random.NextSingle());
                foreach (var (v, r) in centres)
                    if (Vector2.Distance(v, agentPos) <= r + radius)
                        goto again;
                    goalAgain:
                agentGoal = new Vector2(random.NextSingle(), random.NextSingle());
                foreach (var (v, r) in centres)
                    if (Vector2.Distance(v, agentGoal) <= r + radius)
                        goto goalAgain;

                mesh = cdt.Mesh;
                var agent = new Agent(pathfinding, collisionDetection, agentPos, radius);
                agent.GoTo(agentGoal);
                agents.Add(agent);
            }
        }
        public void Update(GameTime gameTime)
        {
            var m = Mouse.GetState();
            var k = Keyboard.GetState();
            cursor = new Vector2(((float)m.X - Offset) / Scale, ((float)m.Y - Offset) / Scale);
            bool inRange = cursor.X > 0 && cursor.X < 1 && cursor.Y > 0 && cursor.Y < 1;
            if (inRange && m.LeftButton == ButtonState.Pressed && !lastDown)
            {
                foreach (Agent agent in agents)
                    agent.GoTo(cursor);
            }
            lastDown = m.LeftButton == ButtonState.Pressed;
            lastState = k;
            collisionDetection.Update();
            foreach(Agent agent in agents)
                agent.Update(gameTime);
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
                var fc = Color.Green;
                ctx.Polygon(new Vector2[] { toScreen(f.Vertex1.Pos), toScreen(f.Vertex3.Pos), toScreen(f.Vertex2.Pos) }, fc, true);
            }
            foreach (Mesh.Edge v in mesh.Edges())
            {
                if (!cdt.IsConstrained(v)) continue;
                var ec = cdt.IsConstrained(v) ? Color.Blue : Color.Olive;
                ctx.Line(toScreen(v.Vertex1.Pos), toScreen(v.Vertex2.Pos), ec);
            }

            ctx.RegularPolygon(toScreen(cursor), 6, 2, 0, Color.Purple, true);

            foreach (Agent agent in agents)
            {
                ctx.RegularPolygon(toScreen(agent.Position), 32, radius * Scale, 0, Color.Red, true);
                ctx.Line(toScreen(agent.Position), toScreen(agent.Target), Color.White);
                if (agent.Waypoints.Count > 1)
                {
                    //Shape.FromVertices(device, agent.Waypoints.ToArray()).Transform(toScreen).Draw(device);
                }
            }
        }
    }
}
