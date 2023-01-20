using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Security.AccessControl;

namespace RTS
{
    interface IDrawAt
    {
        void DrawAt(GraphicsDevice device, Vector2 position);
    }
    interface IPosition
    {
        Vector2 Position { get; }
    }
    interface IEntity
    {
        void Draw(GraphicsDevice device);
        void Update(GameTime gameTime);
    }
    internal class PositionOffset : IPosition
    {
        IPosition pos;
        Vector2 off;
        public PositionOffset(IPosition pos, Vector2 off)
        {
            this.pos = pos;
            this.off = off;
        }
        public Vector2 Position
        {
            get
            {
                return pos.Position + off;
            }
        }
    }
    internal class BuildingSupplier : IEntity, IPosition
    {
        ResourceSource src;
        bool busy;
        float status;
        public Vector2 Position { get; set; }
        public BuildingSupplier(JobManager jobManager, Timer timer, Resource res, Vector2 position)
        {
            this.Position = position;
            src = jobManager.NewSource(res, this);
            busy = true;
            status = 0;
        }
        public void Draw(GraphicsDevice device)
        {
            //new Polygon(device, 4, 32, 45 + 90 * status, Color.Red).DrawAt(device, Position);
            if(src.Have > 0)
                src.Resource.Icon.DrawAt(device, Position);
        }
        public void Update(GameTime gameTime)
        {
            if (!busy)
            {
                if (src.Have < 1)
                {
                    busy = true;
                    status = 0;
                }
            }
            else
            {
                float st = status + (float)gameTime.ElapsedGameTime.TotalSeconds;
                if (st >= 1)
                {
                    src.Add(1);
                    busy = false;
                    status = 0;
                }
                else
                {
                    status = st;
                }
            }
        }
    }
    internal class BuildingConverter : IEntity, IPosition
    {
        ResourceSource src;
        ResourceSink sink;
        bool busy;
        float status;
        public Vector2 Position { get; set; }
        public BuildingConverter(JobManager jobManager, Timer timer, Resource res1, Resource res2, Vector2 position)
        {
            this.Position = position;
            src = jobManager.NewSource(res2, new PositionOffset(this, new Vector2(0.25f, 0)));
            sink = jobManager.NewSink(res1, new PositionOffset(this, new Vector2(-0.25f, 0)));
        }
        public void Draw(GraphicsDevice device)
        {
            //new Polygon(device, 4, 32, 45 + 90 * status, Color.Red).DrawAt(device, Position);
            if (sink.Have > 0)
                sink.Resource.Icon.DrawAt(device, sink.Position.Position);
            if (src.Have > 0)
                src.Resource.Icon.DrawAt(device, src.Position.Position);
        }
        public void Update(GameTime gameTime)
        {
            while (sink.Want < 1)
                sink.Request(1);
            if (!busy)
            {
                if (sink.Have > 0 && src.Have == 0)
                {
                    sink.Take(1);
                    busy = true;
                    status = 0;
                }
            }
            else
            {
                float st = status + (float) gameTime.ElapsedGameTime.TotalSeconds;
                if(st >= 1)
                {
                    src.Add(1);
                    busy = false;
                    status = 0;
                }
                else
                {
                    status = st;
                }
            }
        }
    }
    internal class BuildingConsumer : IEntity, IPosition
    {
        ResourceSink sink;
        public Vector2 Position { get; set; }
        public BuildingConsumer(JobManager jobManager, Timer timer, Resource res, Vector2 position)
        {
            this.Position = position;
            sink = jobManager.NewSink(res, this);
            sink.OnFill += () => sink.Request(1);
            sink.Request(1);
        }
        public void Draw(GraphicsDevice device)
        {
            //new Polygon(device, 4, 32, 45, Color.Red).DrawAt(device, Position);
        }
        public void Update(GameTime gameTime)
        {

        }
    }
    internal class Worker : IEntity
    {
        public Walker Walker;
        public Resource Carrying { get; set; }
        public Worker(Vector2 position)
        {
            Walker = new Walker(position, 2.0f);
        }

        public void Draw(GraphicsDevice device)
        {
            //new Polygon(device, 6, 8, 0, Color.Red).DrawAt(device, Walker.Position);
            if(Carrying != null)
                Carrying.Icon.DrawAt(device, Walker.Position);
        }
        public void Update(GameTime gameTime)
        {
            Walker.Update(gameTime);
        }
    }

    public class Game1 : Game
    {
        private GraphicsDeviceManager _graphics;
        private SpriteBatch _spriteBatch;
        private List<IEntity> _entities;
        private ResourceManager _resourceManager;
        private JobManager _jobManager;
        private Timer _timer;
        private MeshManipulator _meshManipulator;
        private PathfindingManipulator _pathfindingManipulator;
        private AgentManipulator _agentManipulator;

        public Game1()
        {
            _graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";
            IsMouseVisible = true;
            _entities = new List<IEntity>();
            _resourceManager = new ResourceManager();
            _jobManager = new JobManager();
            _timer = new Timer();
            _meshManipulator = new MeshManipulator();
            _pathfindingManipulator = new PathfindingManipulator();
            _agentManipulator = new AgentManipulator();
            //_entities.Add(_meshManipulator);
            //_entities.Add(_pathfindingManipulator);
            _entities.Add(_agentManipulator);
            //_entities.Add(new MazeManipulator());
        }

        protected override void Initialize()
        {
            /*Resource wood = _resourceManager.NewResource("Wood", new Polygon(GraphicsDevice, 3, 4, 0, Color.Green));
            Resource iron = _resourceManager.NewResource("Iron", new Polygon(GraphicsDevice, 5, 4, 0, Color.Green));
            _entities.Add(new BuildingSupplier(_jobManager, _timer, wood, new Vector2(1, 1)));
            _entities.Add(new BuildingConverter(_jobManager, _timer, wood, iron, new Vector2(5, 1)));
            _entities.Add(new BuildingConsumer(_jobManager, _timer, iron, new Vector2(5, 5)));
            var worker = new Worker(new Vector2(3, 3));
            _jobManager.NewWorker(worker);
            _entities.Add(worker);
            worker = new Worker(new Vector2(3, 3));
            _jobManager.NewWorker(worker);
            _entities.Add(worker);*/
            base.Initialize();
        }

        protected override void LoadContent()
        {
            _spriteBatch = new SpriteBatch(GraphicsDevice);

            // TODO: use this.Content to load your game content here
        }

        protected override void Update(GameTime gameTime)
        {
            if (GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed || Keyboard.GetState().IsKeyDown(Keys.Escape))
                Exit();

            _jobManager.Update();
            _timer.Update(gameTime);
            foreach (IEntity entity in _entities)
                entity.Update(gameTime);

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);
            var rs = new RasterizerState();
            rs.CullMode = CullMode.None;
            GraphicsDevice.RasterizerState = rs;

            foreach (IEntity entity in _entities)
                entity.Draw(GraphicsDevice);

            base.Draw(gameTime);
        }
    }
}