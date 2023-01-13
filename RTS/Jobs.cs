using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RTS
{
    internal class ResourceSink
    {
        JobManager jobManager;
        public Resource Resource { get; }
        public IPosition Position { get; }
        public ResourceSink(JobManager JobManager, Resource resource, IPosition position)
        {
            jobManager = JobManager;
            Resource = resource;
            Position = position;
        }
        public int Have { get; private set; }
        public int Want { get; private set; }
        public event Action OnFill;
        public void Request(int qty)
        {
            Debug.Assert(qty >= 0);
            Want += qty;
            for (int i = 0; i < qty; i++)
            {
                jobManager.NewJob(this);
            }
        }
        public void Take(int qty)
        {
            Debug.Assert(qty >= 0 && qty <= Have);
            Have -= qty;
            Want -= qty;
        }
        public void Add(int qty)
        {
            Have++;
            jobManager.CallBack(OnFill);
        }
    }

    internal class ResourceSource
    {
        JobManager jobManager;
        public Resource Resource { get; }
        public IPosition Position { get; }
        public List<Job> Claims { get; }
        public ResourceSource(JobManager jobManager, Resource resource, IPosition position)
        {
            this.jobManager = jobManager;
            Resource = resource;
            Claims = new List<Job>();
            Position = position;
        }
        public event Action OnTake;
        public int Have { get; private set; }
        public void Add(int qty)
        {
            Have += qty;
            jobManager.ProcessSource(this);
        }
        public void Take(Job j)
        {
            Claims.Remove(j);
            Have--;
            jobManager.CallBack(OnTake);
        }
    }
    public enum JobStateT
    {
        Waiting,
        GoToSource,
        AtSource,
        GoToSink,
        AtSink,
        Moribund,
    }
    internal class Job
    {
        JobManager jobManager;
        public ResourceSource Source;
        public ResourceSink Sink;
        public JobWorker Worker;
        public JobStateT State = JobStateT.Waiting;
        public Job(JobManager jobManager, ResourceSink sink)
        {
            this.jobManager = jobManager;
            this.Sink = sink;
        }
    }
    internal class JobWorker
    {
        JobManager jobManager;
        public Job Job;
        public Worker Worker;
        
        public JobWorker(JobManager jobManager, Worker worker)
        {
            this.jobManager = jobManager;
            this.Worker = worker;
        }
    }
    
    internal class JobManager
    {
        Dictionary<int, List<ResourceSink>> sinks;
        Dictionary<int, List<ResourceSource>> sources;
        List<Job> jobs;
        List<JobWorker> jobWorkers;
        Queue<Action> callBacks;
        public JobManager()
        {
            sinks = new Dictionary<int, List<ResourceSink>>();
            sources = new Dictionary<int, List<ResourceSource>>();
            jobs = new List<Job>();
            jobWorkers = new List<JobWorker>();
            callBacks = new Queue<Action>();
        }
        public ResourceSource NewSource(Resource resource, IPosition Position)
        {
            ResourceSource source = new ResourceSource(this, resource, Position);
            if (!sources.TryGetValue(resource.Id, out List<ResourceSource> l))
            {
                l = new List<ResourceSource>();
                sources.Add(resource.Id, l);
            }
            l.Add(source);
            return source;
        }
        public ResourceSink NewSink(Resource resource, IPosition Position)
        {
            ResourceSink sink = new ResourceSink(this, resource, Position);
            if (!sinks.TryGetValue(resource.Id, out List<ResourceSink> l))
            {
                l = new List<ResourceSink>();
                sinks.Add(resource.Id, l);
            }
            l.Add(sink);
            return sink;
        }
        public void NewWorker(Worker worker)
        {
            jobWorkers.Add(new JobWorker(this, worker));
        }
        public void NewJob(ResourceSink sink)
        {
            jobs.Add(new Job(this, sink));
        }
        public void ProcessSource(ResourceSource source)
        {

        }
        public void CallBack(Action action)
        {
            if(action != null)
                callBacks.Enqueue(action);
        }
        ResourceSource findSource(int id)
        {
            if (sources[id] == null)
                return null;
            foreach(ResourceSource source in sources[id])
            {
                if(source.Claims.Count < source.Have)
                {
                    return source;
                }
            }
            return null;
        }
        JobWorker findWorker()
        {
            foreach(JobWorker w in jobWorkers)
            {
                if(w.Job == null)
                {
                    return w;
                }
            }
            return null;
        }
        void jobUpdate(Job j)
        {
            int res = j.Sink.Resource.Id;
            switch(j.State)
            {
                case JobStateT.Waiting:
                    var src = findSource(res);
                    var worker = findWorker();
                    if(src != null && worker != null)
                    {
                        j.Worker = worker;
                        j.Source = src;
                        worker.Job = j;
                        src.Claims.Add(j);
                        j.State = JobStateT.GoToSource;
                        worker.Worker.Walker.GoTo(j.Source.Position.Position,
                            () => j.State = JobStateT.AtSource,
                            null);
                    }
                    break;
                case JobStateT.GoToSource:
                    break;
                case JobStateT.AtSource:
                    j.Source.Take(j);
                    j.Worker.Worker.Carrying = j.Sink.Resource;
                    j.State = JobStateT.GoToSink;
                    j.Worker.Worker.Walker.GoTo(j.Sink.Position.Position,
                        () => j.State = JobStateT.AtSink,
                        null);
                    break;
                case JobStateT.GoToSink:
                    break;
                case JobStateT.AtSink:
                    j.Worker.Worker.Carrying = null;
                    j.Sink.Add(1);
                    j.State = JobStateT.Moribund;
                    j.Worker.Job = null;
                    j.Source = null;
                    j.Worker = null;
                    break;
                case JobStateT.Moribund:
                    break;
            }
        }

        public void Update()
        {
            foreach(Job j in jobs)
            {
                jobUpdate(j);
            }
            jobs.RemoveAll(j => j.State == JobStateT.Moribund);
            while (callBacks.TryDequeue(out Action action))
                action();
        }
    }
}
