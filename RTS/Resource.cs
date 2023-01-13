using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RTS
{
    internal class Resource
    {
        public string Name { get; }
        public int Id { get; }
        public IDrawAt Icon;
        public Resource(string Name, IDrawAt Icon, int Id)
        {
            this.Name = Name;
            this.Id = Id;
            this.Icon = Icon;
        }
    }
    internal class ResourceStack
    {
        public Resource type { get; }
        public int qty { get; }
        public ResourceStack(Resource type_, int qty_)
        {
            type = type_;
            qty = qty_;
        }
    }
    internal class ResourceManager
    {
        List<Resource> resources;
        public ResourceManager()
        {
            resources = new List<Resource>();
        }
        public Resource NewResource(string name, IDrawAt Icon)
        {
            Resource r = new Resource(name, Icon, resources.Count);
            resources.Add(r);
            return r;
        }
        public Resource GetResource(int id) { return resources[id]; }
    }
}
