using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace AeroSim.Physics.FlowFields
{
    public class UniformFlowField : IFlowField
    {
        public Vector3 WindVelocity { get; set; }

        public Vector3 GetVelocity(Vector3 position, double time)
        {
            return WindVelocity;
        }
    }
}   