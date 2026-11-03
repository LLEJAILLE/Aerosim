
using System;
using System.Numerics;
using AeroSim.Physics.FlowFields;

namespace AeroSim.Simulation
{
    public static class StreamlineGenerator
    {
        public static Streamline Generate(
            IFlowField flow,
            Vector3 start,
            float stepSize = 0.25f,
            int maxSteps = 500)
        {
            var streamline = new Streamline();
            Vector3 position = start;

            streamline.Points.Add(position);

            for (int i = 0; i < maxSteps; i++)
            {
                Vector3 velocity = flow.GetVelocity(position, 0);

                if (velocity.LengthSquared() < 1e-8f)
                    break;

                Vector3 displacement = RK4(
                    flow,
                    position,
                    stepSize
                );

                if (!IsFinite(displacement))
                    break;

                position += displacement;
                streamline.Points.Add(position);
            }

            return streamline;
        }

        private static Vector3 Direction(
            IFlowField flow,
            Vector3 position)
        {
            Vector3 velocity = flow.GetVelocity(position, 0);

            if (!IsFinite(velocity) ||
                velocity.LengthSquared() < 1e-8f)
                return Vector3.Zero;

            return Vector3.Normalize(velocity);
        }

        private static Vector3 RK4(
            IFlowField flow,
            Vector3 position,
            float h)
        {
            Vector3 k1 = Direction(flow, position);
            Vector3 k2 = Direction(flow, position + h * k1 / 2f);
            Vector3 k3 = Direction(flow, position + h * k2 / 2f);
            Vector3 k4 = Direction(flow, position + h * k3);

            return h / 6f * (k1 + 2f * k2 + 2f * k3 + k4);
        }

        private static bool IsFinite(Vector3 v)
        {
            return float.IsFinite(v.X) &&
                   float.IsFinite(v.Y) &&
                   float.IsFinite(v.Z);
        }
    }
}
