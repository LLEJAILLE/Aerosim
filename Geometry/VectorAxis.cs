using System.Numerics;

namespace AeroSim.Geometry
{
    public static class VectorAxis
    {
        public static float Get(Vector3 v, int axis)
        {
            return axis switch
            {
                0 => v.X,
                1 => v.Y,
                _ => v.Z
            };
        }

        public static Vector3 With(Vector3 v, int axis, float value)
        {
            return axis switch
            {
                0 => new Vector3(value, v.Y, v.Z),
                1 => new Vector3(v.X, value, v.Z),
                _ => new Vector3(v.X, v.Y, value)
            };
        }
    }
}
