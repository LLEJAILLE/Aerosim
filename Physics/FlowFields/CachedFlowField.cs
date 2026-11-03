
using System;
using System.Numerics;
using AeroSim.Physics;

namespace AeroSim.Physics.FlowFields
{
    public sealed class CachedFlowField : IFlowField
    {
        private readonly Vector3[,,] _velocities;
        private readonly Vector3 _min;
        private readonly Vector3 _max;
        private readonly Vector3 _step;
        private readonly Vector3 _outsideVelocity;

        private readonly int _nx, _ny, _nz;

        public CachedFlowField(
            IFlowField source,
            Vector3 min,
            Vector3 max,
            int nx,
            int ny,
            int nz,
            Vector3 outsideVelocity)
        {
            if (nx < 2 || ny < 2 || nz < 2)
                throw new ArgumentException(
                    "Chaque dimension doit contenir au moins 2 points.");

            if (min.X >= max.X || min.Y >= max.Y ||
                min.Z >= max.Z)
                throw new ArgumentException("Bornes invalides.");

            _min = min;
            _max = max;
            _nx = nx;
            _ny = ny;
            _nz = nz;
            _outsideVelocity = outsideVelocity;

            _step = new Vector3(
                (max.X - min.X) / (nx - 1),
                (max.Y - min.Y) / (ny - 1),
                (max.Z - min.Z) / (nz - 1)
            );

            _velocities = new Vector3[nx, ny, nz];

            // Précalcul du champ stationnaire
            for (int x = 0; x < nx; x++)
                for (int y = 0; y < ny; y++)
                    for (int z = 0; z < nz; z++)
                    {
                        Vector3 position = new(
                            min.X + x * _step.X,
                            min.Y + y * _step.Y,
                            min.Z + z * _step.Z
                        );

                        _velocities[x, y, z] =
                            source.GetVelocity(position, 0);
                    }
        }

        public Vector3 GetVelocity(Vector3 position, double time)
        {
            if (position.X < _min.X || position.X > _max.X ||
                position.Y < _min.Y || position.Y > _max.Y ||
                position.Z < _min.Z || position.Z > _max.Z)
            {
                return _outsideVelocity;
            }

            Vector3 p = new(
                (position.X - _min.X) / _step.X,
                (position.Y - _min.Y) / _step.Y,
                (position.Z - _min.Z) / _step.Z
            );

            int x = Math.Clamp((int)p.X, 0, _nx - 2);
            int y = Math.Clamp((int)p.Y, 0, _ny - 2);
            int z = Math.Clamp((int)p.Z, 0, _nz - 2);

            float tx = Math.Clamp(p.X - x, 0f, 1f);
            float ty = Math.Clamp(p.Y - y, 0f, 1f);
            float tz = Math.Clamp(p.Z - z, 0f, 1f);

            // Interpolation sur les huit coins
            Vector3 c000 = _velocities[x, y, z];
            Vector3 c100 = _velocities[x + 1, y, z];
            Vector3 c010 = _velocities[x, y + 1, z];
            Vector3 c110 = _velocities[x + 1, y + 1, z];

            Vector3 c001 = _velocities[x, y, z + 1];
            Vector3 c101 = _velocities[x + 1, y, z + 1];
            Vector3 c011 = _velocities[x, y + 1, z + 1];
            Vector3 c111 = _velocities[x + 1, y + 1, z + 1];

            Vector3 a = Vector3.Lerp(c000, c100, tx);
            Vector3 b = Vector3.Lerp(c010, c110, tx);
            Vector3 c = Vector3.Lerp(c001, c101, tx);
            Vector3 d = Vector3.Lerp(c011, c111, tx);

            Vector3 bottom = Vector3.Lerp(a, b, ty);
            Vector3 top = Vector3.Lerp(c, d, ty);

            return Vector3.Lerp(bottom, top, tz);
        }
    }
}
