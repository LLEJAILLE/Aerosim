using System;
using System.Numerics;

namespace AeroSim.Geometry
{
    public class SurfacePanel
    {
        public Vector3 A { get; }
        public Vector3 B { get; }
        public Vector3 C { get; }

        public Vector3 Center { get; }
        public Vector3 Normal { get; }
        public float Area { get; }

        public SurfacePanel(Vector3 a, Vector3 b, Vector3 c)
        {
            if (!IsFinite(a) || !IsFinite(b) || !IsFinite(c))
                throw new ArgumentException(
                    "Triangle invalide : coordonnées non finies."
                );

            Vector3 cross = Vector3.Cross(b - a, c - a);
            float magnitude = cross.Length();

            if (!float.IsFinite(magnitude) || magnitude < 1e-8f)
                throw new ArgumentException(
                    "Triangle dégénéré : surface nulle ou trop petite."
                );

            A = a;
            B = b;
            C = c;

            Center = (a + b + c) / 3f;
            Normal = cross / magnitude;
            Area = magnitude * 0.5f;
        }

        private static bool IsFinite(Vector3 v)
        {
            return float.IsFinite(v.X)
                && float.IsFinite(v.Y)
                && float.IsFinite(v.Z);
        }
    }
}