
using System;
using System.Numerics;
using AeroSim.Geometry;

namespace AeroSim.Physics.Solvers
{
    public static class PanelInfluence
    {
        public static Vector3 GetVelocity(
            SurfacePanel panel,
            Vector3 position)
        {
            Vector3 a = panel.A;
            Vector3 b = panel.B;
            Vector3 c = panel.C;

            // Trois points de quadrature du triangle
            Vector3 p1 = (4f * a + b + c) / 6f;
            Vector3 p2 = (a + 4f * b + c) / 6f;
            Vector3 p3 = (a + b + 4f * c) / 6f;

            float weight = panel.Area / (12f * MathF.PI);

            // Longueur caractéristique du panneau
            float panelLength = MathF.Sqrt(panel.Area);

            // Régularisation relative à la taille du panneau
            float epsilon = MathF.Max(
                1e-6f,
                panelLength * 0.01f
            );

            return weight * (
                Kernel(position - p1, epsilon) +
                Kernel(position - p2, epsilon) +
                Kernel(position - p3, epsilon)
            );
        }

        private static Vector3 Kernel(
            Vector3 r,
            float epsilon)
        {
            float distanceSquared = r.LengthSquared();

            // Éviter les valeurs non finies
            if (!float.IsFinite(distanceSquared))
            {
                throw new InvalidOperationException(
                    "Distance non finie dans PanelInfluence."
                );
            }

            // Noyau régularisé
            float epsilonSquared = epsilon * epsilon;

            float regularizedDistanceSquared =
                distanceSquared + epsilonSquared;

            float distance = MathF.Sqrt(
                regularizedDistanceSquared
            );

            return r /
                (regularizedDistanceSquared * distance);
        }
    }
}
