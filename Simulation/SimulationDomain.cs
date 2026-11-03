using System;
using System.Numerics;
using AeroSim.Geometry;

namespace AeroSim.Simulation
{
    /// <summary>
    /// Paramètres de dimensionnement du domaine. Les marges sont exprimées en multiples
    /// de la longueur caractéristique L de l'obstacle (plus grande dimension de sa boîte englobante).
    /// </summary>
    public sealed class DomainSettings
    {
        // Distance entre l'obstacle et les faces amont / aval (le long du vent).
        public float UpstreamMargin { get; init; } = 1.5f;
        public float DownstreamMargin { get; init; } = 2.0f;

        // Distance entre l'obstacle et les faces latérales.
        public float LateralMargin { get; init; } = 0.75f;

        // Élargissement de la zone d'émission autour de la section de l'obstacle.
        public float SpawnLateralMargin { get; init; } = 0.25f;

        // Taille de cellule visée (fraction de L), sans dépasser le budget de nœuds.
        public float TargetCellFraction { get; init; } = 1f / 16f;

        // Nombre approximatif maximal de nœuds du cache : agrandir le domaine à budget
        // constant agrandit les cellules et réduit donc la résolution spatiale.
        public int CacheNodeBudget { get; init; } = 150_000;
    }

    /// <summary>
    /// Domaine de simulation calculé à partir du maillage transformé et de la direction du vent.
    /// Les mêmes bornes servent au cache de vitesse et au système de particules.
    /// </summary>
    public sealed class SimulationDomain
    {
        public Vector3 Min { get; }
        public Vector3 Max { get; }

        public Vector3 ObstacleMin { get; }
        public Vector3 ObstacleMax { get; }

        // Rectangle d'émission (épaisseur nulle selon l'axe du vent), sur la face amont.
        public Vector3 SpawnMin { get; }
        public Vector3 SpawnMax { get; }

        // Axe dominant du vent (0 = X, 1 = Y, 2 = Z) et sens (+1 / -1).
        public int FlowAxis { get; }
        public int FlowSign { get; }

        public float CharacteristicLength { get; }

        public int Nx { get; }
        public int Ny { get; }
        public int Nz { get; }

        public Vector3 CellSize { get; }
        public float MinCellSize => MathF.Min(CellSize.X, MathF.Min(CellSize.Y, CellSize.Z));
        public long NodeCount => (long)Nx * Ny * Nz;

        public float FlowLength =>
            VectorAxis.Get(Max, FlowAxis) - VectorAxis.Get(Min, FlowAxis);

        public static SimulationDomain Create(
            SurfaceMesh mesh,
            Vector3 windVelocity,
            DomainSettings? settings = null)
        {
            return new SimulationDomain(mesh, windVelocity, settings ?? new DomainSettings());
        }

        private SimulationDomain(
            SurfaceMesh mesh,
            Vector3 wind,
            DomainSettings settings)
        {
            Vector3 obstacleMin = new(float.PositiveInfinity);
            Vector3 obstacleMax = new(float.NegativeInfinity);

            foreach (SurfacePanel panel in mesh.Panels)
            {
                obstacleMin = Vector3.Min(obstacleMin,
                    Vector3.Min(panel.A, Vector3.Min(panel.B, panel.C)));
                obstacleMax = Vector3.Max(obstacleMax,
                    Vector3.Max(panel.A, Vector3.Max(panel.B, panel.C)));
            }

            ObstacleMin = obstacleMin;
            ObstacleMax = obstacleMax;

            Vector3 size = obstacleMax - obstacleMin;
            float length = MathF.Max(size.X, MathF.Max(size.Y, size.Z));
            CharacteristicLength = length;

            // Vent nul ou invalide : repli sur +Y, direction historique du projet.
            Vector3 direction = wind.LengthSquared() > 1e-12f && float.IsFinite(wind.LengthSquared())
                ? Vector3.Normalize(wind)
                : Vector3.UnitY;

            FlowAxis = 0;

            for (int axis = 1; axis < 3; axis++)
            {
                if (MathF.Abs(VectorAxis.Get(direction, axis)) >
                    MathF.Abs(VectorAxis.Get(direction, FlowAxis)))
                    FlowAxis = axis;
            }

            FlowSign = VectorAxis.Get(direction, FlowAxis) >= 0 ? 1 : -1;

            // Marges par axe : latérales pour un axe perpendiculaire au vent, amont/aval pour
            // l'axe du vent, interpolées selon la composante de la direction sur cet axe.
            Vector3 min = obstacleMin;
            Vector3 max = obstacleMax;

            for (int axis = 0; axis < 3; axis++)
            {
                float component = VectorAxis.Get(direction, axis);
                float weight = MathF.Abs(component);

                float upstream = Lerp(settings.LateralMargin, settings.UpstreamMargin, weight) * length;
                float downstream = Lerp(settings.LateralMargin, settings.DownstreamMargin, weight) * length;

                float lowMargin = component >= 0 ? upstream : downstream;
                float highMargin = component >= 0 ? downstream : upstream;

                min = VectorAxis.With(min, axis, VectorAxis.Get(obstacleMin, axis) - lowMargin);
                max = VectorAxis.With(max, axis, VectorAxis.Get(obstacleMax, axis) + highMargin);
            }

            Min = min;
            Max = max;

            // Taille de cellule : fraction de L, bornée par le budget de nœuds.
            Vector3 extent = max - min;
            float volume = extent.X * extent.Y * extent.Z;
            float budgetCell = MathF.Cbrt(volume / Math.Max(settings.CacheNodeBudget, 8));
            float cell = MathF.Max(length * settings.TargetCellFraction, budgetCell);

            Nx = NodeCountFor(extent.X, cell);
            Ny = NodeCountFor(extent.Y, cell);
            Nz = NodeCountFor(extent.Z, cell);

            CellSize = new Vector3(
                extent.X / (Nx - 1),
                extent.Y / (Ny - 1),
                extent.Z / (Nz - 1));

            // Émission : face amont, section de l'obstacle élargie, bornée au domaine.
            Vector3 spawnMin = min;
            Vector3 spawnMax = max;
            float spawnMargin = settings.SpawnLateralMargin * length;

            for (int axis = 0; axis < 3; axis++)
            {
                if (axis == FlowAxis)
                    continue;

                spawnMin = VectorAxis.With(spawnMin, axis,
                    MathF.Max(VectorAxis.Get(min, axis), VectorAxis.Get(obstacleMin, axis) - spawnMargin));
                spawnMax = VectorAxis.With(spawnMax, axis,
                    MathF.Min(VectorAxis.Get(max, axis), VectorAxis.Get(obstacleMax, axis) + spawnMargin));
            }

            float inset = 1e-3f * length;
            float plane = FlowSign > 0
                ? VectorAxis.Get(min, FlowAxis) + inset
                : VectorAxis.Get(max, FlowAxis) - inset;

            SpawnMin = VectorAxis.With(spawnMin, FlowAxis, plane);
            SpawnMax = VectorAxis.With(spawnMax, FlowAxis, plane);
        }

        public bool Contains(Vector3 p)
        {
            return p.X >= Min.X && p.X <= Max.X &&
                   p.Y >= Min.Y && p.Y <= Max.Y &&
                   p.Z >= Min.Z && p.Z <= Max.Z;
        }

        public bool ObstacleInside =>
            ObstacleMin.X >= Min.X && ObstacleMin.Y >= Min.Y && ObstacleMin.Z >= Min.Z &&
            ObstacleMax.X <= Max.X && ObstacleMax.Y <= Max.Y && ObstacleMax.Z <= Max.Z;

        public override string ToString()
        {
            return $"x[{Min.X:F1}; {Max.X:F1}] y[{Min.Y:F1}; {Max.Y:F1}] z[{Min.Z:F1}; {Max.Z:F1}] • " +
                   $"{Nx}×{Ny}×{Nz} nœuds • cellule min {MinCellSize:F2}";
        }

        private static int NodeCountFor(float extent, float cell)
        {
            return Math.Max(2, (int)MathF.Ceiling(extent / cell) + 1);
        }

        private static float Lerp(float a, float b, float t) => a + (b - a) * t;
    }
}
