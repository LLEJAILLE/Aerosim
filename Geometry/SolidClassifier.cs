using System;
using System.Collections.Generic;
using System.Numerics;

namespace AeroSim.Geometry
{
    /// <summary>
    /// Test intérieur/extérieur d'un maillage triangulé.
    ///
    /// Méthode : nombre d'enroulement par lancer de rayons le long des trois axes,
    /// avec décompte signé des intersections (sortie +1, entrée -1) puis vote à la
    /// majorité. Le décompte signé reste correct quand des coques fermées se
    /// recouvrent (une simple parité ne le serait pas). Chaque axe dispose d'une
    /// grille 2D qui limite chaque requête aux triangles voisins du rayon.
    ///
    /// Le test n'est fiable que si le maillage est fermé et orienté vers l'extérieur.
    /// Sinon <see cref="IsReliable"/> vaut false et Contains retombe sur la boîte
    /// englobante, estimation volontairement prudente (exclut plus que le solide).
    /// </summary>
    public sealed class SolidClassifier
    {
        private const int MaxGridResolution = 96;

        private sealed class AxisGrid
        {
            public int Axis;
            public int AxisU;
            public int AxisV;
            public int CellsU;
            public int CellsV;
            public double OriginU;
            public double OriginV;
            public double InvCellU;
            public double InvCellV;
            public int[][] Cells = Array.Empty<int[]>();
        }

        // Triangle projeté perpendiculairement à un axe, sommets orientés positivement dans le plan projeté.
        private struct ProjectedTriangle
        {
            public double U0, V0, U1, V1, U2, V2;
            public double OriginAxis, OriginU, OriginV;
            public double NormalAxis, NormalU, NormalV;
            public int Sign;
            public bool Valid;
        }

        private readonly SurfaceMesh _mesh;
        private readonly AxisGrid[] _grids = new AxisGrid[3];
        private readonly ProjectedTriangle[][] _triangles = new ProjectedTriangle[3][];

        public Vector3 BoundsMin { get; }
        public Vector3 BoundsMax { get; }
        public bool IsReliable { get; }
        public string Reliability { get; }

        public SolidClassifier(SurfaceMesh mesh)
        {
            _mesh = mesh ?? throw new ArgumentNullException(nameof(mesh));

            Vector3 min = new(float.PositiveInfinity);
            Vector3 max = new(float.NegativeInfinity);

            foreach (SurfacePanel panel in mesh.Panels)
            {
                min = Vector3.Min(min, Vector3.Min(panel.A, Vector3.Min(panel.B, panel.C)));
                max = Vector3.Max(max, Vector3.Max(panel.A, Vector3.Max(panel.B, panel.C)));
            }

            BoundsMin = min;
            BoundsMax = max;

            IsReliable = AnalyzeReliability(out string note);
            Reliability = note;

            if (IsReliable)
            {
                for (int axis = 0; axis < 3; axis++)
                    BuildGrid(axis);
            }
        }

        /// <summary>Vrai si le point est à l'intérieur d'un solide du maillage.</summary>
        public bool Contains(Vector3 p)
        {
            if (p.X < BoundsMin.X || p.X > BoundsMax.X ||
                p.Y < BoundsMin.Y || p.Y > BoundsMax.Y ||
                p.Z < BoundsMin.Z || p.Z > BoundsMax.Z)
                return false;

            if (!IsReliable)
                return true;

            int votes = 0;

            for (int axis = 0; axis < 3; axis++)
            {
                if (Winding(axis, p) > 0)
                    votes++;
            }

            return votes >= 2;
        }

        /// <summary>Distance exacte du point au triangle le plus proche (O(N), usage diagnostic).</summary>
        public float DistanceToSurface(Vector3 p)
        {
            float best = float.PositiveInfinity;

            foreach (SurfacePanel panel in _mesh.Panels)
            {
                Vector3 q = ClosestPointOnTriangle(p, panel.A, panel.B, panel.C);
                float d = Vector3.DistanceSquared(p, q);

                if (d < best)
                    best = d;
            }

            return MathF.Sqrt(best);
        }

        private int Winding(int axis, Vector3 p)
        {
            AxisGrid grid = _grids[axis];
            ProjectedTriangle[] triangles = _triangles[axis];

            double pu = VectorAxis.Get(p, grid.AxisU);
            double pv = VectorAxis.Get(p, grid.AxisV);
            double pa = VectorAxis.Get(p, axis);

            int cu = (int)((pu - grid.OriginU) * grid.InvCellU);
            int cv = (int)((pv - grid.OriginV) * grid.InvCellV);

            if (cu < 0 || cv < 0 || cu >= grid.CellsU || cv >= grid.CellsV)
                return 0;

            int winding = 0;

            foreach (int index in grid.Cells[cv * grid.CellsU + cu])
            {
                ProjectedTriangle t = triangles[index];

                if (!t.Valid || !CoversPoint(t, pu, pv))
                    continue;

                double hit = t.OriginAxis -
                    (t.NormalU * (pu - t.OriginU) + t.NormalV * (pv - t.OriginV)) / t.NormalAxis;

                if (hit > pa)
                    winding += t.Sign;
            }

            return winding;
        }

        // Règle « haut-gauche » : un point exactement sur une arête partagée n'est compté qu'une fois.
        private static bool CoversPoint(in ProjectedTriangle t, double pu, double pv)
        {
            return EdgeTest(t.U0, t.V0, t.U1, t.V1, pu, pv) &&
                   EdgeTest(t.U1, t.V1, t.U2, t.V2, pu, pv) &&
                   EdgeTest(t.U2, t.V2, t.U0, t.V0, pu, pv);
        }

        private static bool EdgeTest(
            double u0, double v0, double u1, double v1, double pu, double pv)
        {
            double du = u1 - u0;
            double dv = v1 - v0;
            double e = du * (pv - v0) - dv * (pu - u0);

            if (e > 0)
                return true;

            if (e < 0)
                return false;

            return dv > 0 || (dv == 0 && du < 0);
        }

        private void BuildGrid(int axis)
        {
            int axisU = (axis + 1) % 3;
            int axisV = (axis + 2) % 3;
            int count = _mesh.Panels.Count;

            var triangles = new ProjectedTriangle[count];

            double minU = VectorAxis.Get(BoundsMin, axisU);
            double minV = VectorAxis.Get(BoundsMin, axisV);
            double maxU = VectorAxis.Get(BoundsMax, axisU);
            double maxV = VectorAxis.Get(BoundsMax, axisV);

            int resolution = Math.Clamp((int)(Math.Sqrt(count) * 1.5), 4, MaxGridResolution);

            double spanU = Math.Max(maxU - minU, 1e-9);
            double spanV = Math.Max(maxV - minV, 1e-9);

            var grid = new AxisGrid
            {
                Axis = axis,
                AxisU = axisU,
                AxisV = axisV,
                CellsU = resolution,
                CellsV = resolution,
                OriginU = minU,
                OriginV = minV,
                InvCellU = resolution / spanU,
                InvCellV = resolution / spanV
            };

            var buckets = new List<int>?[resolution * resolution];

            for (int i = 0; i < count; i++)
            {
                SurfacePanel panel = _mesh.Panels[i];

                double normalAxis = VectorAxis.Get(panel.Normal, axis);
                var t = new ProjectedTriangle();

                // Triangle presque parallèle au rayon : aucune intersection fiable.
                if (Math.Abs(normalAxis) > 1e-6)
                {
                    double u0 = VectorAxis.Get(panel.A, axisU), v0 = VectorAxis.Get(panel.A, axisV);
                    double u1 = VectorAxis.Get(panel.B, axisU), v1 = VectorAxis.Get(panel.B, axisV);
                    double u2 = VectorAxis.Get(panel.C, axisU), v2 = VectorAxis.Get(panel.C, axisV);

                    double area2 = (u1 - u0) * (v2 - v0) - (v1 - v0) * (u2 - u0);

                    if (area2 < 0)
                    {
                        (u1, u2) = (u2, u1);
                        (v1, v2) = (v2, v1);
                    }

                    t.U0 = u0; t.V0 = v0;
                    t.U1 = u1; t.V1 = v1;
                    t.U2 = u2; t.V2 = v2;
                    t.OriginAxis = VectorAxis.Get(panel.A, axis);
                    t.OriginU = VectorAxis.Get(panel.A, axisU);
                    t.OriginV = VectorAxis.Get(panel.A, axisV);
                    t.NormalAxis = normalAxis;
                    t.NormalU = VectorAxis.Get(panel.Normal, axisU);
                    t.NormalV = VectorAxis.Get(panel.Normal, axisV);
                    t.Sign = normalAxis > 0 ? 1 : -1;
                    t.Valid = Math.Abs(area2) > 0;

                    if (t.Valid)
                    {
                        int cu0 = ClampCell((Math.Min(u0, Math.Min(u1, u2)) - minU) * grid.InvCellU, resolution);
                        int cu1 = ClampCell((Math.Max(u0, Math.Max(u1, u2)) - minU) * grid.InvCellU, resolution);
                        int cv0 = ClampCell((Math.Min(v0, Math.Min(v1, v2)) - minV) * grid.InvCellV, resolution);
                        int cv1 = ClampCell((Math.Max(v0, Math.Max(v1, v2)) - minV) * grid.InvCellV, resolution);

                        for (int cv = cv0; cv <= cv1; cv++)
                        {
                            for (int cu = cu0; cu <= cu1; cu++)
                            {
                                int cell = cv * resolution + cu;
                                (buckets[cell] ??= new List<int>()).Add(i);
                            }
                        }
                    }
                }

                triangles[i] = t;
            }

            grid.Cells = new int[buckets.Length][];

            for (int i = 0; i < buckets.Length; i++)
                grid.Cells[i] = buckets[i]?.ToArray() ?? Array.Empty<int>();

            _grids[axis] = grid;
            _triangles[axis] = triangles;
        }

        private static int ClampCell(double value, int resolution)
        {
            return Math.Clamp((int)Math.Floor(value), 0, resolution - 1);
        }

        // Maillage fermé (chaque arête orientée a exactement son inverse), orienté vers l'extérieur.
        private bool AnalyzeReliability(out string note)
        {
            Vector3 size = BoundsMax - BoundsMin;
            double length = Math.Max(size.X, Math.Max(size.Y, size.Z));
            double tolerance = Math.Max(length * 1e-5, 1e-9);

            var vertexIds = new Dictionary<(long, long, long), int>();
            var directedEdges = new Dictionary<(int, int), int>();
            double signedVolume6 = 0;
            int collapsed = 0;

            int Weld(Vector3 v)
            {
                var key = (
                    (long)Math.Round(v.X / tolerance),
                    (long)Math.Round(v.Y / tolerance),
                    (long)Math.Round(v.Z / tolerance));

                if (!vertexIds.TryGetValue(key, out int id))
                {
                    id = vertexIds.Count;
                    vertexIds[key] = id;
                }

                return id;
            }

            foreach (SurfacePanel panel in _mesh.Panels)
            {
                int a = Weld(panel.A);
                int b = Weld(panel.B);
                int c = Weld(panel.C);

                signedVolume6 += Vector3.Dot(panel.A, Vector3.Cross(panel.B, panel.C));

                if (a == b || b == c || c == a)
                {
                    collapsed++;
                    continue;
                }

                foreach (var edge in new[] { (a, b), (b, c), (c, a) })
                {
                    directedEdges.TryGetValue(edge, out int n);
                    directedEdges[edge] = n + 1;
                }
            }

            int open = 0;
            int inconsistent = 0;

            foreach (var (edge, count) in directedEdges)
            {
                directedEdges.TryGetValue((edge.Item2, edge.Item1), out int reverse);

                if (count != 1 || reverse != 1)
                {
                    if (reverse == 0)
                        open++;
                    else
                        inconsistent++;
                }
            }

            double volume = signedVolume6 / 6.0;

            if (collapsed > 0 || open > 0 || inconsistent > 0)
            {
                note = $"maillage non fermé ou incohérent ({open} arêtes libres, " +
                       $"{inconsistent} arêtes non appariées, {collapsed} triangles dégénérés après soudure) : " +
                       "test intérieur/extérieur remplacé par la boîte englobante";
                return false;
            }

            if (volume <= 1e-9 * length * length * length)
            {
                note = "volume signé ≤ 0 : normales orientées vers l'intérieur ou volume nul ; " +
                       "test intérieur/extérieur remplacé par la boîte englobante";
                return false;
            }

            note = "maillage fermé et orienté : test par nombre d'enroulement";
            return true;
        }

        // Point le plus proche sur un triangle (Ericson, Real-Time Collision Detection).
        private static Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 ap = p - a;
            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);

            if (d1 <= 0 && d2 <= 0)
                return a;

            Vector3 bp = p - b;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);

            if (d3 >= 0 && d4 <= d3)
                return b;

            float vc = d1 * d4 - d3 * d2;

            if (vc <= 0 && d1 >= 0 && d3 <= 0)
                return a + ab * (d1 / (d1 - d3));

            Vector3 cp = p - c;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);

            if (d6 >= 0 && d5 <= d6)
                return c;

            float vb = d5 * d2 - d1 * d6;

            if (vb <= 0 && d2 >= 0 && d6 <= 0)
                return a + ac * (d2 / (d2 - d6));

            float va = d3 * d6 - d5 * d4;

            if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
                return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));

            float denom = 1f / (va + vb + vc);
            return a + ab * (vb * denom) + ac * (vc * denom);
        }
    }
}
