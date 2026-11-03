using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

namespace AeroSim.Geometry
{
    public class SurfaceMesh
    {
        public IReadOnlyList<Triangle> Triangles { get; }
        public IReadOnlyList<SurfacePanel> Panels { get; }

        public int TriangleCount => Triangles.Count;

        public float TotalArea =>
            Panels.Sum(panel => panel.Area);

        public SurfaceMesh(IEnumerable<Triangle> triangles)
        {
            var triangleList = triangles.ToList();

            if (triangleList.Count == 0)
                throw new ArgumentException(
                    "Le maillage ne contient aucun triangle."
                );

            Triangles = triangleList.AsReadOnly();

            var panels = new List<SurfacePanel>();

            foreach (var triangle in triangleList)
            {
                panels.Add(new SurfacePanel(
                    triangle.A,
                    triangle.B,
                    triangle.C
                ));
            }

            Panels = panels.AsReadOnly();
        }
    }
}