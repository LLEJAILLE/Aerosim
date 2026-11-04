
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Windows.Media.Media3D;
using System.Collections.Generic;

namespace AeroSim.Geometry
{
    public static class MeshExtractor
    {
        public static SurfaceMesh Extract(Model3D model)
        {
            var triangles = new List<Triangle>();

            ExtractRecursive(
                model,
                Matrix3D.Identity,
                triangles
            );

            return new SurfaceMesh(triangles);
        }

        public static SurfaceMesh Extract(Model3D model,Matrix3D visualTransform)
        {
            var triangles = new List<Triangle>();

            ExtractRecursive(
                model,
                visualTransform,
                triangles
            );

            return new SurfaceMesh(triangles);
        }

        private static void ExtractRecursive(
            Model3D model,
            Matrix3D parentTransform,
            List<Triangle> triangles)
        {
            Matrix3D transform = model.Transform.Value;

            transform.Append(parentTransform);

            if (model is Model3DGroup group)
            {
                foreach (Model3D child in group.Children)
                {
                    ExtractRecursive(
                        child,
                        transform,
                        triangles
                    );
                }
            }
            else if (model is GeometryModel3D geometry)
            {
                if (geometry.Geometry is not MeshGeometry3D mesh)
                    return;

                for (int i = 0; i + 2 < mesh.TriangleIndices.Count; i += 3)
                {
                    int indexA = mesh.TriangleIndices[i];
                    int indexB = mesh.TriangleIndices[i + 1];
                    int indexC = mesh.TriangleIndices[i + 2];

                    Point3D a = transform.Transform(
                        mesh.Positions[indexA]
                    );

                    Point3D b = transform.Transform(
                        mesh.Positions[indexB]
                    );

                    Point3D c = transform.Transform(
                        mesh.Positions[indexC]
                    );

                    triangles.Add(new Triangle(
                        ToVector3(a),
                        ToVector3(b),
                        ToVector3(c)
                    ));
                }
            }
        }

        private static Vector3 ToVector3(Point3D point)
        {
            return new Vector3(
                (float)point.X,
                (float)point.Y,
                (float)point.Z
            );
        }
    }
}
