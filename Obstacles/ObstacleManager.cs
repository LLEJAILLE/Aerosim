
using System;
using System.Diagnostics;
using System.IO;
using System.Security.Cryptography.Xml;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using AeroSim.Geometry;
using HelixToolkit.Wpf;

namespace AeroSim.Obstacles
{
    public class ObstacleManager
    {
        private readonly HelixViewport3D _viewport;
        private ModelVisual3D? _currentObstacle;

        public SurfaceMesh? CurrentMesh { get; private set; }

        public ObstacleManager(HelixViewport3D viewport)
        {
            _viewport = viewport;
        }

        public SurfaceMesh Load(string type)
        {
            ModelVisual3D obstacle = type switch
            {
                "Cube" => new BoxVisual3D
                {
                    Center = new Point3D(0, 0, 5),
                    Width = 10,
                    Height = 10,
                    Length = 10,
                    Fill = Brushes.Orange
                },

                "Sphere" => new SphereVisual3D
                {
                    Center = new Point3D(0, 0, 5),
                    Radius = 5,
                    ThetaDiv = 8,
                    PhiDiv = 6,
                    Fill = Brushes.Orange
                },

                "low_poly_airplane" => LoadAirplane(),

                _ => throw new ArgumentException(
                    $"Obstacle inconnu : {type}"
                )
            };

            if (obstacle.Content == null)
                throw new InvalidOperationException(
                    "L'obstacle ne possède pas de géométrie."
                );

            SurfaceMesh mesh = MeshExtractor.Extract(
                obstacle.Content,
                obstacle.Transform.Value
            );

            Debug.WriteLine(
                $"[ObstacleManager] {type} : " +
                $"{mesh.TriangleCount} triangles"
            );

            if (_currentObstacle != null)
                _viewport.Children.Remove(_currentObstacle);

            _currentObstacle = obstacle;
            CurrentMesh = mesh;

            _viewport.Children.Add(obstacle);

            return mesh;
        }


        private static ModelVisual3D LoadAirplane()
        {
            string path = Path.Combine(
                AppContext.BaseDirectory,
                "Assets",
                "Models",
                "low_poly_airplane.obj"
            );

            if (!File.Exists(path))
            {
                throw new FileNotFoundException(
                    "Modèle low_poly_airplane introuvable.",
                    path
                );
            }

            var importer = new ModelImporter();

            Model3DGroup model = importer.Load(path)
                ?? throw new InvalidOperationException(
                    "Impossible de charger le modèle low_poly_airplane."
                );

            const double targetSize = 50.0;

            var rotation = new Transform3DGroup();

            rotation.Children.Add(
                new RotateTransform3D(
                    new AxisAngleRotation3D(
                        new Vector3D(1, 0, 0),
                        90
                    )
                )
            );

            rotation.Children.Add(
                new RotateTransform3D(
                    new AxisAngleRotation3D(
                        new Vector3D(0, 0, 1),
                        90
                    )
                )
            );

            Rect3D bounds = rotation.TransformBounds(model.Bounds);

            double maxDimension = Math.Max(
                bounds.SizeX,
                Math.Max(bounds.SizeY, bounds.SizeZ)
            );

            if (maxDimension <= 0 ||
                double.IsNaN(maxDimension) ||
                double.IsInfinity(maxDimension))
            {
                throw new InvalidOperationException(
                    "Dimensions invalides pour le modèle."
                );
            }

            double scale = targetSize / maxDimension;

            double centerX = bounds.X + bounds.SizeX / 2.0;
            double centerY = bounds.Y + bounds.SizeY / 2.0;
            double centerZ = bounds.Z + bounds.SizeZ / 2.0;

            var transforms = new Transform3DGroup();

            transforms.Children.Add(rotation);

            transforms.Children.Add(
                new ScaleTransform3D(
                    scale,
                    scale,
                    scale
                )
            );

            const double airplaneHeight = 10.0;

            transforms.Children.Add(
                new TranslateTransform3D(
                    -centerX * scale,
                    -centerY * scale,
                    airplaneHeight - centerZ * scale
                )
            );

            Debug.WriteLine(
                $"[Airplane] Dimensions : " +
                $"{bounds.SizeX:F2} x " +
                $"{bounds.SizeY:F2} x " +
                $"{bounds.SizeZ:F2}"
            );

            Debug.WriteLine(
                $"[Airplane] Target size : {targetSize}"
            );

            Debug.WriteLine(
                $"[Airplane] Scale factor : {scale:F6}"
            );

            return new ModelVisual3D
            {
                Content = model,
                Transform = transforms
            };
        }
    }
}
