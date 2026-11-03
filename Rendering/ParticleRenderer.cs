
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using AeroSim.Simulation;
using AeroSim.Physics;

namespace AeroSim.Rendering
{
    public class ParticleRenderer
    {
        private readonly HelixViewport3D _viewport;
        private readonly PointsVisual3D[] _visuals;

        private readonly Color[] _colors =
        {
            Color.FromRgb(22, 60, 187),   // Fort ralentissement
            Color.FromRgb(22, 120, 242),  // Ralentissement
            Color.FromRgb(0, 213, 246),   // Vitesse normale
            Color.FromRgb(248, 197, 60),  // Accélération
            Color.FromRgb(238, 73, 56)    // Forte accélération
        };

        private bool _visible = true;

        public ParticleRenderer(HelixViewport3D viewport)
        {
            _viewport = viewport;
            _visuals = new PointsVisual3D[_colors.Length];

            for (int i = 0; i < _colors.Length; i++)
            {
                _visuals[i] = new PointsVisual3D
                {
                    Color = _colors[i],
                    Size = 2.0
                };

                _viewport.Children.Add(_visuals[i]);
            }
        }

        public void UpdateVisuals(
            IReadOnlyList<AirParticle> particles,
            Func<Vector3, Vector3> getVelocity,
            float freeStreamSpeed)
        {
            var groups = new Point3DCollection[_colors.Length];

            for (int i = 0; i < groups.Length; i++)
                groups[i] = new Point3DCollection();

            // Évite une division par zéro.
            if (freeStreamSpeed <= 1e-6f)
            {
                foreach (var particle in particles)
                {
                    var p = particle.Position;

                    groups[2].Add(new Point3D(
                        p.X, p.Y, p.Z
                    ));
                }
            }
            else
            {
                foreach (var particle in particles)
                {
                    Vector3 position = particle.Position;

                    // Vitesse physique au point de la particule.
                    Vector3 velocity = getVelocity(position);

                    float speed = velocity.Length();

                    // Ratio relatif au vent incident.
                    float ratio = speed / freeStreamSpeed;

                    int groupIndex = GetColorIndex(ratio);

                    groups[groupIndex].Add(new Point3D(
                        position.X,
                        position.Y,
                        position.Z
                    ));
                }
            }

            for (int i = 0; i < _visuals.Length; i++)
                _visuals[i].Points = groups[i];
        }

        private static int GetColorIndex(float ratio)
        {
            if (!float.IsFinite(ratio))
                return 2;

            if (ratio < 0.7f)
                return 0;

            if (ratio < 0.95f)
                return 1;

            if (ratio < 1.05f)
                return 2;

            if (ratio < 1.25f)
                return 3;

            return 4;
        }

        public void SetVisible(bool visible)
        {
            if (_visible == visible)
                return;

            _visible = visible;

            foreach (var visual in _visuals)
            {
                if (visible)
                {
                    if (!_viewport.Children.Contains(visual))
                        _viewport.Children.Add(visual);
                }
                else
                {
                    _viewport.Children.Remove(visual);
                }
            }
        }
    }
}
