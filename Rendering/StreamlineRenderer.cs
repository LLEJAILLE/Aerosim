using System;
using System.Collections.Generic;
using System.Numerics;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using AeroSim.Simulation;

namespace AeroSim.Rendering
{
    public class StreamlineRenderer
    {
        private readonly HelixViewport3D _viewport;
        private readonly LinesVisual3D[] _visuals;

        private readonly Color[] _colors =
        {
            Color.FromRgb(22, 60, 187),   // fort ralentissement
            Color.FromRgb(22, 120, 242),  // ralentissement
            Color.FromRgb(0, 213, 246),   // vitesse normale
            Color.FromRgb(248, 197, 60),  // accélération
            Color.FromRgb(238, 73, 56)    // forte accélération
        };

        private bool _visible = true;

        public StreamlineRenderer(HelixViewport3D viewport)
        {
            _viewport = viewport;
            _visuals = new LinesVisual3D[_colors.Length];

            for (int i = 0; i < _colors.Length; i++)
            {
                _visuals[i] = new LinesVisual3D
                {
                    Color = _colors[i],
                    Thickness = 2.5
                };

                _viewport.Children.Add(_visuals[i]);
            }
        }

        public void UpdateVisuals(
            IReadOnlyList<Streamline> streamlines,
            Func<Vector3, Vector3> getVelocity,
            float freeStreamSpeed)
        {
            var groups = new Point3DCollection[_colors.Length];

            for (int i = 0; i < groups.Length; i++)
                groups[i] = new Point3DCollection();

            foreach (Streamline line in streamlines)
            {
                for (int i = 1; i < line.Points.Count; i++)
                {
                    Vector3 a = line.Points[i - 1];
                    Vector3 b = line.Points[i];

                    // On estime la couleur du segment à partir
                    // de sa vitesse au milieu du segment.
                    Vector3 mid = (a + b) * 0.5f;

                    float ratio = 1f;

                    if (freeStreamSpeed > 1e-6f)
                    {
                        Vector3 velocity = getVelocity(mid);
                        float speed = velocity.Length();

                        if (float.IsFinite(speed))
                            ratio = speed / freeStreamSpeed;
                    }

                    int groupIndex = GetColorIndex(ratio);

                    groups[groupIndex].Add(new Point3D(a.X, a.Y, a.Z));
                    groups[groupIndex].Add(new Point3D(b.X, b.Y, b.Z));
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