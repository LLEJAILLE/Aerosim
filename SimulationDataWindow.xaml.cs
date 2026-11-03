using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Numerics;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace AeroSim
{
    public partial class SimulationDataWindow : Window
    {
        private readonly List<Sample> _history = new();
        private const int MaxHistory = 120;

        private readonly record struct Sample(
            double Time, double Mean, double Maximum,
            double Deviation, double Vertical);

        public SimulationDataWindow()
        {
            InitializeComponent();
        }

        public void SetScenario(
            string obstacle, int panels, float area, float windSpeed,
            long solverMs, long cacheMs,
            double normalRms, double normalMax, int normalSamples,
            bool active,
            string domainSummary,
            string solidSummary,
            int particlesInside)
        {
            ObstacleText.Text = obstacle;
            PanelsText.Text = panels.ToString("N0", CultureInfo.CurrentCulture);
            AreaText.Text = area.ToString("F1", CultureInfo.CurrentCulture);
            WindText.Text = $"{windSpeed:F2} m/s";
            SolverText.Text = $"{solverMs} ms";
            CacheText.Text = $"{cacheMs} ms";
            RmsText.Text = double.IsFinite(normalRms) ? $"{normalRms:F4} m/s" : "—";
            NormalMaxText.Text = double.IsFinite(normalMax) ? $"{normalMax:F4} m/s" : "—";
            DomainText.Text = domainSummary;
            SolidText.Text = solidSummary;
            InsideText.Text = particlesInside.ToString("N0", CultureInfo.CurrentCulture);

            StatusText.Text = active
                ? $"Simulation active • {normalSamples} panneaux contrôlés pour le résidu de collocation"
                : "Pas de calcul actif (géométrie trop détaillée ou champ indisponible)";
        }

        public void AddSample(
            double elapsed, double mean, double maximum,
            double deviation, double vertical, int sampleCount,
            long outOfBoundsRecycles,
            long lifetimeRecycles,
            long stagnationRecycles,
            long insideRecycles,
            double averageSubSteps,
            int maxSubSteps,
            int probeCount,
            double probeErrorRms,
            double probeErrorMax,
            double stationarityDriftMax,
            Vector3 worstProbePosition,
            double worstProbeDirectSpeed,
            double worstProbeCachedSpeed,
            Vector3 extremePosition,
            double extremeCachedSpeed,
            double extremeDirectSpeed)
        {
            _history.Add(new Sample(elapsed, mean, maximum, deviation, vertical));

            if (_history.Count > MaxHistory)
                _history.RemoveAt(0);

            RecycleText.Text =
                $"Sortie domaine : {outOfBoundsRecycles:N0}\n" +
                $"Durée de vie : {lifetimeRecycles:N0}\n" +
                $"Blocage : {stagnationRecycles:N0}\n" +
                $"Dans solide : {insideRecycles:N0}";

            SubStepText.Text =
                $"Sous-pas moyen : {averageSubSteps:F2}\n" +
                $"Sous-pas max observé : {maxSubSteps}";

            ProbeText.Text =
                $"Sondes : {probeCount}\n" +
                $"Erreur RMS : {probeErrorRms:F4} m/s\n" +
                $"Erreur max : {probeErrorMax:F4} m/s\n" +
                $"Pire sonde : {FormatVector(worstProbePosition)}\n" +
                $"Direct={worstProbeDirectSpeed:F2} m/s • Cache={worstProbeCachedSpeed:F2} m/s";

            StationaryText.Text =
                $"Dérive max des sondes directes : {stationarityDriftMax:E3} m/s";

            ExtremeText.Text =
                $"Position : {FormatVector(extremePosition)}\n" +
                $"|V| cache : {extremeCachedSpeed:F2} m/s\n" +
                $"|V| direct : {extremeDirectSpeed:F2} m/s";

            StatusText.Text = $"Échantillon particules : {sampleCount} • " +
                $"{_history.Count} points • t = {elapsed:F1} s";

            RedrawCharts();
        }

        public void ClearHistory()
        {
            _history.Clear();
            RedrawCharts();
        }

        private static string FormatVector(Vector3 v)
        {
            return $"({v.X:F1}, {v.Y:F1}, {v.Z:F1})";
        }

        private void ClearHistory_Click(object sender, RoutedEventArgs e)
        {
            ClearHistory();
        }

        private void Chart_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            RedrawCharts();
        }

        private void RedrawCharts()
        {
            if (MeanChart == null) return;

            DrawChart(MeanChart, s => s.Mean, Colors.Cyan, "m/s");
            DrawChart(MaxChart, s => s.Maximum, Colors.Orange, "m/s");
            DrawChart(DeviationChart, s => s.Deviation, Colors.MediumPurple, "%");
            DrawChart(VerticalChart, s => s.Vertical, Colors.LightGreen, "m/s");
        }

        private void DrawChart(
            Canvas canvas, Func<Sample, double> selector,
            Color lineColor, string unit)
        {
            canvas.Children.Clear();

            double width = canvas.ActualWidth;
            double height = canvas.ActualHeight;
            if (width < 70 || height < 70)
                return;

            const double left = 52;
            const double right = 12;
            const double top = 12;
            const double bottom = 27;
            double plotWidth = width - left - right;
            double plotHeight = height - top - bottom;

            if (_history.Count == 0)
            {
                AddLabel(canvas, "Aucune mesure", 20, height / 2 - 8,
                    Brushes.LightGray, 12);
                return;
            }

            double min = _history.Min(selector);
            double max = _history.Max(selector);
            double amplitude = max - min;
            if (amplitude < 1e-8)
            {
                double padding = Math.Max(0.1, Math.Abs(max) * 0.1);
                min -= padding;
                max += padding;
            }
            else
            {
                double padding = amplitude * 0.12;
                min -= padding;
                max += padding;
            }

            for (int i = 0; i <= 4; i++)
            {
                double ratio = i / 4.0;
                double y = top + ratio * plotHeight;
                var grid = new Line
                {
                    X1 = left,
                    X2 = width - right,
                    Y1 = y,
                    Y2 = y,
                    Stroke = new SolidColorBrush(Color.FromRgb(65, 72, 82)),
                    StrokeThickness = 1
                };
                canvas.Children.Add(grid);
                double val = max - ratio * (max - min);
                AddLabel(canvas, val.ToString("F2", CultureInfo.CurrentCulture),
                    2, y - 9, Brushes.LightGray, 10);
            }

            double start = _history[0].Time;
            double duration = Math.Max(1e-6, _history[^1].Time - start);
            var points = new PointCollection();

            foreach (var sample in _history)
            {
                double x = left + (sample.Time - start) / duration * plotWidth;
                double y = top + (max - selector(sample)) / (max - min) * plotHeight;
                points.Add(new Point(x, y));
            }

            canvas.Children.Add(new Polyline
            {
                Points = points,
                Stroke = new SolidColorBrush(lineColor),
                StrokeThickness = 2.3,
                StrokeLineJoin = PenLineJoin.Round
            });

            double latest = selector(_history[^1]);
            AddLabel(canvas, $"{latest:F2} {unit}", left + 8, top + 7,
                new SolidColorBrush(lineColor), 12);
            AddLabel(canvas, $"{start:F1} s", left, height - 19, Brushes.LightGray, 10);
            AddLabel(canvas, $"{_history[^1].Time:F1} s",
                Math.Max(left, width - 52), height - 19, Brushes.LightGray, 10);
        }

        private static void AddLabel(
            Canvas canvas, string value, double x, double y,
            Brush brush, double size)
        {
            var label = new TextBlock
            {
                Text = value,
                Foreground = brush,
                FontSize = size,
                IsHitTestVisible = false
            };
            Canvas.SetLeft(label, x);
            Canvas.SetTop(label, y);
            canvas.Children.Add(label);
        }
    }
}
