
using System;
using System.Diagnostics;
using System.Numerics;
using System.Windows;
using System.Windows.Media;

using AeroSim.Physics;
using AeroSim.Simulation;
using AeroSim.Rendering;
using AeroSim.Physics.FlowFields;

using AeroSim.Geometry;
using System.Windows.Media.Media3D;
using System.Linq;
using AeroSim.Physics.Solvers;
using System.Collections.Generic;

using AeroSim.Obstacles;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;

namespace AeroSim
{
    public partial class MainWindow : Window
    {
        private SimulationEngine? _simulation;
        private ParticleRenderer? _renderer;

        private StreamlineRenderer? _streamlineRenderer;

        private readonly Stopwatch _clock = new();
        private double _lastTime;

        private ObstacleManager? _obstacleManager;
        private IFlowField? _currentFlow;
        private IFlowField? _cachedFlow;

        private double _lastParticleRenderTime = 0;
        private const double ParticleRenderInterval = 1.0 / 120.0;

        private float _freeStreamSpeed;

        // Fenêtre indépendante de diagnostics
        private SimulationDataWindow? _dataWindow;
        private double _lastDataSampleTime = -1;
        private string _currentObstacleName = "Cube";
        private int _panelCount;
        private float _totalArea;
        private long _solverMs;
        private long _cacheMs;
        private double _normalRms;
        private double _normalMax;
        private int _normalSamples;
        private SimulationDomain? _currentDomain;
        private SolidClassifier? _solidClassifier;
        private readonly DomainSettings _domainSettings = new();
        private readonly List<Vector3> _probePoints = new();
        private double[]? _baselineProbeDirectSpeeds;
        private double _probeErrorRms;
        private double _probeErrorMax;
        private double _probeStationDrift;
        private Vector3 _probeWorstPosition;
        private double _probeWorstDirect;
        private double _probeWorstCached;
        private Vector3 _fieldExtremePosition;
        private double _fieldExtremeSpeedCached;
        private double _fieldExtremeSpeedDirect;


        public MainWindow()
        {
            InitializeComponent();

            Loaded += OnLoaded;
            Closed += OnClosed;


            WindSlider.AddHandler(
                Thumb.DragCompletedEvent,
                new DragCompletedEventHandler(
                    WindSlider_DragCompleted
                )
            );
        }



        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            _obstacleManager = new ObstacleManager(MainViewport);

            _renderer = new ParticleRenderer(MainViewport);
            _streamlineRenderer = new StreamlineRenderer(MainViewport);

            LoadObstacle("Cube");

            _clock.Restart();
            _lastTime = 0;

            CompositionTarget.Rendering += OnRendering;
        }





        private void OnRendering(object? sender, EventArgs e)
        {
            double now = _clock.Elapsed.TotalSeconds;
            double frameTime = now - _lastTime;
            _lastTime = now;

            if (_simulation == null ||
                _renderer == null ||
                _cachedFlow == null)
                return;

            // La simulation physique reste mise à jour
            // à chaque frame.
            _simulation.Update(frameTime);

            // Échantillonnage léger pour les graphiques, sans dépendre
            // de l'affichage des particules.
            if (_dataWindow != null &&
                now - _lastDataSampleTime >= 0.5)
            {
                _lastDataSampleTime = now;
                UpdateDataWindow(now);
            }

            if (ParticlesToggle.IsChecked != true)
                return;

            // Limiter le rafraîchissement des particules
            // à environ 30 FPS maximum.
            if (now - _lastParticleRenderTime
                < ParticleRenderInterval)
                return;

            _lastParticleRenderTime = now;

            _renderer.UpdateVisuals(
                _simulation.Particles.Particles,
                position => _cachedFlow.GetVelocity(position, now),
                _freeStreamSpeed
            );
        }





        private void OnClosed(object? sender, EventArgs e)
        {
            CompositionTarget.Rendering -= OnRendering;
            _clock.Stop();
            _dataWindow?.Close();
        }



        private void ParticlesToggle_Changed(
            object sender,
            RoutedEventArgs e)
        {
            _renderer?.SetVisible(
                ParticlesToggle.IsChecked == true
            );
        }



        private void StreamlinesToggle_Changed(
            object sender,
            RoutedEventArgs e)
        {
            _streamlineRenderer?.SetVisible(
                StreamlinesToggle.IsChecked == true
            );
        }



        private void LoadObstacle(string type)
        {
            if (_obstacleManager == null ||
                _renderer == null ||
                _streamlineRenderer == null)
                return;

            var timer = Stopwatch.StartNew();

            // 1. Charger la nouvelle géométrie
            SurfaceMesh mesh = _obstacleManager.Load(type);
            _currentObstacleName = type;
            _panelCount = mesh.TriangleCount;
            _totalArea = mesh.TotalArea;
            _solverMs = 0;
            _cacheMs = 0;
            _normalRms = double.NaN;
            _normalMax = double.NaN;
            _normalSamples = 0;
            _currentDomain = null;
            _solidClassifier = null;
            _probePoints.Clear();
            _baselineProbeDirectSpeeds = null;
            _probeErrorRms = double.NaN;
            _probeErrorMax = double.NaN;
            _probeStationDrift = double.NaN;
            _probeWorstPosition = Vector3.Zero;
            _probeWorstDirect = double.NaN;
            _probeWorstCached = double.NaN;
            _fieldExtremePosition = Vector3.Zero;
            _fieldExtremeSpeedCached = double.NaN;
            _fieldExtremeSpeedDirect = double.NaN;
            _dataWindow?.ClearHistory();
            RefreshDataSummary();

            // Sécurité : ne pas lancer le solveur sur
            // une géométrie trop détaillée.
            const int MaxPanels = 5000;

            if (mesh.TriangleCount > MaxPanels)
            {
                // Arrêter l'ancienne simulation pour éviter
                // d'afficher le champ de l'ancien obstacle.
                _simulation = null;
                _currentFlow = null;
                _cachedFlow = null;
                RefreshDataSummary();

                _renderer?.UpdateVisuals(
                    Array.Empty<AirParticle>(),
                    _ => Vector3.Zero,
                    0f
                );

                _streamlineRenderer?.UpdateVisuals(
                    Array.Empty<Streamline>(),
                    _ => Vector3.Zero,
                    0f
                );

                Debug.WriteLine(
                    $"[AeroSim] {type} : " +
                    $"{mesh.TriangleCount} triangles. " +
                    "Simulation désactivée : maillage trop détaillé."
                );

                MessageBox.Show(
                    $"Modèle chargé avec {mesh.TriangleCount:N0} triangles.\n\n" +
                    $"La simulation est limitée provisoirement à {MaxPanels} panneaux.\n" +
                    "Le modèle est affiché, sans calcul aérodynamique.",
                    "AeroSim - Géométrie trop détaillée"
                );

                return;
            }

            Debug.WriteLine(
                $"[Geometry] Obstacle : {type}, " +
                $"Triangles : {mesh.TriangleCount}, " +
                $"Temps : {timer.ElapsedMilliseconds} ms"
            );

            // 2. Recalculer le champ aérodynamique
            timer.Restart();

            _freeStreamSpeed = (float)(WindSlider.Value / 3.6);

            float windSpeed = _freeStreamSpeed;

            // Angle en degrés vers radians
            float angleRadians = (float)(
                0 * Math.PI / 180.0
            );

            // Direction du vent dans le plan YZ
            Vector3 windVelocity = new Vector3(
                0f,
                windSpeed * MathF.Cos(angleRadians),
                windSpeed * MathF.Sin(angleRadians)
            );

            Debug.WriteLine(
                $"Wind = {windVelocity}"
            );

            // Champ physique calculé à partir des panneaux
            _currentFlow = new PanelFlowField(
                mesh,
                windVelocity
            );

            _solverMs = timer.ElapsedMilliseconds;

            // Diagnostic sur un sous-ensemble de panneaux.
            // Calcul non-caché, réalisé une seule fois par chargement.
            MeasureNormalResidual(mesh, _currentFlow);

            Debug.WriteLine(
                $"[Solver] {_solverMs} ms"
            );

            _solidClassifier = new SolidClassifier(mesh);
            _currentDomain = SimulationDomain.Create(
                mesh,
                windVelocity,
                _domainSettings
            );

            _probePoints.Clear();
            _probePoints.AddRange(BuildProbePoints(_currentDomain, _solidClassifier));
            _baselineProbeDirectSpeeds = null;

            Debug.WriteLine(
                $"[Domain] {_currentDomain}"
            );

            Debug.WriteLine(
                $"[Solid] {_solidClassifier.Reliability}"
            );

            // Construction du cache 3D
            timer.Restart();

            _cachedFlow = new CachedFlowField(
                source: _currentFlow,
                min: _currentDomain.Min,
                max: _currentDomain.Max,
                nx: _currentDomain.Nx,
                ny: _currentDomain.Ny,
                nz: _currentDomain.Nz,
                outsideVelocity: windVelocity
            );

            _cacheMs = timer.ElapsedMilliseconds;
            RefreshDataSummary();

            Debug.WriteLine(
                $"[Cache] {_cacheMs} ms"
            );

            // 3. Recréer le système de particules
            timer.Restart();

            var particleSystem = new ParticleSystem(
                particleCount: 10000,
                flow: _cachedFlow,
                domain: _currentDomain,
                settings: new ParticleSettings
                {
                    Seed = 12345,
                    MaxLifetimeSeconds = 55.0,
                    LifetimeJitter = 0.35,
                    StagnationWindowSeconds = 3.0,
                    StagnationMinNetSpeed = 0.25,
                    RecycleInsideSolid = true,
                    MaxStepDistance = 0.55f * _currentDomain.MinCellSize,
                    MaxSubSteps = 8,
                    MaxEmissionAttempts = 48
                },
                solidClassifier: _solidClassifier
            );

            _simulation = new SimulationEngine(particleSystem);

            _renderer.UpdateVisuals(
                _simulation.Particles.Particles,
                position => _cachedFlow!.GetVelocity(position, 0.0),
                _freeStreamSpeed
            );

            Debug.WriteLine(
                $"[Particles] {timer.ElapsedMilliseconds} ms"
            );

            // 4. Recalculer les lignes de courant
            timer.Restart();

            var streamlines = new List<Streamline>();

            foreach (Vector3 start in BuildStreamlineSeeds(_currentDomain))
            {
                Streamline line = StreamlineGenerator.Generate(
                    _cachedFlow,
                    start,
                    stepSize: MathF.Max(0.1f, _currentDomain.MinCellSize * 0.4f),
                    maxSteps: 400
                );

                streamlines.Add(line);
            }

            _streamlineRenderer.UpdateVisuals(
                streamlines,
                position => _cachedFlow!.GetVelocity(position, 0.0),
                _freeStreamSpeed
            );

            EvaluateProbeDiagnostics(0.0, resetBaseline: true);
            ScanFieldExtremes();
            RefreshDataSummary();

            Debug.WriteLine(
                $"[Streamlines] {timer.ElapsedMilliseconds} ms"
            );

            // 5. Respecter les modes de visualisation actifs
            _renderer.SetVisible(
                ParticlesToggle.IsChecked == true
            );

            _streamlineRenderer.SetVisible(
                StreamlinesToggle.IsChecked == true
            );
        }



        private void ObstacleSelector_SelectionChanged(
            object sender,
            SelectionChangedEventArgs e)
        {
            if (!IsLoaded || _obstacleManager == null)
                return;

            if (ObstacleSelector.SelectedItem is ComboBoxItem item &&
                item.Content is string obstacleType)
            {
                LoadObstacle(obstacleType);
            }
        }



        private void WindSlider_ValueChanged(
            object sender,
            RoutedPropertyChangedEventArgs<double> e)
        {
            // 
        }



        private void WindSlider_DragCompleted(
            object sender,
            DragCompletedEventArgs e)
        {
            if (!IsLoaded || _obstacleManager == null)
                return;

            if (ObstacleSelector.SelectedItem is ComboBoxItem item &&
                item.Content is string obstacleType)
            {
                LoadObstacle(obstacleType);
            }
        }


        private void OpenDataWindow_Click(object sender, RoutedEventArgs e)
        {
            if (_dataWindow != null)
            {
                _dataWindow.Activate();
                return;
            }

            _dataWindow = new SimulationDataWindow
            {
                Owner = this
            };

            _dataWindow.Closed += (_, _) => _dataWindow = null;
            _dataWindow.Show();
            _lastDataSampleTime = -1;
            RefreshDataSummary();
        }

        private void RefreshDataSummary()
        {
            int inside = _simulation?.Particles.CountParticlesInsideSolid() ?? 0;
            string domain = _currentDomain?.ToString() ?? "—";
            string solid = _solidClassifier?.Reliability ?? "—";

            _dataWindow?.SetScenario(
                _currentObstacleName,
                _panelCount,
                _totalArea,
                _freeStreamSpeed,
                _solverMs,
                _cacheMs,
                _normalRms,
                _normalMax,
                _normalSamples,
                _simulation != null && _cachedFlow != null,
                domain,
                solid,
                inside
            );
        }

        private void MeasureNormalResidual(SurfaceMesh mesh, IFlowField flow)
        {
            // Échantillon déterministe de 48 centres, évite de refaire
            // des millions d'interactions pour tous les panneaux.
            const int maxSamples = 48;
            int count = Math.Min(maxSamples, mesh.Panels.Count);
            double sumSquares = 0;
            double maximum = 0;
            double rawSumSquares = 0;
            double rawMaximum = 0;
            int valid = 0;

            // Au centre d'un panneau, le champ évalué ne contient pas le saut
            // 0.5 * sigma imposé par le solveur (diagonale A[i,i] = 0.5).
            // La trace extérieure de la vitesse normale est donc u_centre·n + 0.5 * sigma.
            var strengths = (flow as PanelFlowField)?.Strengths;

            for (int k = 0; k < count; k++)
            {
                int index = k * mesh.Panels.Count / count;
                SurfacePanel panel = mesh.Panels[index];
                Vector3 velocity = flow.GetVelocity(panel.Center, 0.0);
                double rawNormal = Vector3.Dot(velocity, panel.Normal);
                double jump = strengths != null ? 0.5 * strengths[index] : 0.0;
                double vn = Math.Abs(rawNormal + jump);

                if (!double.IsFinite(vn))
                    continue;

                sumSquares += vn * vn;
                maximum = Math.Max(maximum, vn);
                rawSumSquares += rawNormal * rawNormal;
                rawMaximum = Math.Max(rawMaximum, Math.Abs(rawNormal));
                valid++;
            }

            _normalSamples = valid;
            _normalRms = valid > 0 ? Math.Sqrt(sumSquares / valid) : double.NaN;
            _normalMax = valid > 0 ? maximum : double.NaN;

            if (valid > 0)
            {
                Debug.WriteLine(
                    $"[NormalDiag] trace extérieure RMS={_normalRms:F4} max={_normalMax:F4} | " +
                    $"ancienne métrique (sans saut) RMS={Math.Sqrt(rawSumSquares / valid):F4} max={rawMaximum:F4}"
                );
            }
        }

        private static List<Vector3> BuildStreamlineSeeds(SimulationDomain domain)
        {
            var points = new List<Vector3>();
            Vector3 min = domain.Min;
            Vector3 max = domain.Max;
            Vector3 spawnMin = domain.SpawnMin;
            Vector3 spawnMax = domain.SpawnMax;

            int a = (domain.FlowAxis + 1) % 3;
            int b = (domain.FlowAxis + 2) % 3;

            float plane = VectorAxis.Get(spawnMin, domain.FlowAxis);

            for (int i = 0; i < 11; i++)
            {
                float ta = i / 10f;

                for (int j = 0; j < 7; j++)
                {
                    float tb = j / 6f;
                    Vector3 p = spawnMin;

                    p = VectorAxis.With(p, a, Lerp(VectorAxis.Get(spawnMin, a), VectorAxis.Get(spawnMax, a), ta));
                    p = VectorAxis.With(p, b, Lerp(VectorAxis.Get(spawnMin, b), VectorAxis.Get(spawnMax, b), tb));
                    p = VectorAxis.With(p, domain.FlowAxis, plane);

                    // Décaler légèrement dans le domaine pour éviter les points exactement sur la frontière.
                    float inset = 2f * domain.MinCellSize * domain.FlowSign;
                    p = VectorAxis.With(p, domain.FlowAxis, plane + inset);
                    p = ClampToDomain(p, min, max);
                    points.Add(p);
                }
            }

            return points;
        }

        private static IEnumerable<Vector3> BuildProbePoints(
            SimulationDomain domain,
            SolidClassifier? solid)
        {
            var probes = new List<Vector3>();
            int flowAxis = domain.FlowAxis;
            int axisA = (flowAxis + 1) % 3;
            int axisB = (flowAxis + 2) % 3;

            float oMin = VectorAxis.Get(domain.ObstacleMin, flowAxis);
            float oMax = VectorAxis.Get(domain.ObstacleMax, flowAxis);
            float centerFlow = 0.5f * (oMin + oMax);

            float sizeA = VectorAxis.Get(domain.ObstacleMax, axisA) - VectorAxis.Get(domain.ObstacleMin, axisA);
            float sizeB = VectorAxis.Get(domain.ObstacleMax, axisB) - VectorAxis.Get(domain.ObstacleMin, axisB);

            float spanA = MathF.Max(sizeA * 0.6f, domain.MinCellSize * 3f);
            float spanB = MathF.Max(sizeB * 0.6f, domain.MinCellSize * 3f);

            float upstream = oMin - 0.35f * domain.CharacteristicLength;
            float downstream = oMax + 0.8f * domain.CharacteristicLength;
            float[] streamStations = { upstream, centerFlow, downstream };
            float[] lateralA = { 0f, spanA, -spanA };
            float[] lateralB = { 0f, spanB, -spanB };

            Vector3 obstacleCenter = 0.5f * (domain.ObstacleMin + domain.ObstacleMax);

            foreach (float sf in streamStations)
            {
                foreach (float da in lateralA)
                {
                    foreach (float db in lateralB)
                    {
                        if (Math.Abs(da) > 0 && Math.Abs(db) > 0)
                            continue;

                        Vector3 p = obstacleCenter;
                        p = VectorAxis.With(p, flowAxis, sf);
                        p = VectorAxis.With(p, axisA, VectorAxis.Get(obstacleCenter, axisA) + da);
                        p = VectorAxis.With(p, axisB, VectorAxis.Get(obstacleCenter, axisB) + db);
                        p = ClampToDomain(p, domain.Min, domain.Max);

                        if (solid != null && solid.IsReliable && solid.Contains(p))
                        {
                            float nudge = MathF.Max(domain.MinCellSize * 2f, 0.05f * domain.CharacteristicLength);
                            p = VectorAxis.With(
                                p,
                                flowAxis,
                                VectorAxis.Get(p, flowAxis) - domain.FlowSign * nudge);
                            p = ClampToDomain(p, domain.Min, domain.Max);
                        }

                        probes.Add(p);
                    }
                }
            }

            return probes;
        }

        private void EvaluateProbeDiagnostics(double time, bool resetBaseline)
        {
            if (_currentFlow == null || _cachedFlow == null || _probePoints.Count == 0)
                return;

            if (resetBaseline)
                _baselineProbeDirectSpeeds = null;

            int n = _probePoints.Count;
            _baselineProbeDirectSpeeds ??= new double[n];

            double sumSq = 0;
            double maxError = 0;
            double maxDrift = 0;
            Vector3 worstPos = _probePoints[0];
            double worstDirect = 0;
            double worstCached = 0;

            for (int i = 0; i < n; i++)
            {
                Vector3 p = _probePoints[i];
                Vector3 direct = _currentFlow.GetVelocity(p, time);
                Vector3 cached = _cachedFlow.GetVelocity(p, time);
                double directSpeed = direct.Length();
                double cachedSpeed = cached.Length();
                double err = (direct - cached).Length();

                if (i == 0 || err > maxError)
                {
                    maxError = err;
                    worstPos = p;
                    worstDirect = directSpeed;
                    worstCached = cachedSpeed;
                }

                sumSq += err * err;

                if (resetBaseline)
                    _baselineProbeDirectSpeeds[i] = directSpeed;
                else
                    maxDrift = Math.Max(maxDrift, Math.Abs(directSpeed - _baselineProbeDirectSpeeds[i]));
            }

            _probeErrorRms = Math.Sqrt(sumSq / n);
            _probeErrorMax = maxError;
            _probeStationDrift = maxDrift;
            _probeWorstPosition = worstPos;
            _probeWorstDirect = worstDirect;
            _probeWorstCached = worstCached;
        }

        private void ScanFieldExtremes()
        {
            if (_currentDomain == null || _currentFlow == null || _cachedFlow == null)
                return;

            const int nx = 9;
            const int ny = 9;
            const int nz = 7;

            Vector3 min = _currentDomain.Min;
            Vector3 max = _currentDomain.Max;

            double best = double.NegativeInfinity;
            Vector3 bestPos = min;
            double bestDirect = 0;

            for (int ix = 0; ix < nx; ix++)
            {
                float tx = ix / (float)(nx - 1);

                for (int iy = 0; iy < ny; iy++)
                {
                    float ty = iy / (float)(ny - 1);

                    for (int iz = 0; iz < nz; iz++)
                    {
                        float tz = iz / (float)(nz - 1);

                        Vector3 p = new(
                            Lerp(min.X, max.X, tx),
                            Lerp(min.Y, max.Y, ty),
                            Lerp(min.Z, max.Z, tz)
                        );

                        Vector3 vCached = _cachedFlow.GetVelocity(p, 0);
                        double speedCached = vCached.Length();

                        if (speedCached > best)
                        {
                            best = speedCached;
                            bestPos = p;
                            bestDirect = _currentFlow.GetVelocity(p, 0).Length();
                        }
                    }
                }
            }

            _fieldExtremePosition = bestPos;
            _fieldExtremeSpeedCached = best;
            _fieldExtremeSpeedDirect = bestDirect;
        }

        private static Vector3 ClampToDomain(Vector3 p, Vector3 min, Vector3 max)
        {
            return new Vector3(
                Math.Clamp(p.X, min.X, max.X),
                Math.Clamp(p.Y, min.Y, max.Y),
                Math.Clamp(p.Z, min.Z, max.Z)
            );
        }

        private static float Lerp(float a, float b, float t) => a + (b - a) * t;

        private void UpdateDataWindow(double elapsedSeconds)
        {
            if (_dataWindow == null || _simulation == null || _cachedFlow == null)
                return;

            var particles = _simulation.Particles.Particles;
            if (particles.Count == 0)
                return;

            // Mesurer un échantillon déterministe, pas les 10 000 particules.
            int count = Math.Min(150, particles.Count);
            double sumSpeed = 0;
            double maxSpeed = 0;
            double sumVertical = 0;
            int valid = 0;

            for (int k = 0; k < count; k++)
            {
                int index = k * particles.Count / count;
                Vector3 p = particles[index].Position;
                Vector3 velocity = _cachedFlow.GetVelocity(p, elapsedSeconds);
                double speed = velocity.Length();

                if (!double.IsFinite(speed) || !float.IsFinite(velocity.Z))
                    continue;

                sumSpeed += speed;
                maxSpeed = Math.Max(maxSpeed, speed);
                sumVertical += velocity.Z;
                valid++;
            }

            if (valid == 0)
                return;

            EvaluateProbeDiagnostics(elapsedSeconds, resetBaseline: false);

            ParticleSystem ps = _simulation.Particles;
            double mean = sumSpeed / valid;
            double deviation = _freeStreamSpeed > 1e-6f
                ? 100.0 * (mean - _freeStreamSpeed) / _freeStreamSpeed
                : 0.0;

            _dataWindow.AddSample(
                elapsedSeconds,
                mean,
                maxSpeed,
                deviation,
                sumVertical / valid,
                valid,
                ps.RecycleCount(RecycleReason.OutOfBounds),
                ps.RecycleCount(RecycleReason.Lifetime),
                ps.RecycleCount(RecycleReason.Stagnation),
                ps.RecycleCount(RecycleReason.InsideSolid),
                ps.TotalParticleUpdates > 0
                    ? (double)ps.TotalSubSteps / ps.TotalParticleUpdates
                    : 1.0,
                ps.MaxSubStepsUsed,
                _probePoints.Count,
                _probeErrorRms,
                _probeErrorMax,
                _probeStationDrift,
                _probeWorstPosition,
                _probeWorstDirect,
                _probeWorstCached,
                _fieldExtremePosition,
                _fieldExtremeSpeedCached,
                _fieldExtremeSpeedDirect);
        }
    }
}
