using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using AeroSim.Geometry;
using AeroSim.Physics.FlowFields;

namespace AeroSim.Simulation
{
    public class ParticleSystem
    {
        private readonly AirParticle[] _particles;
        private IFlowField _flow;
        private readonly Random _random;
        private readonly ParticleSettings _settings;
        private readonly SolidClassifier? _solid;
        private readonly long[] _recycleCounts = new long[4];
        private readonly int _flowAxis;

        private Func<Vector3>? _spawnGenerator;

        public long TotalSubSteps { get; private set; }
        public long TotalParticleUpdates { get; private set; }
        public long MultiSubStepUpdates { get; private set; }
        public int MaxSubStepsUsed { get; private set; }

        public ParticleSettings Settings => _settings;

        public long RecycleCount(RecycleReason reason) => _recycleCounts[(int)reason];
        public long TotalRecycled => _recycleCounts.Sum();

        public IReadOnlyList<AirParticle> Particles => _particles;

        public Vector3 BoundsMin { get; }
        public Vector3 BoundsMax { get; }

        public Vector3 SpawnMin { get; }
        public Vector3 SpawnMax { get; }

        public ParticleSystem(
            int particleCount,
            IFlowField flow,
            SimulationDomain domain,
            ParticleSettings? settings = null,
            SolidClassifier? solidClassifier = null)
            : this(
                particleCount,
                flow,
                domain.Min,
                domain.Max,
                domain.SpawnMin,
                domain.SpawnMax,
                settings,
                solidClassifier,
                domain.FlowAxis)
        { }

        public ParticleSystem(
            int particleCount,
            IFlowField flow,
            Vector3 boundsMin,
            Vector3 boundsMax,
            Vector3 spawnMin,
            Vector3 spawnMax,
            ParticleSettings? settings = null,
            SolidClassifier? solidClassifier = null,
            int flowAxis = 1)
        {
            if (particleCount <= 0)
                throw new ArgumentOutOfRangeException(nameof(particleCount));

            _flow = flow ?? throw new ArgumentNullException(nameof(flow));
            _settings = settings ?? new ParticleSettings();
            _solid = solidClassifier;
            _random = new Random(_settings.Seed);

            ValidateBounds(boundsMin, boundsMax);
            // La zone d'émission est un rectangle plat sur le plan amont :
            // l'épaisseur nulle selon l'axe du vent est légitime.
            ValidateBounds(spawnMin, spawnMax, allowDegenerate: true);

            if (spawnMin.X < boundsMin.X || spawnMax.X > boundsMax.X ||
                spawnMin.Y < boundsMin.Y || spawnMax.Y > boundsMax.Y ||
                spawnMin.Z < boundsMin.Z || spawnMax.Z > boundsMax.Z)
            {
                throw new ArgumentException(
                    "La zone d'émission doit être dans le domaine."
                );
            }

            BoundsMin = boundsMin;
            BoundsMax = boundsMax;
            SpawnMin = spawnMin;
            SpawnMax = spawnMax;
            _flowAxis = Math.Clamp(flowAxis, 0, 2);

            _particles = new AirParticle[particleCount];

            for (int i = 0; i < particleCount; i++)
            {
                float normalized = (float)(i + _random.NextDouble()) / particleCount;
                Vector3 position = RandomInitialPosition(normalized);

                if (!TryFindValidPosition(
                        p => p,
                        position,
                        out Vector3 valid))
                {
                    valid = RandomSpawnPosition();
                }

                var particle = new AirParticle(valid);
                ConfigureLifecycle(particle, randomizeAge: true);

                _particles[i] = particle;
            }
        }

        private static void ValidateBounds(
            Vector3 min,
            Vector3 max,
            bool allowDegenerate = false)
        {
            bool invalid = allowDegenerate
                ? min.X > max.X || min.Y > max.Y || min.Z > max.Z
                : min.X >= max.X || min.Y >= max.Y || min.Z >= max.Z;

            if (invalid)
            {
                throw new ArgumentException(
                    "Bornes min/max invalides."
                );
            }
        }

        private float RandomRange(float min, float max)
        {
            return min + (float)_random.NextDouble() * (max - min);
        }

        private Vector3 RandomInitialPosition(float normalizedAxis)
        {
            Vector3 p = RandomSpawnPosition();

            float axisMin = VectorAxis.Get(BoundsMin, _flowAxis);
            float axisMax = VectorAxis.Get(BoundsMax, _flowAxis);
            float axisValue = axisMin + normalizedAxis * (axisMax - axisMin);

            return VectorAxis.With(p, _flowAxis, axisValue);
        }

        private Vector3 RandomSpawnPosition()
        {
            return new Vector3(
                RandomRange(SpawnMin.X, SpawnMax.X),
                RandomRange(SpawnMin.Y, SpawnMax.Y),
                RandomRange(SpawnMin.Z, SpawnMax.Z)
            );
        }

        private Vector3 NextSpawnCandidate()
        {
            return _spawnGenerator?.Invoke() ?? RandomSpawnPosition();
        }

        private bool IsOutside(Vector3 p)
        {
            return p.X < BoundsMin.X || p.X > BoundsMax.X ||
                   p.Y < BoundsMin.Y || p.Y > BoundsMax.Y ||
                   p.Z < BoundsMin.Z || p.Z > BoundsMax.Z;
        }

        private bool IsInsideSolid(Vector3 p)
        {
            return _solid != null && _solid.Contains(p);
        }

        private bool TryFindValidPosition(
            Func<Vector3, Vector3> prepare,
            Vector3 firstCandidate,
            out Vector3 valid)
        {
            Vector3 p = prepare(firstCandidate);

            if (!IsOutside(p) && !IsInsideSolid(p))
            {
                valid = p;
                return true;
            }

            for (int i = 0; i < _settings.MaxEmissionAttempts; i++)
            {
                p = prepare(NextSpawnCandidate());

                if (!IsOutside(p) && !IsInsideSolid(p))
                {
                    valid = p;
                    return true;
                }
            }

            valid = prepare(firstCandidate);
            return false;
        }

        private void ConfigureLifecycle(AirParticle particle, bool randomizeAge)
        {
            double jitter = Math.Clamp(_settings.LifetimeJitter, 0.0, 0.9);
            double scale = 1.0 + (2.0 * _random.NextDouble() - 1.0) * jitter;
            double maxAge = Math.Max(1.0, _settings.MaxLifetimeSeconds * scale);
            double age = randomizeAge ? _random.NextDouble() * maxAge : 0.0;
            double window = _settings.StagnationWindowSeconds > 0
                ? _random.NextDouble() * _settings.StagnationWindowSeconds
                : 0.0;

            particle.SetLifecycle(maxAge, age, window);
        }

        private void Recycle(AirParticle particle, RecycleReason reason)
        {
            _recycleCounts[(int)reason]++;

            if (!TryFindValidPosition(
                    p => p,
                    NextSpawnCandidate(),
                    out Vector3 spawn))
            {
                spawn = RandomSpawnPosition();
            }

            particle.Reset(spawn);
            ConfigureLifecycle(particle, randomizeAge: false);
        }

        public void SetFlowField(IFlowField flow)
        {
            _flow = flow ?? throw new ArgumentNullException(nameof(flow));
        }

        public void SetSpawnGenerator(Func<Vector3> generator)
        {
            _spawnGenerator = generator ??
                throw new ArgumentNullException(nameof(generator));
        }

        public int CountParticlesInsideSolid()
        {
            if (_solid == null)
                return 0;

            int count = 0;

            foreach (AirParticle particle in _particles)
            {
                if (IsInsideSolid(particle.Position))
                    count++;
            }

            return count;
        }

        public void Update(double deltaTime, double simulationTime)
        {
            bool checkWindow = _settings.StagnationWindowSeconds > 0;
            bool checkInside = _settings.RecycleInsideSolid && _solid != null;

            foreach (AirParticle particle in _particles)
            {
                Vector3 before = particle.Position;

                int subSteps = particle.Update(
                    _flow,
                    deltaTime,
                    simulationTime,
                    _settings.MaxStepDistance,
                    Math.Max(1, _settings.MaxSubSteps)
                );

                TotalParticleUpdates++;
                TotalSubSteps += subSteps;

                if (subSteps > 1)
                    MultiSubStepUpdates++;

                if (subSteps > MaxSubStepsUsed)
                    MaxSubStepsUsed = subSteps;

                if (IsOutside(particle.Position))
                {
                    Recycle(particle, RecycleReason.OutOfBounds);
                    continue;
                }

                if (particle.Age >= particle.MaxAge)
                {
                    Recycle(particle, RecycleReason.Lifetime);
                    continue;
                }

                if (checkWindow && particle.WindowElapsed >= _settings.StagnationWindowSeconds)
                {
                    Vector3 delta = particle.Position - particle.WindowOrigin;
                    double netSpeed = delta.Length() / particle.WindowElapsed;

                    bool stagnant = netSpeed < _settings.StagnationMinNetSpeed;
                    bool insideSolid = checkInside && IsInsideSolid(particle.Position);

                    if (insideSolid)
                    {
                        Recycle(particle, RecycleReason.InsideSolid);
                        continue;
                    }

                    if (stagnant)
                    {
                        Recycle(particle, RecycleReason.Stagnation);
                        continue;
                    }

                    particle.RestartWindow();
                }
            }
        }

        public void Reset()
        {
            for (int i = 0; i < _particles.Length; i++)
            {
                float normalized = (float)(i + _random.NextDouble()) / _particles.Length;
                Vector3 p = RandomInitialPosition(normalized);

                if (!TryFindValidPosition(v => v, p, out Vector3 valid))
                    valid = RandomSpawnPosition();

                _particles[i].Reset(valid);
                ConfigureLifecycle(_particles[i], randomizeAge: true);
            }
        }
    }
}
