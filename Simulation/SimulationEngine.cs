using System;

namespace AeroSim.Simulation
{
    public class SimulationEngine
    {
        private readonly ParticleSystem _particleSystem;

        private double _accumulator;
        private double _simulationTime;

        private const double FixedDeltaTime = 0.02;

        public ParticleSystem Particles => _particleSystem;

        public SimulationEngine(ParticleSystem particleSystem)
        {
            _particleSystem = particleSystem;
        }

        public void Update(double frameTime)
        {
            frameTime = Math.Clamp(frameTime, 0.0, 0.1);

            _accumulator += frameTime;

            while (_accumulator >= FixedDeltaTime)
            {
                _particleSystem.Update(
                    FixedDeltaTime,
                    _simulationTime
                );

                _simulationTime += FixedDeltaTime;
                _accumulator -= FixedDeltaTime;
            }
        }
    }
}