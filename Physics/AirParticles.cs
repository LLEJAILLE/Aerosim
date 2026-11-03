using AeroSim.Physics.FlowFields;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace AeroSim.Simulation
{
    public class AirParticle
    {
        public Vector3 Position { get; private set; }

        public Queue<Vector3> Trail { get; } = new();

        public int MaxTrailLength { get; set; } = 10;

        // Temps écoulé depuis la dernière émission et durée de vie autorisée (s).
        public double Age { get; private set; }
        public double MaxAge { get; private set; } = double.PositiveInfinity;

        // Fenêtre de détection des particules bloquées : position de départ et durée écoulée.
        public Vector3 WindowOrigin { get; private set; }
        public double WindowElapsed { get; private set; }

        public AirParticle(Vector3 initialPosition)
        {
            Position = initialPosition;
            WindowOrigin = initialPosition;
            Trail.Enqueue(initialPosition);
        }

        /// <summary>
        /// Avance la particule par Euler explicite avec sous-pas : le nombre de sous-pas est choisi
        /// pour que chaque déplacement reste inférieur à <paramref name="maxStepDistance"/>, dans la
        /// limite de <paramref name="maxSubSteps"/>. La vitesse du champ n'est jamais plafonnée.
        /// Retourne le nombre de sous-pas effectués.
        /// </summary>
        public int Update(
            IFlowField flow,
            double deltaTime,
            double simulationTime,
            float maxStepDistance = float.PositiveInfinity,
            int maxSubSteps = 1)
        {
            Vector3 velocity = flow.GetVelocity(Position, simulationTime);
            int subSteps = 1;

            if (float.IsFinite(maxStepDistance) && maxStepDistance > 0f && maxSubSteps > 1)
            {
                double distance = velocity.Length() * deltaTime;

                if (double.IsFinite(distance))
                {
                    subSteps = Math.Clamp(
                        (int)Math.Ceiling(distance / maxStepDistance),
                        1,
                        maxSubSteps);
                }
            }

            double subDelta = deltaTime / subSteps;

            for (int i = 0; i < subSteps; i++)
            {
                if (i > 0)
                    velocity = flow.GetVelocity(Position, simulationTime + i * subDelta);

                if (!float.IsFinite(velocity.X) ||
                    !float.IsFinite(velocity.Y) ||
                    !float.IsFinite(velocity.Z))
                    break;

                Position += velocity * (float)subDelta;
            }

            Age += deltaTime;
            WindowElapsed += deltaTime;

            Trail.Enqueue(Position);

            while (Trail.Count > MaxTrailLength)
                Trail.Dequeue();

            return subSteps;
        }

        /// <summary>Redémarre la fenêtre de détection de blocage à la position courante.</summary>
        public void RestartWindow()
        {
            WindowOrigin = Position;
            WindowElapsed = 0.0;
        }

        /// <summary>Décale le cycle de vie pour étaler les réémissions dans le temps.</summary>
        public void SetLifecycle(double maxAge, double age, double windowElapsed)
        {
            MaxAge = maxAge;
            Age = age;
            WindowElapsed = windowElapsed;
        }

        public void Reset(Vector3 newPosition)
        {
            Position = newPosition;
            Age = 0.0;
            WindowOrigin = newPosition;
            WindowElapsed = 0.0;
            Trail.Clear();
            Trail.Enqueue(newPosition);
        }
    }
}