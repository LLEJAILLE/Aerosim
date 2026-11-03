
using System;
using System.Collections.Generic;
using System.Numerics;
using AeroSim.Geometry;
using AeroSim.Physics.Solvers;

namespace AeroSim.Physics.FlowFields
{
    public class PanelFlowField : IFlowField
    {
        private readonly SurfaceMesh _mesh;
        private readonly float[] _strengths;
        private readonly Vector3 _windVelocity;

        public PanelFlowField(
            SurfaceMesh mesh,
            Vector3 windVelocity)
        {
            _mesh = mesh;
            _windVelocity = windVelocity;

            var solver = new PanelMethodSolver(mesh);
            _strengths = solver.Solve(windVelocity);
        }

        public IReadOnlyList<float> Strengths => _strengths;

        public Vector3 GetVelocity(Vector3 position, double time)
        {
            Vector3 velocity = _windVelocity;

            for (int i = 0; i < _mesh.Panels.Count; i++)
            {
                Vector3 influence = PanelInfluence.GetVelocity(
                    _mesh.Panels[i],
                    position
                );

                velocity += _strengths[i] * influence;
            }

            return velocity;
        }
    }
}
