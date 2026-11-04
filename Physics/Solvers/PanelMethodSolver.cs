
using System;
using System.Numerics;
using AeroSim.Geometry;

namespace AeroSim.Physics.Solvers
{
    public class PanelMethodSolver
    {
        private readonly SurfaceMesh _mesh;

        public PanelMethodSolver(SurfaceMesh mesh)
        {
            _mesh = mesh;
        }

        public float[] Solve(Vector3 windVelocity)
        {
            int n = _mesh.Panels.Count;

            double[,] matrix = new double[n, n];
            double[] rhs = new double[n];

            for (int i = 0; i < n; i++)
            {
                SurfacePanel target = _mesh.Panels[i];

                rhs[i] = -Vector3.Dot(
                    windVelocity,
                    target.Normal
                );

                for (int j = 0; j < n; j++)
                {
                    if (i == j)
                    {
                        matrix[i, j] = 0.5;
                        continue;
                    }

                    Vector3 influence =
                        PanelInfluence.GetVelocity(
                            _mesh.Panels[j],
                            target.Center
                        );

                    matrix[i, j] = Vector3.Dot(
                        influence,
                        target.Normal
                    );
                }
            }


            double[,] matrixCopy = (double[,])matrix.Clone();
            double[] rhsCopy = (double[])rhs.Clone();

            double[] solution = SolveLinearSystem(matrixCopy, rhsCopy);

            // Résidu du système A * sigma = b
            double maxResidual = 0.0;
            double sumSquaredResidual = 0.0;

            for (int i = 0; i < n; i++)
            {
                double calculated = 0.0;

                for (int j = 0; j < n; j++)
                {
                    calculated += matrix[i, j] * solution[j];
                }

                double residual = calculated - rhs[i];

                maxResidual = Math.Max(
                    maxResidual,
                    Math.Abs(residual)
                );

                sumSquaredResidual += residual * residual;
            }

            double rmsResidual = Math.Sqrt(
                sumSquaredResidual / n
            );

            System.Diagnostics.Debug.WriteLine(
                $"PanelSolver | Max residual: {maxResidual:E4} m/s"
            );

            System.Diagnostics.Debug.WriteLine(
                $"PanelSolver | RMS residual: {rmsResidual:E4} m/s"
            );

            float[] strengths = new float[n];

            for (int i = 0; i < n; i++)
                strengths[i] = (float)solution[i];

            return strengths;
        }

        private static double[] SolveLinearSystem(
            double[,] a,
            double[] b)
        {
            int n = b.Length;

            for (int k = 0; k < n; k++)
            {
                int pivot = k;

                for (int i = k + 1; i < n; i++)
                {
                    if (Math.Abs(a[i, k]) >
                        Math.Abs(a[pivot, k]))
                    {
                        pivot = i;
                    }
                }

                if (Math.Abs(a[pivot, k]) < 1e-12)
                    throw new InvalidOperationException(
                        "Matrice singulière ou mal conditionnée."
                    );

                for (int j = k; j < n; j++)
                {
                    (a[k, j], a[pivot, j]) =
                        (a[pivot, j], a[k, j]);
                }

                (b[k], b[pivot]) = (b[pivot], b[k]);

                for (int i = k + 1; i < n; i++)
                {
                    double factor = a[i, k] / a[k, k];

                    for (int j = k + 1; j < n; j++)
                        a[i, j] -= factor * a[k, j];

                    b[i] -= factor * b[k];
                    a[i, k] = 0;
                }
            }

            double[] x = new double[n];

            for (int i = n - 1; i >= 0; i--)
            {
                double sum = b[i];

                for (int j = i + 1; j < n; j++)
                    sum -= a[i, j] * x[j];

                x[i] = sum / a[i, i];
            }

            return x;
        }
    }
}
