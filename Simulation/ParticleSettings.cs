using System;

namespace AeroSim.Simulation
{
    /// <summary>
    /// Paramètres du cycle de vie des particules. Ces critères agissent uniquement sur la
    /// représentation (réémission des particules) : le champ de vitesse n'est jamais modifié.
    /// </summary>
    public sealed class ParticleSettings
    {
        // Graine du générateur pseudo-aléatoire : même graine et même configuration = mêmes trajectoires.
        public int Seed { get; init; } = 12345;

        // Durée de vie moyenne (s) avant réémission, quelle que soit la vitesse de la particule.
        public double MaxLifetimeSeconds { get; init; } = 60.0;

        // Dispersion relative de la durée de vie (0,25 = ±25 %), pour éviter une réémission simultanée.
        public double LifetimeJitter { get; init; } = 0.25;

        // Fenêtre de détection des particules bloquées (s). Le critère est le déplacement net
        // entre le début et la fin de la fenêtre : une particule qui oscille sur place sans
        // avancer est détectée même si son chemin parcouru est long.
        public double StagnationWindowSeconds { get; init; } = 3.0;

        // Vitesse nette moyenne minimale (m/s) sur la fenêtre ; en dessous, la particule est recyclée.
        // Une zone de stagnation physique légitime peut donc aussi être vidée : ce choix est un
        // compromis de visualisation, et chaque recyclage est compté par cause.
        public double StagnationMinNetSpeed { get; init; } = 0.25;

        // Recycler les particules qui se retrouvent dans un solide (il n'y a pas de collision).
        // Testé à la fin de chaque fenêtre, pas à chaque pas.
        public bool RecycleInsideSolid { get; init; } = true;

        // Déplacement maximal par sous-pas (m), typiquement une fraction de la cellule du cache.
        public float MaxStepDistance { get; init; } = float.PositiveInfinity;

        // Nombre maximal de sous-pas Euler par pas de simulation. Au-delà, le pas reste plus
        // grand que MaxStepDistance : la vitesse n'est jamais plafonnée.
        public int MaxSubSteps { get; init; } = 8;

        // Tentatives de tirage d'une position d'émission hors solide.
        public int MaxEmissionAttempts { get; init; } = 32;
    }

    public enum RecycleReason
    {
        OutOfBounds = 0,
        Lifetime = 1,
        Stagnation = 2,
        InsideSolid = 3
    }
}
