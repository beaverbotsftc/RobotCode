package org.beaverbots.beaver.pathing.path.pathbuilder;

import org.apache.commons.math3.linear.RealVector;

public class MaxSpeed {
    /// An approximation, but rather good; also a strict upper bound
    public static double cubicBezier(RealVector p0, RealVector p1, RealVector p2, RealVector p3) {
        // 1. Calculate the velocity control vectors (scaled by 3 for cubic derivative)
        RealVector v0 = p1.subtract(p0).mapMultiply(3.0);
        RealVector v1 = p2.subtract(p1).mapMultiply(3.0);
        RealVector v2 = p3.subtract(p2).mapMultiply(3.0);

        // 2. Perform one iteration of subdivision (De Casteljau) on the velocity vectors
        RealVector ma = v0.add(v1).mapMultiply(0.5);
        RealVector mb = v1.add(v2).mapMultiply(0.5);
        RealVector mSplit = ma.add(mb).mapMultiply(0.5); // This is exactly Velocity at t=0.5

        // 3. The curve is strictly contained within the convex hulls of the two halves.
        // We check the norm of all defining points of these hulls.
        double maxV0 = v0.getNorm();
        double maxMa = ma.getNorm();
        double maxSplit = mSplit.getNorm();
        double maxMb = mb.getNorm();
        double maxV2 = v2.getNorm();

        // 4. Return the maximum.
        // This is GUARANTEED to be >= True Max Speed.
        // This is significantly tighter than the original single convex hull.
        return Math.max(maxV0, Math.max(maxMa, Math.max(maxSplit, Math.max(maxMb, maxV2))));
    }
}
