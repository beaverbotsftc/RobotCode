package org.beaverbots.beaver.util;

import android.util.Pair;

import java.util.ArrayList;
import java.util.List;

public class Geometry {
    public enum Orientation {
        COLINEAR,
        CLOCKWISE,
        COUNTERCLOCKWISE,
    }

    public static Orientation orientation(double px, double py, double qx, double qy, double rx, double ry) {
        double val = (qy - py) * (rx - qx) - (qx - px) * (ry - qy);
        if (Math.abs(val) < 1e-10) return Orientation.COLINEAR;
        return (val > 0) ? Orientation.CLOCKWISE : Orientation.COUNTERCLOCKWISE;
    }

    public static boolean onSegment(double px, double py, double qx, double qy, double rx, double ry) {
        return px >= Math.min(qx, rx) && px <= Math.max(qx, rx)
                && py >= Math.min(qy, ry) && py <= Math.max(qy, ry);
    }

    public static boolean edgeEdgeIntersects(
            double x1, double y1, double x2, double y2,
            double x3, double y3, double x4, double y4
    ) {
        Orientation o1 = orientation(x1, y1, x2, y2, x3, y3);
        Orientation o2 = orientation(x1, y1, x2, y2, x4, y4);
        Orientation o3 = orientation(x3, y3, x4, y4, x1, y1);
        Orientation o4 = orientation(x3, y3, x4, y4, x2, y2);

        // General case
        if (o1 != o2 && o3 != o4) return true;

        // Special cases (collinear points)
        if (o1 == Orientation.COLINEAR && onSegment(x3, y3, x1, y1, x2, y2)) return true;
        if (o2 == Orientation.COLINEAR && onSegment(x4, y4, x1, y1, x2, y2)) return true;
        if (o3 == Orientation.COLINEAR && onSegment(x1, y1, x3, y3, x4, y4)) return true;
        if (o4 == Orientation.COLINEAR && onSegment(x2, y2, x3, y3, x4, y4)) return true;

        return false; // no intersection
    }

    public static boolean pointPolygonIntersects(double px, double py, List<Double> vx, List<Double> vy) {
        int n = vx.size();
        boolean inside = false;

        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = vx.get(i), yi = vy.get(i);
            double xj = vx.get(j), yj = vy.get(j);

            // Check if point is on edge
            if (onSegment(px, py, xi, yi, xj, yj)) return true;

            // Ray-casting algorithm
            boolean intersect = ((yi > py) != (yj > py)) &&
                    (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }

        return inside;
    }

    public static boolean polygonPolygonIntersects(
            List<Double> ax, List<Double> ay,
            List<Double> bx, List<Double> by
    ) {
        int nA = ax.size();
        int nB = bx.size();

        // 1. Check all edge pairs for intersection
        for (int i = 0; i < nA; i++) {
            int nextA = (i + 1) % nA;
            double ax1 = ax.get(i), ay1 = ay.get(i);
            double ax2 = ax.get(nextA), ay2 = ay.get(nextA);

            for (int j = 0; j < nB; j++) {
                int nextB = (j + 1) % nB;
                double bx1 = bx.get(j), by1 = by.get(j);
                double bx2 = bx.get(nextB), by2 = by.get(nextB);

                if (edgeEdgeIntersects(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2)) {
                    return true; // edges intersect
                }
            }
        }

        // 2. Check if polygon A contains a vertex of B
        if (pointPolygonIntersects(bx.get(0), by.get(0), ax, ay)) return true;

        // 3. Check if polygon B contains a vertex of A
        if (pointPolygonIntersects(ax.get(0), ay.get(0), bx, by)) return true;

        // No intersection
        return false;
    }

    public static Pair<List<Double>, List<Double>> generateBox(
            double cx, double cy, double width, double height, double theta
    ) {
        // Half-dimensions
        double hw = width / 2.0;
        double hh = height / 2.0;

        // Precompute sin and cos
        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        // Local corners (relative to center)
        double[] localX = {-hw, hw, hw, -hw};
        double[] localY = {-hh, -hh, hh, hh};

        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();

        for (int i = 0; i < 4; i++) {
            // Rotate and translate
            double x = cx + localX[i] * cos - localY[i] * sin;
            double y = cy + localX[i] * sin + localY[i] * cos;

            xs.add(x);
            ys.add(y);
        }

        return new Pair<>(xs, ys);
    }
}