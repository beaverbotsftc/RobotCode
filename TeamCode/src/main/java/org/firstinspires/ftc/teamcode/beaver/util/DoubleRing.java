package org.firstinspires.ftc.teamcode.beaver.util;

public class DoubleRing {
    private final int size;
    private final double[] ring;

    private int index = 0;
    private int count = 0;

    public DoubleRing(int size) {
        this.size = size;
        ring = new double[size];
    }

    public void append(double d) {
        ring[index] = d;
        index = (index + 1) % size;
        count++;
    }

    private int getIndex(int i) {
        // When not full yet, oldest starts at 0; when full, oldest is at index
        int base = (i >= 0 && count < size) ? 0 : index;
        return Math.floorMod(base + i, size);
    }

    public void set(int i, double n) {
        ring[getIndex(i)] = n;
    }

    // 0 -> oldest, 1 -> second oldest, -1 -> latest, etc.
    public double get(int i) {
        return ring[getIndex(i)];
    }

    public int search(double target) {
        int stored = getStoredCount();
        for (int i = 0; i < stored; i++) {
            if (get(i) == target) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Binary search assuming monotonically increasing elements.
     * If the exact value doesn't exist, returns the index of the value right under it.
     * If that index would be -1, returns 0.
     */
    public int binarySearch(double target) {
        int low = 0;
        int high = getStoredCount() - 1;

        while (low <= high) {
            int mid = (low + high) >>> 1;
            double val = get(mid);

            if (val < target) {
                low = mid + 1;
            } else if (val > target) {
                high = mid - 1;
            } else {
                return mid; // Exact match found
            }
        }

        return Math.max(0, high);
    }

    public int getCount() {
        return count;
    }

    public int getStoredCount() {
        return Math.min(count, size);
    }
}