package org.firstinspires.ftc.teamcode.beaver.util;

public class IntRing {
    private final int size;
    private final int[] ring;

    private int index = 0;
    private int count = 0;

    public IntRing(int size) {
        this.size = size;
        ring = new int[size];
    }

    public void append(int n) {
        ring[index] = n;
        index = (index + 1) % size;
        count++;
    }

    private int getIndex(int i) {
        // When not full yet, oldest starts at 0; when full, oldest is at index
        int base = (i >= 0 && count < size) ? 0 : index;
        return Math.floorMod(base + i, size);
    }

    public void set(int i, int n) {
        ring[getIndex(i)] = n;
    }


    // 0 -> oldest, 1 -> second oldest, -1 -> latest, etc.
    public int get(int i) {
        return ring[getIndex(i)];
    }

    public int search(int target) {
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
    public int binarySearch(int target) {
        int low = 0;
        int high = getStoredCount() - 1;

        while (low <= high) {
            int mid = (low + high) >>> 1;
            int val = get(mid);

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