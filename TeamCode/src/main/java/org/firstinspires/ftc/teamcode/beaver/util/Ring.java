package org.firstinspires.ftc.teamcode.beaver.util;

public class Ring<T> {
    private final int size;
    private final Object[] ring;

    private int index = 0;
    private int count = 0;

    public Ring(int size) {
        this.size = size;
        ring = new Object[size];
    }

    public void append(T o) {
        ring[index] = o;
        index = (index + 1) % size;
        count++;
    }

    private int getIndex(int i) {
        // When not full yet, oldest starts at 0; when full, oldest is at index
        int base = (i >= 0 && count < size) ? 0 : index;
        return Math.floorMod(base + i, size);
    }

    public void set(int i, T o) {
        ring[getIndex(i)] = o;
    }

    // 0 -> oldest, 1 -> second oldest, -1 -> latest, etc.
    @SuppressWarnings("unchecked")
    public T get(int i) {
        return (T) ring[getIndex(i)];
    }

    public int search(T target) {
        int stored = getStoredCount();
        for (int i = 0; i < stored; i++) {
            if (get(i) == target) {
                return i;
            }
        }
        return -1;
    }

    public int getCount() {
        return count;
    }

    public int getStoredCount() {
        return Math.min(count, size);
    }
}
