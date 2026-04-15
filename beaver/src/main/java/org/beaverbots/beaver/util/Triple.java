package org.beaverbots.beaver.util;

public final class Triple<A, B, C> {
    public final A first;
    public final B second;
    public final C third;

    public Triple(A first, B second, C third) {
        this.first = first;
        this.second = second;
        this.third = third;
    }

    public static <A, B, C> Triple<A, B, C> of(A first, B second, C third) {
        return new Triple<>(first, second, third);
    }

    @Override
    public String toString() {
        return "Triple(" + first + ", " + second + ", " + third + ")";
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Triple)) return false;

        Triple<?, ?, ?> t = (Triple<?, ?, ?>) o;

        return java.util.Objects.equals(first, t.first)
                && java.util.Objects.equals(second, t.second)
                && java.util.Objects.equals(third, t.third);
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(first, second, third);
    }
}