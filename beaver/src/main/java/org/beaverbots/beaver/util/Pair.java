package org.beaverbots.beaver.util;

public final class Pair<A, B> {
    public final A first;
    public final B second;

    public Pair(A first, B second) {
        this.first = first;
        this.second = second;
    }

    public static <A, B> Pair<A, B> of(A first, B second) {
        return new Pair<>(first, second);
    }

    @Override
    public String toString() {
        return "Pair(" + first + ", " + second + ")";
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Pair)) return false;

        Pair<?, ?> p = (Pair<?, ?>) o;

        return java.util.Objects.equals(first, p.first)
                && java.util.Objects.equals(second, p.second);
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(first, second);
    }
}