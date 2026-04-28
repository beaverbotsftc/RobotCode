package org.beaverbots.beaver.util;

///  In order to bypass "effectively final" for lambda expressions
public class Box<T> {
    private T x;
    public Box(T x) {
        this.x = x;
    }

    public T get() {
        return x;
    }

    public void set(T x) {
        this.x = x;
    }
}
