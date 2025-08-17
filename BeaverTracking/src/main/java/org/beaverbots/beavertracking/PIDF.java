package org.beaverbots.beavertracking;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public final class PIDF {
    private final List<PIDFAxis> pidf;

    public PIDF(List<PIDFAxis> pidf) {
        this.pidf = pidf;
    }

    public int dimensions() {
        return pidf.size();
    }

    public List<Double> update(List<Double> error, List<Double> feedforward, double dt) {
        return IntStream.range(0, pidf.size()).mapToDouble(i -> pidf.get(i).update(error.get(i), feedforward.get(i), dt)).boxed().collect(Collectors.toList());
    }
}
