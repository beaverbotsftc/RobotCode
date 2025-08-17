package org.firstinspires.ftc.teamcode.beaverbotv0;

import android.media.ResourceBusyException;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public abstract class Subsystem {
    private static Set<Resource> reserved = new HashSet<>();

    private Set<Resource> requirements;

    public Subsystem(Set<Resource> requirements) throws ResourceBusyException {
        this.requirements = requirements;

        if (!Collections.disjoint(requirements, reserved)) {
            throw new ResourceBusyException(String.format("Requirements are not disjoint with reserved resources : %s required and %s reserved.", requirements, reserved));
        }

        reserved.addAll(requirements);
    }
}
