package org.beaverbots.beaver.command;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

public final class HardwareManager {
    private static final Set<String> claimedHardwareIds = new HashSet<>();
    private static final Map<String, HardwareDevice> hardwareCache = new HashMap<>();
    private static HardwareMap hardwareMap = null;

    private HardwareManager() {}

    public static void reset() {
        claimedHardwareIds.clear();
        hardwareCache.clear();
        hardwareMap = null;
    }

    public static void init(HardwareMap hardwareMap) {
        reset();
        HardwareManager.hardwareMap = hardwareMap;
    }

    public static <T extends HardwareDevice> T claim(String id) {
        if (claimedHardwareIds.contains(id))
            throw new AlreadyClaimedException(String.format("Hardware with ID: '%s' already claimed, cannot claim it again.", id));
        claimedHardwareIds.add(id);
        hardwareCache.put(id, hardwareMap.get(id));
        //noinspection unchecked
        return (T) hardwareCache.get(id);
    }

    public static <T extends HardwareDevice> T claim(Class<? extends T> classOrInterface, String id) {
        if (claimedHardwareIds.contains(id))
            throw new AlreadyClaimedException(String.format("Hardware with ID: '%s' already claimed, cannot claim it again.", id));
        claimedHardwareIds.add(id);
        hardwareCache.put(id, hardwareMap.get(classOrInterface, id));
        //noinspection unchecked
        return (T) hardwareCache.get(id);
    }

    public static void release(String id) {
        if (!claimedHardwareIds.contains(id))
            throw new IllegalStateException(String.format("Hardware with ID: '%s' not claimed, cannot release it.", id));
        claimedHardwareIds.remove(id);
    }
}