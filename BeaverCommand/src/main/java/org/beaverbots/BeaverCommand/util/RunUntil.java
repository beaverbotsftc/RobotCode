package org.beaverbots.BeaverCommand.util;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandGroup;

import java.util.Arrays;

public class RunUntil extends CommandGroup {
    public RunUntil(Command primary, Command... commands) {
        super(prepend(primary, commands));
    }

    @Override
    public void start() {
        for (Command command : commands) {
            command.start();
        }
    }

    @Override
    public boolean periodic() {
        for (int i = commands.size() - 1; i >= 0; i--) {
            Command command = commands.get(i);
            if (command.periodic()) {
                command.stop();
                commands.remove(i);
                if (i == 0) {
                    return true;
                }
            }
        }
        return false;
    }

    @Override
    public void stop() {
        for (Command command : commands) {
            command.stop();
        }
    }
    private static <T> T[] prepend(T element, T[] array) {
        T[] newArr = Arrays.copyOf(array, array.length + 1);
        System.arraycopy(newArr, 0, newArr, 1, array.length);
        newArr[0] = element;
        return newArr;
    }

}