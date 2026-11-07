package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;

public class Parallel extends SimultaneousCommandGroup {
    public Parallel(Command... commands) {
        super(commands);
    }

    @Override
    public void start() {
        super.start();

        for (Command command : commands) {
            command.start();
        }
    }

    @Override
    public boolean periodic() {
        for (int i = 0; i < commands.size(); i++) {
            Command command = commands.get(i);
            if (command.periodic()) {
                command.stop();
                commands.remove(i);
                i--;
            }
        }
        return commands.isEmpty();
    }

    @Override
    public void stop() {
        for (Command command : commands) {
            command.stop();
        }
    }
}
