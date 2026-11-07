package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;
import org.firstinspires.ftc.teamcode.beaver.util.Beaver;

public class RunUntil extends SimultaneousCommandGroup {
    public RunUntil(Command primary, Command... commands) {
        super(Beaver.prepend(primary, commands));
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
                if (i == 0) {
                    return true;
                }
                i--;
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
}
