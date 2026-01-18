package org.firstinspires.ftc.teamcode.config.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Subsystem;

/**
 * FILE: OpModeCommand.java
 * PURPOSE: A helper class that automatically runs the CommandScheduler.
 * Bridge between OpMode and Commands.
 */
public abstract class OpModeCommand extends OpMode {

    public void reset() { CommandScheduler.getInstance().reset(); }
    public void run() { CommandScheduler.getInstance().run(); }
    public void schedule(Command... commands) { CommandScheduler.getInstance().schedule(commands); }

    @Override public void init() { initialize(); }
    @Override public void loop() { run(); }
    @Override public void stop() { reset(); }

    public abstract void initialize();
}