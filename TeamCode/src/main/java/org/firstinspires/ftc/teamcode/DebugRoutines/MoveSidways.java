package org.firstinspires.ftc.teamcode.DebugRoutines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomi.Auton;

@Autonomous(name = "Move Sideways", group = "Debug Routines")
public class MoveSidways extends Auton {
    @Override
    public void runActualSteps() {
        robot.strafeRightInches(24.0f, 0.5);
    }
}
