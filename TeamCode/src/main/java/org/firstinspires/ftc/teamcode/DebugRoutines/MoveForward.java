package org.firstinspires.ftc.teamcode.DebugRoutines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomi.Auton;

@Autonomous(name = "Move Forward", group = "Debug Routines")
public class MoveForward extends Auton {
    @Override
    public void runActualSteps() {

        robot.moveForwardInches(24, 0.5);
    }
}
