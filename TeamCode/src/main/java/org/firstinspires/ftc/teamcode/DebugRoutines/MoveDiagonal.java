package org.firstinspires.ftc.teamcode.DebugRoutines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomi.Auton;

@Autonomous(name = "Move Diagonal", group = "Debug Routines")
public class MoveDiagonal extends Auton {
    @Override
    public void runActualSteps() {
        robot.moveNorthwestInches(70, 0.5);
        robot.moveNortheastInches(70, 0.5);
        robot.moveSoutheastInches(70, 0.5);
        robot.moveSouthwestInches(70, 0.5);
    }
}
