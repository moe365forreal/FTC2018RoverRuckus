package org.firstinspires.ftc.teamcode.DebugRoutines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomi.Auton;

@Autonomous(name = "Turn Right 180", group = "Debug Routines")
public class TurnRightOneEighty extends Auton {
    @Override
    public void runActualSteps() {
//        robot.turn(0.5);
//        waitMilliseconds(2000);
//        robot.moveAllMotors(0);
//        telemetry.addData("turn 1", "1");
        for (int i = 90; i <= 360; i += 90) {
            robot.turnToDegreesAndStop(i);
            waitMilliseconds(200);
        }
        robot.turnToDegreesAndStop(5);
//        telemetry.addData("turn 2", "1");
//        for (int i = -90; i >= -360; i-= 90) {
//            robot.turnDegrees(i);
//            waitMilliseconds(200);
//        }
//        telemetry.addData("turn 3", "1");
//        robot.turnDegrees(10);
//        waitMilliseconds(200);
//        telemetry.addData("turn4", "1");
//        robot.turnDegrees(-1);
//        waitMilliseconds(200);
    }
}
