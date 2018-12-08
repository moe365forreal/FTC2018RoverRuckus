package org.firstinspires.ftc.teamcode.DebugRoutines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomi.Auton;

import java.util.ArrayList;

@Autonomous(name = "Gyro Test", group = "Debug Routines")
public class GyroTest extends Auton {
    @Override
    public void runActualSteps() {
        while (opModeIsActive()) {
//            for (double ang : robot.getAngleOrientations()) {
//                telemetry.addData("angle: ", ang);
//            }
//            telemetry.addData("ROBOT ANGLE: ", robot.getDegreesTurned());
//            telemetry.addData("MMS ROT: ", robot.mmsRotationMotor.getCurrentPosition());
//            telemetry.update();
            telemetry.addData("location: ", getSamplingMineral().getLoc());
            telemetry.update();
        }
    }
}