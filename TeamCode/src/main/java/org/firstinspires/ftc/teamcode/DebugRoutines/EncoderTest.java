package org.firstinspires.ftc.teamcode.DebugRoutines;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomi.Auton;

import java.util.ArrayList;


@Autonomous(name = "Encoder Test", group = "Debug Routines")
public class EncoderTest extends Auton {
    @Override
    public void runActualSteps() {
        while (opModeIsActive()) {
            ArrayList<DcMotor> arr = robot.getDriveMotorList();
            for (int i = 0; i < 4; ++i) {
                telemetry.addData(String.valueOf(i) + ": ", arr.get(i).getCurrentPosition());
            }
            telemetry.update();
        }
    }
}