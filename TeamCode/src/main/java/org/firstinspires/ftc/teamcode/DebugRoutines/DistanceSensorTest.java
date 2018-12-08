package org.firstinspires.ftc.teamcode.DebugRoutines;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomi.Auton;
import org.firstinspires.ftc.teamcode.OtherStuff.MOEBot;

@Autonomous(name = "Distance Sensor Test", group = "Debug Routines")
public class DistanceSensorTest extends Auton {
    @Override
    public void runActualSteps() {
        robot = new MOEBot(hardwareMap, telemetry, this, null, true);
        while (opModeIsActive()) {
            telemetry.addData("Front Sensor: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Sensor: ", robot.rightDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Front Corrected: ", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) + robot.frontSensorOffset);
            telemetry.addData("Right Corrected: ", robot.rightDistanceSensor.getDistance(DistanceUnit.INCH) + robot.rightSensorOffset);
            telemetry.addData("Front Sensor: ", robot.getFrontDistance() + " A* Units");
            telemetry.addData("Right Sensor: ", robot.getRightDistance() + " A* Units");
            telemetry.update();
        }
    }
}
