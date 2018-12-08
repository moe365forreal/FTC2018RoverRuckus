package org.firstinspires.ftc.teamcode.Autonomi;

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OtherStuff.MOEBot;
import org.firstinspires.ftc.teamcode.OtherStuff.SamplingMineral;


/**
 * This file is the new Swerve TeleOp for MOE 365 FTC
 * The file contains both the main drive code, as well as the operations of the secondary driver.
 */

@Autonomous(name = "RedCraterAuton", group = "Autonomous")
public class RedCraterAuton extends Auton {
    @Override
    public void runActualSteps() {
        SamplingMineral goldMineral = dropAndFindMineral();
        debugPoint("found mineral");
        int backwardsInches = craterSamplingRoutine(goldMineral.getLoc());
        debugPoint("found mineral");
        robot.moveBackwardInches(backwardsInches, 0.6);
        debugPoint("found mineral");
        robot.turnToDegreesAndStop(245);
        robot.moveAllMotors(0.52);
        et.reset();
        //  while(et.seconds()<0.5){};
        int xPoint = -1;
        int yPoint = -1;
        while (lastMark == null && robot.getX() < 66 && opModeIsActive()) {
            telemetry.addData("" + robot.getX() + ",", robot.getY());
            telemetry.update();
            if (et.seconds() > 4) {
                et.reset();
                xPoint = 5;
                yPoint = 37;
                break;
            }
        }
        robot.stop();
        newAStarAlgo(5, 12, xPoint, yPoint, 315, 180);
//newAStarAlgo(5,12,13, 40,315,180);
        int frontDist;
        int rightDist;
        et.reset();

        boolean foundReadings = true;
        do {
            frontDist = robot.getFrontDistance();
            rightDist = robot.getRightDistance();

            if (et.seconds() > 1.5) {
                foundReadings = false;
                break;
            }

            pleaseDontCrash();
        }
        while ((frontDist < 3 || frontDist > 25 || rightDist < 3 || rightDist > 25) && isAStarvalidPath(66, 32, 72 - rightDist, 72 - frontDist) && opModeIsActive());

        robot.turnToDegreesAndStop(225);
        waitMilliseconds(250);//ESSENTIAL DO NOT REMOVE
        robot.setTeamMarkerServo(0);
        waitMilliseconds(500);//ESSENTIAL DO NOT REMOVE
        // robot.debug("Front: ",robot.getFrontDistance()+" A* Units","Right: ",robot.getRightDistance()+" A* Units");

        if (foundReadings) {
            newAStarAlgo(66, 32, 72 - rightDist, 72 - frontDist, 315, 0);
            telemetry.addData("Front: ", frontDist);
            telemetry.addData("Right: ", rightDist);
            telemetry.update();
        } else {
            switch (goldMineral.getLoc()) {
                case "CENTER":
                    newAStarAlgo(66, 32, 66, 60, 315, 0);
                    break;
                case "LEFT":
                    newAStarAlgo(66, 32, 66, 60, 315, 0);
                    break;
                case "RIGHT":
                    newAStarAlgo(66, 32, 66, 60, 315, 0);
                    break;
            }

            // debugPoint("Front Distance", );
//        switch (mineralLocation){
//            case "CENTER":
//                newAStarAlgo(31, 66, 63, 60);
//            case "LEFT":
//                newAStarAlgo(31,66,59,65);
//            case "RIGHT":
//                newAStarAlgo(31, 66, 63,64);
//     //   robot.strafeLeftInches(4,0.3);

        }

        robot.setMMSMotorServo(MOEBot.MMM_Auton_Position, 0.7);
        robot.moveBackwardInches(2, 0.5);
        tts.speak("Go MOE", TextToSpeech.QUEUE_ADD, null);
    }
}