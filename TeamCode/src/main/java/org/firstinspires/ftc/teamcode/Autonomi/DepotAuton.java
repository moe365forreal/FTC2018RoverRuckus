package org.firstinspires.ftc.teamcode.Autonomi;

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OtherStuff.MOEBot;
import org.firstinspires.ftc.teamcode.OtherStuff.SamplingMineral;

@Autonomous(name = "DepotAuton", group = "Autonomous")
public class DepotAuton extends Auton {
    @Override
    public void runActualSteps() {
        SamplingMineral goldMineral = dropAndFindMineral();
        String mineralLocation = depotSamplingMarkerRoutine(goldMineral.getLoc());
        tts.speak("Go MOE", TextToSpeech.QUEUE_ADD, null);
        robot.turnToDegreesAndStop(225);
        int frontDist;
        int rightDist;
        et.reset();

        boolean foundReadings = true;
        do {
            frontDist = robot.getFrontDistance();
            rightDist = robot.getRightDistance();

            if (et.seconds() > 4) {
                foundReadings = false;
                break;
            }

            pleaseDontCrash();
        }
        //TODO: check isAstarValid() method and add it
        while ((frontDist < 3 || frontDist > 25 || rightDist < 3 || rightDist > 25) && isAStarvalidPath(36, 66, 72 - rightDist, 72 - frontDist) && opModeIsActive());

        robot.turnToDegreesAndStop(90);
        waitMilliseconds(250);//ESSENTIAL DO NOT REMOVE
        robot.setTeamMarkerServo(0);
        waitMilliseconds(500);//ESSENTIAL DO NOT REMOVE
        // robot.debug("Front: ",robot.getFrontDistance()+" A* Units","Right: ",robot.getRightDistance()+" A* Units");
        robot.strafeRightInches(2, 0.5);
        robot.turnToDegreesAndStop(135);

        robot.moveBackwardInches(10, 0.5);
        robot.strafeLeftInches(6, 0.5);
        if (foundReadings) {
            debugPoint("made it past distance localization - using dist sensors");
            newAStarAlgo(36, 66, 72 - rightDist, 72 - frontDist, -1, -90, true);
            telemetry.addData("Front: ", frontDist);
            telemetry.addData("Right: ", rightDist);
            telemetry.update();
        } else {
            debugPoint("using fallback plan");
            tts.speak("Distance Servos don't work", TextToSpeech.QUEUE_ADD, null);
            switch (mineralLocation) {


                case "CENTER":
                    newAStarAlgo(36, 66, 63, 60, -1, -90, false);
                    break;
                case "LEFT":
                    newAStarAlgo(36, 66, 61, 59, -1, -90, false);
                    break;
                case "RIGHT":
                    newAStarAlgo(36, 66, 63, 61, -1, -90, false);
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
            robot.setMMSMotorServo(MOEBot.MMM_Auton_Position);
        }

        tts.speak("Go MOE", TextToSpeech.QUEUE_ADD, null);
        //     robot.parkingServo.setPosition(0);

        robot.setMMSMotorServo(MOEBot.MMM_Auton_Position);
        //robot.moveBackwardInches(3, 1);
    }
}
