package org.firstinspires.ftc.teamcode.Autonomi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OtherStuff.SamplingMineral;

@Autonomous(name = "RedDepotAuton", group = "Autonomous")
public class RedDepotAuton extends Auton {
    @Override
    public void runActualSteps() {
        SamplingMineral goldMineral = dropAndFindMineral();
        String mineralLocation = depotSamplingMarkerRoutine(goldMineral.getLoc());
        robot.turnToDegreesAndStop(225);
//        switch (mineralLocation){
//            case "CENTER":
//                newAStarAlgo(31, 66, 63, 60);
//                robot.strafeLeftInches(4,0.3);
//
//        }
    }
}