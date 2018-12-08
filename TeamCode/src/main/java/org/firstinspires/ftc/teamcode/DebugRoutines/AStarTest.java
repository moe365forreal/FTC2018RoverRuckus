package org.firstinspires.ftc.teamcode.DebugRoutines;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomi.Auton;
import org.firstinspires.ftc.teamcode.OtherStuff.AStarAlgorithmm;
import org.firstinspires.ftc.teamcode.OtherStuff.RobotMovement;

@Autonomous(name = "AStarTest", group = "Autonomous")
public class AStarTest extends Auton {
    private static final String ASTAR_COORDINATES = "a1x";

    @Override
    public void runActualSteps() {
//        String aStarStr = hardwareMap.appContext.getSharedPreferences("RobotPrefs", 0).getString(ASTAR_COORDINATES, "error");
//        if (aStarStr.equals("error")) {
//            telemetry.addData("Error", "error");
//            telemetry.update();
//        } else {
//            String[] arr = aStarStr.split(",");
//            int coordinates[] = new int[4];
//            for (int i = 0; i < arr.length; i++) {
//                coordinates[i] = Integer.valueOf(arr[i]);
//            }
//            newAStarAlgo(coordinates[0], coordinates[1]);
//        }
        String coords = hardwareMap.appContext.getSharedPreferences("RobotPrefs", 0).getString(ASTAR_COORDINATES, "");
        String[] arr = coords.split(",");

        AStarAlgorithmm algorithmm = new AStarAlgorithmm(Integer.valueOf(arr[1]), Integer.valueOf(arr[0]), Integer.valueOf(arr[3]), Integer.valueOf(arr[2]));
        algorithmm.process();
        telemetry.addData("Made it? ", algorithmm.getRobotMovements().size());
        for (RobotMovement movement : algorithmm.getRobotMovements()
                ) {
            telemetry.addData("" + movement.direction, "" + movement.inches);
        }
        telemetry.update();
        waitMilliseconds(3000);

//        newAStarAlgo(Integer.valueOf(arr[0]), Integer.valueOf(arr[1]), -1, -1, -1, 0);
    }
}
