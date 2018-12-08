/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomi;

import android.speech.tts.TextToSpeech;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.OtherStuff.MOEBot;
import org.firstinspires.ftc.teamcode.OtherStuff.PointMap;
import org.firstinspires.ftc.teamcode.OtherStuff.SamplingMineral;

@Autonomous(name = "BlueCraterAuton", group = "Autonomous")
public class BlueCraterAuton extends Auton {
    @Override
    public void runActualSteps() {
        SamplingMineral goldMineral = dropAndFindMineral();
        debugPoint("found mineral");
        int backwardsInches = craterSamplingRoutine(goldMineral.getLoc());
        debugPoint("did sampling routine");
        robot.moveBackwardInches(backwardsInches, 0.6);
        debugPoint("moved backwards");
        if (goldMineral.getLoc().equals("CENTER")) {
            robot.turnToDegreesAndStop(246);
        } else {
            robot.turnToDegreesAndStop(257);
        }
        robot.moveAllMotors(0.55);
        while (lastMark == null && robot.getX() < 66 && opModeIsActive()) {
            telemetry.addData("" + robot.getX() + ",", robot.getY());
            telemetry.update();
        }
        robot.stop();
        newAStarAlgo(66, 60, -1, -1, -1, 0, false);

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
        while ((frontDist < 3 || frontDist > 25 || rightDist < 3 || rightDist > 25) && opModeIsActive());

        robot.veryFastTurnToDegreesAndStop(180);
        waitMilliseconds(250);//ESSENTIAL DO NOT REMOVE
        robot.setTeamMarkerServo(0);
        waitMilliseconds(500);//ESSENTIAL DO NOT REMOVE
        // robot.debug("Front: ",robot.getFrontDistance()+" A* Units","Right: ",robot.getRightDistance()+" A* Units");

        if (foundReadings) {
            newAStarAlgo(66, 32, 72 - rightDist, 72 - frontDist, 315, 0, false);
            telemetry.addData("Front: ", frontDist);
            telemetry.addData("Right: ", rightDist);
            telemetry.update();
            robot.setMMSMotorServo(MOEBot.MMM_Auton_Position, 0.7);
            robot.moveBackwardInches(2, 0.5);
        } else {
            switch (goldMineral.getLoc()) {
                case "CENTER":
                    newAStarAlgo(66, 32, 66, 60, 315, 0, false);
                    break;
                case "LEFT":
                    newAStarAlgo(66, 32, 66, 60, 315, 0, false);
                    break;
                case "RIGHT":
                    newAStarAlgo(66, 32, 66, 60, 315, 0, false);
                    break;
            }
            robot.setMMSMotorServo(MOEBot.MMM_Auton_Position, 0.7);
            robot.moveBackwardInches(2, 0.5);

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

        tts.speak("Go MOE", TextToSpeech.QUEUE_ADD, null);
    }
}
