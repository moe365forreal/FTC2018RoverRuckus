package org.firstinspires.ftc.teamcode;

/**
 * Created by rohankanchana on 9/22/18.
 */

/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.OtherStuff.PointMap;

import java.util.List;


/**
 * This file is the new Swerve TeleOp for MOE 365 FTC
 * The file contains both the main drive code, as well as the operations of the secondary driver.
 */

@TeleOp(name = "JoeBotLocalizationTest", group = "TeleOp")
public class JoeBotLocalizationTest extends OpMode {
    class MOEBot {
        double x;
        double y;

        public MOEBot() {
            x = -1;
            y = -1;
        }

        public double getX() {
            return x;
        }

        public void setX(double x) {
            this.x = x;
        }

        public double getY() {
            return y;
        }

        public void setY(double y) {
            this.y = y;
        }

        public String toString() {
            return String.valueOf(this.x) + " " + String.valueOf(this.y);
        }
    }

    DcMotor leftMotor, rightMotor;
    ElapsedTime et = new ElapsedTime();
    List<VuforiaTrackable> allTrackables;
    List<VuforiaTrackable> allTrackablesPhone;

    OpenGLMatrix lastLocation = null;
    VuforiaTrackable lastMark = null;
    VuforiaLocalizer vuforia;
    //VuforiaLocalizer vuforia2;
    WebcamName webcamName;

    double[] xyz;
    MOEBot robot;

    void waitMilliseconds(double ms) {
        et.reset();

        while (et.milliseconds() < ms) {
        }
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//        et.reset();
////        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
////        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
//
//        robot = new MOEBot();
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        parameters.vuforiaLicenseKey =
//                "AVmJqSr/////AAABmeGcV96ftk3KgMDBC6fRLL4Qn/iAktqBNx29ewqo4NPP1TWeg6cpqfRV3dQ3vtDc4LxnLDckSBYdv9v3b1pe2Rq1v+stEVsKhckekqcdpbffWqz+QAyFJF7Mg9q/vOVBXIjvH7CPFVVKiM/M+J3vFw87SFxJKQlZuOM0WGi0hMJf8CE21ZJKh3h9tCg+/dqEux2FmB7XpezHFFeIqE8EK/3skt8Gjui+ywRSmgyzr+C3GswiIWsUn3YYCS6udgB8O6ntF5RZyrq4dQJxrdV1Mh1P7dlpGgyml+yiBAKDgoHZPiHKBx1TIY0Gg9QBebnuHdMvEOhK9oOJqtlR7XBO+fRJrXSCBY+9zBHRpZ6zSE0P";
//
//        telemetry.addData(">", "license keys");
//        telemetry.update();
//        waitMilliseconds(500);
//        parameters.cameraName = webcamName;
//
//        ClassFactory instance = ClassFactory.getInstance();
//        vuforia = instance.createVuforia(parameters);
//
//        VuforiaTrackables roverRuckusVuMarks = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
//
//        roverRuckusVuMarks.get(0).setName("Rover");
//        roverRuckusVuMarks.get(1).setName("Moon");
//        roverRuckusVuMarks.get(2).setName("Mars");
//        roverRuckusVuMarks.get(3).setName("Galaxy");
//
//        allTrackables = new ArrayList<>();
//        allTrackables.addAll(roverRuckusVuMarks);
//
//        float mmPerInch        = 25.4f;
//        float mmBotWidth       = 17 * mmPerInch;            // ... or whatever is right for your robot
//        float mmBotLength      = 16 * mmPerInch;            // ... or whatever is right for your robot
//        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels=
//
//        roverRuckusVuMarks.activate();
//
//        waitForStart(); //-------------------------------------------------------------------------------------------------------------------------------------------
//
//
//    }

    public void driveToPoint(double x, double y) {
        while (Math.abs(robot.getX() - x) > 1 && Math.abs(robot.getY() - y) < 1) {
            for (int i = 0; i < 4; i++) {
                VuforiaTrackable trackable = allTrackables.get(i);

                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    lastMark = trackable;
                    telemetry.addData(lastMark.getName() + " x:", PointMap.getMark(lastMark.getName())[0]);
                    telemetry.addData(lastMark.getName() + " y:", PointMap.getMark(lastMark.getName())[1]);
                }


                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;

                    xyz = getXYZFromMatrix(robotLocationTransform);

                    if (trackable.getName().equals("Rover")) {
                        int[] mark = PointMap.getMark("Rover");
                        robot.setX(72 - xyz[2] / 50);
                        robot.setY(-xyz[0] / 50 + 36);
                    } else if (trackable.getName().equals("Mars")) {
                        int[] mark = PointMap.getMark("Mars");
                        robot.setX(xyz[0] / 50 + 36);
                        robot.setY(72 - xyz[2] / 50);
                    } else if (trackable.getName().equals("Moon")) {
                        int[] mark = PointMap.getMark("Moon");
                        robot.setX(xyz[2] / 50);
                        robot.setY(xyz[0] / 50 + 36);
                    } else if (trackable.getName().equals("Galaxy")) {
                        int[] mark = PointMap.getMark("Galaxy");
                        robot.setX(-xyz[0] / 50 + 36);
                        robot.setY(xyz[2] / 50);
                    }
                }
            }
        }
    }

    boolean buttonPressed = false;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
//        leftMotor.setPower(deadzone(-gamepad1.left_stick_y));
//        rightMotor.setPower(deadzone(-gamepad1.right_stick_y));

//        if (gamepad1.a && !buttonPressed) {
//            captureFrameToFile();
//        }
        buttonPressed = gamepad1.a;

        for (int i = 0; i < 4; i++) {
            VuforiaTrackable trackable = allTrackables.get(i);

            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                lastMark = trackable;
                telemetry.addData(lastMark.getName() + " x:", PointMap.getMark(lastMark.getName())[0]);
                telemetry.addData(lastMark.getName() + " y:", PointMap.getMark(lastMark.getName())[1]);
            }


            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;

                xyz = getXYZFromMatrix(robotLocationTransform);

                if (trackable.getName().equals("Rover")) {
                    int[] mark = PointMap.getMark("Rover");
                    robot.setX(72 - xyz[2] / 50);
                    robot.setY(-xyz[0] / 50 + 36);
                } else if (trackable.getName().equals("Mars")) {
                    int[] mark = PointMap.getMark("Mars");
                    robot.setX(xyz[0] / 50 + 36);
                    robot.setY(72 - xyz[2] / 50);
                } else if (trackable.getName().equals("Moon")) {
                    int[] mark = PointMap.getMark("Moon");
                    robot.setX(xyz[2] / 50);
                    robot.setY(xyz[0] / 50 + 36);
                } else if (trackable.getName().equals("Galaxy")) {
                    int[] mark = PointMap.getMark("Galaxy");
                    robot.setX(-xyz[0] / 50 + 36);
                    robot.setY(xyz[2] / 50);
                }
            }
        }

        /**
         * Provide feedback as to where the robot was last located (if we know).
         */
        if (lastLocation != null) {
            //RobotLog.vv("JoeBotTest", "robot=%s", format(lastLocation));
            telemetry.addData("Pos", format(lastLocation));
            telemetry.addData("Robot pos:", robot.toString());
        } else {
            telemetry.addData("Pos", "Unknown");
        }

        telemetry.update();
    }


    @Override
    public void stop() {//RUNS ONCE ON STOP
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
    }

    public double[] getXYZFromMatrix(OpenGLMatrix matrix) {
        String str = matrix.formatAsTransform();

        int state = 4;
        StringBuilder holder = new StringBuilder();
        double[] xyz = new double[3];
        for (int i = str.length() - 1; i >= 0; i--) {
            char c = str.charAt(i);

            if (c == '}' || c == ' ') {
                state -= 1;

                if (state < 3) {
                    xyz[state] = Double.valueOf(holder.toString());
                    holder = new StringBuilder();
                }
            } else if (c == '{') {
                xyz[state - 1] = Double.valueOf(holder.toString());
                break;
            } else if (state <= 3) {
                holder.insert(0, String.valueOf(c));
            }
        }

        return xyz;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public double deadzone(double value) {
        if ((value < .15 && value > -.15)) {
            value = 0;
        } else if (value > .85) {
            value = 1;
        } else if (value < -.85) {
            value = -1;
        }
        return value;
    }
}

//motor 3 is front left, motor 2 is front right, motor 1 is back left, motor 0 is back right

