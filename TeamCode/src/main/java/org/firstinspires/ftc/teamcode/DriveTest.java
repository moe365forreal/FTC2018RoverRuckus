package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.OtherStuff.MOEBot;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.prefs.Preferences;


/**
 * This file is the new Swerve TeleOp for MOE 365 FTC
 * The file contains both the main drive code, as well as the operations of the secondary driver.
 */

@TeleOp(name = "DriveTest", group = "TeleOp")
public class DriveTest extends LinearOpMode {
    private double rawy, rawx1;
    private double fwd, str, rot;
    boolean fieldcentric = true;
    boolean yWasPressed = false;
    double angle, angleOffset;
    int toggle = 0;
    double FRP, FLP, BRP, BLP;
    ElapsedTime et = new ElapsedTime();
    public static final double ROBOT_RADIUS = 5;
    public static final int ENCODER_END_HANG_TICS = 5379;
    boolean overrideMMS = false;
    boolean wasBPressed = false;
    int mmsPositions[] = {0, 0, 0, 0};
    int manualOverridePosition = 0;
    int manualOverrideTicks = 0;
    int mmsMotorServoPosition = 0;
    WebcamName webcamName;

    double liftPower = 1;
    MOEBot robot;

    @Override
    public void runOpMode() {
        et.reset();
//        DcMotor topLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
//        DcMotor topRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
//        DcMotor bottomLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
//        DcMotor bottomRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        //   DcMotor liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

//        BNO055IMU imu;
//
//        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
//        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        gyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        gyroParameters.loggingEnabled = true;
//        gyroParameters.loggingTag = "IMU";
//        gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(gyroParameters);

//        Servo teamMarkerServo = hardwareMap.get(Servo.class, "markerServo");

//        robot = new MOEBot(imu, topLeftMotor, topRightMotor, bottomLeftMotor,
//                bottomRightMotor, ROBOT_RADIUS, this, teamMarkerServo,
//                hardwareMap.appContext.getSharedPreferences("RobotPrefs", 0), liftMotor, telemetry);
        //SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences("RobotPrefs",0);
        robot = new MOEBot(hardwareMap, telemetry, this, null, false);


        telemetry.addData("ready to start", "ready to start");
        telemetry.update();
        waitForStart();
        robot.parkingServo.setPosition(robot.PARKING_SERVO_STORED_POS);
        telemetry.addData("started", "started");
        telemetry.update();
        while (opModeIsActive()) {

            rawy = -deadzone(gamepad1.left_stick_y);
            rawx1 = deadzone(gamepad1.left_stick_x);
            rot = deadzone(gamepad1.right_stick_x);

            double angle = robot.getDegreesTurned() - angleOffset;

            if (gamepad1.a) {
                angleOffset = robot.getDegreesTurned();
                robot.parkingServo.setPosition(MOEBot.PARKING_SERVO_STORED_POS);
            }
            fwd = -rawx1 * Math.sin(Math.toRadians(angle)) + rawy * Math.cos(Math.toRadians(angle));
            str = rawx1 * Math.cos(Math.toRadians(angle)) + rawy * Math.sin(Math.toRadians(angle));

            if (fieldcentric) {
                FRP = fwd - str - rot;
                FLP = fwd + str + rot;
                BRP = fwd + str - rot;
                BLP = fwd - str + rot;
            }

            if (!fieldcentric) {
                FRP = rawy - rawx1 - rot;
                FLP = rawy + rawx1 + rot;
                BRP = rawy + rawx1 - rot;
                BLP = rawy - rawx1 + rot;
            }

            if (gamepad1.y) {
                yWasPressed = true;
            } else if (yWasPressed) {
                yWasPressed = false;
                fieldcentric = !fieldcentric;
            }

            double max = Math.abs(FRP) > 1 ? Math.abs(FRP) : 1;

            if (Math.abs(FLP) > max) {
                max = Math.abs(FLP);
            } else if (Math.abs(BRP) > max) {
                max = Math.abs(BRP);
            } else if (Math.abs(BLP) > max) {
                max = Math.abs(BLP);
            }
            FRP /= max;
            FLP /= max;
            BRP /= max;
            BLP /= max;
            FRP *= FRP * FRP;
            FLP *= FLP * FLP;
            BRP *= BRP * BRP;
            BLP *= BLP * BLP;

            if (gamepad1.right_trigger > .15)
                robot.liftMotor.setPower(liftPower);
            else if (gamepad1.left_trigger > 0.15)
                robot.liftMotor.setPower(-liftPower);
            else
                robot.liftMotor.setPower(0);

            if (gamepad1.dpad_up) {
                FRP = -.5;
                FLP = .5;
                BRP = .5;
                BLP = -.5;
            } else if (gamepad1.dpad_left) {
                FRP = .3;
                FLP = .3;
                BRP = .3;
                BLP = .3;
            } else if (gamepad1.dpad_right) {
                FRP = -.3;
                FLP = -.3;
                BRP = -.3;
                BLP = -.3;
            } else if (gamepad1.dpad_down) {
                FRP = .5;
                FLP = -.5;
                BRP = -.5;
                BLP = .5;
            }
            if (gamepad1.left_bumper) {
                FRP = 0.25;
                FLP = -0.25;
                BRP = 0.25;
                BLP = -0.25;
            }
            if (gamepad1.right_bumper) {
                FRP = -0.25;
                FLP = 0.25;
                BRP = -0.25;
                BLP = 0.25;
            }
            if (gamepad2.left_bumper) {
                FRP = 0.25;
                FLP = -0.25;
                BRP = 0.25;
                BLP = -0.25;
            }
            if (gamepad2.right_bumper) {
                FRP = -0.25;
                FLP = 0.25;
                BRP = -0.25;
                BLP = 0.25;
            }

            if (manualOverridePosition == 0) {
                if (gamepad2.left_trigger > 0.15) {
//                    manualOverridePosition = 1;
//                    manualOverrideTicks = robot.mmsRotationMotor.getCurrentPosition() +
//                            (int) Methods.scaleToRange(gamepad2.left_trigger, 0.15, 1, 400, 900);
//                    robot.mmsRotationMotor.setPower(0.4);
                    robot.mmsRotationMotor.setPower(Methods.scaleToRange(gamepad2.left_trigger, 0.15, 1, 0, 0.5));
                } else if (gamepad2.right_trigger > 0.15) {
//                    manualOverridePosition = 2;
//                    manualOverrideTicks =robot.mmsRotationMotor.getCurrentPosition() -
//                            (int) Methods.scaleToRange(gamepad2.right_trigger, 0.15, 1, 400, 900);
//                    robot.mmsRotationMotor.setPower(-0.4);
                    robot.mmsRotationMotor.setPower(-Methods.scaleToRange(gamepad2.right_trigger, 0.15, 1, 0, 0.5));
                } else {
                    robot.mmsRotationMotor.setPower(0);
                }
            }

            if (-gamepad2.right_stick_y > 0.15) {
                robot.mmsSlideMotor.setPower(Methods.scaleToRange(-gamepad2.right_stick_y, 0.15, 1, 0.1, 0.7));
            } else if (-gamepad2.right_stick_y < -0.15) {
                robot.mmsSlideMotor.setPower(-Methods.scaleToRange(gamepad2.right_stick_y, 0.15, 1, 0.1, 0.7));
            } else {
                robot.mmsSlideMotor.setPower(0);
            }

//            if (gamepad2.y) {
//                //robot.setMMSMotorServo(MOEBot.MMS_Middle_Position);
//                for (int i = 0; i < 4; i++) {
//                    if (i != 0) {
//                        mmsPositions[i] = 0;
//                    }
//                }
//                if (mmsPositions[0] == 0) {
//                    mmsPositions[0] = robot.mmsRotationMotor.getCurrentPosition() > MOEBot.MMS_Middle_Position ? 1 : 2;
//                }
//
//                if (mmsPositions[0] == 1) {
//                    robot.mmsRotationMotor.setPower(-0.6);
//                } else if (mmsPositions[0] == 2) {
//                    robot.mmsRotationMotor.setPower(0.6);
//                }
//            } else if (gamepad2.x) {
////                robot.setMMSMotorServo(MOEBot.MMS_Extended_Position);
//                for (int i = 0; i < 4; i++) {
//                    if (i != 1) {
//                        mmsPositions[i] = 0;
//                    }
//                }
//
//                if (mmsPositions[1] == 0) {
//                    mmsPositions[1] = robot.mmsRotationMotor.getCurrentPosition() > MOEBot.MMS_Extended_Position ? 1 : 2;
//                }
//
//                if (mmsPositions[1] == 1) {
//                    robot.mmsRotationMotor.setPower(-0.7);
//                } else if (mmsPositions[1] == 2) {
//                    robot.mmsRotationMotor.setPower(0.7);
//                }
//            } else if (!overrideMMS && gamepad2.a) {
////                robot.setMMSMotorServo(MOEBot.MMS_Before_Extended_Position);
//                for (int i = 0; i < 4; i++) {
//                    if (i != 2) {
//                        mmsPositions[i] = 0;
//                    }
//                }
//
//                if (mmsPositions[2] == 0) {
//                    mmsPositions[2] = robot.mmsRotationMotor.getCurrentPosition() > MOEBot.MMS_Before_Extended_Position ? 1 : 2;
//                }
//
//                if (mmsPositions[2] == 1) {
//                    robot.mmsRotationMotor.setPower(-0.7);
//                } else if (mmsPositions[2] == 2) {
//                    robot.mmsRotationMotor.setPower(0.7);
//                }
//            } else if (gamepad2.b) {
////                robot.setMMSMotorServo(0);
//                for (int i = 0; i < 4; i++) {
//                    if (i != 3) {
//                        mmsPositions[i] = 0;
//                    }
//                }
//
//                if (mmsPositions[3] == 0) {
//                    mmsPositions[3] = robot.mmsRotationMotor.getCurrentPosition() > 0 ? 1 : 2;
//                }
//
//                if (mmsPositions[3] == 1) {
//                    robot.mmsRotationMotor.setPower(-0.7);
//                } else if (mmsPositions[3] == 2) {
//                    robot.mmsRotationMotor.setPower(0.7);
//                }
//            }
//
//            if (mmsPositions[0] == 1 && robot.mmsRotationMotor.getCurrentPosition() < MOEBot.MMS_Middle_Position) {
//                mmsPositions[0] = 0;
//                robot.mmsRotationMotor.setPower(0);
//            } else if (mmsPositions[0] == 2 && robot.mmsRotationMotor.getCurrentPosition() > MOEBot.MMS_Middle_Position) {
//                mmsPositions[0] = 0;
//                robot.mmsRotationMotor.setPower(0);
//            }
//
//            if (mmsPositions[1] == 1 && robot.mmsRotationMotor.getCurrentPosition() < MOEBot.MMS_Extended_Position) {
//                mmsPositions[1] = 0;
//                robot.mmsRotationMotor.setPower(0);
//            } else if (mmsPositions[1] == 2 && robot.mmsRotationMotor.getCurrentPosition() > MOEBot.MMS_Extended_Position) {
//                mmsPositions[1] = 0;
//                robot.mmsRotationMotor.setPower(0);
//            }
//
//            if (mmsPositions[2] == 1 && robot.mmsRotationMotor.getCurrentPosition() < MOEBot.MMS_Before_Extended_Position) {
//                mmsPositions[2] = 0;
//                robot.mmsRotationMotor.setPower(0);
//            } else if (mmsPositions[2] == 2 && robot.mmsRotationMotor.getCurrentPosition() > MOEBot.MMS_Before_Extended_Position) {
//                mmsPositions[2] = 0;
//                robot.mmsRotationMotor.setPower(0);
//            }
//
//            if (mmsPositions[3] == 1 && robot.mmsRotationMotor.getCurrentPosition() < 0) {
//                mmsPositions[3] = 0;
//                robot.mmsRotationMotor.setPower(0);
//            } else if (mmsPositions[3] == 2 && robot.mmsRotationMotor.getCurrentPosition() > 0) {
//                mmsPositions[3] = 0;
//                robot.mmsRotationMotor.setPower(0);
//            }

//            if (manualOverridePosition == 1 && robot.mmsRotationMotor.getCurrentPosition() < manualOverrideTicks) {
//                manualOverridePosition = 0;
//                robot.mmsRotationMotor.setPower(0);
//            } else if (manualOverridePosition == 2 && robot.mmsRotationMotor.getCurrentPosition() > manualOverrideTicks) {
//                manualOverridePosition = 0;
//                robot.mmsRotationMotor.setPower(0);
//            }

            if (gamepad2.b && !wasBPressed) {
                wasBPressed = true;
                overrideMMS = !overrideMMS;
                robot.setMMSMotorServo(MOEBot.MMS_Extended_Position);
            } else if (!gamepad2.b) {
                wasBPressed = false;
            }


            robot.topRightMotor.setPower(FRP);
            robot.topLeftMotor.setPower(FLP);
            robot.bottomLeftMotor.setPower(BLP);
            robot.bottomRightMotor.setPower(BRP);


            telemetry.addData("Is Field Centric: ", fieldcentric);
            telemetry.addData("Angle: ", angle);
            telemetry.addData("Lift Encoder: ", robot.liftMotor.getCurrentPosition());
            telemetry.addData(manualOverridePosition + " :", manualOverrideTicks);
            telemetry.update();
        }
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

