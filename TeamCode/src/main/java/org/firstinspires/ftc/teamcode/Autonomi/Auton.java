package org.firstinspires.ftc.teamcode.Autonomi;

import android.os.IInterface;
import android.speech.tts.TextToSpeech;
import android.support.annotation.Nullable;
import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.OtherStuff.AStarAlgorithm;
import org.firstinspires.ftc.teamcode.OtherStuff.AStarAlgorithmm;
import org.firstinspires.ftc.teamcode.OtherStuff.MOEBot;
import org.firstinspires.ftc.teamcode.OtherStuff.Pair;
import org.firstinspires.ftc.teamcode.OtherStuff.Pathing;
import org.firstinspires.ftc.teamcode.OtherStuff.RobotMovement;
import org.firstinspires.ftc.teamcode.OtherStuff.RobotMovementPair;
import org.firstinspires.ftc.teamcode.OtherStuff.SamplingMineral;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_GOLD_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.LABEL_SILVER_MINERAL;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodRoverRuckus.TFOD_MODEL_ASSET;


//@Autonomous(name="RedCraterAuton", group="Autonomous")
@SuppressWarnings("StatementWithEmptyBody")
public class Auton extends LinearOpMode {
    public final ElapsedTime et = new ElapsedTime();
    //   List<VuforiaTrackable> allTrackablesPhone;
    private static final String ROBOT_RADIUS_KEY = "robot_radius_key";
    //    public static final int rotation_margin = 5;
//    public static final double ROBOT_RADIUS = 5;
    private final AtomicBoolean aStarIsRunning = new AtomicBoolean(false);
    private final AtomicBoolean restartAStar = new AtomicBoolean(false);
    private final AtomicInteger aStarLimit = new AtomicInteger(3);
    private final AtomicInteger aStarCount = new AtomicInteger(0);
    TextToSpeech tts;
    public VuforiaTrackable lastMark = null;
    //WebcamName webcamName;
    private TFObjectDetector tfod;
    //double[] xyz;
    public MOEBot robot;

    private String lastTagName = "NULL";
    private boolean enableTTS = true;

    class MOEListener extends VuforiaTrackableDefaultListener {
        int numsOfTimesCalled = 0;
        final VuforiaTrackable trackable;

        MOEListener(@Nullable VuforiaTrackable trackable) {
            super(trackable);
            this.trackable = trackable;
        }

        @Override
        public void onTracked(TrackableResult trackableResult, CameraName cameraName, Camera camera, VuforiaTrackable child) {
            super.onTracked(trackableResult, cameraName, camera, child);

            /*DONT' USE CHILD VARIABLE -- VUFORIA IS STUPID*/

            synchronized (this.lock) {
                if (trackable == null) {
                    return;
                }
                if (trackable.getName().equals(lastTagName)) {
                    lastMark = trackable;
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    robot.localizePosition(trackable, robotLocationTransform);
                    return;
                }

                telemetry.addData(trackable.getName(), numsOfTimesCalled);
                telemetry.addData(trackable.getName(), lastTagName);
                telemetry.update();
                numsOfTimesCalled += 1;

                lastTagName = trackable.getName();

                lastMark = trackable;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();

                if (robotLocationTransform != null) {
                    //cameraL = ((VuforiaTrackableDefaultListener) trackable.getListener()).getFtcCameraFromTarget();


                    //xyz = getXYZFromMatrix(robotLocationTransform);

                    robot.localizePosition(trackable, robotLocationTransform);

                    if (aStarIsRunning.get()) {
                        restartAStar.set(true);
                    }
//
//                if (trackable.getName().equals("Rover")) {
//                    int[] mark = PointMap.getMark("Rover");
//                    robot.setX(72 - xyz[2] / 50);
//                    robot.setY(-xyz[0] / 50 + 36);
//                } else if (trackable.getName().equals("Mars")) {
//                    int[] mark = PointMap.getMark("Mars");
//                    robot.setX(xyz[0] / 50 + 36);
//                    robot.setY(72 - xyz[2] / 50);
//                } else if (trackable.getName().equals("Moon")) {
//                    int[] mark = PointMap.getMark("Moon");
//                    robot.setX(xyz[2] / 50);
//                    robot.setY(xyz[0] / 50 + 36);
//                } else if (trackable.getName().equals("Galaxy")) {
//                    int[] mark = PointMap.getMark("Galaxy");
//                    robot.setX(-xyz[0] / 50 + 36);
//                    robot.setY(xyz[2] / 50);
//                }
                }
            }
        }

        @Override
        public void onNotTracked() {
            super.onNotTracked();

//            telemetry.addData("not tracked", trackable.getName());

            assert trackable != null;
            if (trackable.getName().equals(lastTagName)) {
                lastTagName = "NULL";
            }
        }
    }

    protected void waitMilliseconds(double ms) {
        et.reset();

        while (opModeIsActive() && et.milliseconds() < ms) {
        }
    }


    //public GoldAlignDetector detector;
//    boolean buttonPressed = false;
    String currentMarkName = "mars";

    @Override
    public void runOpMode() {
        et.reset();
        if (enableTTS) {
            tts = new TextToSpeech(hardwareMap.appContext, null);
            //tts.setLanguage(Locale.US);

            tts.setLanguage(Locale.US);


        }
//        DcMotor topLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
//        DcMotor topRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
//        DcMotor bottomLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
//        DcMotor bottomRightMotor = hardwareMap.get(DcMotor.class, "backRight");
//        DcMotor liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
//        Servo teamMarkerServo = hardwareMap.get(Servo.class, "markerServo");
//        //DcMotor liftMotor = hardwareMap.get(DcMotor.class,"lift");
//
//
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
//        while (!imu.isGyroCalibrated()) {
//        }

        float prefRobotRadius = hardwareMap.appContext.getSharedPreferences("RobotPrefs", 0).getFloat(ROBOT_RADIUS_KEY, -1);
        if (prefRobotRadius == -1) {
            Toast.makeText(hardwareMap.appContext, "Error with radius, defaulting to 5.", Toast.LENGTH_SHORT).show();
            prefRobotRadius = 5.0f;
        }

//        robot = new MOEBot(imu, topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor,
//                prefRobotRadius, this, teamMarkerServo,
//                hardwareMap.appContext.getSharedPreferences("RobotPrefs", 0), liftMotor, telemetry);
        robot = new MOEBot(hardwareMap, telemetry, this, tts, true);
        robot.liftMotor.setTargetPosition(0);

        robot.setTeamMarkerServo(1);
        robot.parkingServo.setPosition(MOEBot.PARKING_SERVO_STORED_POS);
        //CAMERA INITIALIZATION
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "AVmJqSr/////AAABmeGcV96ftk3KgMDBC6fRLL4Qn/iAktqBNx29ewqo4NPP1TWeg6cpqfRV3dQ3vtDc4LxnLDckSBYdv9v3b1pe2Rq1v+stEVsKhckekqcdpbffWqz+QAyFJF7Mg9q/vOVBXIjvH7CPFVVKiM/M+J3vFw87SFxJKQlZuOM0WGi0hMJf8CE21ZJKh3h9tCg+/dqEux2FmB7XpezHFFeIqE8EK/3skt8Gjui+ywRSmgyzr+C3GswiIWsUn3YYCS6udgB8O6ntF5RZyrq4dQJxrdV1Mh1P7dlpGgyml+yiBAKDgoHZPiHKBx1TIY0Gg9QBebnuHdMvEOhK9oOJqtlR7XBO+fRJrXSCBY+9zBHRpZ6zSE0P";

        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables roverRuckusVuMarks = vuforia.loadTrackablesFromAsset("RoverRuckus");

        roverRuckusVuMarks.get(0).setName("Rover");
        roverRuckusVuMarks.get(1).setName("Moon");
        roverRuckusVuMarks.get(2).setName("Mars");
        roverRuckusVuMarks.get(3).setName("Galaxy");

        float mmRobotLength = 431.8F;
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(0, 0, mmRobotLength / 2);
        for (int i = 0; i < 4; i++) {
            roverRuckusVuMarks.get(i).setListener(new MOEListener(roverRuckusVuMarks.get(i)));
            assert parameters.cameraName != null;
            ((MOEListener) roverRuckusVuMarks.get(i).getListener())
                    .setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        }

        List<VuforiaTrackable> allTrackables = new ArrayList<>(roverRuckusVuMarks);

        float mmPerInch = 25.4f;
//        float mmBotWidth = 17 * mmPerInch;            // ... or whatever is right for your robot
//        float mmBotLength = 16 * mmPerInch;            // ... or whatever is right for your robot
//        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels=

        roverRuckusVuMarks.activate();
        tts.speak("Initialized Vuforia", TextToSpeech.QUEUE_ADD, null);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // int[] curMarkCoords = PointMap.getMark(currentMarkName);

        //
        if (tfod != null) {
            tfod.activate();
        }
        if (tts != null) {
            tts.speak("Initialized Tensor Flow", TextToSpeech.QUEUE_ADD, null);
        }
        //7        telemetry.addData(">", "license keys");
        telemetry.addData("Robot radius:", prefRobotRadius);
        telemetry.update();
        if (tts != null) {
            tts.speak("Initialization Complete", TextToSpeech.QUEUE_ADD, null);
        }
//        waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
//        while(tfod.getRecognitions().size()<2) {
//            robot.turnDegrees(10);
        runActualSteps();
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    SamplingMineral dropAndFindMineral() {
        robot.runLiftRoutine();
        //        while (robot.liftMotor.isBusy() && opModeIsActive()) {}
        //        robot.turnDegrees(-100);
        //        waitMilliseconds(300);
        robot.turnDegrees(-75);
        //        turnUntilTwoMinerals(false);
        //        tfod.activate();/
        SamplingMineral goldMineral = getSamplingMineral();
        while (goldMineral == null && opModeIsActive()) {
            waitMilliseconds(100);
            goldMineral = getSamplingMineral();
        }
        return goldMineral;
    }


    int craterSamplingRoutine(String mineralLocation) {
        telemetry.addData(mineralLocation, " :LABEL");
        telemetry.update();
        switch (mineralLocation) {
            case "LEFT":
                robot.turnDegrees(-35);
                robot.moveForwardInches(26, 0.5);
                return 12;
            case "CENTER":
                robot.turnDegrees(-12.375);
                robot.moveForwardInches(23, 0.5);
                return 11;
            case "RIGHT":
                robot.turnDegrees(16.825);
                robot.moveForwardInches(26, 0.5);
                return 12;
        }
        return 11;
    }

    public String depotSamplingMarkerRoutine(String mineralLocation) {
        telemetry.addData(mineralLocation, " :LABEL");
        telemetry.update();
        double power = 0.60;
        switch (mineralLocation) {
            case "LEFT":
                robot.turnDegrees(-40);
                robot.moveForwardInches(34.5, power);
                robot.turnDegrees(80);
                robot.moveForwardInches(36, power);
                break;
            case "CENTER":
                robot.turnDegrees(-10.375);
                robot.moveForwardInches(58, power);
                break;
            case "RIGHT":
                robot.turnDegrees(16.825);
                robot.moveForwardInches(35.5, power);
                robot.turnDegrees(-58);
                robot.moveForwardInches(28, power);
                break;

        }
        return mineralLocation;
    }

    public SamplingMineral getSamplingMineral() {
        //        if (tfod != null) {
//            tfod.shutdown();
//        }
        return getSamplingMineralHelper(true);
    }

    private SamplingMineral getSamplingMineralHelper(boolean isCameraUpright) {
        if (opModeIsActive()) {
            /* Activate Tensor Flow Object Detection. */

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int lowestRecognitionIndex = 0;
                        double lowestRecoognitionVal = 2;

                        for (int i = 0; i < 3; ++i) {
                            if (updatedRecognitions.get(i).getConfidence() < lowestRecoognitionVal) {
                                lowestRecoognitionVal = updatedRecognitions.get(i).getConfidence();
                                lowestRecognitionIndex = i;
                            }
                        }

                        updatedRecognitions.remove(lowestRecognitionIndex);

//                        int goldMineralX = -1;
//                        int silverMineral1X = -1;
//                        int silverMineral2X = -1;
//                        double goldEstimatedAngle = 10000;
//                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                                goldMineralX = (int) ((recognition.getLeft() + recognition.getRight()) / 2);
//                                goldEstimatedAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
//                            } else if (silverMineral1X == -1) {
//                                silverMineral1X = (int) ((recognition.getLeft() + recognition.getRight()) / 2);
//                            } else {
//                                silverMineral2X = (int) ((recognition.getLeft() + recognition.getRight()) / 2);
//                            }
//                        }
//                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                telemetry.addData("Gold Mineral Position", "Left");
//                                return new SamplingMineral("GOLD", goldMineralX, "LEFT", goldEstimatedAngle);
//                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                telemetry.addData("Gold Mineral Position", "Right");
//                                return new SamplingMineral("GOLD", goldMineralX, "RIGHT", goldEstimatedAngle);
//                            } else {
//                                telemetry.addData("Gold Mineral Position", "Center");
//                                return new SamplingMineral("GOLD", goldMineralX, "CENTER", goldEstimatedAngle);
//                            }
//                        }
                        /*WE KNOW THAT THE 2 MINERALS WILL BE ON THE LEFT**/
                    }

                    if (updatedRecognitions.size() == 2) {
                        SamplingMineral min1 = new SamplingMineral("null", -1, "null", 10000);
                        SamplingMineral min2 = new SamplingMineral("null", -1, "null", 10000);
                        for (Recognition recognition : updatedRecognitions) {
                            if (min1.getLabel().equals("null")) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    min1 = new SamplingMineral("GOLD",
                                            (int) ((recognition.getLeft() + recognition.getRight()) / 2),
                                            "null", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                } else {
                                    min1 = new SamplingMineral("SILVER",
                                            (int) ((recognition.getLeft() + recognition.getRight()) / 2), "null",
                                            recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                }
                            } else {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    min2 = new SamplingMineral("GOLD",
                                            (int) ((recognition.getLeft() + recognition.getRight()) / 2), "null",
                                            recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                } else {
                                    min2 = new SamplingMineral("SILVER",
                                            (int) ((recognition.getLeft() + recognition.getRight()) / 2), "null",
                                            recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                }
                            }
                        }

                        SamplingMineral leftMineral;
                        SamplingMineral rightMineral;
                        if (min1.getXpos() < min2.getXpos()) {
                            leftMineral = min1;
                            rightMineral = min2;
                        } else {
                            leftMineral = min2;
                            rightMineral = min1;
                        }

                        if (isCameraUpright) {
                            if (leftMineral.getLabel().equals("GOLD")) {
                                return new SamplingMineral("GOLD", leftMineral.getXpos(), "CENTER", leftMineral.getEstimatedAngle());
                            } else if (rightMineral.getLabel().equals("GOLD")) {
                                return new SamplingMineral("GOLD", rightMineral.getXpos(), "RIGHT", rightMineral.getEstimatedAngle());
                            } else {
//                                robot.turnDegrees(rightMineral.getEstimatedAngle() + 5);
//                                waitMilliseconds(600);
                                return new SamplingMineral("GOLD", rightMineral.getXpos() + (rightMineral.getXpos() - leftMineral.getXpos()), "LEFT", -1);
                                //return new SamplingMineral("GOLD", rightMineral.getXpos()+(rightMineral.getXpos()-leftMineral.getXpos()),
                                //"RIGHT", rightMineral.getEstimatedAngle()+(rightMineral.getEstimatedAngle()-leftMineral.getEstimatedAngle()));
                            }
                        } else {
                            if (leftMineral.getLabel().equals("GOLD")) {
                                return new SamplingMineral("GOLD", leftMineral.getXpos(), "RIGHT", -leftMineral.getEstimatedAngle());
                            } else if (rightMineral.getLabel().equals("GOLD")) {
                                return new SamplingMineral("GOLD", rightMineral.getXpos(), "CENTER", -rightMineral.getEstimatedAngle());
                            } else {
//                                robot.turnDegrees(-rightMineral.getEstimatedAngle()-5);
//                                waitMilliseconds(600);
//                                return getSamplingMineralHelper(isCameraUpright);
                                return new SamplingMineral("GOLD", rightMineral.getXpos() + (rightMineral.getXpos() - leftMineral.getXpos()), "LEFT", -1);
                                //"RIGHT", rightMineral.getEstimatedAngle()+(rightMineral.getEstimatedAngle()-leftMineral.getEstimatedAngle()));
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        return null;
    }

    public void runActualSteps() {
    }

    @Deprecated
    void aStarAlgoHelper(int gotoX, int gotoY, int robotX, int robotY) {
        aStarCount.set(0);
        aStarAlgo(gotoX, gotoY, robotX, robotY);
    }

    protected void newAStarAlgo(int gotoX, int gotoY, int rotationDegrees) {
        aStarCount.set(0);
        restartAStar.set(false);
        newAStarAlgo(gotoX, gotoY, -1, -1, -1, rotationDegrees, false);
    }

    @Deprecated
    protected void aStarAlgo(int gotoX, int gotoY) {
        aStarAlgo(gotoX, gotoY, -1, -1);
    }

    /**
     * aStarAlgo method
     *
     * @param gotoX  - the x pos to go to
     * @param gotoY  - the y pos to go to
     * @param robotX - to the x point where the robot is --BUT-- if this is -1, then it will figure it out itself
     * @param robotY - to the y point where the robot is --BUT-- if this is -1, then it will figure it out itself
     */
    @Deprecated
    private void aStarAlgo(int gotoX, int gotoY, int robotX, int robotY) {
        aStarCount.incrementAndGet();
        aStarIsRunning.set(true);

        boolean wasNegativeOne = robotX != -1;
        if (robotX == -1) {
            robotX = (int) robot.getX();
        }

        if (robotY == -1) {
            robotY = (int) robot.getY();
        }

        //flipped bc of annoying shit.
        Pathing tempPathing = AStarAlgorithm.getAStarRobotPathing(
                new Pair(robotY, robotX),
                new Pair(gotoY, gotoX),
                robot.getStrafeCounterAction(),
                telemetry
        );

        if (tempPathing.pathing.size() == 0) {
            telemetry.addData("NOTHING IN PATHING", "nothing");
        }

//        for (Pair pair : tempPathing.pathing) {
//            telemetry.addData(String.valueOf(pair.first), pair.second);
//        }
        telemetry.addData("error", tempPathing.error);
        if (tempPathing.extra != null) {
            for (Pair pair : tempPathing.extra) {
                telemetry.addData(String.valueOf(pair.first), pair.second);
            }
        }

        telemetry.update();


        ArrayList<RobotMovementPair> pathing = AStarAlgorithm.convertPathingToRobotMovements(tempPathing);

        telemetry.addData("RobotX " + String.valueOf(robotX), "Robot Y " + String.valueOf(robotY));
        telemetry.addData(String.valueOf(gotoX), String.valueOf(gotoY));
        for (RobotMovementPair pair : pathing) {
            telemetry.addData(pair.direction, pair.inches);
        }
        telemetry.update();

        if (!wasNegativeOne) {
            robot.alignWithGlobalFront(lastMark, (MOEListener) lastMark.getListener());
        }

        double beforeRobotAngle = robot.getDegreesTurned();

        for (int i = 0; i < pathing.size(); i++) {
            RobotMovementPair movements = pathing.get(i);
            String direction = movements.direction;

            telemetry.addData(direction, movements.inches);
            telemetry.update();

            double inches = movements.inches;
            double power = 0.5;
            switch (direction) {
                case "UP": {
                    int[] endTics = robot.getEncoderTics();
                    endTics[0] += ((int) inches * MOEBot.COUNTS_PER_INCH);
                    endTics[1] += ((int) inches * MOEBot.COUNTS_PER_INCH);
                    int[] currentTics = robot.getEncoderTics();

                    robot.moveAllMotors(power);
                    while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] < endTics[1]) {
                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
                            restartAStar.set(false);
                            aStarAlgo(gotoX, gotoY, -1, -1);
                            return;
                        }
                        currentTics = robot.getEncoderTics();
                    }
                    robot.moveAllMotors(0);
                    break;
                }
                case "DOWN": {
                    int[] endTics = robot.getEncoderTics();
                    endTics[0] -= ((int) inches * MOEBot.COUNTS_PER_INCH);
                    endTics[1] -= ((int) inches * MOEBot.COUNTS_PER_INCH);

                    int[] currentTics = robot.getEncoderTics();

                    robot.moveAllMotors(-power);
                    while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] > endTics[1]) {
                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
                            restartAStar.set(false);
                            aStarAlgo(gotoX, gotoY, -1, -1);
                            return;
                        }
                        currentTics = robot.getEncoderTics();
                    }
                    robot.moveAllMotors(0);
                    break;
                }
                case "Right": {
                    int[] endTics = robot.getEncoderTics();
                    endTics[0] += ((int) inches * MOEBot.STRAFE_COUNTS_PER_INCH);
                    endTics[1] -= ((int) inches * MOEBot.STRAFE_COUNTS_PER_INCH);
                    int[] currentTics = robot.getEncoderTics();

                    robot.strafe(power);
                    while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] > endTics[1]) {
                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
                            restartAStar.set(false);
                            aStarAlgo(gotoX, gotoY, -1, -1);
                            return;
                        }
                        currentTics = robot.getEncoderTics();
                    }
                    robot.moveAllMotors(0);
                    break;
                }
                default: {
                    int[] endTics = robot.getEncoderTics();
                    endTics[0] -= ((int) inches * MOEBot.STRAFE_COUNTS_PER_INCH);
                    endTics[1] += ((int) inches * MOEBot.STRAFE_COUNTS_PER_INCH);
                    int[] currentTics = robot.getEncoderTics();

                    robot.strafe(-power);
                    while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] < endTics[1]) {
                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
                            restartAStar.set(false);
                            aStarAlgo(gotoX, gotoY, -1, -1);
                            return;
                        }
                        currentTics = robot.getEncoderTics();
                    }
                    robot.moveAllMotors(0);
                    break;
                }
            }

            if (Math.abs(robot.getDegreesTurned() - beforeRobotAngle) > 2) {
                robot.turnToDegreesAndStop(beforeRobotAngle);
            }
        }
        aStarIsRunning.set(false);
        robot.turnToDegreesAndStop(beforeRobotAngle);
    }

    public boolean isAStarvalidPath(int gotoX, int gotoY, int robotX, int robotY) {
        //        if (robotX == -1) {
//            actualRobotX = (int) Math.round(robot.getX());
//        }
//
//        if (robotY == -1) {
//            actualRobotY = (int) Math.round(robot.getY());
//        }
        try {
            AStarAlgorithmm algo = new AStarAlgorithmm(robotY, robotX, gotoY, gotoX);
            algo.process();
            ArrayList<RobotMovement> movements = algo.getRobotMovements();
            return movements.size() == 0;
        } catch (ArrayIndexOutOfBoundsException e) {
            return false;
        }
    }

    public void newAStarAlgo(int gotoX, int gotoY, int robotX, int robotY, int degreesToTurnTo, int rotationDegrees) {
        newAStarAlgo(gotoX, gotoY, robotX, robotY, degreesToTurnTo, rotationDegrees, false);
    }

    public void newAStarAlgo(int gotoX, int gotoY, int robotX, int robotY, int degreesToTurnTo, int rotationDegrees, boolean shortCircuit) {
        aStarCount.incrementAndGet();
        aStarIsRunning.set(true);

        int actualRobotX = robotX;
        int actualRobotY = robotY;
        if (robotX == -1) {
            actualRobotX = (int) Math.round(robot.getX());
        }

        if (robotY == -1) {
            actualRobotY = (int) Math.round(robot.getY());
        }

        AStarAlgorithmm algo = new AStarAlgorithmm(actualRobotY, actualRobotX, gotoY, gotoX);
        algo.process();
        ArrayList<RobotMovement> movements = algo.getRobotMovements();

        if (movements.size() == 0) {
            if (tts != null) {
                tts.speak("Path not found", TextToSpeech.QUEUE_ADD, null);
            }
            telemetry.addData("NOTHING IN PATHING", "nothing");
        }

        telemetry.addData("RobotX " + String.valueOf(actualRobotX), "Robot Y " + String.valueOf(actualRobotY));
        telemetry.addData(String.valueOf(gotoX), String.valueOf(gotoY));
        for (RobotMovement pair : movements) {
            telemetry.addData(pair.direction + "", pair.inches);
        }
        telemetry.update();

        //  waitMilliseconds(10000);

        if (robotX == -1 && degreesToTurnTo == -1) {
            robot.alignWithGlobalFront(lastMark, (MOEListener) lastMark.getListener());
        }
        if (degreesToTurnTo != -1) {
            robot.turnToDegreesAndStop(degreesToTurnTo);
        }

//        movements.clear();
//        movements.add(new RobotMovement(RobotMovement.Directions.NW, 75));
//        movements.add(new RobotMovement(RobotMovement.Directions.NE, 75));
//        movements.add(new RobotMovement(RobotMovement.Directions.SE, 75));
//        movements.add(new RobotMovement(RobotMovement.Directions.SW, 75));

        double beforeRobotAngle = robot.getDegreesTurned();

        for (int i = 0; i < movements.size(); i++) {
            RobotMovement robotMovement = movements.get(i);
            int direction = robotMovement.direction;
            if (rotationDegrees != 0) {
                direction = RobotMovement.rotate(rotationDegrees, robotMovement.direction);
            }

            telemetry.addData(String.valueOf(direction), robotMovement.inches);
            telemetry.update();

            double inches = robotMovement.inches;
            double power = 0.65;
            double verticalPower = 0.7;
//            switch (direction) {
//                case RobotMovement.Directions.N: {
//                    int[] endTics = robot.getEncoderTics();
//                    endTics[0] += ((int) inches * MOEBot.COUNTS_PER_INCH);
//                    endTics[1] += ((int) inches * MOEBot.COUNTS_PER_INCH);
//                    int[] currentTics = robot.getEncoderTics();
//
//                    robot.moveAllMotors(power);
//                    while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] < endTics[1]) {
//                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
//                            restartAStar.set(false);
//                            newAStarAlgo(gotoX, gotoY, -1, -1);
//                            return;
//                        }
//                        currentTics = robot.getEncoderTics();
//                    }
//                    robot.moveAllMotors(0);
//                    break;
//                }
//                case RobotMovement.Directions.S: {
//                    int[] endTics = robot.getEncoderTics();
//                    endTics[0] -= ((int) inches * MOEBot.COUNTS_PER_INCH);
//                    endTics[1] -= ((int) inches * MOEBot.COUNTS_PER_INCH);
//
//                    int[] currentTics = robot.getEncoderTics();
//
//                    robot.moveAllMotors(-power);
//                    while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] > endTics[1]) {
//                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
//                            restartAStar.set(false);
//                            newAStarAlgo(gotoX, gotoY, -1, -1);
//                            return;
//                        }
//                        currentTics = robot.getEncoderTics();
//                    }
//                    robot.moveAllMotors(0);
//                    break;
//                }
//                case RobotMovement.Directions.E: {
//                    int[] endTics = robot.getEncoderTics();
//                    endTics[0] += ((int) inches * MOEBot.STRAFE_COUNTS_PER_INCH);
//                    endTics[1] -= ((int) inches * MOEBot.STRAFE_COUNTS_PER_INCH);
//                    int[] currentTics = robot.getEncoderTics();
//
//                    robot.strafe(power);
//                    while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] > endTics[1]) {
//                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
//                            restartAStar.set(false);
//                            newAStarAlgo(gotoX, gotoY, -1, -1);
//                            return;
//                        }
//                        currentTics = robot.getEncoderTics();
//                    }
//                    robot.moveAllMotors(0);
//                    break;
//                }
//                case RobotMovement.Directions.W: {
//                    int[] endTics = robot.getEncoderTics();
//                    endTics[0] -= ((int) inches * MOEBot.STRAFE_COUNTS_PER_INCH);
//                    endTics[1] += ((int) inches * MOEBot.STRAFE_COUNTS_PER_INCH);
//                    int[] currentTics = robot.getEncoderTics();
//
//                    robot.strafe(-power);
//                    while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] < endTics[1]) {
//                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
//                            restartAStar.set(false);
//                            newAStarAlgo(gotoX, gotoY, -1, -1);
//                            return;
//                        }
//                        currentTics = robot.getEncoderTics();
//                    }
//                    robot.moveAllMotors(0);
//                    break;
//                }
//                case RobotMovement.Directions.NW: {
//                    int[] endTics = robot.getEncoderTics();
//                    //  endTics[0] += ((int) inches * DIAGONAL_COUNTS_PER_INCH);
//                    endTics[1] += ((int) (inches * robot.DIAGONAL_COUNTS_PER_INCH));
//                    int[] currentTics = robot.getEncoderTics();
//
//                    //   this.moveAllMotors(power);
//                    robot.bottomLeftMotor.setPower(power);
//                    robot.topRightMotor.setPower(power);
//                    robot.topLeftMotor.setPower(power * robot.getStrafeCounterAction());
//                    robot.bottomRightMotor.setPower(power * robot.getStrafeCounterAction());
//                    while (opModeIsActive() && currentTics[1] < endTics[1]) {
//                        currentTics = robot.getEncoderTics();
//                        if (Math.abs(robot.getDegreesTurned() - beforeRobotAngle) > MOEBot.DIAGONAL_ANGLE_DIFFERENCE_ALLOWED) {
//                            robot.turnToDegreesAndStop(beforeRobotAngle);
//                            robot.bottomLeftMotor.setPower(power);
//                            robot.topRightMotor.setPower(power);
//                            robot.topLeftMotor.setPower(power * robot.getStrafeCounterAction());
//                            robot.bottomRightMotor.setPower(power * robot.getStrafeCounterAction());
//                        }
//                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
//                            restartAStar.set(false);
//                            newAStarAlgo(gotoX, gotoY, -1, -1);
//                            return;
//                        }
//                    }
//                    robot.stop();
//                }
//                case RobotMovement.Directions.NE: {
//                    int[] endTics = robot.getEncoderTics();
//                    //  endTics[0] += ((int) inches * DIAGONAL_COUNTS_PER_INCH);
//                    endTics[0] -= ((int) (inches * robot.DIAGONAL_COUNTS_PER_INCH));
//                    int[] currentTics = robot.getEncoderTics();
//
//                    //   this.moveAllMotors(power);
//                    robot.topLeftMotor.setPower(power);
//                    robot.bottomRightMotor.setPower(power);
//                    robot.topRightMotor.setPower(power * robot.getStrafeCounterAction());
//                    robot.bottomLeftMotor.setPower(power * robot.getStrafeCounterAction());
//                    while (opModeIsActive() && currentTics[0] > endTics[0]) {
//                        currentTics = robot.getEncoderTics();
//                        if (Math.abs(robot.getDegreesTurned() - beforeRobotAngle) > MOEBot.DIAGONAL_ANGLE_DIFFERENCE_ALLOWED) {
//                            robot.turnToDegreesAndStop(beforeRobotAngle);
//                            robot.topLeftMotor.setPower(power);
//                            robot.bottomRightMotor.setPower(power);
//                            robot.topRightMotor.setPower(power * robot.getStrafeCounterAction());
//                            robot.bottomLeftMotor.setPower(power * robot.getStrafeCounterAction());
//                        }
//                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
//                            restartAStar.set(false);
//                            newAStarAlgo(gotoX, gotoY, -1, -1);
//                            return;
//                        }
//                    }
//                    robot.stop();
//                }
//                case RobotMovement.Directions.SE: {
//                    int[] endTics = robot.getEncoderTics();
//                    //  endTics[0] += ((int) inches * DIAGONAL_COUNTS_PER_INCH);
//                    endTics[1] -= ((int) (inches * robot.DIAGONAL_COUNTS_PER_INCH));
//                    int[] currentTics = robot.getEncoderTics();
//
//                    power = -power;
//
//                    //   this.moveAllMotors(power);
//                    robot.bottomLeftMotor.setPower(power);
//                    robot.topRightMotor.setPower(power);
//                    robot.topLeftMotor.setPower(power * robot.getStrafeCounterAction());
//                    robot.bottomRightMotor.setPower(power * robot.getStrafeCounterAction());
//                    while (opModeIsActive() && currentTics[1] > endTics[1]) {
//                        currentTics = robot.getEncoderTics();
//                        if (Math.abs(robot.getDegreesTurned() - beforeRobotAngle) > MOEBot.DIAGONAL_ANGLE_DIFFERENCE_ALLOWED) {
//                            robot.turnToDegreesAndStop(beforeRobotAngle);
//                            robot.bottomLeftMotor.setPower(power);
//                            robot.topRightMotor.setPower(power);
//                            robot.topLeftMotor.setPower(power * robot.getStrafeCounterAction());
//                            robot.bottomRightMotor.setPower(power * robot.getStrafeCounterAction());
//                        }
//                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
//                            restartAStar.set(false);
//                            newAStarAlgo(gotoX, gotoY, -1, -1);
//                            return;
//                        }
//                    }
//                    robot.stop();
//                }
//                case RobotMovement.Directions.SW: {
//                    int[] endTics = robot.getEncoderTics();
//                    //  endTics[0] += ((int) inches * DIAGONAL_COUNTS_PER_INCH);
//                    endTics[0] += ((int) (inches * robot.DIAGONAL_COUNTS_PER_INCH));
//                    int[] currentTics = robot.getEncoderTics();
//
//                    power = -power;
//
//                    //   this.moveAllMotors(power);
//                    robot.topLeftMotor.setPower(power);
//                    robot.bottomRightMotor.setPower(power);
//                    robot.topRightMotor.setPower(power * robot.getStrafeCounterAction());
//                    robot.bottomLeftMotor.setPower(power * robot.getStrafeCounterAction());
//                    while (opModeIsActive() && currentTics[0] < endTics[0]) {
//                        currentTics = robot.getEncoderTics();
//                        if (Math.abs(robot.getDegreesTurned() - beforeRobotAngle) > MOEBot.DIAGONAL_ANGLE_DIFFERENCE_ALLOWED) {
//                            robot.turnToDegreesAndStop(beforeRobotAngle);
//                            robot.topLeftMotor.setPower(power);
//                            robot.bottomRightMotor.setPower(power);
//                            robot.topRightMotor.setPower(power * robot.getStrafeCounterAction());
//                            robot.bottomLeftMotor.setPower(power * robot.getStrafeCounterAction());
//                        }
//                        if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
//                            restartAStar.set(false);
//                            newAStarAlgo(gotoX, gotoY, -1, -1);
//                            return;
//                        }
//                    }
//                    robot.stop();
//                }
//            }

            switch (direction) {
                case RobotMovement.Directions.N: {
                    robot.moveForwardInches(inches, verticalPower);
                    break;
                }
                case RobotMovement.Directions.S: {
                    robot.moveBackwardInches(inches, verticalPower);
                    break;
                }
                case RobotMovement.Directions.W: {
                    robot.strafeLeftInches(inches, power);
                    break;
                }
                case RobotMovement.Directions.E: {
                    robot.strafeRightInches(inches, power);
                    break;
                }
                case RobotMovement.Directions.NE: {
                    robot.moveNortheastInches(inches, power);
                    break;
                }
                case RobotMovement.Directions.NW: {
                    robot.moveNorthwestInches(inches, power);
                    break;
                }
                case RobotMovement.Directions.SE: {
                    robot.moveSoutheastInches(inches, power);
                    break;
                }
                case RobotMovement.Directions.SW: {
                    robot.moveSouthwestInches(inches, power);
                    break;
                }
            }
            if (Math.abs(robot.getDegreesTurned() - beforeRobotAngle) > MOEBot.ANGLE_DIFFERENCE_ALLOWED) {
                robot.turnToDegreesAndStop(beforeRobotAngle);
            }
        }
        aStarIsRunning.set(false);
        robot.turnToDegreesAndStop(beforeRobotAngle);
    }

    public void debugPoint(String name) {
        telemetry.addData("testing:", name);
        telemetry.update();
        waitMilliseconds(50);
    }

    public void pleaseDontCrash() {
        telemetry.addData("I ain't gonna crash.", "");
        telemetry.update();
    }

//    String format(OpenGLMatrix transformationMatrix) {
//        return transformationMatrix.formatAsTransform();
//    }


//    public double deadzone(double value) {
//        if ((value < .15 && value > -.15)) {
//            value = 0;
//        }
//        else if (value > .85){
//            value = 1;
//        }
//        else if (value < -.85){
//            value = -1;
//        }
//        return value;
//    }

}
