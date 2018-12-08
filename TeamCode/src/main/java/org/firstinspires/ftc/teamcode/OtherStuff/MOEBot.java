package org.firstinspires.ftc.teamcode.OtherStuff;


import android.content.SharedPreferences;
import android.speech.tts.TextToSpeech;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.Methods;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Methods.scaleToRange;

public class MOEBot {
    public DistanceSensor frontDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    private double x;
    private double y;
    private double angle;
    double globalAngle;
    private double StrafeCounterAction = 5.0;
    private double globalOffset;
    private final BNO055IMU gyro;
    public final DcMotor topLeftMotor;
    public final DcMotor topRightMotor;
    public final DcMotor bottomLeftMotor;
    public DcMotor bottomRightMotor;
    public DcMotor liftMotor;
    public double frontSensorOffset = 4.5;
    public double rightSensorOffset = 5.5;
    public RevTouchSensor liftLimitSwitch;
    private final Telemetry telemetry;
    private final Servo teamMarkerServo;
    public Servo parkingServo;
    public DcMotor mmsRotationMotor;
    public DcMotor mmsSlideMotor;
    private LinearOpMode opMode;
    public final double ACTUAL_COUNTS_PER_INCH;
    public final double ACTUAL_STRAFE_COUNTS_PER_INCH;
    public final double DIAGONAL_COUNTS_PER_INCH;
    private final int LIFT_TICS_TO_TOP;
    private static final float DIAGONAL_POWER_RATIO = -0.45f;
    public static final float ANGLE_DIFFERENCE_ALLOWED = 3;
    public static final float STRAFE_ANGLE_DIFFERENCE_ALLOWED = 4;
    public static final float DIAGONAL_ANGLE_DIFFERENCE_ALLOWED = 5;
    private static final double COUNTS_PER_MOTOR_REV = 1120.0;    // eg: TETRIX Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    private static final double OFFSET_COUNTS_PER_INCH = 3;       //configure this number
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /

            (WHEEL_DIAMETER_INCHES * 3.1415 * 2) - OFFSET_COUNTS_PER_INCH; //about 90
    public static final double STRAFE_COUNTS_PER_INCH = COUNTS_PER_INCH + 13;
    public static final int MMS_Extended_Position = -680;
    public static final int MMS_Middle_Position = -280;
    public static final int MMS_Before_Extended_Position = -613;
    public static final int MMM_Auton_Position = -400;

    public static final double PARKING_SERVO_STORED_POS = 0.6;

    private double[] getXYZFromMatrix(OpenGLMatrix matrix) {
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

    public static final String ROBOT_RADIUS_KEY = "robot_radius_key";
    private static final String ENCODER_TICS_KEY = "encoder_tics_key";
    private static final String LIFT_ENCODER_TICS_TO_TOP = "lettt";
    private static final String DIAGONAL_ENCODER_TICS_KEY = "detkslke";

    private static final String STRAFE_ENCODER_TICS_KEY = "strafe_encoder_tics_key";

    public MOEBot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, TextToSpeech tts, boolean isAuton) {
        this.x = -1;
        this.y = -1;
        this.telemetry = telemetry;
        SharedPreferences prefs = hardwareMap.appContext.getSharedPreferences("RobotPrefs", 0);
        this.topLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        this.topRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        this.bottomLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.bottomRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        bottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        this.frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistance");
        this.rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistance");
        this.teamMarkerServo = hardwareMap.get(Servo.class, "markerServo");
        this.opMode = opMode;
        this.liftLimitSwitch = hardwareMap.get(RevTouchSensor.class, "liftStop");
        this.bottomRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bottomLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mmsSlideMotor = hardwareMap.get(DcMotor.class, "MMSslide");
        this.topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.mmsRotationMotor = hardwareMap.get(DcMotor.class, "MMSrot");
//        if (isAuton) {
//            this.mmsRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            this.mmsRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
        this.parkingServo = hardwareMap.get(Servo.class, "parkingStick");
        for (DcMotor motor : this.getDriveMotorList()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (!isAuton) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        this.StrafeCounterAction = prefs.getFloat(ROBOT_RADIUS_KEY, 100);
        this.StrafeCounterAction = -0.45;
        ACTUAL_COUNTS_PER_INCH = prefs.getInt(ENCODER_TICS_KEY, (int) COUNTS_PER_INCH);
        ACTUAL_STRAFE_COUNTS_PER_INCH = prefs.getInt(STRAFE_ENCODER_TICS_KEY, (int) STRAFE_COUNTS_PER_INCH);
        LIFT_TICS_TO_TOP = prefs.getInt(LIFT_ENCODER_TICS_TO_TOP, 100);
        DIAGONAL_COUNTS_PER_INCH = prefs.getInt(DIAGONAL_ENCODER_TICS_KEY, 50);
        this.gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyroParameters.loggingEnabled = true;
        gyroParameters.loggingTag = "IMU";
        gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        initGyro(gyro, gyroParameters);
//        if (tts != null) {
//            tts.speak("Initialized jyro", TextToSpeech.QUEUE_ADD, null);
//        }
//
//        this.mmsRotationMotor.setTargetPosition(-MMM_Auton_Position);
//        this.mmsRotationMotor.setPower(-0.3);
//        while (this.mmsRotationMotor.isBusy()) { }
//        this.mmsRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.mmsRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initGyro(BNO055IMU gyro, BNO055IMU.Parameters gyroParameters) {
        gyro.initialize(gyroParameters);
        while (!gyro.isGyroCalibrated()) {
        }
    }

    public MOEBot(BNO055IMU gyro, DcMotor tl, DcMotor tr, DcMotor bl, DcMotor br,
                  double StrafeCounterAction, LinearOpMode opMode, Servo teamMarkerServo, SharedPreferences prefs, DcMotor liftMotor,
                  Telemetry telemetry) {
        this.x = -1;
        this.y = -1;
        this.telemetry = telemetry;
        this.gyro = gyro;
        this.topLeftMotor = tl;
        this.topRightMotor = tr;
        this.bottomLeftMotor = bl;
        // this.bottomRightMotor = br;

        this.bottomRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bottomLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.teamMarkerServo = teamMarkerServo;
        this.StrafeCounterAction = StrafeCounterAction;
        this.opMode = opMode;
        this.globalOffset = 0;
        this.liftMotor = liftMotor;
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.bottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        for (DcMotor motor : this.getDriveMotorList()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        ACTUAL_COUNTS_PER_INCH = prefs.getInt(ENCODER_TICS_KEY, (int) COUNTS_PER_INCH);
        ACTUAL_STRAFE_COUNTS_PER_INCH = prefs.getInt(STRAFE_ENCODER_TICS_KEY, (int) STRAFE_COUNTS_PER_INCH);
        LIFT_TICS_TO_TOP = prefs.getInt(LIFT_ENCODER_TICS_TO_TOP, 100);
        DIAGONAL_COUNTS_PER_INCH = prefs.getInt(DIAGONAL_ENCODER_TICS_KEY, 50);
    }

    public void runLiftRoutine() {
        this.liftMotor.setTargetPosition(LIFT_TICS_TO_TOP);
        this.liftMotor.setPower(1);
        while (this.liftMotor.isBusy() && opMode.opModeIsActive()) {
        }
        this.strafeLeftInches(3, 0.5);
        this.liftMotor.setTargetPosition(200);
        this.liftMotor.setPower(-1);
    }

    public ArrayList<DcMotor> getDriveMotorList() {
        ArrayList<DcMotor> list = new ArrayList<>();
        list.add(this.topLeftMotor);
        list.add(this.topRightMotor);
        list.add(this.bottomLeftMotor);
        list.add(this.bottomRightMotor);
        return list;

    }

    public double getDegreesTurned() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double degrees = -AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        if (degrees >= 0 && degrees <= 90) {
            degrees = 90 - degrees;
        } else if (degrees >= 90 && degrees <= 180) {
            degrees = 360 - (degrees - 90);
        } else if (degrees <= 0 && degrees >= -90) {
            degrees = 90 - degrees;
        } else { //degrees >= -90 && degrees <= -180
            degrees = (-degrees) - 90 + 180;
        }
        angle = degrees;
        return angle;
    }

    private void setGlobalOffset(double offset) {
        this.globalOffset = offset;
    }

    public int[] getEncoderTics() {
        return new int[]{this.topLeftMotor.getCurrentPosition(), this.topRightMotor.getCurrentPosition()};
    }

    public void moveNorthwestInches(double inches, double power) {
        int[] endTics = this.getEncoderTics();
        //  endTics[0] += ((int) inches * DIAGONAL_COUNTS_PER_INCH);
        endTics[1] += ((int) (inches * DIAGONAL_COUNTS_PER_INCH));
        int[] currentTics = this.getEncoderTics();

        //   this.moveAllMotors(power);
        this.bottomLeftMotor.setPower(power);
        this.topRightMotor.setPower(power);
        this.topLeftMotor.setPower(power * this.getStrafeCounterAction());
        this.bottomRightMotor.setPower(power * this.getStrafeCounterAction());
        double beforeRobotAngle = this.getDegreesTurned();
        while (currentTics[1] < endTics[1] && opMode.opModeIsActive()) {
            currentTics = this.getEncoderTics();
            if (Math.abs(this.getDegreesTurned() - beforeRobotAngle) > DIAGONAL_ANGLE_DIFFERENCE_ALLOWED) {
                this.turnToDegreesAndStop(beforeRobotAngle);
                this.bottomLeftMotor.setPower(power);
                this.topRightMotor.setPower(power);
                this.topLeftMotor.setPower(power * this.getStrafeCounterAction());
                this.bottomRightMotor.setPower(power * this.getStrafeCounterAction());
            }
        }
        this.turnToDegreesAndStop(beforeRobotAngle);
        this.stop();
    }


    public void moveSouthwestInches(double inches, double power) {
        int[] endTics = this.getEncoderTics();
        //  endTics[0] += ((int) inches * DIAGONAL_COUNTS_PER_INCH);
        endTics[0] += ((int) (inches * DIAGONAL_COUNTS_PER_INCH));
        int[] currentTics = this.getEncoderTics();

        power = -power;

        //   this.moveAllMotors(power);
        this.topLeftMotor.setPower(power);
        this.bottomRightMotor.setPower(power);
        this.topRightMotor.setPower(power * this.getStrafeCounterAction());
        this.bottomLeftMotor.setPower(power * this.getStrafeCounterAction());
        double beforeRobotAngle = this.getDegreesTurned();
        while (currentTics[0] < endTics[0] && opMode.opModeIsActive()) {
            currentTics = this.getEncoderTics();
            if (Math.abs(this.getDegreesTurned() - beforeRobotAngle) > DIAGONAL_ANGLE_DIFFERENCE_ALLOWED) {
                this.turnToDegreesAndStop(beforeRobotAngle);
                this.topLeftMotor.setPower(power);
                this.bottomRightMotor.setPower(power);
                this.topRightMotor.setPower(power * this.getStrafeCounterAction());
                this.bottomLeftMotor.setPower(power * this.getStrafeCounterAction());
            }
        }
        this.turnToDegreesAndStop(beforeRobotAngle);
        this.stop();
    }

    public int getFrontDistance() {//returns A* Units
        return (int) Math.round((frontDistanceSensor.getDistance(DistanceUnit.INCH) + frontSensorOffset) / 2);
    }

    public int getRightDistance() {//returns A* Units
        return (int) Math.round((rightDistanceSensor.getDistance(DistanceUnit.INCH) + rightSensorOffset) / 2);
    }

    public void setMMSMotorServo(double position) {
        int targetPos = (int) Methods.scaleToRange(position, 0, 1, -680, 0);
        targetPos = deadzoneMMS(targetPos);
        mmsRotationMotor.setTargetPosition(targetPos);
        if (mmsRotationMotor.getCurrentPosition() > targetPos) {
            mmsRotationMotor.setPower(-0.8);
        } else {
            mmsRotationMotor.setPower(0.8);
        }
    }

    public void setMMSMotorServo(int num) {
        num = deadzoneMMS(num);
        mmsRotationMotor.setTargetPosition(num);
        if (mmsRotationMotor.getCurrentPosition() > num) {
            mmsRotationMotor.setPower(-0.2);
        } else {
            mmsRotationMotor.setPower(0.2);
        }
    }

    public void setMMSMotorServo(int num, double power) {
        num = deadzoneMMS(num);
        mmsRotationMotor.setTargetPosition(num);
        if (mmsRotationMotor.getCurrentPosition() > num) {
            mmsRotationMotor.setPower(-power);
        } else {
            mmsRotationMotor.setPower(power);
        }
    }

    public int deadzoneMMS(int num) {
        if (num < MMS_Extended_Position) {
            return MMS_Extended_Position;
        } else if (num > 0) {
            return 0;
        } else {
            return num;
        }
    }


    public void moveNortheastInches(double inches, double power) {
        int[] endTics = this.getEncoderTics();
        //  endTics[0] += ((int) inches * DIAGONAL_COUNTS_PER_INCH);
        endTics[0] -= ((int) (inches * DIAGONAL_COUNTS_PER_INCH));
        int[] currentTics = this.getEncoderTics();

        //   this.moveAllMotors(power);
        this.topLeftMotor.setPower(power);
        this.bottomRightMotor.setPower(power);
        this.topRightMotor.setPower(power * this.getStrafeCounterAction());
        this.bottomLeftMotor.setPower(power * this.getStrafeCounterAction());
        double beforeRobotAngle = this.getDegreesTurned();
        while (currentTics[0] > endTics[0] && opMode.opModeIsActive()) {
            telemetry.addData("current", currentTics[0]);
            telemetry.addData("goal", endTics[0]);
            telemetry.addData("current2", currentTics[1]);
            telemetry.addData("goal2", endTics[1]);
            telemetry.update();
            currentTics = this.getEncoderTics();
            if (Math.abs(this.getDegreesTurned() - beforeRobotAngle) > DIAGONAL_ANGLE_DIFFERENCE_ALLOWED) {
                this.turnToDegreesAndStop(beforeRobotAngle);
                this.topLeftMotor.setPower(power);
                this.bottomRightMotor.setPower(power);
                this.topRightMotor.setPower(power * this.getStrafeCounterAction());
                this.bottomLeftMotor.setPower(power * this.getStrafeCounterAction());
            }
        }
        this.turnToDegreesAndStop(beforeRobotAngle);
        this.stop();
    }

    public void moveSoutheastInches(double inches, double power) {
        int[] endTics = this.getEncoderTics();
        //  endTics[0] += ((int) inches * DIAGONAL_COUNTS_PER_INCH);
        endTics[1] -= ((int) (inches * DIAGONAL_COUNTS_PER_INCH));
        int[] currentTics = this.getEncoderTics();

        power = -power;

        //   this.moveAllMotors(power);
        this.bottomLeftMotor.setPower(power);
        this.topRightMotor.setPower(power);
        this.topLeftMotor.setPower(power * this.getStrafeCounterAction());
        this.bottomRightMotor.setPower(power * this.getStrafeCounterAction());
        double beforeRobotAngle = this.getDegreesTurned();
        while (currentTics[1] > endTics[1] && opMode.opModeIsActive()) {
            currentTics = this.getEncoderTics();
            if (Math.abs(this.getDegreesTurned() - beforeRobotAngle) > DIAGONAL_ANGLE_DIFFERENCE_ALLOWED) {
                this.turnToDegreesAndStop(beforeRobotAngle);
                this.bottomLeftMotor.setPower(power);
                this.topRightMotor.setPower(power);
                this.topLeftMotor.setPower(power * this.getStrafeCounterAction());
                this.bottomRightMotor.setPower(power * this.getStrafeCounterAction());
            }
        }
        this.turnToDegreesAndStop(beforeRobotAngle);
        this.stop();
    }

    public void moveForwardInchesWithPowerscaling(double inches, double power) {
        int[] endTics = this.getEncoderTics();
        endTics[0] += ((int) inches * ACTUAL_COUNTS_PER_INCH);
        endTics[1] += ((int) inches * ACTUAL_COUNTS_PER_INCH);
        int[] currentTics = this.getEncoderTics();

        double max = inches > 5 ? inches : 5;
        this.moveAllMotors(scaleToRange(currentTics[0], 0.0, max, 0.4, 0.8));
        double beforeRobotAngle = this.getDegreesTurned();
        while (currentTics[0] < endTics[0] && currentTics[1] < endTics[1] && opMode.opModeIsActive()) {
            currentTics = this.getEncoderTics();
            if (Math.abs(this.getDegreesTurned() - beforeRobotAngle) > ANGLE_DIFFERENCE_ALLOWED) {
                this.turnToDegreesAndStop(beforeRobotAngle);
            }
            this.moveAllMotors(scaleToRange(currentTics[0], 0.0, max, 0.4, 0.8));
        }
        this.moveAllMotors(0);
    }

    public void moveForwardInches(double inches, double power) {
        int[] endTics = this.getEncoderTics();
        endTics[0] += ((int) inches * ACTUAL_COUNTS_PER_INCH);
        endTics[1] += ((int) inches * ACTUAL_COUNTS_PER_INCH);
        int[] currentTics = this.getEncoderTics();

        this.moveAllMotors(power);
        double beforeRobotAngle = this.getDegreesTurned();
        while (currentTics[0] < endTics[0] && currentTics[1] < endTics[1] && opMode.opModeIsActive()) {
            currentTics = this.getEncoderTics();
            if (Math.abs(this.getDegreesTurned() - beforeRobotAngle) > ANGLE_DIFFERENCE_ALLOWED) {
                this.turnToDegreesAndStop(beforeRobotAngle);
                this.moveAllMotors(power);
            }
        }
        this.moveAllMotors(0);
    }

    public void moveBackwardInches(double inches, double power) {
        int[] endTics = this.getEncoderTics();
        endTics[0] -= ((int) inches * ACTUAL_COUNTS_PER_INCH);
        endTics[1] -= ((int) inches * ACTUAL_COUNTS_PER_INCH);
        int[] currentTics = this.getEncoderTics();

        this.moveAllMotors(-power);
        double beforeRobotAngle = this.getDegreesTurned();
        while (currentTics[0] > endTics[0] && currentTics[1] > endTics[1] && opMode.opModeIsActive()) {
            currentTics = this.getEncoderTics();
            if (Math.abs(this.getDegreesTurned() - beforeRobotAngle) > ANGLE_DIFFERENCE_ALLOWED) {
                this.turnToDegreesAndStop(beforeRobotAngle);
                this.moveAllMotors(power);
            }
        }
        this.moveAllMotors(0);
    }

    public void setTeamMarkerServo(double pos) {
        this.teamMarkerServo.setPosition(pos);
    }

    public double getStrafeCounterAction() {
        return StrafeCounterAction;
    }

    public double getX() {
        return x;
    }

    private void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    private void setY(double y) {
        this.y = y;
    }

    public void moveAllMotors(double power) {
        this.topLeftMotor.setPower(power);
        this.topRightMotor.setPower(power);
        this.bottomLeftMotor.setPower(power);
        this.bottomRightMotor.setPower(power);
    }

    public void strafe(double power) {
        this.topLeftMotor.setPower(power);
        this.topRightMotor.setPower(-power);
        this.bottomLeftMotor.setPower(-power);
        this.bottomRightMotor.setPower(power);
    }

    public void strafeLeftInches(double inches, double power) {
        int[] tics = this.getEncoderTics();
        double[] endTics = {((double) tics[0]) - ACTUAL_STRAFE_COUNTS_PER_INCH * inches, ((double) tics[1]) + ACTUAL_STRAFE_COUNTS_PER_INCH * inches};
        this.strafe(-power);
        double beforeRobotAngle = this.getDegreesTurned();
        while (tics[0] > endTics[0] && tics[1] < endTics[1] && opMode.opModeIsActive()) {
            tics = this.getEncoderTics();
            if (Math.abs(this.getDegreesTurned() - beforeRobotAngle) > STRAFE_ANGLE_DIFFERENCE_ALLOWED) {
                this.turnToDegrees(beforeRobotAngle);
                this.strafe(-power);
            }
        }
        this.moveAllMotors(0);
    }

    public void strafeRightInches(double inches, double power) {
        int[] tics = this.getEncoderTics();
        double[] endTics = {((double) tics[0]) + ACTUAL_STRAFE_COUNTS_PER_INCH * inches, ((double) tics[1]) - ACTUAL_STRAFE_COUNTS_PER_INCH * inches};
        this.strafe(power);
        double beforeRobotAngle = this.getDegreesTurned();
        while (tics[0] < endTics[0] && tics[1] > endTics[1] && opMode.opModeIsActive()) {
            tics = this.getEncoderTics();
            if (Math.abs(this.getDegreesTurned() - beforeRobotAngle) > STRAFE_ANGLE_DIFFERENCE_ALLOWED) {
                this.turnToDegrees(beforeRobotAngle);
                this.strafe(power);
            }
        }
        this.moveAllMotors(0);
    }

    public void turn(double power) {
        this.topLeftMotor.setPower(power);
        this.topRightMotor.setPower(-power);
        this.bottomLeftMotor.setPower(power);
        this.bottomRightMotor.setPower(-power);
    }

    private double convertToJonasGlobal(double degreesOffset, VuforiaTrackable trackable) {
        double val = -1;
        if (trackable.getName().equals("Rover")) {
            val = 0 - degreesOffset;
        } else if (trackable.getName().equals("Mars")) {
            val = 90 - degreesOffset;
        } else if (trackable.getName().equals("Moon")) {
            val = 180 - degreesOffset;
        } else if (trackable.getName().equals("Galaxy")) {
            val = 270 - degreesOffset;
        }

        if (val < 0) {
            val += 360;
        }

        return val;
    }

    public void alignWithGlobalFront(VuforiaTrackable trackable, VuforiaTrackableDefaultListener listener) {
        OpenGLMatrix lastPose = listener.getFtcCameraFromTarget();

        Orientation rot = Orientation.getOrientation(lastPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double angle = rot.secondAngle;
        if (angle < 0) {
            angle = 180 - Math.abs(angle);
        } else {
            angle = -(180 - angle);
        }

        angle = this.convertToJonasGlobal(angle, trackable);
        double turningAmount = Methods.closestAngleDifference(90, angle);

        this.setGlobalOffset(angle);

        this.turnDegrees(turningAmount);
    }

    public double[] getAngleOrientations() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return new double[]{angles.firstAngle, angles.secondAngle, angles.thirdAngle};
    }

    public void turnToDegreesAndStop(double degrees) {
        turnToDegrees(degrees);
        this.stop();
    }

    public void veryFastTurnToDegreesAndStop(double degrees) {
        veryFastTurnToDegrees(degrees);
        this.stop();
    }

    public void turnToDegrees(double degrees) {
        double difference = Methods.closestAngleDifference(this.getDegreesTurned(), degrees);
        double absDifference = Math.abs(difference);
        while (absDifference > 1 && opMode.opModeIsActive()) {
            int sign = difference > 0 ? -1 : 1;
            this.turn(sign * scaleToRange(absDifference, 0.0, 180.0, 0.25, 0.6));
            difference = Methods.closestAngleDifference(this.getDegreesTurned(), degrees);
            absDifference = Math.abs(difference);
        }
    }

    public void veryFastTurnToDegrees(double degrees) {
        double difference = Methods.closestAngleDifference(this.getDegreesTurned(), degrees);
        double absDifference = Math.abs(difference);
        while (absDifference > 1 && opMode.opModeIsActive()) {
            int sign = difference > 0 ? -1 : 1;
            this.turn(sign * scaleToRange(absDifference, 0.0, 180.0, 0.25, 0.85));
            difference = Methods.closestAngleDifference(this.getDegreesTurned(), degrees);
            absDifference = Math.abs(difference);
        }
    }

    public void turnDegrees(double degreeAmnt) {
        double degrees = this.getDegreesTurned() - degreeAmnt;
        if (degrees < 0) {
            degrees += 360;
        } else if (degrees > 360) {
            degrees -= 360;
        }
        turnToDegreesAndStop(degrees);
    }

    public void stop() {
        this.moveAllMotors(0);
    }

    public void localizePosition(VuforiaTrackable trackable, OpenGLMatrix matrix) {
        double[] xyz = getXYZFromMatrix(matrix);

        if (trackable.getName().equals("Rover")) {
            int[] mark = PointMap.getMark("Rover");
            this.setX(72 - xyz[2] / 50);
            this.setY(-xyz[0] / 50 + 36);
        } else if (trackable.getName().equals("Mars")) {
            int[] mark = PointMap.getMark("Mars");
            this.setX(xyz[0] / 50 + 36);
            this.setY(72 - xyz[2] / 50);
        } else if (trackable.getName().equals("Moon")) {
            int[] mark = PointMap.getMark("Moon");
            this.setX(xyz[2] / 50);
            this.setY(xyz[0] / 50 + 36);
        } else if (trackable.getName().equals("Galaxy")) {
            int[] mark = PointMap.getMark("Galaxy");
            this.setX(-xyz[0] / 50 + 36);
            this.setY(xyz[2] / 50);
        }
    }

    public String toString() {
        return String.valueOf(this.x) + " " + String.valueOf(this.y) + " " + String.valueOf(this.getDegreesTurned());
    }

    public void debug(String... debugMessages) {
        for (int i = 0; i < debugMessages.length; i += 2) {
            telemetry.addData(debugMessages[i], debugMessages[i + 1]);
        }
        telemetry.update();
        while (!liftLimitSwitch.isPressed() && opMode.opModeIsActive()) {
        }
        ;
    }
}
