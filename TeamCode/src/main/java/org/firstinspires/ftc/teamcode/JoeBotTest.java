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

package org.firstinspires.ftc.teamcode;

import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.OtherStuff.AStarAlgorithm;
import org.firstinspires.ftc.teamcode.OtherStuff.PointMap;
import org.firstinspires.ftc.teamcode.OtherStuff.Pair;
import org.firstinspires.ftc.teamcode.OtherStuff.Pathing;
import org.firstinspires.ftc.teamcode.OtherStuff.RobotMovementPair;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;


/**
 * This file is the new Swerve TeleOp for MOE 365 FTC
 * The file contains both the main drive code, as well as the operations of the secondary driver.
 */

@TeleOp(name = "JoeBotTest", group = "TeleOp")


public class JoeBotTest extends LinearOpMode {
    public static int[][] mappedFTCField = new int[][]{{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, {0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0}, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1}, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1}, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1}, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0}, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0}, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0}, {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0}};
    DcMotor leftMotor, rightMotor;
    ElapsedTime et = new ElapsedTime();
    List<VuforiaTrackable> allTrackables;
    List<VuforiaTrackable> allTrackablesPhone;
    public static final double ROBOT_RADIUS = 5.0;
    AtomicBoolean aStarIsRunning = new AtomicBoolean(false);
    AtomicBoolean restartAStar = new AtomicBoolean(false);
    AtomicInteger aStarLimit = new AtomicInteger(3);
    AtomicInteger aStarCount = new AtomicInteger(0);

    OpenGLMatrix lastLocation = null;
    VuforiaTrackable lastMark = null;
    VuforiaLocalizer vuforia;
    //VuforiaLocalizer vuforia2;
    WebcamName webcamName;

    double[] xyz;
    MOEBot robot;

    public double scaleToRange(double number, double numberMin, double numberMax, double newMin, double newMax) {
        return ((number - numberMin) / (numberMax - numberMin)) * (newMax - newMin) + newMin;
    }

    String lastTagName = "NULL";

    public class MOEListener extends VuforiaTrackableDefaultListener {
        int numsOfTimesCalled = 0;
        VuforiaTrackable trackable;

        public MOEListener(@Nullable VuforiaTrackable trackable) {
            super(trackable);
            this.trackable = trackable;
        }

        @Override
        public void onTracked(TrackableResult trackableResult, CameraName cameraName, Camera camera, VuforiaTrackable child) {
            super.onTracked(trackableResult, cameraName, camera, child);

            /**DONT' USE CHILD VARIABLE -- VUFORIA IS STUPID**/

            synchronized (this.lock) {
                if (trackable.getName().equals(lastTagName)) {
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
                    lastLocation = robotLocationTransform;
                    //cameraL = ((VuforiaTrackableDefaultListener) trackable.getListener()).getFtcCameraFromTarget();


                    //xyz = getXYZFromMatrix(robotLocationTransform);

                    robot.localizePosition(trackable, lastLocation);

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

            telemetry.addData("not tracked", trackable.getName());

            if (trackable.getName().equals(lastTagName)) {
                lastTagName = "NULL";
            }
        }
    }

    public class MOEBot {
        double x;
        double y;
        double angle;
        double globalAngle;
        double radius = 5.0;
        BNO055IMU gyro;
        DcMotor topLeftMotor;
        DcMotor topRightMotor;
        DcMotor bottomLeftMotor;
        DcMotor bottomRightMotor;

        static final double COUNTS_PER_MOTOR_REV = 1120.0;    // eg: TETRIX Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        static final double OFFSET_COUNTS_PER_INCH = 3;       //configure this number
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /

                (WHEEL_DIAMETER_INCHES * 3.1415) - OFFSET_COUNTS_PER_INCH; //about 90
        static final double STRAFE_COUNTS_PER_INCH = COUNTS_PER_INCH + 13;


        public MOEBot(BNO055IMU gyro, DcMotor tl, DcMotor tr, DcMotor bl, DcMotor br, double radius) {
            this.x = -1;
            this.y = -1;
            this.gyro = gyro;
            this.topLeftMotor = tl;
            this.topRightMotor = tr;
            this.bottomLeftMotor = bl;
            this.bottomRightMotor = br;
            this.bottomRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.bottomLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.radius = radius;
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

            return degrees;
        }

        public int[] getEncoderTics() {
            return new int[]{this.bottomLeftMotor.getCurrentPosition(), this.bottomRightMotor.getCurrentPosition()};
        }

        public void moveForwardInches(double inches, double power) {
            int[] endTics = this.getEncoderTics();
            endTics[0] += ((int) inches * COUNTS_PER_INCH);
            endTics[1] += ((int) inches * COUNTS_PER_INCH);
            int[] currentTics = this.getEncoderTics();

            this.moveAllMotors(power);
            while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] < endTics[1]) {
                currentTics = this.getEncoderTics();
            }
            this.moveAllMotors(0);
        }

        public void moveBackwardInches(double inches, double power) {
            int[] endTics = this.getEncoderTics();
            endTics[0] -= ((int) inches * COUNTS_PER_INCH);
            endTics[1] -= ((int) inches * COUNTS_PER_INCH);
            int[] currentTics = this.getEncoderTics();

            this.moveAllMotors(-power);
            while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] > endTics[1]) {
                currentTics = this.getEncoderTics();
            }
            this.moveAllMotors(0);
        }

        public void strafeLeft(double inches, double power) {
            int[] endTics = this.getEncoderTics();
            endTics[0] += ((int) inches * STRAFE_COUNTS_PER_INCH);
            endTics[1] -= ((int) inches * STRAFE_COUNTS_PER_INCH);
            int[] currentTics = this.getEncoderTics();

            this.strafe(power);
            while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] > endTics[1]) {
                currentTics = this.getEncoderTics();
            }
            this.moveAllMotors(0);

        }

        public void strafeRight(double inches, double power) {
            int[] endTics = this.getEncoderTics();
            endTics[0] -= ((int) inches * STRAFE_COUNTS_PER_INCH);
            endTics[1] += ((int) inches * STRAFE_COUNTS_PER_INCH);
            int[] currentTics = this.getEncoderTics();

            this.strafe(-power);
            while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] < endTics[1]) {
                currentTics = this.getEncoderTics();
            }
            this.moveAllMotors(0);

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

        public void moveMotor(String name, double power) {
            switch (name) {
                case "TL":
                    this.topLeftMotor.setPower(power);
                    break;
                case "TR":
                    this.topRightMotor.setPower(power);
                    break;
                case "BL":
                    this.bottomLeftMotor.setPower(power);
                    break;
                default:
                    this.bottomRightMotor.setPower(power);
                    break;
            }
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

        public void turn(double power) {
            this.topLeftMotor.setPower(power);
            this.topRightMotor.setPower(-power);
            this.bottomLeftMotor.setPower(power);
            this.bottomRightMotor.setPower(-power);
        }

        public double convertToJonasGlobal(double degreesOffset, VuforiaTrackable trackable) {
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

        public void alignWithGlobalFront(VuforiaTrackable trackable) {
            OpenGLMatrix lastPose = ((MOEListener) trackable.getListener()).getFtcCameraFromTarget();

            Orientation rot = Orientation.getOrientation(lastPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            double angle = rot.secondAngle;
            if (angle < 0) {
                angle = 180 - Math.abs(angle);
            } else {
                angle = -(180 - angle);
            }

            telemetry.addData("offset", angle);
            telemetry.addData("name", trackable.getName());

            angle = robot.convertToJonasGlobal(angle, trackable);
            double turningAmount = Methods.closestAngleDifference(90, angle);

            telemetry.addData("turning amount: ", turningAmount);

            robot.turnDegrees(turningAmount);
        }

//        public void followRobotPathing(ArrayList<RobotMovementPair> pathing, VuforiaTrackable trackable) {
//            this.alignWithGlobalFront(trackable);
//
//            for (int i = 0; i < pathing.size(); i++) {
//                RobotMovementPair movements = pathing.get(i);
//                String direction = movements.direction;
//
//                telemetry.addData(direction, movements.inches);
//                telemetry.update();
//
//                double inches = movements.inches;
//                double power = 0.3;
//                if (direction.equals("UP")) {
//                    int[] endTics = this.getEncoderTics();
//                    endTics[0] += ((int) inches*COUNTS_PER_INCH);
//                    endTics[1] += ((int) inches*COUNTS_PER_INCH);
//                    int[] currentTics = this.getEncoderTics();
//
//                    this.moveAllMotors(power);
//                    while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] < endTics[1]) {
//                        if (restartAStar.get()) {
//                            followRobotPathing();
//                            return;
//                        }
//                        currentTics = this.getEncoderTics();
//                    }
//                    this.moveAllMotors(0);
//                } else if (direction.equals("DOWN")) {
//                    int[] endTics = this.getEncoderTics();
//                    endTics[0] -= ((int) inches*COUNTS_PER_INCH);
//                    endTics[1] -= ((int) inches*COUNTS_PER_INCH);
//                    int[] currentTics = this.getEncoderTics();
//
//                    this.moveAllMotors(-power);
//                    while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] > endTics[1]) {
//                        currentTics = this.getEncoderTics();
//                    }
//                    this.moveAllMotors(0);
//                } else if (direction.equals("LEFT")) {
//                    int[] endTics = this.getEncoderTics();
//                    endTics[0] += ((int) inches*STRAFE_COUNTS_PER_INCH);
//                    endTics[1] -= ((int) inches*STRAFE_COUNTS_PER_INCH);
//                    int[] currentTics = this.getEncoderTics();
//
//                    this.strafe(power);
//                    while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] > endTics[1]) {
//                        currentTics = this.getEncoderTics();
//                    }
//                    this.moveAllMotors(0);
//                } else {
//                    int[] endTics = this.getEncoderTics();
//                    endTics[0] -= ((int) inches*STRAFE_COUNTS_PER_INCH);
//                    endTics[1] += ((int) inches*STRAFE_COUNTS_PER_INCH);
//                    int[] currentTics = this.getEncoderTics();
//
//                    this.strafe(-power);
//                    while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] < endTics[1]) {
//                        currentTics = this.getEncoderTics();
//                    }
//                    this.moveAllMotors(0);
//                }
//            }
//
//
//        }

        public void getGlobalDegreesTurned(VuforiaTrackable trackable, OpenGLMatrix lastPose) {
            Orientation rot = Orientation.getOrientation(lastPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            double angle = rot.secondAngle;
            if (angle < 0) {
                angle = 180 - Math.abs(angle);
            } else {
                angle = -(180 - angle);
            }

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

        public void turnToDegrees(double degrees) {
            double difference = Methods.closestAngleDifference(this.getDegreesTurned(), degrees);
            double absDifference = Math.abs(difference);

            while (absDifference > 1) {
                int sign = difference > 0 ? 1 : -1;
                this.turn(sign * scaleToRange(absDifference, 0.0, 180.0, 0.2, 0.8));
                difference = Methods.closestAngleDifference(this.getDegreesTurned(), degrees);
                absDifference = Math.abs(difference);
            }
            this.moveAllMotors(0);
        }

        public void turnDegrees(double degreeAmnt) {
            double degrees = this.getDegreesTurned() - degreeAmnt;
            if (degrees < 0) {
                degrees += 360;
            } else if (degrees > 360) {
                degrees -= 360;
            }

            double difference = Methods.closestAngleDifference(this.getDegreesTurned(), degrees);
            double absDifference = Math.abs(difference);

            while (absDifference > 1) {
                int sign = difference > 0 ? 1 : -1;
                this.turn(sign * scaleToRange(absDifference, 0.0, 180.0, 0.2, 0.8));
                difference = Methods.closestAngleDifference(this.getDegreesTurned(), degrees);
                absDifference = Math.abs(difference);
            }
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
    }

    public void waitMilliseconds(double ms) {
        et.reset();

        while (et.milliseconds() < ms) {
        }
    }

    public void driveToPoint(double x, double y) {
        int count = 0;
        OpenGLMatrix robotLocationTransform = null;
        OpenGLMatrix lastPose = null;
        while (Math.abs(robot.getX() - x) > 1 && Math.abs(robot.getY() - y) > 1) {
//            telemetry.addData("asdf:", Math.abs(robot.getX()-x));
//            telemetry.addData("asdf:", Math.abs(robot.getY()-y));
//            telemetry.addData("asdf:", x);
//            telemetry.addData("asdf:", y);
//            telemetry.addData("asdf:", count);
            count += 1;

            for (int i = 0; i < 4; i++) {
                VuforiaTrackable trackable = allTrackables.get(i);

                robotLocationTransform = null;
                lastMark = null;
                MOEListener trackListener = ((MOEListener) trackable.getListener());
                if (trackListener.isVisible()) {
                    lastMark = trackable;
                    robotLocationTransform = trackListener.getUpdatedRobotLocation();
                    lastPose = trackListener.getFtcCameraFromTarget();
                    break;
                }

                //                    double distance = Math.sqrt(Math.pow(vumarkCoords[1]-robot.getY(), 2)+Math.pow(vumarkCoords[0]-robot.getX(), 2));
                //
                //                    robot.moveAllMotors(0);
                //                    robot.turnDegrees(degreesToTurn);
                //                    robot.moveAllMotors(scaleToRange(distance, 0, 101, 0.3, 1));
            }

            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;

//                int[] temp = PointMap.getMark(lastMark.getName());
//                double[] vumarkCoords = new double[]{(double) temp[0], (double) temp[1]};
                robot.localizePosition(lastMark, lastLocation);

//                double angle =
                Orientation rot = Orientation.getOrientation(lastPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                double angle = rot.secondAngle;
                if (angle < 0) {
                    angle = 180 - Math.abs(angle);
                } else {
                    angle = -(180 - angle);
                }
                telemetry.addData("offset", angle);
                telemetry.addData("name", lastMark.getName());
                angle = robot.convertToJonasGlobal(angle, lastMark);

                double mAngle = Math.toDegrees(Math.atan2(y - robot.getY(), x - robot.getX()));
                if (mAngle < 0) {
                    mAngle += 360;
                }

                double angDiff = Methods.closestAngleDifference(mAngle, angle);

                double degreesToTurn = angDiff;
//                if (degreesToTurn < 0) {
//                    degreesToTurn += 360;
//                } else if (degreesToTurn > 360) {
//                    degreesToTurn -= degreesToTurn;
//                }


                telemetry.addData("ang diff:", angDiff);
                telemetry.addData("degs to turn:", degreesToTurn);
                telemetry.addData("coordinate angle: ", mAngle);
                telemetry.addData("global angle", angle);
                telemetry.update();

                robot.turnDegrees(angDiff);
                robot.moveAllMotors(0.3);
                //                    double distance = Math.sqrt(Math.pow(vumarkCoords[1]-robot.getY(), 2)+Math.pow(vumarkCoords[0]-robot.getX(), 2));
                //
                //                    robot.moveAllMotors(0);
                //                    robot.turnDegrees(degreesToTurn);
                //                    robot.moveAllMotors(scaleToRange(distance, 0, 101, 0.3, 1));
            }
        }
    }

    boolean buttonPressed = false;
    String currentMarkName = "mars";

    @Override
    public void runOpMode() {
        et.reset();
//        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
//        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        DcMotor topLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor topRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor bottomLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor bottomRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU imu;

        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyroParameters.loggingEnabled = true;
        gyroParameters.loggingTag = "IMU";
        gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        robot = new MOEBot(imu, topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor, ROBOT_RADIUS);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        parameters.vuforiaLicenseKey =
                "AVmJqSr/////AAABmeGcV96ftk3KgMDBC6fRLL4Qn/iAktqBNx29ewqo4NPP1TWeg6cpqfRV3dQ3vtDc4LxnLDckSBYdv9v3b1pe2Rq1v+stEVsKhckekqcdpbffWqz+QAyFJF7Mg9q/vOVBXIjvH7CPFVVKiM/M+J3vFw87SFxJKQlZuOM0WGi0hMJf8CE21ZJKh3h9tCg+/dqEux2FmB7XpezHFFeIqE8EK/3skt8Gjui+ywRSmgyzr+C3GswiIWsUn3YYCS6udgB8O6ntF5RZyrq4dQJxrdV1Mh1P7dlpGgyml+yiBAKDgoHZPiHKBx1TIY0Gg9QBebnuHdMvEOhK9oOJqtlR7XBO+fRJrXSCBY+9zBHRpZ6zSE0P";

        telemetry.addData(">", "license keys");
        telemetry.update();
        waitMilliseconds(500);
        parameters.cameraName = webcamName;

        ClassFactory instance = ClassFactory.getInstance();
        vuforia = instance.createVuforia(parameters);

        VuforiaTrackables roverRuckusVuMarks = this.vuforia.loadTrackablesFromAsset("RoverRuckus");

        roverRuckusVuMarks.get(0).setName("Rover");
        roverRuckusVuMarks.get(1).setName("Moon");
        roverRuckusVuMarks.get(2).setName("Mars");
        roverRuckusVuMarks.get(3).setName("Galaxy");

        float mmRobotLength = 431.8F;
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(0, 0, mmRobotLength / 2);
        for (int i = 0; i < 4; i++) {
            roverRuckusVuMarks.get(i).setListener(new MOEListener(roverRuckusVuMarks.get(i)));
            ((MOEListener) roverRuckusVuMarks.get(i).getListener())
                    .setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        }

        allTrackables = new ArrayList<>();
        allTrackables.addAll(roverRuckusVuMarks);

        float mmPerInch = 25.4f;
        float mmBotWidth = 17 * mmPerInch;            // ... or whatever is right for your robot
        float mmBotLength = 16 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels=

        roverRuckusVuMarks.activate();

        //leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
//        leftMotor.setPower(deadzone(-gamepad1.left_stick_y));
//        rightMotor.setPower(deadzone(-gamepad1.right_stick_y));

//        if (gamepad1.a && !buttonPressed) {
//            captureFrameToFile();
//        }

        buttonPressed = gamepad1.a;

//        for (VuforiaTrackable trackable : allTrackables) {
//            if (((MOEListener)trackable.getListener()).isVisible()) {
//                lastMark = trackable;
//            }
//
//            OpenGLMatrix robotLocationTransform =
//                    ((MOEListener) lastMark.getListener()).getUpdatedRobotLocation();
//            if (robotLocationTransform != null) {
//                lastLocation = robotLocationTransform;
//                xyz = getXYZFromMatrix(robotLocationTransform);
//                robot.localizePosition(trackable, lastLocation);
//            }
//        }

        int[] curMarkCoords = PointMap.getMark(currentMarkName);

        aStarAlgoHelper(curMarkCoords[0], curMarkCoords[1] - 10);

        telemetry.update();
    }

    public void aStarAlgoHelper(int gotoX, int gotoY) {
        aStarCount.set(0);
        aStarAlgo(gotoX, gotoY);
    }

    public void aStarAlgo(int gotoX, int gotoY) {
        aStarCount.incrementAndGet();
        aStarIsRunning.set(true);
        //flipped bc of annoying shit.
        Pathing tempPathing = AStarAlgorithm.getAStarRobotPathing(
                new Pair((int) robot.getY(), (int) robot.getX()),
                new Pair(gotoY, gotoX),
                robot.radius, telemetry
        );

        for (Pair pair : tempPathing.pathing) {
            telemetry.addData(String.valueOf(pair.first), pair.second);
        }

        ArrayList<RobotMovementPair> pathing = AStarAlgorithm.convertPathingToRobotMovements(tempPathing);

        telemetry.addData("RobotX " + String.valueOf(robot.getX()), "Robot Y " + String.valueOf(robot.getY()));
        telemetry.addData(String.valueOf(gotoX), String.valueOf(gotoY));
        for (RobotMovementPair pair : pathing) {
            telemetry.addData(pair.direction, pair.inches);
        }
        telemetry.update();

        robot.alignWithGlobalFront(lastMark);

        for (int i = 0; i < pathing.size(); i++) {
            RobotMovementPair movements = pathing.get(i);
            String direction = movements.direction;

            telemetry.addData(direction, movements.inches);
            telemetry.update();

            double inches = movements.inches;
            double power = 0.3;
            if (direction.equals("UP")) {
                int[] endTics = robot.getEncoderTics();
                endTics[0] += ((int) inches * robot.COUNTS_PER_INCH);
                endTics[1] += ((int) inches * robot.COUNTS_PER_INCH);
                int[] currentTics = robot.getEncoderTics();

                robot.moveAllMotors(power);
                while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] < endTics[1]) {
                    if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
                        restartAStar.set(false);
                        aStarAlgo(gotoX, gotoY);
                        return;
                    }
                    currentTics = robot.getEncoderTics();
                }
                robot.moveAllMotors(0);
            } else if (direction.equals("DOWN")) {
                int[] endTics = robot.getEncoderTics();
                endTics[0] -= ((int) inches * robot.COUNTS_PER_INCH);
                endTics[1] -= ((int) inches * robot.COUNTS_PER_INCH);
                int[] currentTics = robot.getEncoderTics();

                robot.moveAllMotors(-power);
                while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] > endTics[1]) {
                    if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
                        restartAStar.set(false);
                        aStarAlgo(gotoX, gotoY);
                        return;
                    }
                    currentTics = robot.getEncoderTics();
                }
                robot.moveAllMotors(0);
            } else if (direction.equals("LEFT")) {
                int[] endTics = robot.getEncoderTics();
                endTics[0] += ((int) inches * robot.STRAFE_COUNTS_PER_INCH);
                endTics[1] -= ((int) inches * robot.STRAFE_COUNTS_PER_INCH);
                int[] currentTics = robot.getEncoderTics();

                robot.strafe(power);
                while (opModeIsActive() && currentTics[0] < endTics[0] && currentTics[1] > endTics[1]) {
                    if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
                        restartAStar.set(false);
                        aStarAlgo(gotoX, gotoY);
                        return;
                    }
                    currentTics = robot.getEncoderTics();
                }
                robot.moveAllMotors(0);
            } else {
                int[] endTics = robot.getEncoderTics();
                endTics[0] -= ((int) inches * robot.STRAFE_COUNTS_PER_INCH);
                endTics[1] += ((int) inches * robot.STRAFE_COUNTS_PER_INCH);
                int[] currentTics = robot.getEncoderTics();

                robot.strafe(-power);
                while (opModeIsActive() && currentTics[0] > endTics[0] && currentTics[1] < endTics[1]) {
                    if (restartAStar.get() && aStarCount.get() < aStarLimit.get()) {
                        restartAStar.set(false);
                        aStarAlgo(gotoX, gotoY);
                        return;
                    }
                    currentTics = robot.getEncoderTics();
                }
                robot.moveAllMotors(0);
            }
        }
        aStarIsRunning.set(false);
    }

    public void debugPoint(String name) {
        telemetry.addData("testing:", name);
        telemetry.update();
        waitMilliseconds(1000);
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
