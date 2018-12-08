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
import android.widget.Toast;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.OtherStuff.MOEBot;
import org.firstinspires.ftc.teamcode.OtherStuff.PointMap;
import org.firstinspires.ftc.teamcode.OtherStuff.SamplingMineral;
import org.w3c.dom.Text;

@Autonomous(name = "ResetAuton", group = "Autonomous")
public class ResetAuton extends LinearOpMode {
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {
        elapsedTime = new ElapsedTime();
        TextToSpeech tts = new TextToSpeech(hardwareMap.appContext, null);
        MOEBot bot = new MOEBot(hardwareMap, telemetry, this, tts, false);
        waitForStart();
        bot.setTeamMarkerServo(1);
        bot.liftMotor.setPower(0.3);
        bot.mmsRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.mmsRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elapsedTime.reset();
        while (elapsedTime.milliseconds() < 500 && opModeIsActive()) {
        }
        bot.liftMotor.setPower(-0.3);
        while (!bot.liftLimitSwitch.isPressed() && opModeIsActive()) {
        }

        bot.liftMotor.setPower(0);
        tts.speak("Finished resetting Autonomous", TextToSpeech.QUEUE_ADD, null);
        telemetry.addData("Finished", "");
        telemetry.update();
        elapsedTime.reset();
        while (elapsedTime.seconds() < 2 && opModeIsActive()) {
        }
    }
}