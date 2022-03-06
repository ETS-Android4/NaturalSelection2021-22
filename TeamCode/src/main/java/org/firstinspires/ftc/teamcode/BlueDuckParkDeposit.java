package org.firstinspires.ftc.teamcode;
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "BlueDuckParkDeposit", group = "Final")
public class BlueDuckParkDeposit extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware layerCake = new RobotHardware();
    ElapsedTime runtime = new ElapsedTime();
    String currentStep = "Waiting for Start";
    int slideHeight = Constants.LOW_POSITION;
    Thread telemetryHandler = new Thread(){
        @Override
        public void run() {
            while (opModeIsActive()) {
                telemetry.addData("Current Task: ", currentStep);
                telemetry.addData("Runtime(s): ", runtime.seconds());
                telemetry.addData("Slide Target: ", slideHeight);
                telemetry.update();
            }
        }
    };
    @Override
    public void runOpMode() {
        layerCake.init(hardwareMap, telemetry);
        layerCake.initSlides();
        waitForStart();
        runtime.reset();
        telemetryHandler.start();
        currentStep = "deposit";
        layerCake.driveByAngleEncoder(-90, 1200, 0,.1,3);
        layerCake.driveByAngleEncoder(0, 1400, .5,.1,3);
        layerCake.setSlidePosition(Constants.HIGH_POSITION);
        layerCake.output(true);
        sleep(1000);
        layerCake.output(false);
        layerCake.driveByAngleEncoder(180, 1500, -.5,.1,3);
        layerCake.setSlidePosition(Constants.LOW_POSITION);
        layerCake.driveByAngleEncoder(90, 1300, 0,.1,3);
        currentStep = "go to spin";
        layerCake.driveByAngleEncoder(90, 700, 0,.1,3);
        sleep(2000);
        layerCake.spinnerPower(Constants.DUCK_POWER);
        sleep(500);
        layerCake.spinnerPower(0);
        layerCake.driveByAngleEncoder(0, 1500, 0,.1,3);
        layerCake.driveByAngleEncoder(90, 100, 0,.1,3);
        layerCake.driveByAngleEncoder(0, 1000, 0,.1,3);
        layerCake.angle(0);
        layerCake.setSlidePosition(0);
    }
}







