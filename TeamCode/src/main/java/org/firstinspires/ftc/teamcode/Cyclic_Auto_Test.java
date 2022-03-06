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


    @Autonomous(name = "CyclicBlueV1", group = "Final")
    public class Cyclic_Auto_Test extends LinearOpMode {

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
            currentStep = "scanning code";
            slideHeight = layerCake.getSlideHeight();
            layerCake.setSlidePosition(Constants.LOW_POSITION);
            currentStep = "travel to shipping hub";
            layerCake.driveByAngleEncoder(-90,500, 0, .25, 5);
            layerCake.driveByAngleSensor(180, layerCake.getDistBack(), 60,  5);
            layerCake.setSlidePosition(layerCake.getSlideHeight());
            currentStep = "deposit";
            layerCake.output(true);
            sleep(1000);
            layerCake.output(false);
            currentStep = "return to neutral";
            layerCake.driveByAngleSensor(180, layerCake.getDistBack(), 50,  5);
            layerCake.setSlidePosition(Constants.LOW_POSITION);
            layerCake.driveByAngleSensor(180, layerCake.getDistBack(), 10,  5);
            layerCake.angle(-90);
            layerCake.driveByAngleEncoder(90,500, -90, .25, .5);
            //layerCake.driveByAngleEncoder(0,60.9, -90, .25, .25);
            currentStep = "begin detection";
            layerCake.setCubePipeline();
            layerCake.getGroundDistance();
            layerCake.driveByAngleEncoder(0,layerCake.getGroundDistance() + 4, 0, .25, .25);
            layerCake.intake(true);
            sleep(200);
            layerCake.driveByAngleEncoder(180,layerCake.getGroundDistance() + 4, 0, .25, .25);
            layerCake.driveByAngleEncoder(0,0, -90, .25, .25);
            layerCake.driveByAngleEncoder(-90,60.9, 0, .25, .5);
            layerCake.driveByAngleEncoder(0,45.72, 0, .25, .5);
            layerCake.setSlidePosition(Constants.HIGH_POSITION);
            layerCake.output(true);
            sleep(1000);
            layerCake.output(false);
            currentStep = "end cycle 1";
            layerCake.setSlidePosition(Constants.LOW_POSITION);
            layerCake.driveByAngleEncoder(180,25.72, 0, .25, .5);
            layerCake.driveByAngleEncoder(0,0, 90, .25, .25);
            layerCake.driveByAngleEncoder(90,20, 0, .25, .5);
            layerCake.driveByAngleEncoder(0,60.9, 0, .25, .25);
            currentStep = "park";
            layerCake.driveByAngleEncoder(0,60.9, 0, .25, .25);
            sleep(90);
        }

    }

