package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Full Alt Auto Blue")
public class LayerCake_V4_blue_altpark extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware layerCake = new RobotHardware();
    private String currentStep = "waiting for start";
    double barScan = 0;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        layerCake.init(hardwareMap);
        Thread telemetryHandler = new Thread() {
            @Override
            public void run() {
                while (runtime.seconds() < 30) {
                    telemetry.addData("Runtime(s): ", runtime.seconds());
                    telemetry.addData("Distance Left(cm): ", layerCake.getLeftDistance());
                    telemetry.addData("Distance Right(cm): ", layerCake.getRightDistance());
                    telemetry.addData("Distance Back(cm): ", layerCake.getBackDistance());
                    telemetry.addData("Bar Scan: ", barScan);
                    telemetry.addData("Current Task: ", currentStep);
                    telemetry.update();
                }
            }
        };
        telemetryHandler.start();
        layerCake.initSlides();
        waitForStart();
        runtime.reset();
        //moves away from wall
        sleep(50);
        layerCake.forwardDrive(0.2,250,2);
        layerCake.strafeRight(0.2);
        layerCake.setSlidePosition(Constants.LOW_POSITION);
        //moves towards spinner
        while(layerCake.getRightDistance() > 5 && runtime.seconds() < 5) {}
        layerCake.forwardDrive(-0.2);
        while(layerCake.getBackDistance() > 24.5 && runtime.seconds() < 7) {}
        layerCake.stopDrive();
        layerCake.spinnerPower(-1);
        sleep(3000);
        layerCake.spinnerPower(0);
        layerCake.forwardDrive(0.2);
        while(layerCake.getBackDistance() < 65 && runtime.seconds() < 13) {}
        layerCake.strafeRight(-0.1);
        while(layerCake.getRightDistance() < 35 && runtime.seconds() < 16) {}
        layerCake.stopDrive();
        sleep(50);
        barScan = layerCake.getLeftDistance();
        sleep(50);
        layerCake.strafeRight(0.3,200,1);
        layerCake.rotateLeft(0.5,Constants.FULL_SPIN/4,2);
        //set the slide to the correct position
        if(barScan < Constants.BAR_1_MAX){
            layerCake.setSlidePosition(Constants.HIGH_POSITION);
        }else if(barScan >= Constants.BAR_1_MAX && barScan <= Constants.BAR_2_MAX){
            layerCake.setSlidePosition(Constants.MID_POSITION);
        }else{
            layerCake.setSlidePosition(Constants.LOW_POSITION);
        }
        layerCake.strafeRight(0.5, 400, 3);
        //move up to the shipping hub
        layerCake.forwardDrive(0.5, 1050, 4);
        layerCake.output(true);
        sleep(1000);
        layerCake.output(false);
        currentStep = "Parking";
       layerCake.forwardDrive(0.5, -1200, 4);
        layerCake.setSlidePosition(0);
        layerCake.strafeRight(0.5, -400, 2);
        layerCake.stopDrive();
        currentStep = "Waiting";

        sleep(1000);

    }

}
