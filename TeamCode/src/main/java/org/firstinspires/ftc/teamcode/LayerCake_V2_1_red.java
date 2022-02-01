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
@Autonomous(name = "Duck Truck Red")
public class LayerCake_V2_1_red extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware layerCake = new RobotHardware();
    private String currentStep = "waiting for start";
    private double bar1 = 0;//high
    private double bar2 = 0;//middle
    private double bar3 = 0;//low
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        layerCake.init(hardwareMap,telemetry);
        Thread telemetryHandler = new Thread() {
            @Override
            public void run() {
                while (runtime.seconds() < 30) {
                    telemetry.addData("Runtime(s): ", runtime.seconds());
                    telemetry.addData("Distance(cm): ", layerCake.getLeftDistance());
                    telemetry.addData("Current Task: ", currentStep);
                    telemetry.addData("BAR 1(high): ", bar1);
                    telemetry.addData("BAR 2(middle): ", bar2);
                    telemetry.addData("BAR 3(low): ", bar3);
                    telemetry.update();
                }
            }
        };
        telemetryHandler.start();
        layerCake.initSlides();
        waitForStart();
        runtime.reset();
        //moves away from wall
        layerCake.getRightDistance();
        sleep(50);
        layerCake.strafeRight(0.2,-150,2);
            layerCake.forwardDrive(-0.2);
        //moves towards spinner
        while(layerCake.getBackDistance() > 8) {}
        layerCake.strafeRight(0.2);
        while(layerCake.getRightDistance() >15 && runtime.seconds() < 6) {}
        layerCake.stopDrive();
        layerCake.spinnerPower(1);
        sleep(3000);
        layerCake.spinnerPower(0);
        layerCake.forwardDrive(0.75, 750, 3);
        layerCake.strafeRight(0.5, 185, 1.5);
        //Check first bar
        bar1 = layerCake.getLeftDistance();
        sleep(50);
        currentStep = "Moving to position 2";
        layerCake.forwardDrive(0.25, 250, 1.5);
        //Check second bar
        bar2 = layerCake.getLeftDistance();
        layerCake.stopDrive();
        sleep(50);
        currentStep = "Moving to position 3";
        layerCake.forwardDrive(0.25, 250, 1.5);
        layerCake.stopDrive();
        //Check last bar
        bar3 = layerCake.getLeftDistance();
        sleep(50);
        currentStep = "Moving to shipping hub";
        layerCake.rotateLeft(0.25,Constants.FULL_SPIN/4,3);
        //line up with the shipping hub
        layerCake.strafeRight(0.25, 650, 2);
        //Check which bars are possible
        if(bar1>50||bar1<38){bar1=-1;}
        if(bar2>50||bar2<38){bar2=-1;}
        if(bar3>50||bar3<38){bar3=-1;}
        //set the slide to the correct position
        if(bar1 > bar2 && bar1 > bar3){
            layerCake.setSlidePosition(Constants.LOW_POSITION);
        }else if(bar2 > bar3){
            layerCake.setSlidePosition(Constants.MID_POSITION);
        }else{
            layerCake.setSlidePosition(Constants.HIGH_POSITION);
        }
        //move up to the shipping hub
        layerCake.forwardDrive(0.5, 800, 4);
        layerCake.output(true);
        sleep(1000);
        layerCake.output(false);
        currentStep = "Parking";
        layerCake.forwardDrive(0.5, -1000, 5);
        layerCake.strafeRight(0.5, 2000, 10);
        layerCake.stopDrive();
        currentStep = "Waiting";
        layerCake.setSlidePosition(0);
        sleep(1000);

    }

}
