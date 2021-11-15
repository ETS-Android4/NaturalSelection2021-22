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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Layer Cake V2!")
public class LayerCake_V2 extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware layerCake = new RobotHardware();
    private String currentStep = "waiting for start";
    private double bar1 = 0;//high
    private double bar2 = 0;//middle
    private double bar3 = 0;//low
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        layerCake.init(hardwareMap);
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
        waitForStart();
        layerCake.initSlides();
        runtime.reset();
        sleep(500);
        bar1 = layerCake.getRightDistance();
        sleep(500);
        currentStep = "Moving to position 2";
        layerCake.forwardDrive(0.25, 250, 10);
        sleep(500);
        bar2 = layerCake.getRightDistance();
        layerCake.stopDrive();
        sleep(500);
        currentStep = "Moving to position 3";
        layerCake.forwardDrive(0.25, 250, 10);
        layerCake.stopDrive();
        sleep(500);
        bar3 = layerCake.getRightDistance();
        sleep(500);
        currentStep = "Moving to shipping hub";
        layerCake.rotateLeft(0.25,-Constants.FULL_SPIN/4,10);
        layerCake.strafeRight(0.25, -650, 10);
        if(bar1 > 35 && bar1 < 45){
            layerCake.setSlidePosition(Constants.HIGH_POSITION);
        }else if(bar2 > 35 && bar2 < 45){
            layerCake.setSlidePosition(Constants.MID_POSITION);
        }else{
            layerCake.setSlidePosition(Constants.LOW_POSITION);
        }
        layerCake.forwardDrive(0.5, 850, 10);
        sleep(500);
        layerCake.output(true);
        sleep(1000);
        layerCake.output(false);
        currentStep = "Parking";
        layerCake.forwardDrive(0.5, -1000, 1);
        layerCake.strafeRight(0.5, 2000, 10);
        layerCake.stopDrive();
        currentStep = "Waiting";
        layerCake.setSlidePosition(0);
        sleep(1000);

    }

}
