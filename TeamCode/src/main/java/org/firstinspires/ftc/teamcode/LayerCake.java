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


@Autonomous(name = "Layer Cake V1!")
public class LayerCake extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware layerCake = new RobotHardware();
    private String currentStep = "waiting for start";
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        layerCake.init(hardwareMap);
        Thread telemetryHandler = new Thread() {
            @Override
            public void run() {
                while (runtime.seconds() < 30 &&opModeIsActive()) {
                    telemetry.addData("Runtime(s): ", runtime.seconds());
                    telemetry.addData("Distance(cm): ", layerCake.getLeftDistance());
                    telemetry.addData("Current Task: ", currentStep);
                    telemetry.update();
                }
            }
        };
        telemetryHandler.start();
        waitForStart();
        runtime.reset();

        currentStep = "Moving away from wall";
        layerCake.forwardDrive(0.25, 100, 0.2);
        currentStep = "Moving to carousel";
        layerCake.strafeRight(-0.5);
        double startMove = runtime.seconds();
        while(layerCake.getLeftDistance() > 19){
            if(runtime.seconds() > startMove + 4){
                break;
            }
        }
        layerCake.stopDrive();
        currentStep = "Delivering Duck";
        layerCake.spinnerPower(1);
        sleep(4000);
        layerCake.spinnerPower(0);
        currentStep = "Moving away from carousel";
        layerCake.forwardDrive(0.5, 1000, 2);
        currentStep = "Going forward";
        layerCake.forwardDrive(0.5, 1000, 1);
        currentStep = "Waiting for teleop";
    }

}