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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Insta Park Red")
public class LayerCake_V1_red_instapark extends LinearOpMode {

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

        sleep(1000);
        layerCake.output(false);
        currentStep = "Parking";
        layerCake.setSlidePosition(Constants.LOW_POSITION);
        layerCake.forwardDrive(0.5, -200, 2);
        layerCake.strafeRight(0.5, -375, 2);
        layerCake.stopDrive();
        layerCake.setSlidePosition(0);
        currentStep = "Waiting";
        sleep(1000);

    }

}
