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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that rus in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Test_2", group = "Iterative Opmode")

public class Iterative_Opmode_V_2 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor spin = null;
    private DcMotor slides = null;
    private DcMotor intake = null;
    private DistanceSensor distLeft = null;
    private DistanceSensor distRight = null;
    private DistanceSensor distBack = null;
    private DigitalChannel magSwitch = null;
    private Thread slideZeroer = null;
    private int slidesTarget = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backRight = hardwareMap.get(DcMotor.class, "br");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        spin = hardwareMap.get(DcMotor.class, "spin");
        slides = hardwareMap.get(DcMotor.class, "slides");
        intake = hardwareMap.get(DcMotor.class, "nom");

        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        spin.setDirection(DcMotorSimple.Direction.FORWARD);
        slides.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        //set zero behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //reset encoders for all the motors
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        distLeft = hardwareMap.get(DistanceSensor.class, "distLeft");
        distRight = hardwareMap.get(DistanceSensor.class, "distRight");
        distBack = hardwareMap.get(DistanceSensor.class, "distBack");
        magSwitch = hardwareMap.get(DigitalChannel.class, "mag");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);
        //init slides
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(Constants.SLIDE_POWER);
        slides.setTargetPosition(0);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //do stick position things(I know its bad but its ok and it works)
        double stickX = -gamepad1.left_stick_x;
        double stickY = gamepad1.left_stick_y;
        double rotatedX = (stickX * Math.cos(Math.PI / 4)) - (stickY * Math.sin(Math.PI / 4));
        double rotatedY = (stickY * Math.cos(Math.PI / 4)) + (stickX * Math.sin(Math.PI / 4));
        double rotation = gamepad1.left_trigger - gamepad1.right_trigger;

        //do math to it
        if (Math.sqrt((stickX * stickX) + (stickY * stickY)) > Constants.STICK_THRESH) {
            frontLeft.setPower(rotatedY);
            backRight.setPower(-rotatedY);
            frontRight.setPower(rotatedX);
            backLeft.setPower(-rotatedX);
        } else if (Math.abs(rotation) > Constants.STICK_THRESH) {
            turnLeft(rotation);
        }else {
            stopDrive();
        }
        //ducky thingy
        if (gamepad2.a) {
            spin.setPower(1);
        } else if(gamepad2.b){
            spin.setPower(-1);
        }else{
            spin.setPower(0);
        }

        //intake
        if (gamepad2.left_bumper) {
            intake.setPower(Constants.INTAKE_POWER);
        } else if (gamepad2.right_bumper) {
            intake.setPower(Constants.OUTPUT_POWER);
        } else {
            intake.setPower(0);
        }
        //slides
        if(!slideZeroer.isAlive()) {
            if (gamepad2.dpad_up) {
                slidesTarget = Constants.HIGH_POSITION;
            } else if (gamepad2.dpad_right) {
                slidesTarget = Constants.MID_POSITION;
            } else if (gamepad2.dpad_left) {
                slidesTarget = Constants.LOW_POSITION;
            } else if (gamepad2.dpad_down) {
                slidesTarget = 0;
            }
            //manual adjustments
            slidesTarget += -gamepad2.right_stick_y * 25;
            //keep it in a range
            slidesTarget = Math.min(slidesTarget, Constants.SLIDE_MAX);
            slidesTarget = Math.max(slidesTarget, 0);
            slides.setTargetPosition(slidesTarget);
            if(gamepad2.x){
                slideZeroer = makeZeroer();
                slideZeroer.start();
            }
        }

        telemetry.addData("Slide Position: ", slides.getCurrentPosition());
        telemetry.addData("Distance on the left(cm): ", distLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance on the right(cm): ", distRight.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance on the back(cm): ", distBack.getDistance(DistanceUnit.CM));
        telemetry.addData("FL: ", frontLeft.getCurrentPosition());
        telemetry.addData("Switch: ", magSwitch.getState());

    }

    private void turnLeft(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    private Thread makeZeroer(){
        return new Thread(){
            public void run(){
                double start = runtime.seconds();
                boolean timeout = false;
                slides.setTargetPosition(0);
                while(slides.isBusy()){
                    telemetry.addData("Zeroing:", "Please Wait");
                    telemetry.update();
                }
                slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slides.setPower(-0.1);
                while(!magSwitch.getState() && !timeout){
                    telemetry.addData("Zeroing:", "Looking for limit switch");
                    telemetry.update();
                    if(runtime.seconds() - start > 1){
                        timeout = true;
                    }
                }
                if(!timeout) {
                    slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slides.setTargetPosition(0);
                slides.setPower(0.5);
                telemetry.addData("Zeroing:", "Finished");
                telemetry.update();
            }
        };
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop(){
    }

}
