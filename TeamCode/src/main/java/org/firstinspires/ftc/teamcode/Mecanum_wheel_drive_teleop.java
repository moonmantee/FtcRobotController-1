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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Wheel teleop", group="Linear Opmode")
//@Disabled
public class Mecanum_wheel_drive_teleop extends LinearOpMode {

    // Declare OpMode members.

    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;

    private Servo intakeflip=null;
    private Servo claw=null;
    private DcMotor lift=null;
    private Servo coneflip=null;
    private DcMotor slider = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intakeflip = hardwareMap.get(Servo.class, "intakeflip");
        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(DcMotor.class, "lift");
        coneflip = hardwareMap.get(Servo.class, "coneflip");
        slider = hardwareMap.get(DcMotor.class, "slider");
        double speed;
        double strafe;
        double turn;
        int intakeliftarget=0;
        int sliderstepcount = 20;
        int targetposition = 0;
        double liftpower;
        boolean clawpositon = true;


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);




        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeflip.setDirection(Servo.Direction.FORWARD);

        coneflip.setPosition(1);


        intakeflip.setPosition(0);
        claw.setPosition(1);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(200);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();

        // Wait for the game to start (driver presses PLAY)



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            speed = -gamepad1.left_stick_y;
//            turn = gamepad1.right_stick_x;
//            strafe = gamepad1.left_stick_x;

//            telemetry.addData("speed", speed);
//            telemetry.addData("turn", turn);
//            telemetry.addData("strafe", strafe);
//            telemetry.update();


            speed = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;


            double LB = speed + turn - strafe;
            double LF = speed + turn + strafe;
            double RB = speed - turn + strafe;
            double RF = speed - turn - strafe;

            leftBack.setPower(LB);
            leftFront.setPower(LF);
            rightBack.setPower(RB);
            rightFront.setPower(RF);
            if (gamepad2.x) {
                targetposition += sliderstepcount;
            }
            if (gamepad2.y) {
                targetposition -= sliderstepcount;
            }
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setTargetPosition(targetposition);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(1);
            while(slider.isBusy()){
                telemetry.addData("Position",slider.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("Position",slider.getCurrentPosition());
            telemetry.update();
            slider.setPower(0);
            if(gamepad1.a){

            }
        }
    }
}