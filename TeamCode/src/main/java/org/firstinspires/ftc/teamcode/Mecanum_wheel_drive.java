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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


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

@Autonomous(name="Mecanum Wheel", group="Linear Opmode")
//@Disabled
public class Mecanum_wheel_drive extends LinearOpMode {

    // Declare OpMode members.

    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;

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

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY
        MecanumHardware robot = new MecanumHardware(this);

        waitForStart();


        robot.init();

        //robot.sideDistance(-0.5, -50 , 9.7, 20);




        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//            //robot.ForwardTime(0.5,1000);
//            //robot.ForwardTime(-0.5,1000);
//            //
////            robot.ForwardDistance(0.5,24,4,20);
////            robot.sideTime(0.5,1000);
////            robot.ForwardDistance(-0.5,-24,4,20);
////            robot.sideTime(-0.5,1000);
//
///*
//            leftFront.setPower(0.5);
//            leftBack.setPower(0.5);
//            rightFront.setPower(0.5);
//            rightBack.setPower(0.5);
//            sleep(1100);
//            leftFront.setPower(0);
//            leftBack.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//
//            leftFront.setPower(-0.5);
//            leftBack.setPower(-0.5);
//            rightFront.setPower(-0.5);
//            rightBack.setPower(-0.5);
//            sleep(1100);
//            leftFront.setPower(0);
//            leftBack.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//*/
///*            leftFront.setPower(0.5);
//            leftBack.setPower(-0.5);
//            rightFront.setPower(-0.5);
//            rightBack.setPower(0.5);
//            sleep(1100);
//            leftFront.setPower(0);
//            leftBack.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);
//
//            leftFront.setPower(-0.5);
//            leftBack.setPower(0.5);
//            rightFront.setPower(0.5);
//            rightBack.setPower(-0.5);
//            sleep(1100);
//            leftFront.setPower(0);
//            leftBack.setPower(0);
//            rightFront.setPower(0);
//            rightBack.setPower(0);*/
//
//        }
    }
}
