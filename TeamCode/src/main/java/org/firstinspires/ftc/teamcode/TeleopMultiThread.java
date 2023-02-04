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

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


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

@TeleOp(name="TestopMT", group="Linear Opmode")
//@Disabled
public class TeleopMultiThread extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx slider = null;
    private DcMotorEx lift = null;
    private Servo coneflip=null;
    private Servo intakeflip=null;
    private Servo claw=null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;

    private boolean keyTrigger_lift = false;
    private boolean keyTrigger_lift_down = false;
    private boolean KeyTrigger_slider = false;
    private boolean KeyTrigger_slider_back = false;

    @Override
    public void runOpMode()
    {

        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        ElapsedTime timer = new ElapsedTime();

        // Important Step 3: Option A. Set all Expansion hubs to use the AUTO Bulk Caching mode
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        slider = hardwareMap.get(DcMotorEx.class, "slider");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        coneflip = hardwareMap.get(Servo.class, "coneflip");
        intakeflip = hardwareMap.get(Servo.class, "intakeflip");
        claw = hardwareMap.get(Servo.class, "claw");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        ShowOnTelemetry("0 Status Initialized");
        //telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        int slider_stepcount = 50;
        int creepincrement = 1;
        int slider_position = 0;
        int slider_initialposition = 0;
        int slider_targetposition = 0;
        double slider_power = 0.9;
        double slider_velocity = 1;
        double speed;
        double strafe;
        double turn;

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Thread  liftThread = new LiftThread();
        liftThread.start();

        while (opModeIsActive()) {
            ReadGamePadKeys();
            speed = -gamepad2.left_stick_y;
            turn = gamepad2.right_stick_x;
            strafe = gamepad2.left_stick_x;

            double LB = (speed + turn - strafe)/creepincrement;
            double LF = (speed + turn + strafe)/creepincrement;
            double RB = (speed - turn + strafe)/creepincrement;
            double RF = (speed - turn - strafe)/creepincrement;

            if(gamepad2.left_bumper){
                creepincrement=1;
                telemetry.addData("creeepincrement", creepincrement);
                telemetry.update();
            } else if (gamepad2.right_bumper){
                creepincrement=5;
                telemetry.addData("creeepincrement", creepincrement);
                telemetry.update();
            }

            leftBack.setPower(LB);
            leftFront.setPower(LF);
            rightBack.setPower(RB);
            rightFront.setPower(RF);



            if (gamepad1.left_trigger>0.2){
                setConeflipPosition(ConeflipPosition.deliver);
                slider_targetposition = slider_position;
                ShowOnTelemetry(String.format("4 trigger set slider target position %s", slider_targetposition));
            }
            if (gamepad1.right_trigger>0.2){
                PickupConeThenRelease();
            }
            if (gamepad2.x){
                setClawPosition(ClawPosition.intake);
            }
            if (gamepad2.b){
                setClawPosition(ClawPosition.release);
            }
            if (gamepad2.y){
                setIntakeflipPosition(IntakePosition.up);
            }
            if (gamepad2.a){
                setIntakeflipPosition(IntakePosition.down);
            }


            slider.setMotorEnable();

            ShowOnTelemetry("1 set Motor Enabled");

            //check slider
            slider_position = slider.getCurrentPosition();
            if (KeyTrigger_slider) {
                slider_position += slider_stepcount;
            }
            if (KeyTrigger_slider_back) {
                slider_position -= slider_stepcount;
            }
            ShowOnTelemetry(String.format("4 set slider target position %s", slider_position));
            slider.setTargetPosition(slider_position);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(slider_power);

            //telemetry.update();

            while (slider.isBusy()) {
                //slider_position = slider.getCurrentPosition();
                slider_velocity = slider.getVelocity();
                ShowOnTelemetry(String.format("5 slider busy slider position %s, velocity %s", slider_position, slider_velocity));
                telemetry.update();
            }

//            slider_targetposition = slider_position;

            while (gamepad1.left_bumper) {
                ShowOnTelemetry(String.format("4 set slider target position %s", slider_targetposition));

                //start the automatic process
                setConeflipPosition(ConeflipPosition.intake);
                //setIntakeflipPosition(IntakePosition.down);
                slider.setTargetPosition(slider_initialposition);
                slider.setPower(0.4);

                while (slider.isBusy()) {
                    //slider_position = slider.getCurrentPosition();
                    //ShowOnTelemetry(String.format("4 set slider target position %s", slider_position));
                    //setClawPosition(ClawPosition.intake)

                }
                if (!gamepad1.left_bumper){
                    break;
                }
                sleep(500);
                //loading the cone

//                setIntakeflipPosition(IntakePosition.up);
//                sleep(800);
//                setClawPosition(ClawPosition.release);
//                sleep(450);
                //extend to target position
                PickupConeThenRelease();
                slider.setTargetPosition(slider_targetposition);
                ShowOnTelemetry(String.format("4 set slider target position %s", slider_targetposition));

                slider.setPower(slider_power);

                while (slider.isBusy()) {

                    if (!gamepad1.left_bumper){
                        break;
                    }
                    //slider_position = slider.getCurrentPosition();
                    //ShowOnTelemetry(String.format("4 set slider target position %s", slider_position));
                    //setIntakeflipPosition(IntakePosition.down);
                }
                sleep(500);

                if (!gamepad1.left_bumper){
                    break;
                }
                setConeflipPosition(ConeflipPosition.deliver);
                sleep(1000);
            }
        }

        liftThread.interrupt();
    }

    public void ReadGamePadKeys()
    {
        keyTrigger_lift = gamepad1.y;
        keyTrigger_lift_down = gamepad1.a;
        KeyTrigger_slider = gamepad1.b;
        KeyTrigger_slider_back = gamepad1.x;
    }
    public void PickupConeThenRelease()
    {
        setConeflipPosition(ConeflipPosition.intake);
        setClawPosition(ClawPosition.intake);
        sleep(1000);
        setIntakeflipPosition(IntakePosition.up);
        sleep(800); //eians'fix
        setClawPosition(ClawPosition.release);
        sleep(1000);
        setIntakeflipPosition(IntakePosition.down);
        sleep(1000);
    }

    enum ConeflipPosition{
        intake,
        deliver
    }
    public void setConeflipPosition(ConeflipPosition position) {
        double offset = 0;
        if (position == ConeflipPosition.intake)
            offset = 1;
        else
            offset = 0;
        offset = Range.clip(offset, 0.2, 0.8);
        coneflip.setPosition(offset);
    }
    enum IntakePosition{
        up,
        down
    }
    public void setIntakeflipPosition(IntakePosition position) {
        double offset = 0;
        if (position == IntakePosition.up)
            offset = 1;
        else
            offset = 0;
        offset = Range.clip(offset, 0.04, 0.55);
        intakeflip.setPosition(offset);
    }
    //offset: 0: release cone
    //        1: intake cone
    enum ClawPosition {
        release,
        intake
    }
    public void setClawPosition(ClawPosition position) {
        double offset = 0;
        if (position == ClawPosition.release)
            offset = 1;
        else
            offset = 0;

        offset = Range.clip(offset, 0, 1); //Eian fix
        claw.setPosition(offset);
    }
    public void ShowOnTelemetry(String msg)
    {
        telemetry.log().add(msg);
    }

    private class LiftThread extends Thread
    {
        public LiftThread()
        {
            this.setName("LiftThread");

            ShowOnTelemetry(String.format("%s", this.getName()));
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            ShowOnTelemetry(String.format("Starting thread %s",this.getName()));
            lift.setMotorEnable();
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            try
            {
                int lift_stepcount = 20;
                int lift_position = 0;
                int lift_target_position = 0;

                while (!isInterrupted())
                {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.
                    lift_position = lift.getCurrentPosition();
                    //ShowOnTelemetry(String.format("2 lift position %s", lift_position));

                    if (gamepad1.y) {
                        lift_position += lift_stepcount;
                        if(lift_target_position < lift_position){
                            lift_target_position = lift_position;
                        }
                    } else if (gamepad1.a) {

                        lift_target_position -= 1;
                    }
                    //ShowOnTelemetry(String.format("3 set lift target position %s", lift_target_position));
                    lift.setTargetPosition(lift_target_position);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(0.5);


                    idle();
                }
            }

            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {ShowOnTelemetry(String.format("%s interrupted", this.getName()));}
            // an error occurred in the run loop.
            catch (Exception e) {
                throw e;
            }

            ShowOnTelemetry(String.format("end of thread %s", this.getName()));
        }
    }
}