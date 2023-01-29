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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@TeleOp(name="Testop", group="Linear Opmode")
//@Disabled
public class Testop extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx slider = null;
    private DcMotorEx lift = null;
    private Servo coneflip=null;
    private Servo intakeflip=null;
    private Servo claw=null;

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

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        coneflip = hardwareMap.get(Servo.class, "coneflip");
        intakeflip = hardwareMap.get(Servo.class, "intakeflip");
        claw = hardwareMap.get(Servo.class, "claw");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        ShowOnTelemetry("0 Status Initialized");
        //telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        int slider_stepcount = 20;
        int slider_position = 0;
        int slider_initialposition = 0;
        int slider_targetposition = 0;
        double slider_power = 0.5;
        double slider_velocity = 1;

        int lift_busycounter = 0;
        int lift_stepcount = 20;
        int lift_position = 0;
        int lift_target_position = 0;
        double lift_power = 0.5;
        double lift_velocity = 0;
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {
            if (gamepad1.dpad_up){
                setConeflipPosition(ConeflipPosition.deliver);
            }
            if (gamepad1.dpad_down){
                setConeflipPosition(ConeflipPosition.intake);
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

            lift.setMotorEnable();
            slider.setMotorEnable();

            ShowOnTelemetry("1 set Motor Enabled");

            //check lift
            lift_position = lift.getCurrentPosition();
            ShowOnTelemetry(String.format("2 lift position %s", lift_position));

            if (gamepad1.y) {
                lift_position += lift_stepcount;
                if(lift_target_position < lift_position){
                    lift_target_position = lift_position;
                }
            } else if (gamepad1.a) {
                lift_target_position -= lift_stepcount;
            }
            ShowOnTelemetry(String.format("3 set lift target position %s", lift_target_position));
            lift.setTargetPosition(lift_target_position);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(0.7);

            //check slider
            slider_position = slider.getCurrentPosition();
            if (gamepad1.x) {
                slider_position += slider_stepcount;
            }
            if (gamepad1.b) {
                slider_position -= slider_stepcount;
            }
            ShowOnTelemetry(String.format("4 set slider target position %s", slider_position));
            slider.setTargetPosition(slider_position);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(slider_power);

            telemetry.update();

            while (slider.isBusy() && slider_velocity >= 0.001) {
                //slider_position = slider.getCurrentPosition();
                slider_velocity = slider.getVelocity();
                ShowOnTelemetry(String.format("5 slider busy slider position %s, velocity %s", slider_position, slider_velocity));
                telemetry.update();
            }

            while (lift.isBusy()) {
                ShowOnTelemetry(String.format("6 lift busy"));
                telemetry.update();

                //lift.setTargetPosition(lift_position);
            }
            slider_targetposition = slider_position;

            while (gamepad1.left_bumper) {

                //start the automatic process
                setConeflipPosition(ConeflipPosition.intake);
                slider.setTargetPosition(slider_initialposition);
                slider.setPower(slider_power);

                while (slider.isBusy()) {
                    slider_position = slider.getCurrentPosition();
                    ShowOnTelemetry(String.format("4 set slider target position %s", slider_position));
                }

                //loading the cone
                setClawPosition(ClawPosition.intake);
                sleep(500);
                setIntakeflipPosition(IntakePosition.up);
                sleep(800);
                setClawPosition(ClawPosition.release);
                sleep(500);
                //extend to target position
                slider.setTargetPosition(slider_targetposition);
                slider.setPower(slider_power);

                if (!gamepad1.left_bumper){
                    break;
                }

                while (slider.isBusy()) {
                    slider_position = slider.getCurrentPosition();
                    ShowOnTelemetry(String.format("4 set slider target position %s", slider_position));
                }
                setConeflipPosition(ConeflipPosition.deliver);
                setIntakeflipPosition(IntakePosition.down);
                sleep(1000);
            }
        }
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
        offset = Range.clip(offset, 0.3, 0.8);
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
        offset = Range.clip(offset, 0, 0.5);
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

        offset = Range.clip(offset, 0, 0.8);
        claw.setPosition(offset);
    }
    public void ShowOnTelemetry(String msg)
    {
        telemetry.log().add(msg);
    }
}