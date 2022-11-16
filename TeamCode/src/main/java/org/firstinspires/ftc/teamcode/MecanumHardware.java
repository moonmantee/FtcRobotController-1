package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MecanumHardware {
    private LinearOpMode myOpMode = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    public ElapsedTime runtime = new ElapsedTime();

    public MecanumHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public void init() {
        leftBack = myOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = myOpMode.hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

    }

    /*    public void ForwardDistance(double Power,int distance) {
            if (Power > 1.0){
                Power = 1.0;
            }
            else if (Power < 1.0){
                Power = -1.0;
            }
            int Distance = (int) (distance/3*3.14)*560;
            leftBack.setTargetPosition(Distance);
            leftFront.setTargetPosition(Distance);
            rightBack.setTargetPosition(Distance);
            rightFront.setTargetPosition(Distance);
            leftFront.setPower(Power);
            rightFront.setPower(Power);
            leftBack.setPower(Power);
            rightBack.setPower(Power);
            while (myOpMode.opModeIsActive() && leftBack.isBusy()){
                telemetry.update();
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);*/
    public void ForwardDistance(double Power, int distance, double diameter, int gear_reduction) {

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Distance = leftBack.getCurrentPosition() + (int) ((distance / (diameter * 3.14)) * 28 * gear_reduction);

        leftBack.setTargetPosition(Distance);
        leftFront.setTargetPosition(Distance);
        rightBack.setTargetPosition(Distance);
        rightFront.setTargetPosition(Distance);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBack.setPower(Math.abs(Power));
        leftFront.setPower(Math.abs(Power));
        rightBack.setPower(Math.abs(Power));
        rightFront.setPower(Math.abs(Power));
        runtime.reset();
        while (leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", " %7d :%7d", Distance, Distance);
            myOpMode.telemetry.addData("Currently at", " at %7d :%7d", leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
            myOpMode.telemetry.update();
            myOpMode.sleep(1);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void ForwardTime(double Power, long Time) {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(Power);
        rightFront.setPower(Power);
        leftBack.setPower(Power);
        rightBack.setPower(Power);
        myOpMode.sleep(Time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void sideTime(double Power, long Time) {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(Power);
        rightFront.setPower(-Power);
        leftBack.setPower(-Power);
        rightBack.setPower(Power);
        myOpMode.sleep(Time);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public void sideDistance(double Power, int distance, double diameter, int gear_reduction) {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Distance = leftBack.getCurrentPosition() + (int) ((distance / (diameter * 3.14)) * 28 * gear_reduction);

        leftBack.setTargetPosition(Distance);
        leftFront.setTargetPosition(-Distance);
        rightBack.setTargetPosition(-Distance);
        rightFront.setTargetPosition(Distance);

        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBack.setPower(Math.abs(Power));
        leftFront.setPower(Math.abs(Power));
        rightBack.setPower(Math.abs(Power));
        rightFront.setPower(Math.abs(Power));
        runtime.reset();
        while (leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {

            // Display it for the driver.
            myOpMode.telemetry.addData("Running to", " %7d :%7d", Distance, Distance);
            myOpMode.telemetry.addData("Currently at", " at %7d :%7d", leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
            myOpMode.telemetry.update();
            myOpMode.sleep(1);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
}
