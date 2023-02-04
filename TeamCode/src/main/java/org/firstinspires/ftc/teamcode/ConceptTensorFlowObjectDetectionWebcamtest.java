/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

//@TeleOp(name = "Auto_test", group = "Linear Opmode")
@Autonomous(name="Robot: Auto_test", group="Robot")
//@Disabled
public class ConceptTensorFlowObjectDetectionWebcamtest extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "1dot2dot3dot.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/jonahsbadleftarm.tflite";
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private Servo intakeflip=null;

    float speed = 0;
    float turn = 0;
    float strafe = 0;
    int creepincrement = 4;
    boolean WorkDone = false;
    int lb_initposition = 0;
    int lb_loc2position = 0;
    int lb_loc1position = 0;


    private static final String[] LABELS = {
            "1dot",
            "2dot",
            "3dot"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Acc1Mkr/////AAABmY3MIgzTk0/vv/F6wi5F2MM+sGvjRSSjmHCxMpBjeH1FQakUSFyh5mdAiP5j69oymjPBaXTYy1XCy5UxSPU6jT1lqmVUG/z0bLDgFOxA8QQpym48FwmkGNBetyCwknfUG5QnfYDt5s9K1A/neXh+tNGMrfFX9c0JiIV8INPoDzOFyL7AO7hZ5+6War/ZQIPNSYu2RMK7owq4d6MBmGSOHE/OMjK3cEQcVufOcA3u9nX1qJCob1MGNRiG4mnkNR8d8RWHiq6rzJeH+GhIOQakqPWAMojrV5o3L9+QOQgEsiOG4hoey+jwnw8hoFbGAKHwpyIS9OedMgkKzHAkHdpjKaJhOYKeOM9jLs4ab2InSlvR";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        MecanumHardware robot = new MecanumHardware(this);
        robot.init();
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        intakeflip = hardwareMap.get(Servo.class, "intakeflip");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive() && !WorkDone) {
                setIntakeflipPosition(TeleopMultiThread.IntakePosition.up);
                sleep(1000);
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                            if (recognition.getLabel() == "2dot"){
                                telemetry.addData("Label", "2dot");
                                telemetry.update();
                                parking2();
                                sleep(10000);

                                WorkDone = true;
                                break;
                            }
                            else if(recognition.getLabel() == "1dot") {
                                telemetry.addData("Label", "1dot");
                                telemetry.update();
                                parking1();
                                sleep(10000);

                                WorkDone = true;
                                break;
                            }
                            else if(recognition.getLabel() == "3dot"){
                                telemetry.addData("Label", "3dot");
                                telemetry.update();
                                parking3();
                                sleep(10000);

                                WorkDone = true;
                                break;
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    //move straight, then move left
    public void parking1() {
        parking2();

        speed = 0;
        //turn = gamepad2.right_stick_x;
        strafe = -1;

        double LB = (speed + turn - strafe)/creepincrement;
        double LF = (speed + turn + strafe)/creepincrement;
        double RB = (speed - turn + strafe)/creepincrement;
        double RF = (speed - turn - strafe)/creepincrement;
        while (leftBack.getCurrentPosition() < 3200){
            leftBack.setPower(LB);
            leftFront.setPower(LF);
            rightBack.setPower(RB);
            rightFront.setPower(RF);
            idle();
        }
        lb_loc1position = leftBack.getCurrentPosition();
        telemetry.addData("2pos",lb_loc2position);
        telemetry.addData("1pos",lb_loc1position);
        telemetry.update();
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }

    //move straight
    public void parking2() {
        speed = 1;
        //turn = gamepad2.right_stick_x;
        strafe = 0; //gamepad2.left_stick_x;

        double LB = (speed + turn - strafe)/creepincrement;
        double LF = (speed + turn + strafe)/creepincrement;
        double RB = (speed - turn + strafe)/creepincrement;
        double RF = (speed - turn - strafe)/creepincrement;
        while(leftBack.getCurrentPosition() < 1200){
            leftBack.setPower(LB);
            leftFront.setPower(LF);
            rightBack.setPower(RB);
            rightFront.setPower(RF);
            idle();
        }
        lb_loc2position = leftBack.getCurrentPosition();
        telemetry.addData("position", leftBack.getCurrentPosition());
        //after was encoder position 3774
        telemetry.update();
        sleep(1000);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }

    //move straight, then move right
    public void parking3() {

        parking2();
        sleep(500);
        speed = 0;
        //turn = gamepad2.right_stick_x;
        strafe = 1;

        double LB = (speed + turn - strafe)/creepincrement;
        double LF = (speed + turn + strafe)/creepincrement;
        double RB = (speed - turn + strafe)/creepincrement;
        double RF = (speed - turn - strafe)/creepincrement;

        while(leftBack.getCurrentPosition()<3200){
            leftBack.setPower(LB);
            leftFront.setPower(LF);
            rightBack.setPower(RB);
            rightFront.setPower(RF);
        }
        lb_loc1position = leftBack.getCurrentPosition();
        telemetry.addData("2pos",lb_loc2position);
        telemetry.addData("1pos",lb_loc1position);
        telemetry.update();
        sleep(1000);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

    }

    enum IntakePosition{
        up,
        down
    }
    public void setIntakeflipPosition(TeleopMultiThread.IntakePosition position) {
        double offset = 0;
        if (position == TeleopMultiThread.IntakePosition.up)
            offset = 1;
        else
            offset = 0;
        offset = Range.clip(offset, 0, 0.5);
        intakeflip.setPosition(offset);
    }
}
