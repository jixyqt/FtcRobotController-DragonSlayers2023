package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.*;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

//TODO: This is the actual code that will be used for driving

@TeleOp(name="RoBits Drive", group = "Driving")
public class RoBitsDriveCode extends LinearOpMode {
    // Motor Define
    DcMotor leftSide = null;
    DcMotor rightSide = null;

    DcMotor armMotor = null;
    // Servo Define
    CRServo claw = null;
    Servo armServo = null;
    Servo planeServo = null;

    Servo lancelotV2 = null;

    // Camera
    //AprilTagProcessor myAprilTagProcessor;
    //VisionPortal.Builder myVisionPortalBuilder;
    //VisionPortal myVisionPortal;

    // Variables
    int goalPosition = 0;
    int goalIncrement = 2;
    double x = 0;
    // Autonomous Movement Copying
    private MotherAutonomous movements = new MotherAutonomous();

    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<DcMotor> motors = new ArrayList<>();
        // Motor Init
        leftSide = hardwareMap.get(DcMotor.class, "left");
        rightSide= hardwareMap.get(DcMotor.class, "right");
        motors.add(leftSide); motors.add(rightSide);
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Servo Init
        claw = hardwareMap.get(CRServo.class, "claw");
        armServo = hardwareMap.get(Servo.class, "armServo");
        lancelotV2 = hardwareMap.get(Servo.class, "lancelotV2");
        planeServo = hardwareMap.get(Servo.class, "planeServo");

        // TODO - Try to get camera working?
        /*
        // Camera
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Camera"));
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        Camera Customization
        myVisionPortalBuilder.setCameraResolution(new Size(1280, 720));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        myVisionPortalBuilder.enableLiveView(true);      // Enable LiveView (RC preview).         myVisionPortalBuilder.setAutoStopLiveView(false);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.
         */
        lancelotV2.setPosition(0);
        waitForStart();
        while(opModeIsActive()) {
            // Forward-Backward Motion
            if (gamepad1.left_stick_y < 0) {
                setArrayPower(motors, 2);
                leftSide.setDirection(DcMotorSimple.Direction.FORWARD);
                rightSide.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (gamepad1.left_stick_y > 0) {
                setArrayPower(motors, 2);
                leftSide.setDirection(DcMotorSimple.Direction.REVERSE);
                rightSide.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            // Turning Motion
            if (gamepad1.right_stick_x < 0) {
                setArrayPower(motors, 1.5);
                leftSide.setDirection(DcMotorSimple.Direction.REVERSE);
                rightSide.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (gamepad1.right_stick_x > 0) {
                setArrayPower(motors, 1.5);
                leftSide.setDirection(DcMotorSimple.Direction.FORWARD);
                rightSide.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
                setArrayPower(motors, 0);
            }
            //Plane Launcher

            if (gamepad2.a){
                planeServo.setDirection(Servo.Direction.FORWARD);
                planeServo.setPosition(1);
            } else if (gamepad2.b) {
                planeServo.setPosition(0);
            }



            // Claw
            if (gamepad2.left_trigger > 0) {
                claw.setDirection(DcMotorSimple.Direction.FORWARD);
                claw.setPower(100);
                lancelotV2.setPosition(.55);
            } else {
                lancelotV2.setPosition(0);
            }
            if (gamepad2.right_trigger > 0) {
                claw.setDirection(DcMotorSimple.Direction.REVERSE);
                claw.setPower(100);
            }
            if(gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
                claw.setPower(0);
            }
            // Arm Servos
            if (gamepad2.right_bumper) {
                armServo.setDirection(Servo.Direction.REVERSE);
                armServo.setPosition(.1);
            }
            if (gamepad2.left_bumper) {
                armServo.setDirection(Servo.Direction.FORWARD);
                armServo.setPosition(0);
            }
            // Arm Motor
            if (gamepad2.left_stick_y < 0) {
                goalPosition+=goalIncrement;
            }
            if (gamepad2.left_stick_y > 0) {
                if (goalPosition >= goalIncrement) {
                    goalPosition-=goalIncrement;
                }
            }
            // Encoder Running
            armMotor.setTargetPosition(goalPosition);
            if(armMotor.getCurrentPosition() > goalPosition) {
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                armMotor.setPower(0.5);
            }
            if(armMotor.getCurrentPosition() < goalPosition) {
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                armMotor.setPower(0.5);
            }
            if(armMotor.getCurrentPosition() == goalPosition) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armMotor.setPower(0.01);
            }

            telemetry.addData("Lancelot V2 Position", lancelotV2.getPosition());
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Target Arm Position", goalPosition);
            telemetry.update();
            if (gamepad2.dpad_up){
                goalPosition = 700;
            }
            if (gamepad2.dpad_down){
                goalPosition = 100;
            }
            if (gamepad2.dpad_left || gamepad2.dpad_right){
                goalPosition = 300;
            }
        }
    }

    public void setArrayPower(ArrayList<DcMotor> myMotors, double myPower) {
        for (DcMotor motor: myMotors) {
            motor.setPower(myPower);
        }
    }
}
