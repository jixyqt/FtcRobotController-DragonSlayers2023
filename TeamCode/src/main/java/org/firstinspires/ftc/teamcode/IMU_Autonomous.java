package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;

@Autonomous(name="IMU Auto Test", group = "Driving")
public class IMU_Autonomous extends LinearOpMode {
    // Defining All the Hardware
    private DcMotor armMotor = null;
    private DcMotor left, right;
    Servo lancelotV2 = null;
    Servo armServo = null;
    CRServo claw = null;

    // IMU Stuffs
    IMU imu = null;
    IMU.Parameters myIMUparameters = null;
    // Grabbing the Mother Movements.
    private MotherAutonomous movements = new MotherAutonomous();
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<DcMotor> motors = new ArrayList<>();
        // Initalizing Hardware
        imu = hardwareMap.get(IMU.class, "imu");
        left = hardwareMap.get(DcMotor.class, "left");
        motors.add(left);
        right = hardwareMap.get(DcMotor.class, "right");
        motors.add(right);
        armServo = hardwareMap.get(Servo.class, "armServo");
        claw = hardwareMap.get(CRServo.class, "claw");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        lancelotV2 = hardwareMap.get(Servo.class, "lancelotV2");
        // Defining IMU Stuffs
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        YawPitchRollAngles robotOrientation;
        imu.resetYaw();
        waitForStart();
        while(opModeIsActive()) {

        }
    }
}
