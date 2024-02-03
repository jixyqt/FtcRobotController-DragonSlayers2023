package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="IMU Test", group = "Driving")
public class imu_test extends LinearOpMode {
    IMU.Parameters myIMUparameters;
    IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUparameters);
        YawPitchRollAngles robotOrientation;
        imu.resetYaw();
        waitForStart();
        while(opModeIsActive()) {
            robotOrientation = imu.getRobotYawPitchRollAngles();
            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
            telemetry.addData("Yaw", Yaw);
            telemetry.addData("Pitch", Pitch);
            telemetry.addData("Roll", Roll);
            telemetry.update();
        }
    }
}
