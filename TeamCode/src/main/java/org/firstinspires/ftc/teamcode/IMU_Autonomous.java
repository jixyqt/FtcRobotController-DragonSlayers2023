package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;

/*TODO: This is Modeled after A2_A
   We need to redo this so that the turning is more acurate as we can now use IMU to determine stuff
*/

@Autonomous(name="IMU Auto Test(A2_A)", group = "Driving")
public class IMU_Autonomous extends LinearOpMode {

    // Logic for Sequencing
    private boolean moving = false;
    private boolean move1Done = false;
    private boolean move2Done = false;
    private boolean move3Done = false;
    private boolean turning = false;
    private boolean complete = false;

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
    public boolean within(double x, int y){
        return x >= y;
    }
    public boolean within2(double x, int y) {
        return x<= y;
    }

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
        // Defining IMU
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(myIMUparameters);
        YawPitchRollAngles robotOrientation;
        imu.resetYaw();
        resetEncoders(motors);
        waitForStart();
        while(opModeIsActive()) {
            robotOrientation = imu.getRobotYawPitchRollAngles();
            double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            if(!move1Done) {
                moving = true;
                movements.move(21.5, motors, true, left, right);
            }
            else if(!move2Done) {
                turning = true;
                left.setDirection(DcMotorSimple.Direction.FORWARD);
                right.setDirection(DcMotorSimple.Direction.FORWARD);
                setAllMotorPower(motors, .1);
            }
            if(moving && !move1Done && within(left.getCurrentPosition(), left.getTargetPosition()) && within(right.getCurrentPosition(), right.getTargetPosition())) {
                move1Done = true;
                moving = false;
                imu.resetYaw();
                left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(turning && !move2Done && within2(Yaw, -90)) {
                move2Done = true;
                turning = false;
                setAllMotorPower(motors, 0);
            }

            // For Testing: Adds positions and goals to the driver hub
            telemetry.addData("Left Position", left.getCurrentPosition());
            telemetry.addData("Left Goal", left.getTargetPosition());
            telemetry.addData("Right Position", right.getCurrentPosition());
            telemetry.addData("Right Goal", right.getTargetPosition());
            telemetry.addData("Arm Position", armServo.getPosition());
            telemetry.addData("Move 1 Done", move1Done);
            telemetry.addData("Move 2 Done", move2Done);
            telemetry.addData("Move 3 Done", move3Done);
            telemetry.addData("Current Completed", complete);
            telemetry.addData("Yaw", Yaw);
            telemetry.update();
        }
    }

    public void resetEncoders(ArrayList<DcMotor> motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setAllMotorPower(ArrayList<DcMotor> motors, double power){
        // Through the power of arrays we make this teeny!
        for (DcMotor motor : motors){
            motor.setPower(power);
        }
    }
}
