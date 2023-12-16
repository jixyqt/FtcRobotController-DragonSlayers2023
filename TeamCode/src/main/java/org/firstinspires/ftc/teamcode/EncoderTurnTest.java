package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;
@TeleOp
public class EncoderTurnTest extends LinearOpMode {

    DcMotor left = null, right = null;
    TouchSensor touchSensor0 = null, touchSensor1 = null;
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<DcMotor> motors = new ArrayList<>();
        // Motor Initialization!
        left = hardwareMap.get(DcMotor.class, "left");
        motors.add(left);
        right = hardwareMap.get(DcMotor.class, "right");
        motors.add(right);
        touchSensor0 = hardwareMap.get(TouchSensor.class, "sensor_touch");
        touchSensor1 = hardwareMap.get(TouchSensor.class, "sensor_touch");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        boolean turned = false;
        while(opModeIsActive()) {
            telemetry.addData("Left Pos", left.getCurrentPosition());
            telemetry.addData("Right Pos", right.getCurrentPosition());
            telemetry.update();
            if(touchSensor0.isPressed()) {
                turned = true;
                left.setPower(0);
                right.setPower(0);
            }
            if(touchSensor1.isPressed()) {
                turned = true;
            }
            if(turned) {
                left.setPower(0);
                right.setPower(0);
            }
            if(!turned && gamepad1.a) {
                left.setDirection(DcMotor.Direction.FORWARD);
                right.setDirection(DcMotor.Direction.FORWARD);
                left.setPower(.1);
                right.setPower(.1);
            }
            if(!turned && gamepad1.a) {
                left.setDirection(DcMotor.Direction.REVERSE);
                right.setDirection(DcMotor.Direction.REVERSE);
                left.setPower(.1);
                right.setPower(.1);
            }
        }
    }
}
