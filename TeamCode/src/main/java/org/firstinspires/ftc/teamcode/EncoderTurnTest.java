package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
@TeleOp
public class EncoderTurnTest extends LinearOpMode {

    DcMotor left = null, right = null;
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<DcMotor> motors = new ArrayList<>();
        // Motor Initialization!
        left = hardwareMap.get(DcMotor.class, "left");
        motors.add(left);
        right = hardwareMap.get(DcMotor.class, "right");
        motors.add(right);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setTargetPosition(0);
        right.setTargetPosition(0);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Left Pos", left.getTargetPosition());
            telemetry.addData("Right Pos", right.getTargetPosition());
            telemetry.update();
            left.setPower(.1);
            right.setPower(.1);
            if(gamepad1.a) {
                left.setTargetPosition(left.getCurrentPosition()+5);
                right.setTargetPosition(right.getCurrentPosition()+5);
            } else if (gamepad1.b) {
                left.setTargetPosition(left.getCurrentPosition()-5);
                right.setTargetPosition(right.getCurrentPosition()-5);
            }
        }
    }
}
