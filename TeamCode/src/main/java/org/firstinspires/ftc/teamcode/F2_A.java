package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;


@Autonomous (name = "Red Side - F2")
public class F2_A extends LinearOpMode {

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
    // Grabbing the Mother Movements.
    private MotherAutonomous movements = new MotherAutonomous();


    public boolean within(double x, int y){
        return x >= y-10;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<DcMotor> motors = new ArrayList<>();
        // Motor Initialization!
        left = hardwareMap.get(DcMotor.class, "left");
        motors.add(left);
        right = hardwareMap.get(DcMotor.class, "right");
        motors.add(right);
        armServo = hardwareMap.get(Servo.class, "armServo");
        claw = hardwareMap.get(CRServo.class, "claw");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        lancelotV2 = hardwareMap.get(Servo.class, "lancelotV2");

        // Resetting motor encoder position to 0
        resetEncoders(motors);
        waitForStart();
        while(opModeIsActive()) {
            movements.raiseArm(175, armMotor);
            double leftCurrPos = left.getCurrentPosition();
            int leftTargetPos = left.getTargetPosition();
            double rightCurrPos = right.getCurrentPosition()
                    ;            int rightTargetPos = right.getTargetPosition();
            // Deem whether or not the current movement is done
            complete = within(leftCurrPos, leftTargetPos) && within(rightCurrPos, rightTargetPos);
            // Logic for figuring out where we are in sequence
            if(moving && complete && !move1Done) {
                lancelotV2.setPosition(0);
                moving = false;
                move1Done = true;
                resetEncoders(motors);
            } else if(turning && complete && !move2Done) {
                turning = false;
                move2Done = true;
                resetEncoders(motors);
            } else if(moving && complete && !move3Done) {
                turning = false;
                move3Done = true;
                resetEncoders(motors);
            }

            if (!move1Done){
                moving = true;
                movements.move(21.51,motors,true,left,right);// Is this the right direction?                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            , motors, true, left, right);
            } else if (!move2Done) {
                turning = true;
                movements.turn(90, motors, true);
            } else if (!move3Done) {
                moving = true;
                movements.move(31.5, motors, true, left, right);
            } else {
                moving = false;
                turning = false;
                setAllMotorPower(motors, 0);
                claw.setDirection(DcMotorSimple.Direction.REVERSE);
                claw.setPower(100);
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
