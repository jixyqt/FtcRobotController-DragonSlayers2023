package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;


public class MotherAutonomous {

    // Defining All the Motors!!
    private DcMotor left, right;
    Servo armServo = null;


    public void resetEncoders(ArrayList<DcMotor> motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // For whatever reason, 10 inches is around 1 square according to this math.
    public void move(double inchGoal, ArrayList<DcMotor> motors, boolean direction, DcMotor left, DcMotor right) {
        double y;
        // 3 is the diameter of the wheel, and multiplying by pi gets us the circumference.
        y = 1440*(inchGoal/(3*Math.PI));
        for (DcMotor motor : motors) {
            motor.setTargetPosition((int)y);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (direction) {
            left.setDirection(DcMotor.Direction.FORWARD);
            right.setDirection(DcMotor.Direction.REVERSE);
        } else {
            left.setDirection(DcMotor.Direction.REVERSE);
            right.setDirection(DcMotor.Direction.FORWARD);
        }
        setAllMotorPower(motors, .3);

    }

    public void setAllMotorPower(ArrayList<DcMotor> motors, double power){
        // Through the power of arrays we make this teeny!
        for (DcMotor motor : motors){
            motor.setPower(power);
        }
    }

    public void turn(int degreeGoal, ArrayList<DcMotor> motors, boolean direction) {
        double y;
        left = motors.get(0);
        right = motors.get(1);
        if(degreeGoal == 90) {
            left.setTargetPosition(610);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setTargetPosition(610);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            y = 1440/((double)(degreeGoal/33.5));
            for (DcMotor motor : motors) {
                motor.setTargetPosition((int)y);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        if (direction) {
            for (DcMotor motor : motors) {
                motor.setDirection(DcMotor.Direction.FORWARD);
            }
        } else {
            for (DcMotor motor : motors) {
                motor.setDirection(DcMotor.Direction.REVERSE);
            }
        }
        setAllMotorPower(motors, .1);

    }
    public void raiseArm(int goalPosition, DcMotor armMotor){
        armMotor.setTargetPosition(-goalPosition);
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
    }
}
