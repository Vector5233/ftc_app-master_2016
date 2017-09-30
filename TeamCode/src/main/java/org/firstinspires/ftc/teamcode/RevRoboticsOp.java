package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by CCA on 8/16/2017.
 */

@TeleOp(name="RevRoboticsOp", group = "myGroup")
public class RevRoboticsOp extends OpMode {


    DcMotor frontLeft, frontRight, backLeft, backRight;
    ColorSensor colorSensor;
    float red, green, blue;
    Servo jewelKnocker;

    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight= hardwareMap.dcMotor.get("backRight");

        colorSensor = hardwareMap.colorSensor.get("color");
        jewelKnocker= hardwareMap.servo.get("jewel");
        jewelKnocker.setPosition(0.1);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void loop () {

        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        /*
        Left.setPower(gamepad1.left_stick_y);
        Right.setPower(gamepad1.right_stick_y);
        */


        frontLeft.setPower(gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x);
        frontRight.setPower(gamepad1.right_stick_y+gamepad1.right_stick_x-gamepad1.right_stick_x);
        backLeft.setPower(gamepad1.left_stick_y+gamepad1.left_stick_x-gamepad1.right_stick_x);
        backRight.setPower(gamepad1.right_stick_y-gamepad1.right_stick_x+gamepad1.right_stick_x);


        telemetry.addData("Red: ", red);
        telemetry.addData("Green: ", green);
        telemetry.addData("Blue: ", blue);
        telemetry.update();

        if (gamepad1.a) {
            jewelKnocker.setPosition(-0.5);
        }
        else if (gamepad1.b) {
            jewelKnocker.setPosition(0.3);
        }
        else {
            ;
        }
    }






}
