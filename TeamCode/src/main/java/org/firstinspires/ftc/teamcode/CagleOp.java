package org.firstinspires.ftc.teamcode;

/**
 * Created by Jeff Cagle on 2/21/2017.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CagleOp", group = "myGroup")

public class CagleOp extends OpMode {

    DcMotor backLeft, frontLeft, backRight, frontRight;

    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        frontRight = hardwareMap.dcMotor.get("frontRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {
       /* arcade style */
        frontRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        backRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        frontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        backLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);

        /* tank style
         * frontRight.setPower(gamepad1.right_stick_y);
         * backRight.setPower(gamepad1.right_stick_y);
         * frontLeft.setPower(gamepad1.left_stick_y);
         * backLeft.setPower(gamepad1.left_stick_y);
          */
    }
}
