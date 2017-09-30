package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by CCA on 9/18/2017.
 */

@TeleOp (name="SophiaOp", group="myGroup")
public class SophiaOp extends OpMode {

    DcMotor left, right;

    public void init() {
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");

        right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);

        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void loop(){
        right.setPower(Math.pow(gamepad1.right_stick_y,3));
        left.setPower(Math.pow(gamepad1.right_stick_y,3));
    }
}
