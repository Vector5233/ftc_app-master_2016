package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

/**
 * Created by CCA on 4/18/2017.
 */
@TeleOp(name="BigDaddyOp2", group = "myGroup")
public class BigDaddyOp extends OpMode {

    DcMotor right, left, boxer, lift;
    //IrSeekerSensor irSeeker;


    public void init() {
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        boxer = hardwareMap.dcMotor.get("boxer");
        lift = hardwareMap.dcMotor.get("lift");
        //irSeeker = hardwareMap.irSeekerSensor.get("irSeeker");

        right.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        boxer.setDirection(DcMotor.Direction.FORWARD);

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAll(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        right.setPower(gamepad1.right_stick_y);
        left.setPower(gamepad1.left_stick_y);
        if (gamepad1.x) {
            lift.setPower(0.5);
        }
        else if (gamepad1.a) {
            lift.setPower(-1);
        }
        else {
            lift.setPower(0);
        }


        if (gamepad1.y) {
            boxer.setPower(1);
        }
        else if (gamepad1.b) {
            boxer.setPower(-1);
        }
        else {
            boxer.setPower(0);
        }

        telemetry.addData("boxer @", boxer.getCurrentPosition());
        telemetry.addData("lift @,", lift.getCurrentPosition());
        //telemetry.addData("irSeeker @", irSeeker.getAngle());
    }

    public void setModeAll(DcMotor.RunMode RunMode) {
        right.setMode(RunMode);
        left.setMode(RunMode);
        lift.setMode(RunMode);
        boxer.setMode(RunMode);

    }


}
