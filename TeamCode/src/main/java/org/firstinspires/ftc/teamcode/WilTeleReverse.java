import android.graphics.Bitmap;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name="WilTeleReverse", group = "myGroup")

/**
 * Created Jan 17, 2017 by Wil Orlando
 *
 * Normal teleop, but reversed controls (so that the collector is the "front" while driving)
 *
 */

public class WilTeleReverse extends OpMode {

    // Declare motors, servos, and variables
    DcMotor frontRight, frontLeft, backRight, backLeft, flicker, collector;
    Servo gate;
    ModernRoboticsI2cGyro gyro;

    String gateState = "";

    boolean targetSet = false;
    int encVal, targetVal;
    int FLICKER_ERROR = 184; // Number of encoder ticks the physical flicker runs past the target value; last measured as between 104 and 181

    public ElapsedTime runTime = new ElapsedTime();
    double timeMark = 0;

    // Initialize robot before match starts
    public void init() {

        // Map this program's names for motors to the RC Phone's config file
        frontRight = hardwareMap.dcMotor.get ("frontRight");
        frontLeft = hardwareMap.dcMotor.get ("frontLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        backLeft = hardwareMap.dcMotor. get("backLeft");
        flicker = hardwareMap.dcMotor.get("flicker");
        collector = hardwareMap.dcMotor.get("collector");
        gate = hardwareMap.servo.get("gate");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        // Reverse motors that are physically reversed on bot (so that setPower(1) means "forward" for every motor)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        // Make sure that drive motors are using encoders
        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reset flicker encoder to 0, then set it to run using encoders
        flicker.setDirection(DcMotor.Direction.REVERSE);
        flicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize flicker's target value for one rotation to be its current position, so that it doesn't start flicking if the reset failed (for some reason)
        targetVal = flicker.getCurrentPosition();

        // Calibrate gyro to take initial bearings as references for axes
        gyro.calibrate();

        // POTENTIAL TROUBLE!  EMPTY LOOP MAY NOT SYNCHRONIZE
        while (gyro.getIntegratedZValue() != 0 && (runTime.time() - timeMark) < 1.5) {

        }

        telemetry.addLine("Fully Initialized!");
    }

    public void loop() {

        telemetry.addData(">", "Robot Heading Z = %d", gyro.getIntegratedZValue());

        // Drive Controls //

        // Normal tank drive, binding the left wheels to the left joystick and right wheels to right
        frontLeft.setPower(gamepad1.right_stick_y);
        backLeft.setPower(gamepad1.right_stick_y);
        frontRight.setPower(gamepad1.left_stick_y);
        backRight.setPower(gamepad1.left_stick_y);


        // Gate Controls //

        // Opens gate when x is pressed
        if (gamepad2.x) {
            gate.setPosition(.3); // Last set at .3
            gateState = "Open";
        }
        // Closes gate by default
        else {
            gate.setPosition(0); // Last set at 0
            gateState = "Closed";
        }

        telemetry.addData("Gate: ", gateState);


        // Collector Controls //

        // Collector out when dpad_up is pressed
        if (gamepad2.dpad_up) {
            collector.setPower(1);
        }
        // Collector in when dpad_down is pressed
        else if (gamepad2.dpad_down) {
            collector.setPower(-1);
        }
        // Collector off by default
        else {
            collector.setPower(0);
        }


        // Flicker Controls //

        //Working note: flicker.setPower(1) is the right direction

        if (gamepad2.y && flicker.getPower() == 0) {
            encVal = Math.abs(flicker.getCurrentPosition());
            targetVal = encVal + 3360;

            flicker.setTargetPosition((targetVal - FLICKER_ERROR));
            timeMark = runTime.time();
            flicker.setPower(1);
        }
        // Manual reset, in case the flicker gets stuck, to rotate it a third of a rotation
        else if (gamepad2.right_bumper && flicker.getPower() == 0) {
            encVal = Math.abs(flicker.getCurrentPosition());
            targetVal = encVal + 3360/3;

            flicker.setTargetPosition((targetVal - FLICKER_ERROR));
            timeMark = runTime.time();
            flicker.setPower(1);
        }

        if ((runTime.time() - timeMark) >= 2) {
            flicker.setPower(0);
        }

        telemetry.addData("Flicker Enc: ", flicker.getCurrentPosition());
        telemetry.addData("Flicker Power: ", flicker.getPower());
    }



    // Sets all drive motors to a specified RunMode
    public void setModeAll(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }
}

