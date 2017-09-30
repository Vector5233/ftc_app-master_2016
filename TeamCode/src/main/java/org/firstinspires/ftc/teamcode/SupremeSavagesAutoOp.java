import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Timer;

/**
 * Created by Olu Ogungbesan on 5/3/2017.
 */

@Autonomous (name="SupremeSavagesAutoOp", group = "MyGroup")
public class SupremeSavagesAutoOp extends LinearOpMode {

    DcMotor backRight;
    DcMotor backLeft;
    IrSeekerSensor IrSeeker;

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        stop();
    }
    public void initialize() {
        backRight = hardwareMap.dcMotor.get ("backRight");
        backLeft = hardwareMap.dcMotor.get ("backLeft");


        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void powerAllMotors(double power){

        backRight.setPower(power);
        backLeft.setPower(power);
        }


    public void forwardForDistance (int clicks, double power){
        resetAll();
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && backRight.getCurrentPosition() < clicks){
        powerAllMotors(power);
        }
        powerAllMotors(0);

        }

public void resetAll() {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        }