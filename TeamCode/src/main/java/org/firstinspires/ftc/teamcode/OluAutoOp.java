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
 * Created by Olu Ogungbesan on 3/6/2017.
 */
@Autonomous (name="OluAutoOp", group = "MyGroup")
public class OluAutoOp extends LinearOpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    //IrSeekerSensor IrSeeker;

    public void runOpMode() throws InterruptedException{
        initialize();
        waitForStart();
        forwardForDistance(2000, 0.5);
        //forwardForTime(4,.5);
        //moveLeft();
        //readIrSeeker();







    }
    public void initialize(){
        frontRight = hardwareMap.dcMotor.get ("frontRight");
        frontLeft = hardwareMap.dcMotor.get ("frontLeft");
        backRight = hardwareMap.dcMotor.get ("backRight");
        backLeft = hardwareMap.dcMotor.get ("backLeft");
        //IrSeeker = hardwareMap.irSeekerSensor.get ("IrSeeker");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


    }
    public void powerAllMotors(double power){
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    public void forwardForTime(double time, double power) {
        ElapsedTime myTimer = new ElapsedTime();

        myTimer.reset();
        while (opModeIsActive() && myTimer.time() < time) {
            powerAllMotors(power);
        }
        powerAllMotors(0);
    }

    public void forwardForDistance (int clicks, double power){
        resetAll();
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      while (opModeIsActive() && frontRight.getCurrentPosition() < clicks){
          powerAllMotors(power);
      }
        powerAllMotors(0);

    }

    public void resetAll() {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
