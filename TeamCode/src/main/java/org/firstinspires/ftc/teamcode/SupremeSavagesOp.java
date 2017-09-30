import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Olu Ogungbesan on 3/28/2017.
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

@TeleOp (name="SupremeSavagesOp" , group = "myGroup")
public class SupremeSavagesOp extends OpMode {
    DcMotor backLeft, backRight, liftMotor;
    Servo extender;
    public void init(){
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        extender = hardwareMap.servo.get("extender");

        backLeft.setDirection(DcMotor.Direction.REVERSE );
        backRight.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop(){
        backRight.setPower(gamepad1.left_stick_y);
        backLeft.setPower(gamepad1.right_stick_y);

        if (gamepad1.y){
            liftMotor.setPower(1);
        }
        else if (gamepad1.a){
            liftMotor.setPower(-1);
        }
        else {
            liftMotor.setPower(0);
        }
        if (gamepad1.x) {
            extender.setPosition(1);
        }
        else if (gamepad1.b){
            extender.setPosition(0);
        }
        else {
            extender.setPosition(.5);
        }
    }

}
