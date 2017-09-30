/**
 * Created by CCA on 1/25/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

//@Autonomous(name="ServoRun", group="MyGroup")

public class ServoRun extends LinearOpMode {

    Servo gate;

    public void runOpMode() throws InterruptedException {
        gate = hardwareMap.servo.get("gate");

        waitForStart();

        while(opModeIsActive()) {
            gate.setPosition(1);
        }
    }



}
