import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.*;

@TeleOp
public class connveyorTest extends LinearOpMode {
    double move;
    DcMotor ConveyM;

    waitForStart();

    public void runOpMode() {
        ConveyM = hardwareMap.get(DcMotor.class, "cm");
        while(!isStopRequested()){

        move = -gamepad1.left_stick_y;
        ConveyM.setPower(move);

        }
    }
}
