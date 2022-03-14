package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class TesteColeta extends LinearOpMode {
    private DcMotor Coleta;
    public void runOpMode() {
        Coleta = hardwareMap.get(DcMotor.class, "Coleta");
        Coleta.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_bumper || gamepad1.left_bumper){
                Coleta.setPower(0);
            }
            if(gamepad1.a) {
                Coleta.setPower(0.5);
            }
            else if(gamepad1.b){
                Coleta.setPower(-0.5);
            }
            else if(gamepad1.y){
                Coleta.setPower(1.0);
            }
            else if(gamepad1.x){
                Coleta.setPower(-1.0);
            }
        }
    }
}
