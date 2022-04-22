package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends LinearOpMode {
    private Servo SB = null;        //Servo - Braço
    @Override
    public void runOpMode() {
        SB = hardwareMap.get(Servo.class, "SB");
        SB.setDirection(Servo.Direction.REVERSE) ;
        waitForStart();
        while (opModeIsActive()) {
            double i = SB.getPosition();
            if(gamepad2.a){
                SB.setPosition(i+0.1);
            }else if (gamepad2.b){
                SB.setPosition(i-0.1);
            }
            telemetry.addData("POSICAO SERVO: ", SB.getPosition());
            telemetry.update();
            sleep(1000);
        }
    }
}