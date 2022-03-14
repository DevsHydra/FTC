package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;

@Disabled
@TeleOp
public class HydraTeleOp extends LinearOpMode {
    private DcMotor rightMotorF;
    private DcMotor rightMotorB;
    private DcMotor leftMotorF;
    private DcMotor leftMotorB;

    @Override
    public void runOpMode(){
        rightMotorF = hardwareMap.get(DcMotor.class, "rightMotorF");
        rightMotorB = hardwareMap.get(DcMotor.class, "rightMotorB");
        leftMotorF = hardwareMap.get(DcMotor.class, "leftMotorF");
        leftMotorB = hardwareMap.get(DcMotor.class, "leftMotorB");

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double d = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            double sen = y/d;
            double forceMotorR = 0;
            double forceMotorL = 0;

            // 1º QUADRANTE
            if((x >= 0 ) && (y >= 0)){
                forceMotorR = ((2 * sen) - 1) * d;
                forceMotorL = d;
            }
            // 2º QUADRANTE
            else if((x < 0 ) && (y >= 0)) {
                forceMotorR = d;
                forceMotorL = ((2 * sen) - 1) * d;
            }
            // 3º QUADRANTE
            else if((x < 0 ) && (y < 0)){
                forceMotorR = -d;
                forceMotorL = ((2 * sen) + 1) * d;
            }
            // 4º QUADRANTE
            else if((x >= 0 ) && (y < 0)) {
                forceMotorR = ((2 * sen) + 1) * d;
                forceMotorL = -d;
            }

            leftMotorF.setPower(-forceMotorL);
            leftMotorB.setPower(-forceMotorL);
            rightMotorF.setPower(forceMotorR);
            rightMotorB.setPower(forceMotorR);
        }
    }
}