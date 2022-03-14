package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp
public class TestarMotor extends LinearOpMode {
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;
    private DcMotor GARRA1;
    private DcMotor GARRA2;

    @Override
    public void runOpMode() {
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        GARRA1 = hardwareMap.get(DcMotor.class, "GARRA1");
        GARRA2 = hardwareMap.get(DcMotor.class, "GARRA2");
        waitForStart();

        while (opModeIsActive()) {
        double jao = 0;
        double silvas = 0;

        GARRA1.setDirection(DcMotorSimple.Direction.REVERSE);
        GARRA2.setDirection(DcMotorSimple.Direction.FORWARD);

        if(gamepad1.left_stick_y > 0){
            GARRA1.setPower(0.1);
        }
        else if(gamepad1.left_stick_y < 0){
            GARRA1.setPower(-0.1);
        }
        else{
            GARRA1.setPower(0);
        }

        if(gamepad1.right_stick_y > 0){
            GARRA2.setPower(0.1);
        }
        else if(gamepad1.right_stick_y < 0){
            GARRA2.setPower(-0.1);
        }
        else{
            GARRA2.setPower(0);
        }

            // FRENTE
            if(gamepad1.dpad_up){
                silvas = 1;
            }
            // RE
            else if(gamepad1.dpad_down){
                silvas = - 1;
            }

            if(gamepad1.dpad_right){
                jao = 1;
            }
            if(gamepad1.dpad_left){
                jao = - 1;
            }


            if(gamepad1.right_bumper){
                jao /= 2;
                silvas /= 2;
            }
            if(gamepad1.left_bumper){
                jao /= 4;
                silvas /= 4;
            }
            LMF.setPower(silvas);
            LMB.setPower(silvas);
            RMF.setPower(silvas);
            RMB.setPower(silvas);
            GARRA1.setPower(jao);
            GARRA2.setPower(jao);


        }
    }
}