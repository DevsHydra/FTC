package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TeleOpFinal extends LinearOpMode {
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;

    double speed;
    boolean toggle = false;
//    int stateSpeed = 1; //1 100%, 2 50%, 3 25%.

    public void runOpMode(){
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;

            if(!toggle) {
                if(gamepad1.x) {
                    speed = 1 / 4;
                    toggle = true;
                }
                if(gamepad1.a) {
                    speed = 1 / 2;
                    toggle = true;
                }
                if(gamepad1.b) {
                    speed = 1;
                    toggle = true;
                }
            } else if(!gamepad1.x && !gamepad1.a && !gamepad1.b) toggle = false;


            RMF.setPower(((y - x) - x2) * speed);
            LMF.setPower(((y + x) + x2) * speed);
            RMB.setPower(((y + x) - x2) * speed);
            LMB.setPower(((y - x) + x2) * speed);

        }
    }
}
