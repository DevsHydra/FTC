package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Apresentacao extends LinearOpMode {

    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;

    @Override
    public void runOpMode() {
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");

        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double TickAng = 1120 / 360;
                double gmax = TickAng * 90;
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double x2 = gamepad1.right_stick_x;
                double y2 = -gamepad1.right_stick_y;

//==================================================================================================
//LOCOMOCAO
                if (gamepad2.right_bumper) {
                    RMF.setPower(((y - x) - x2) / 2);
                    LMF.setPower(((y + x) + x2) / 2);
                    RMB.setPower(((y + x) - x2) / 2);
                    LMB.setPower(((y - x) + x2) / 2);
                    telemetry.update();
                } else if (gamepad2.left_bumper) {
                    RMF.setPower(((y - x) - x2) / 4);
                    LMF.setPower(((y + x) + x2) / 4);
                    RMB.setPower(((y + x) - x2) / 4);
                    LMB.setPower(((y - x) + x2) / 4);
                    telemetry.update();
                } else {
                    RMF.setPower(((y - x) - x2)/1.5);
                    LMF.setPower(((y + x) + x2)/1.5);
                    RMB.setPower(((y + x) - x2)/1.5);
                    LMB.setPower(((y - x) + x2)/1.5);
                    telemetry.update();
                }
//==================================================================================================
            }
        }
    }
}