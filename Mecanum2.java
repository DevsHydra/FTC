package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumTeste")
public class Mecanum2 extends LinearOpMode {

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

        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        GARRA1.setDirection(DcMotorSimple.Direction.REVERSE);
        GARRA2.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double TickAng = 1120 / 360;
                double gmax = TickAng * 90;
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double x2 = gamepad1.right_stick_x;
                double y2 = -gamepad1.right_stick_y;
                double Ang1 = GARRA1.getCurrentPosition() /360;
                double Ang2 = GARRA2.getCurrentPosition() /360;

//==================================================================================================
//SISTEMA
/*
                if((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0) || (GARRA1.getCurrentPosition() >= gmax) || (GARRA1.getCurrentPosition() <= 0) || (GARRA2.getCurrentPosition() >= gmax) || (GARRA2.getCurrentPosition() <= 0)){
                    GARRA1.setPower(0);
                    GARRA2.setPower(0);

                }
                if ((GARRA1.getCurrentPosition() < gmax) || (GARRA2.getCurrentPosition() < gmax) && (gamepad1.right_trigger != 0)) {
                    GARRA1.setPower(y2);
                    GARRA2.setPower(y2);
                }
                if ((GARRA1.getCurrentPosition() > 0) || (GARRA2.getCurrentPosition() > 0) && (gamepad1.left_trigger != 0)) {
                    GARRA1.setPower(-y2);
                    GARRA2.setPower(-y2);
                }

                GARRA1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.addLine()
                        .addData("Angulo1 - :", Ang1)
                        .addData("Angulo2 - ", Ang2)
                        .addData("Força1 - ", GARRA1.getPower())
                        .addData("Força2 - ", GARRA2.getPower());
 */
//==================================================================================================
//LOCOMOCAO
                if (gamepad1.right_bumper) {
                    RMF.setPower(((y - x) - x2) / 2);
                    LMF.setPower(((y + x) + x2) / 2);
                    RMB.setPower(((y + x) - x2) / 2);
                    LMB.setPower(((y - x) + x2) / 2);
                    telemetry.update();
                } else if (gamepad1.left_bumper) {
                    RMF.setPower(((y - x) - x2) / 4);
                    LMF.setPower(((y + x) + x2) / 4);
                    RMB.setPower(((y + x) - x2) / 4);
                    LMB.setPower(((y - x) + x2) / 4);
                    telemetry.update();
                } else {
                    RMF.setPower((y - x) - x2);
                    LMF.setPower((y + x) + x2);
                    RMB.setPower((y + x) - x2);
                    LMB.setPower((y - x) + x2);
                    telemetry.update();
                }
//==================================================================================================
            }
        }
    }
}