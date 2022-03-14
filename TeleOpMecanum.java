package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp
public class TeleOpMecanum extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor RMF;
    private DcMotor RMB;
    private DcMotor LMF;
    private DcMotor LMB;
    boolean mecanumButton = false;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF = hardwareMap.get(DcMotor.class, "RMF");
        LMB = hardwareMap.get(DcMotor.class, "RMB");
        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double d = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            final double pi = Math.PI;
            double cos = x/d;
            double sen = y/d;
            double y2 = -gamepad1.right_stick_y;
            double x2 = gamepad1.right_stick_x;
            double d2 = Math.sqrt(Math.pow(x2,2) + Math.pow(y2,2));
            double sen2 = y2/d2;
            double forceMotorR = 0;
            double forceMotorL = 0;
            double forceMotorVE = 0;
            double forceMotorVD = 0;
            double angulorad = Math.asin(sen2);
            double angulo = angulorad * (180/pi);
            boolean butonY = gamepad1.y;

            // if(butonY) this.mecanumButton = !this.mecanumButton;
            //FUNÇÕES NORMAIS
            // 1º QUADRANTE
            // if((x >= 0 ) && (y >= 0)){
            //     forceMotorR = ((2 * sen) - 1) * d;
            //     forceMotorL = d;
            // }
            // // 2º QUADRANTE
            // else if((x < 0 ) && (y >= 0)) {
            //     forceMotorR = d;
            //     forceMotorL = ((2 * sen) - 1) * d;
            // }
            // // 3º QUADRANTE
            // else if((x < 0 ) && (y < 0)){
            //     forceMotorR = -d;
            //     forceMotorL = ((2 * sen) + 1) * d;
            // }
            // // 4º QUADRANTE
            // else if((x >= 0 ) && (y < 0)) {
            //     forceMotorR = ((2 * sen) + 1) * d;
            //     forceMotorL = -d;
            // }
            
            //FUNÇÕES MECANUM
            
            //1º QUADRANTE
            if((angulo >= 0) && (angulo <= 90) && (x2 > 0)) {
                forceMotorVE = (((4 / pi) * angulorad) - 1) * d2;
                forceMotorVD = d2;
            }
            //2º QUADRANTE
            else if ((angulo >= 0) && (angulo <= 90) && (x2 < 0)) {
                forceMotorVE = d2;
                forceMotorVD = (((4 / pi) * angulorad) - 1) * d2;
            }
            //3º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x2 < 0)) {
                forceMotorVE = (((4 / pi) * angulorad) + 1) * d2 ;
                forceMotorVD = d2;
            }
            //4º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x2 > 0)) {
                forceMotorVE = d2;
                forceMotorVD = (((4 / pi) * angulorad) + 1) * d2;
            }
            
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

            // if(!mecanumButton){
            //     RMF.setPower(forceMotorL);
            //     RMB.setPower(forceMotorL);
            //     LMF.setPower(forceMotorR);
            //     LMB.setPower(forceMotorR);
            // }else{
                RMF.setPower(forceMotorVE);
                RMB.setPower(forceMotorVD);
                LMF.setPower(forceMotorVD);
                LMB.setPower(forceMotorVE);
            //}
            telemetry.addData("Status ", "Running");
            telemetry.addData("Mecanum? ",mecanumButton);
            telemetry.update();
        }
    }
}