package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TreinandoGyro extends LinearOpMode {
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;

    BNO055IMU imu;

    double speed;
    boolean toggle = false;

    public void runOpMode(){
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;
            double d = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            final double pi = Math.PI;
            double sen = y/d;
            double y2 = -gamepad1.right_stick_y;
            double d2 = Math.sqrt(Math.pow(x2,2) + Math.pow(y2,2));
            double sen2 = y2/d2;
            double angulorad = Math.asin(sen);
            double angulo = angulorad * (180/pi);
            double fMD = 0;
            double fME = 0;
            double forceMotorR = 0;
            double forceMotorL = 0;
            //Control Speed

            // gamepad1.b ? speed = 1 : gamepad1.a ? speed = 1/2 : gamepad1.x ? speed = 1/4 ;
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

            // Funções Mecanum

            //1º QUADRANTE
            if((angulo >= 0) && (angulo <= 90) && (x > 0)) {
                fME = (((4 / pi) * angulorad) - 1) * d;
                fMD = d;
            }
            //2º QUADRANTE
            else if ((angulo >= 0) && (angulo <= 90) && (x < 0)) {
                fME = d;
                fMD = (((4 / pi) * angulorad) - 1) * d;
            }
            //3º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x < 0)) {
                fME = (((4 / pi) * angulorad) + 1) * d ;
                fMD = d;
            }
            //4º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x > 0)) {
                fME = d;
                fMD = (((4 / pi) * angulorad) + 1) * d;
            }

            // Funções Normais

            //1º QUADRANTE
            if((x2 >= 0 ) && (y2 >= 0)){
                forceMotorR = ((2 * sen2) - 1) * d2;
                forceMotorL = d2;
            }
            // 2º QUADRANTE
            else if((x2 < 0 ) && (y2 >= 0)) {
                forceMotorR = d2;
                forceMotorL = ((2 * sen2) - 1) * d2;
            }
            // 3º QUADRANTE
            else if((x2 < 0 ) && (y2 < 0)){
                forceMotorR = -d2;
                forceMotorL = ((2 * sen2) + 1) * d2;
            }
            // 4º QUADRANTE
            else if((x2 >= 0 ) && (y2 < 0)) {
                forceMotorR = ((2 * sen2) + 1) * d2;
                forceMotorL = -d2;
            }

            if(fMD != 0 && fME != 0){
                RMF.setPower(fMD * speed);
                LMF.setPower(fME * speed);
                RMB.setPower(fME * speed);
                LMB.setPower(fMD * speed);
            }else{
                RMF.setPower(forceMotorR * speed);
                RMB.setPower(forceMotorR * speed);
                LMF.setPower(forceMotorL * speed);
                LMB.setPower(forceMotorL * speed);
            }

        }
    }
}
