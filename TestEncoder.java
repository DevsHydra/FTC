package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestEncoder extends LinearOpMode {
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;
    static final double MOTOR_TICK_COUNT_GOBILDA =	1425.1; // GOBILDA 5202 SERIE 117 RPM

    //private DcMotor ML;

    double speed;
    boolean toggle = false;

    @Override
    public void runOpMode() {
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        //ML = hardwareMap.get(DcMotor.class, "ML");
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        //ML.setDirection(DcMotorSimple.Direction.FORWARD);
        ZeroEncoder();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;
            double d = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
            final double pi = Math.PI;
            double sen = y / d;
            double y2 = -gamepad1.right_stick_y;
            double d2 = Math.sqrt(Math.pow(x2, 2) + Math.pow(y2, 2));
            double sen2 = y2 / d2;
            double angulorad = Math.asin(sen);
            double angulo = angulorad * (180 / pi);
            double fMD = 0;
            double fME = 0;
            double forceMotorR = 0;
            double forceMotorL = 0;


            //double mlf = 0;


            //Control Speed

            // gamepad1.b ? speed = 1 : gamepad1.a ? speed = 1/2 : gamepad1.x ? speed = 1/4 ;
            if (!toggle) {
                if (gamepad1.x) {
                    speed = 1 / 4;
                    toggle = true;
                }
                if (gamepad1.a) {
                    speed = 1 / 2;
                    toggle = true;
                }
                if (gamepad1.b) {
                    speed = 1;
                    toggle = true;
                }
            } else if (!gamepad1.x && !gamepad1.a && !gamepad1.b) toggle = false;

            // Funções Mecanum

            //1º QUADRANTE
            if ((angulo >= 0) && (angulo <= 90) && (x > 0)) {
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
                fME = (((4 / pi) * angulorad) + 1) * d;
                fMD = d;
            }
            //4º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x > 0)) {
                fME = d;
                fMD = (((4 / pi) * angulorad) + 1) * d;
            }

            // Funções Normais

            //1º QUADRANTE
            if ((x2 >= 0) && (y2 >= 0)) {
                forceMotorR = ((2 * sen2) - 1) * d2;
                forceMotorL = d2;
            }
            // 2º QUADRANTE
            else if ((x2 < 0) && (y2 >= 0)) {
                forceMotorR = d2;
                forceMotorL = ((2 * sen2) - 1) * d2;
            }
            // 3º QUADRANTE
            else if ((x2 < 0) && (y2 < 0)) {
                forceMotorR = -d2;
                forceMotorL = ((2 * sen2) + 1) * d2;
            }
            // 4º QUADRANTE
            else if ((x2 >= 0) && (y2 < 0)) {
                forceMotorR = ((2 * sen2) + 1) * d2;
                forceMotorL = -d2;
            }

            // Funções Linear
            /*
            if(!toggle) {
                if(gamepad2.x) {
                    mlf = 0.3;
                    toggle = true;
                }
                if(gamepad2.a) {
                    mlf = 0.5;
                    toggle = true;
                }
                if(gamepad2.b) {
                    mlf = 0.7;
                    toggle = true;
                }
            } else if(!gamepad2.x && !gamepad2.a && !gamepad2.b) toggle = false;


            if(gamepad2.dpad_up) {
                ML.setPower(mlf);
            }
            else if(gamepad2.dpad_down){
                ML.setPower(-mlf);
            }

             */


            // Set Force

            if (fMD != 0 && fME != 0) {
                RMF.setPower(fMD * speed);
                LMF.setPower(fME * speed);
                RMB.setPower(fME * speed);
                LMB.setPower(fMD * speed);
            } else {
                RMF.setPower(forceMotorR * speed);
                RMB.setPower(forceMotorR * speed);
                LMF.setPower(forceMotorL * speed);
                LMB.setPower(forceMotorL * speed);
            }

            // Test Esncoder
            if(gamepad1.left_bumper){
                ZeroEncoder();
            }

            if(gamepad1.right_bumper){

                LMF.setTargetPosition((int)MOTOR_TICK_COUNT_GOBILDA/4);
                LMB.setTargetPosition((int)MOTOR_TICK_COUNT_GOBILDA/4);
                RMF.setTargetPosition((int)MOTOR_TICK_COUNT_GOBILDA/4);
                RMB.setTargetPosition((int)MOTOR_TICK_COUNT_GOBILDA/4);

                LMF.setPower(0.5);
                LMB.setPower(0.5);
                RMF.setPower(0.5);
                RMB.setPower(0.5);

                LMF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LMB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RMF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RMB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(LMF.isBusy() || LMB.isBusy() || RMF.isBusy() || RMB.isBusy()){
                telemetry.addLine()
                        .addData("Atual1:", LMF.getCurrentPosition())
                        .addData("Atual2:", LMB.getCurrentPosition())
                        .addData("Atual1:", RMF.getCurrentPosition())
                        .addData("Atual2:", RMB.getCurrentPosition())

                        .addData("Atual1:", LMF.getTargetPosition())
                        .addData("Atual2:", LMB.getTargetPosition())
                        .addData("Atual1:", RMF.getTargetPosition())
                        .addData("Atual2:", RMB.getTargetPosition());
                telemetry.update();

            }

        }
    }

    public void ZeroEncoder(){
        LMF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LMB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}

/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TeleOpFinal extends LinearOpMode {

                    }
                    }

    private DcMotor GARRA1;
    private DcMotor GARRA2;
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;

    static final double MOTOR_TICK_COUNT = 288; // CORE HEX
    static final double MOTOR_TICK_COUNT_HDHEX = 1120; // HD HEX 40:1


    @Override
    public void runOpMode(){
        GARRA1 = hardwareMap.get(DcMotor.class, "GARRA1");
        GARRA2 = hardwareMap.get(DcMotor.class, "GARRA2");
        waitForStart();

        boolean qqw = true;

        GARRA1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //GARRA2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (qqw) {
            getRuntime();
            //GARRA1.setDirection(DcMotorSimple.Direction.REVERSE);
            //GARRA2.setDirection(DcMotorSimple.Direction.FORWARD);
            double quarterTurn = 1120 / 16;
            double TickAang = 1120/360;

            //double newTarget1 = GARRA1.getCurrentPosition() + quarterTurn;
            //double newTarget2 = GARRA2.getCurrentPosition() + quarterTurn;

            if((GARRA1.getCurrentPosition() != newTarget1) && (GARRA2.getCurrentPosition() != newTarget2)) {
                GARRA1.setTargetPosition((int) newTarget1);
                GARRA2.setTargetPosition((int) newTarget2);

                GARRA1.setPower(1);
                GARRA2.setPower(1);

                GARRA1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                GARRA2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            while (GARRA1.isBusy() || GARRA2.isBusy()) {
                telemetry.addLine()
                        .addData("Atual1:", GARRA1.getCurrentPosition())
                        .addData("Atual2:", GARRA2.getCurrentPosition())
                        .addData("Meta1", GARRA1.getTargetPosition())
                        .addData("Meta2", GARRA2.getTargetPosition())
                        .addData("Tempo:", time);
                telemetry.update();
                if (getRuntime() > 15) {
                    GARRA1.setPower(0);
                    GARRA1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    GARRA2.setPower(0);
                    GARRA2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    qqw = false;
                    stop();
                }
            }
        }
        stop();
    }
        */