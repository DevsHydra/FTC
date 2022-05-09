package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Robot extends LinearOpMode {
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;
    Orientation angles;
    HardwareMap hwMap = null;
    BNO055IMU imu;
    double speed = 1;
    long timer = System.currentTimeMillis();
    int iGravacao = 0;
    boolean gravar = false;
    boolean gravar2 = false;
    double valorArmazenado[][] = new double[500][4];
    double[] forcaMotor = {0, 0}; // 0 ESQUERDA, E 1 DIREITA EM RELAÇÃO A MECANUM.
    double[] forcax = {0, 0}; // 0 ESQUERDA E 1 DIREITA
    long timer2 = System.currentTimeMillis();
    boolean executar = false;
    ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void runTeleOp() {
        initHardware(hwMap);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if(System.currentTimeMillis() - timer < 20) {
                    continue;
                }
                timer = System.currentTimeMillis();
                // Declarando Vetores joystick esquerdo, [0] = alavanca esquerdo, [1] alavanca direita
                double[] joyY = {-gamepad1.left_stick_y, -gamepad1.right_stick_y,};
                double[] joyX = {gamepad1.left_stick_x, gamepad1.right_stick_x,};
                double[] d = {Math.hypot(joyX[0], joyY[0]), Math.hypot(joyX[1], joyY[1])};
                double[] angulorad = {Math.asin(joyY[0] / d[0]), Math.asin(joyY[1] / d[1])};
                double[] angulo = {Math.toDegrees(angulorad[0]), Math.toDegrees(angulorad[1])};
                double forcay = 0;
                double[] anguloJoy = {0, 0}; // O ESQUERDA , 1 DIREITA
                double[] anguloReal = {0, 0}; // O ESQUERDA E 1 DIREITA
                double anguloRobo = getAngle();

                //VARIAVEIS E CONSTANTES
                final double PI = Math.PI;


                speed = gamepad1.b ? 1 : gamepad1.a ? 0.5 : gamepad1.x ? 0.25 : speed;

                gravacao();
                if (gamepad1.dpad_up) {
                    if (executar) {
                        setPowerGravado();
                    }
                    executar = false;
                } else {
                    executar = true;
                }

                anguloJoy[0] = getAngleJoyEsquerdo(angulo[0], joyX[0]);
                anguloJoy[1] = getAngleJoyDireito(joyY[1], joyX[1], angulo[1]);

                forcay = forceyJoy(anguloRobo, anguloJoy[0], d[0]);
                forcax[0] = forcexJoy(anguloRobo, anguloJoy[0], d[0]);
                forcax[1] = forcexJoy(anguloRobo, anguloJoy[1], d[1]);

                forcaMotor[0] = getForce(d[0], forcax[0], forcay, "ESQUERDA");
                forcaMotor[1] = getForce(d[0], forcax[0], forcay, "DIREITA");

                RMF.setPower((forcaMotor[1] - forcax[1]) * speed);
                LMF.setPower((forcaMotor[0] + forcax[1]) * speed);
                RMB.setPower((forcaMotor[0] - forcax[1]) * speed);
                LMB.setPower((forcaMotor[1] + forcax[1]) * speed);
            }
        }
    }

    public void setOnePowerAll(double power) {
        RMF.setPower(power);
        LMF.setPower(power);
        RMB.setPower(power);
        LMB.setPower(power);
    }

    public void setPowerAll(double power1, double power2) {
        LMF.setPower(power1);
        RMB.setPower(power1);
        RMF.setPower(power2);
        LMB.setPower(power2);
    }

    public void initHardware(HardwareMap hwMap) {
        HardwareMap hMap = hwMap;
        // init imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Init Motores
        RMF = hMap.get(DcMotor.class, "RMF");
        LMF = hMap.get(DcMotor.class, "LMF");
        RMB = hMap.get(DcMotor.class, "RMB");
        LMB = hMap.get(DcMotor.class, "LMB");

        // Set Direction
        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        RMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public double getAngleJoyEsquerdo(double angulo, double x) {
        double anguloFinal = 0;
        if ((angulo >= 0) && (angulo <= 90) && (x >= 0)) {
            anguloFinal = angulo;
        }
        //2º QUADRANTE
        else if ((angulo >= 0) && (angulo <= 90) && (x <= 0)) {
            anguloFinal = 180 - angulo;
        }
        //3º QUADRANTE
        else if ((angulo <= 0) && (angulo >= -90) && (x <= 0)) {
            anguloFinal = -angulo - 180;
        }
        //4º QUADRANTE
        else if ((angulo <= 0) && (angulo >= -90) && (x >= 0)) {
            anguloFinal = angulo;
        }
        return anguloFinal;
    }

    public double getAngleJoyDireito(double x2, double y2, double angulo) {
        double anguloFinal = 0;
        // Pegar ângulo Joy Direito
        //1º QUADRANTE
        if ((x2 >= 0) && (y2 >= 0)) {
            anguloFinal = angulo;
        }
        // 2º QUADRANTE
        else if ((x2 < 0) && (y2 >= 0)) {
            anguloFinal = 180 - angulo;
        }
        // 3º QUADRANTE
        else if ((x2 < 0) && (y2 < 0)) {
            anguloFinal = -angulo - 180;
        }
        // 4º QUADRANTE
        else if ((x2 >= 0) && (y2 < 0)) {
            anguloFinal = angulo;
        }
        return anguloFinal;
    }

    public double forceyJoy(double anguloRobo, double angulofinal, double d) {
        double forcay = 0;
        double anguloReal = 0;

        anguloReal = angulofinal - anguloRobo;
        forcay = Math.sin(Math.toRadians(anguloReal)) * d;

        return forcay;
    }

    public double forcexJoy(double anguloRobo, double angulofinal, double d) {
        double forcax = 0;
        double anguloReal = 0;

        anguloReal = angulofinal - anguloRobo;
        forcax = Math.cos(Math.toRadians(anguloReal)) * d;
        return forcax;
    }

    public double getForce(double d, double forcax, double forcay, String lado) {
        double angulorad = (d != 0) ? Math.asin(forcay / d) : 0;
        double fME = 0;
        double fMD = 0;

        if ((forcax >= 0) && (forcay >= 0)) {
            fME = d;
            fMD = (((4 / Math.PI) * angulorad) - 1) * d;
        }
        //2º QUADRANTE FORÇA
        else if ((forcax < 0) && (forcay >= 0)) {
            fME = (((4 / Math.PI) * angulorad) - 1) * d;
            fMD = d;
        }
        //3º QUADRANTE FORÇA
        else if ((forcax < 0) && (forcay < 0)) {
            fME = -d;
            fMD = (((4 / Math.PI) * angulorad) + 1) * d;
        }
        //4º QUADRANTE FORÇA
        else if ((forcax >= 0) && (forcay < 0)) {
            fME = (((4 / Math.PI) * angulorad) + 1) * d;
            fMD = -d;
        }
        if (lado == "ESQUERDA") {
            return fME;
        } else if (lado == "DIREITA") {
            return fMD;
        }
        return 0;
    }

    public double getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

    // Autonomo
    public void turn(double anguloAlvo) {
        double erro = angles.firstAngle - anguloAlvo;
        double anguloAtual = angles.firstAngle;
        // ANGULO ROBÔ SEMPRE MAIOR QUE O ERRO
        if (erro > 0) {
            while (anguloAtual > erro) {
                // move(Esquerda)
            }

        } else if (erro < 0) {
            while (anguloAtual > erro) {
                // move(Direita)
            }
        }
        anguloAtual = angles.firstAngle;
        if ((erro > 0)) {
            if (erro < anguloAtual - 1) {
                while (anguloAtual != erro) {
                    // move(Direita);
                }
            }
            if (erro < 0) {
                if (erro > anguloAtual + 1) {
                    while (anguloAtual != erro) {
                        // move(Esquerda);
                    }
                }
            }
        }
    }

    public double gravacao() {
        if (gamepad1.y) {
            if (gravar) {
                gravar2 = !gravar2;
            }
            gravar = false;
        } else {
            gravar = true;
        }
        if (gravar2) {
            // GRAVAR
            valorArmazenado[iGravacao][0] = (forcaMotor[1] - forcax[1]) * speed; // RMF
            valorArmazenado[iGravacao][1] = (forcaMotor[0] + forcax[1]) * speed; // LMF
            valorArmazenado[iGravacao][2] = (forcaMotor[0] - forcax[1]) * speed; // RMB
            valorArmazenado[iGravacao][3] = (forcaMotor[1] + forcax[1]) * speed; // LMB
            iGravacao++;
        } else {
            iGravacao = 0;
        }
        return 0;
    }

    public void setPowerGravado() {
        timer2 = System.currentTimeMillis();
        while (System.currentTimeMillis() - timer2 < 10000) {
            if (System.currentTimeMillis() - timer < 20) {
                continue;
            }
            timer = System.currentTimeMillis();
            for (int i = 0; i < 500; i++) {
                RMF.setPower(valorArmazenado[i][0]);
                LMF.setPower(valorArmazenado[i][1]);
                RMB.setPower(valorArmazenado[i][2]);
                LMB.setPower(valorArmazenado[i][3]);
            }
        }
    }
}
