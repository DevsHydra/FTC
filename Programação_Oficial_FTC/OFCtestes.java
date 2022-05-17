package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class OFCtestes extends LinearOpMode {

//    HardwarePushbot         robot   = new HardwarePushbot();    //Objeto para manipular os motores

    DigitalChannel digitalTouch;  // Hardware Device Object

//==============================CRIAÇÃO DOS MOTORES E SERVOS========================================

//  Motores de locomoção:

    public DcMotor  LMF;    //Motor Mecanum - Esquerda Frontal
    public DcMotor  RMF;    //Motor Mecanum - Direita Frontal
    public DcMotor  LMB;    //Motor Mecanum - Esquerda Traseiro
    public DcMotor  RMB;    //Motor Mecanum - Direita Traseiro;
    public DcMotor Esteira; //Motor Core Hex - Esteira

//  Motores de sistema:

    public DcMotor  MB;     //Motor UltraPlanetary (100:1) - Sistema de elevação (Braço)
    public DcMotor  ML;     //Motor Core Hex - Lagosta
    public DcMotor  MC;     //Motor Core Hex - Coleta

//  Servos:

    public CRServo SPR;     //Servo Continuo Girar Carrossel - Direita
    public CRServo SPL;     //Servo Continuo Girar Carrossel - Esqueda

    public Servo SL;        //Servo Posição - Pinça

//============================================VARIÁVEIS=============================================

//  Locomoção:

    double angulofinalE;
    double angulofinalD;
    double angulorealE = 0;
    double angulorealD = 0;

    double fMD = 0;             //Força para motores Meacanum - Sentido: Direita
    double fME = 0;             //Força para motores Meacanum - Sentido: Esqueda

    double forcax = 0;
    double forcax2 = 0;
    double forcay = 0;

    double speed = 1;           //Controle de Velocidade

// Controle do Servo e Motores da Lagosta:

    double  servoLagosta = 0;        //Define a posição que ele deve ir
    double  MIN_PALETA = 0;         //Define a posição minima
    double  MAX_PALETA = 1;         //Define a posição máxima
    double  speedPaleta = 0.01;     //É somada ou subtraida a "servoLagosta"

    int  position = 0;

//  Braço:

    String nivel = "";              //Controle da posição
    String ultNivel = "";
    int pointBraco = 0;             //Define a posição do braço

    double powerBraco = 0;          //Força para locomoção do braço

    int controleBraco = 0;

//  Biblioteca matematica:

    final double PI = Math.PI;

    BNO055IMU imu;      //Chamando o sensor IMU

    Orientation angles;


    @Override public void runOpMode() {

        double switchAngle = 0;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


//==========================DEFININDO INNICIALIZAÇÃO DOS MOTORES E SERVOS===========================

        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");

        Esteira = hardwareMap.get(DcMotor.class, "MMT");

        MB = hardwareMap.get(DcMotor.class, "MB");
        MC = hardwareMap.get(DcMotor.class,"MC");
        ML = hardwareMap.get(DcMotor.class, "ML");

        SL = hardwareMap.get(Servo.class, "SL");

        SPR = hardwareMap.crservo.get("SPR");
        SPL = hardwareMap.crservo.get("SPL");

//==========================DEFININDO DIREÇÃO DOS MOTORES E SERVOS==================================

        LMF.setDirection(DcMotorSimple.Direction.REVERSE);      //REVERSE - Direção Reversa
        LMB.setDirection(DcMotorSimple.Direction.FORWARD);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);      //FORWARD - Direção Normal
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);

        Esteira.setDirection(DcMotorSimple.Direction.REVERSE);

        MB.setDirection(DcMotorSimple.Direction.FORWARD);
        MC.setDirection(DcMotorSimple.Direction.FORWARD);
        ML.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setDirection(Servo.Direction.FORWARD);

//====================================TRAVANDO OS MOTORES===========================================

//        LMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        MB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        MC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        ML.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//==========================DEFININDO MOTORES QUE NÃO IRAM USAR ENCODER=============================

        LMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//==========================DEFININDO MOTORES QUE IRAM USAR ENCODER=================================

        MB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ML.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//=======================================RESETANDO ENCODERS=========================================

        MB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ML.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        angles= imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();

        while(opModeIsActive()){

//            robot.init(hardwareMap);

//===========================================VARIÁVEIS==============================================

//  Botões do controle - 1:

            double y_1 = -gamepad1.left_stick_y;      //Analogico Esquedo - Eixo Y
            double x_1 = gamepad1.left_stick_x;       //Analogico Esquedo - Eixo X

            double y2_1 = -gamepad1.right_stick_y;    //Analogico Direita - Eixo Y
            double x2_1 = gamepad1.right_stick_x;     //Analogico Direita - Eixo X

            boolean r1_1 = gamepad1.right_bumper;     //Gatilo booleano - Direita
            boolean l1_1 = gamepad1.left_bumper;      //Gatilo booleano - Esqueda

//  Botões do controle - 2:

            double y_2 = -gamepad2.left_stick_y;      //Analogico Esquedo - Eixo Y

            double y2_2 = -gamepad2.right_stick_y;    //Analogico Direita - Eixo Y

            boolean povU = gamepad2.dpad_up;          //Seta - Cima
            boolean povR = gamepad2.dpad_right;       //Seta - Direita
            boolean povL = gamepad2.dpad_left;        //Seta - Esquerda
            boolean povD = gamepad2.dpad_down;        //Seta - Baixo

            boolean r1_2= gamepad2.right_bumper;     //Gatilo booleano - Direita
            boolean l1_2 = gamepad2.left_bumper;      //Gatilo booleano - Esqueda

//  Formulas:

            double d = Math.hypot(x_1,y_1);         //Hipotenusa - Analogico Esquerdo
            double d2 = Math.hypot(x2_1, y2_1);     //Hipotenusa - Analogico Direito

            double angulorad = (d!=0)?Math.asin(y_1/d):0;     //Seno (Em radiandos) - Analogico Esquerdo
            double angulo = Math.toDegrees(angulorad);      //Seno (Em graus) - Analogico Esquerdo

            double angulorad2 = (d2!=0)?Math.asin(y2_1/d2):0; //Seno (Em radiandos) - Analogico Direito
            double anguloD = Math.toDegrees(angulorad2);    //Seno (Em graus) - Analogico Direito

            double anguloRobo;      //Recebe o angulo do robo

//========================================== CONTROLE - 1 ==========================================

//  Controle de Velocidade:

            speed = gamepad1.b ? 1 : gamepad1.a ? 0.5 : gamepad1.x ?0.25:speed ;

            switchAngle = gamepad1.dpad_down ? getAngle() : switchAngle;

//  Recebe o angulo do Analogico Esquedo:

            //1º QUADRANTE
            if((angulo >= 0) && (angulo <= 90) && (x_1 >= 0)) {
                angulofinalE = angulo;
            }
            //2º QUADRANTE
            else if ((angulo >= 0) && (angulo <= 90) && (x_1 <= 0)) {
                angulofinalE = 180-angulo;
            }
            //3º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x_1 <= 0)) {
                angulofinalE = -angulo-180;
            }
            //4º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x_1 >= 0)) {
                angulofinalE = angulo;
            }

//  Recebe o angulo do Analogico Direito:

            //1º QUADRANTE
            if((x2_1 >= 0 ) && (y2_1 >= 0)){
                angulofinalD = anguloD;
            }
            // 2º QUADRANTE
            else if((x2_1 < 0 ) && (y2_1 >= 0)) {
                angulofinalD = 180-anguloD;
            }
            // 3º QUADRANTE
            else if((x2_1 < 0 ) && (y2_1 < 0)){
                angulofinalD = -anguloD-180;
            }
            // 4º QUADRANTE
            else if((x2_1 >= 0) && (y2_1 < 0)) {
                angulofinalD = anguloD;
            }

            // MOVIMENTAÇÃO POR ÂNGULO

            anguloRobo = getAngle();
            //Analógico Esquerdo
            angulorealE =  angulofinalE - anguloRobo + switchAngle;
            forcay = Math.sin(Math.toRadians(angulorealE)) * d;
            forcax = Math.cos(Math.toRadians(angulorealE)) * d;

            //Analógico Direito
            angulorealD =  angulofinalD - anguloRobo + switchAngle;
            forcax2 = Math.cos(Math.toRadians(angulorealD)) * d2;

            //1º QUADRANTE FORÇA

            angulorad = (d!=0)?Math.asin(forcay/d):0;

//  Funções da Locomoção:

            if((forcax>= 0) && (forcay>=0)) {
                fME = d;
                fMD = (((4 / PI) * angulorad) - 1) * d;

            }
            //2º QUADRANTE FORÇA
            else if ((forcax< 0) && (forcay>=0)) {
                fME = (((4 / PI) * angulorad) - 1) * d;
                fMD = d;
            }
            //3º QUADRANTE FORÇA
            else if ((forcax< 0) && (forcay<0)) {
                fME = -d;
                fMD = (((4 / PI) * angulorad) + 1) * d ;
            }
            //4º QUADRANTE FORÇA
            else if ((forcax>= 0) && (forcay<0)) {
                fME = (((4 / PI) * angulorad) + 1) * d;
                fMD = -d;
            }

//  Esteira:

            if(r1_1){                             //Definindo força a Esteira
                Esteira.setPower(1);
            }else if(l1_1){
                Esteira.setPower(-1);
            }else{
                Esteira.setPower(0);
            }

//========================================== CONTROLE - 2 ==========================================

//  Sistema de Elevação:

            nivel =  gamepad2.a ? "Coletar" : gamepad2.x ? "3" : gamepad2.b ? "1" : gamepad2.y ? "2" : nivel;
            pointBraco = nivel == "Coletar" ? 0 : nivel == "1" ? 978 : nivel == "2" ? 845 : nivel == "3" ? 718 : pointBraco;

            powerBraco = 0.3;

//            position = nivel == "Coletar" ? 0 : 70;       //Controle da posição

//            ML.setTargetPosition(0);               //Definindo posição alvo
//            ML.setPower(1);                               //Definindo força
//            ML.setMode(DcMotor.RunMode.RUN_TO_POSITION);  //Mandando ir a posição

//            MB.setTargetPosition(pointBraco);
//            MB.setPower(powerBraco);
//            MB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            if(nivel != "Coletar"){
//                controleBraco = 0;
//            }
//
//            if(digitalTouch.getState() == false && controleBraco == 0){
//                MB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                controleBraco++;
//            }
//
//            ultNivel = nivel;

            if (nivel == "3"){
                moveMotor(0.5, pointBraco,MB);
            }

            telemetry.addData("Braço - ", MB.getCurrentPosition());
            telemetry.addData("Força Braco -", MB.getPower());
            telemetry.addData("Nivel - ", nivel);
            telemetry.addData("SENSOR TOQUE: ", digitalTouch.getState());
            telemetry.addData("Controle: ", controleBraco);

//  Servo do Carrossel:

            if (r1_2){          //Definindo força aos servos do Carrossel
                SPR.setPower(1);
                SPL.setPower(1);
            } else if (l1_2) {
                SPR.setPower(-1);
                SPL.setPower(-1);
            } else {
                SPR.setPower(0);
                SPL.setPower(0);
            }

//  Motor da Lagosta:

            telemetry.addData("", ML.getCurrentPosition());

//  Servo da Pinça:

            if (povU) {                 //Abrindo a pinça
                servoLagosta += speedPaleta;
            }
            else if (povD) {            //Fechando a pinça
                servoLagosta -= speedPaleta;
            }

            servoLagosta = Range.clip(servoLagosta, MIN_PALETA, MAX_PALETA);    //Limitando o Servo da pinça
            SL.setPosition(servoLagosta);                                 //Mandado para posição

//==================================DEFININDO FORÇAS - MOTORES LOCOMOÇÃO============================

            RMF.setPower((fMD-x2_1) * speed);
            LMF.setPower((fME+x2_1) * speed);
            RMB.setPower((fME-x2_1) * speed);
            LMB.setPower((fMD+x2_1) * speed);

            MB.setPower(-gamepad2.left_stick_y * 0.5);

//            MC.setPower(y_2);
            telemetry.update();
            sleep(20);
        }
    }


    public double getAngle(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }
    public void moveMotor(double power, int position, DcMotor motor){
        motor.setTargetPosition(pointBraco);
        while(motor.getCurrentPosition() != position){
            motor.setPower(powerBraco);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

}