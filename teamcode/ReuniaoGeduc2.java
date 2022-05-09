package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;
import java.util.Locale;

@TeleOp
public class ReuniaoGeduc2 extends LinearOpMode {
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;
    private DcMotor Esteira;
    private DcMotor MP;

    private Servo Paleta;
    private Servo SC;       //Sero - coleta
    private CRServo SPR;
//    private CRServo SPL;

    double  MIN_PALETA = 0.59; // 0.09
    double  MAX_PALETA = 1;  // 0.55
    double  inicialPaleta = 0.78;
    double  travaCargaPaleta = 0.88;
    double  speedPaleta = 0.01;
//    private DcMotor MDP = null;     //Motor - Derrubar Pato
    private DcMotor MC = null;      //Motor - Coletar
    //private Servo SMC = null;        //Servo - Movimentar Coleta
//    private Servo SB = null;        //Servo - Braço
    private DcMotor MB = null;      //Motor - Braço - Ticks 1120
//    private DcMotor MB2 = null;      //Motor - Braço - Ticks 1120
    boolean gravar = false;
    boolean gravar2 = false;
    double valorArmazenado[][] = new double[500][5];
    int iGravacao = 0;
    int jGravacao = 0;
    double speed = 1;
    double angulofinalE;
    double angulofinalD;
    double angulorealE = 0;
    double angulorealD = 0;
    double MOTOR_TICKS_NULL = 28;
    double MOTOR_TICKS_20 = MOTOR_TICKS_NULL * 20;
    double angMOTOR_TICKS_MB_20 = MOTOR_TICKS_20 / 360;
    final double PI = Math.PI;
    double fMD = 0;
    double fME = 0;
    double forcax = 0;
    double forcax2 = 0;
    double forcay = 0;
    double timer = System.currentTimeMillis();
    double timer2 = System.currentTimeMillis();
    ElapsedTime timer3 = new ElapsedTime();
    double timer5 = 0;
    double positionPaleta = 0;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;


    @Override public void runOpMode() {
        boolean povD = gamepad2.dpad_right;
        boolean povE = gamepad2.dpad_left;
        boolean povUp = gamepad2.dpad_up;
        boolean toggle = true;
        boolean antPov = false;
        boolean qqw = true, qqw2=false;
        double ticksBraco = 1120;
        double integral = 0, erro = 0, somatorioErro = 0, ki = 0;
        double pointBraco = 0;
        double controleBraco = 0;
        double powerBraco = 0;
        double pointServo = 0;
        String nivel = "";
        //double pointServo = SB.getPosition();
        double switchAngle = 0;
        boolean c = false;



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        Esteira = hardwareMap.get(DcMotor.class, "MMT");
        MC = hardwareMap.get(DcMotor.class,"MC");
//        MDP = hardwareMap.get(DcMotor.class, "MDP");
        //SMC = hardwareMap.get(Servo.class, "SMC");
        MB = hardwareMap.get(DcMotor.class, "MB");
//        MB2 = hardwareMap.get(DcMotor.class, "MB2");
        SC = hardwareMap.get(Servo.class, "SC");
        Paleta = hardwareMap.get(Servo.class,"Paleta");

        MP = hardwareMap.get(DcMotor.class, "MP");

        SPR = hardwareMap.crservo.get("SPR");
//        SPL = hardwareMap.crservo.get("SPL");

        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        Esteira.setDirection(DcMotorSimple.Direction.FORWARD);
        MB.setDirection(DcMotorSimple.Direction.FORWARD);
//        MB2.setDirection(DcMotorSimple.Direction.FORWARD);
        MC.setDirection(DcMotorSimple.Direction.FORWARD);
        Paleta.setDirection(Servo.Direction.FORWARD);
        SC.setDirection(Servo.Direction.REVERSE);
//        MDP.setDirection(DcMotorSimple.Direction.FORWARD);
        //SMC.setDirection(Servo.Direction.REVERSE);
        MP.setDirection(DcMotorSimple.Direction.REVERSE);


        MB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        MB2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        MB2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
;
//        SC.setPosition(1);


        waitForStart();

        timer3.time();
        while(opModeIsActive()){
            telemetry.addData("TIMER: ", timer3.time() - timer5);
            double[] powerYJoy2 = {-gamepad2.left_stick_y,-gamepad2.right_stick_y};
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double x2 = gamepad1.right_stick_x;
            double y2 = -gamepad1.right_stick_y;
            double d = Math.hypot(x,y);
            double d2 = Math.hypot(x2, y2);
            double angulorad = (d!=0)?Math.asin(y/d):0;
            double angulo = Math.toDegrees(angulorad);
            double angulorad2 = (d2!=0)?Math.asin(y2/d2):0;
            double anguloD = Math.toDegrees(angulorad2);
            boolean r1 = gamepad1.right_bumper;
            boolean l1 = gamepad1.left_bumper;
            double currangle = angles.firstAngle;
            double anguloRobo = 0, tickspAng=0, posicaoSMC=0;
            double posicionAnteriorServo = 0, posicionAtualServo = 0, difPosicionServo = 0;


//========================================== CONTROLE - 1 ==========================================

            //Control Speed
            speed = gamepad1.b ? 1 : gamepad1.a ? 0.5 : gamepad1.x ?0.25:speed ;

            if (gamepad1.dpad_up){
                setPowerGravado();
            }
            switchAngle = gamepad1.dpad_down ? getAngle() : switchAngle;

            // Pegar ângulo Joy Esquerdo
            //1º QUADRANTE
            if((angulo >= 0) && (angulo <= 90) && (x >= 0)) {
                angulofinalE = angulo;
            }
            //2º QUADRANTE
            else if ((angulo >= 0) && (angulo <= 90) && (x <= 0)) {
                angulofinalE = 180-angulo;
            }
            //3º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x <= 0)) {
                angulofinalE = -angulo-180;
            }
            //4º QUADRANTE
            else if ((angulo <= 0) && (angulo >= -90) && (x >= 0)) {
                angulofinalE = angulo;
            }

            // Pegar ângulo Joy Direito
            //1º QUADRANTE
            if((x2 >= 0 ) && (y2 >= 0)){
                angulofinalD = anguloD;
                telemetry.addData("Condicao","1 quadrante");
            }
            // 2º QUADRANTE
            else if((x2 < 0 ) && (y2 >= 0)) {
                angulofinalD = 180-anguloD;
                telemetry.addData("Condicao","2 quadrante");
            }
            // 3º QUADRANTE
            else if((x2 < 0 ) && (y2 < 0)){
                angulofinalD = -anguloD-180;
                telemetry.addData("Condicao","3 quadrante");
            }
            // 4º QUADRANTE
            else if((x2 >= 0) && (y2 < 0)) {
                angulofinalD = anguloD;
                telemetry.addData("Condicao","4 quadrante");
            }
            // MOVIMENTAÇÃO POR ÂNGULO


            anguloRobo = getAngle();
            //Analógico Esquerdo
            angulorealE =  angulofinalE - anguloRobo + switchAngle;
            forcay = Math.sin(Math.toRadians(angulorealE)) * d;
            forcax = Math.cos(Math.toRadians(angulorealE)) * d;
            telemetry.addData("forcay: ",forcay);

            //Analógico Direito
            angulorealD =  angulofinalD - anguloRobo + switchAngle;
            telemetry.addData("AnguloREALD: ", angulorealD);
            telemetry.addData("D2: ", d2);
            forcax2 = Math.cos(Math.toRadians(angulorealD)) * d2;
            telemetry.addData("forcax2: ",forcax2);

            //1º QUADRANTE FORÇA

            angulorad = (d!=0)?Math.asin(forcay/d):0;

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

//             Set Force

            if(r1){
                Esteira.setPower(1);
            }else if(l1){
                Esteira.setPower(-1);
            }else{
                Esteira.setPower(0);
            }

/*
            if (povD && !antPov){
                SB.setPosition(toggle ? 1 : 0);
                toggle = !toggle;
                antPov = true;
            } else if (!povD){
                antPov = false;
            }

 */

//========================================== CONTROLE - 2 ==========================================

            //Coleta
/*
            posicaoSMC = gamepad2.right_bumper ? -1 : 0;
            SMC.setPosition(posicaoSMC);
            MC.setPower(-gamepad2.right_stick_y);

            // Derrubar Pato

            if(gamepad2.dpad_right){
                MDP.setPower(0.7);
            }else if(gamepad2.dpad_left){
                MDP.setPower(-0.7);
            }

 */
            double MOTOR_TICKS_NULL = 28;
            double MOTOR_TICKS_20 = MOTOR_TICKS_NULL * 20;
            double angMOTOR_TICKS_MB_20 = MOTOR_TICKS_20 / 360;
            double angAtualBraco = MB.getCurrentPosition() / angMOTOR_TICKS_MB_20 , angAtualBraco2 = 0;
            double difAngular = 0;
            double ang_pointBraco = 0;
            double kP = 0, kD = 0, proporcional = 0, derivada = 0;
            double erro2 = 0;

            // Nível do Braço
            if(gamepad2.a && gamepad2.left_bumper && gamepad2.right_bumper){
                qqw = false;
            }

            if(qqw){

                nivel = gamepad2.a ? "Coletar": gamepad2.x ? "3": gamepad2.b ? "1": gamepad2.y ? "2" : nivel;
                pointBraco = nivel == "Coletar" ? -10 : nivel == "1" ? 950 : nivel == "2" ? 800 : nivel == "3" ? 650 : pointBraco;

                c = gamepad2.a ? true : ((gamepad2.x) || (gamepad2.y) || (gamepad2.b)) ? false : c;

//                RobotLog.d("NIVEL :" + nivel);
//                RobotLog.d("PointBraco : " + pointBraco);
                telemetry.addData("NIVEL :", nivel);
                telemetry.addData("PointBraco : ", pointBraco);


                if(c){
                    positionPaleta = inicialPaleta;
                }else if(gamepad2.dpad_right){
                    positionPaleta = MAX_PALETA;
                }else if(gamepad2.dpad_left){
                    positionPaleta = MIN_PALETA;
                }else{
                    positionPaleta = travaCargaPaleta;
                }


//                if(gamepad2.dpad_up){
//                    positionPaleta += speedPaleta;
//                }else if(gamepad2.dpad_down){
//                    positionPaleta -= speedPaleta;
//                } else if (gamepad2.dpad_right) {
//                    positionPaleta = MAX_PALETA;
//                }else if (gamepad2.dpad_left){
//                    positionPaleta = MIN_PALETA;
//                }
                telemetry.addData("Paleta Posicion", Paleta.getPosition());
                telemetry.update();
                positionPaleta = Range.clip(positionPaleta,MIN_PALETA,MAX_PALETA);
                Paleta.setPosition(positionPaleta);


                if(gamepad2.right_bumper){
                    SPR.setPower(1);
                    MP.setPower(1);
//                    SPL.setPower(-1);
                }else if(gamepad2.left_bumper){
                    SPR.setPower(-1);
                    MP.setPower(-1);
//                    SPL.setPower(1);
                }else {
                    SPR.setPower(0);
                    MP.setPower(0);
//                    SPL.setPower(0);
                }

                if (MB.getCurrentPosition() != pointBraco) {

                    powerBraco = 0.6;

//                    Atualizar no caderno
                    if (nivel == "Coleta") {
                        powerBraco = 0.2;
                        MB.setTargetPosition(80);
                        MB.setPower(powerBraco);
                        MB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sleep(500);
                    }

                    controleBraco += Math.signum(gamepad2.right_stick_y);

                    MB.setTargetPosition((int)(pointBraco + controleBraco));
                    MB.setPower(powerBraco);
                    MB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    telemetry.addData("1 -\t", MB.getCurrentPosition());
                    telemetry.update();


                } else {
                    MB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

            } else{
                MB.setPower(-gamepad2.left_stick_y/6);
                MB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            RobotLog.d("Posição servo" + pointServo);
            RobotLog.d("Ang Braco" + ang_pointBraco + "Ticks Braco" + MB.getCurrentPosition());


            telemetry.addData("Angulo do Braço atual:", angMOTOR_TICKS_MB_20);
            telemetry.addData("Posição atual:", MB.getCurrentPosition());
            telemetry.update();
            qqw = true;



            //          TESTAR TUDO SOBRE O SERVO


            // Caixa - Servo

/*
            Ver quantos ticks tem um angulo
            Dividir esse valor pela posição atual do braço
            Subtrair o valor de ticks p/ ang na posição atual do servo

 */
            // Soltar Carga

//            if(MB.getCurrentPosition() == pointBraco){
//                SB.setPosition(-1);
//            }






//========================================SET POWER MOTORS==========================================

            //MC.setPower(powerYJoy2[0]);
            //MDP.setPower(powerYJoy2[1]);
            RMF.setPower((fMD-x2) * speed);
            LMF.setPower((fME+x2) * speed);
            RMB.setPower((fME-x2) * speed);
            LMB.setPower((fMD+x2) * speed);
            MC.setPower(gamepad2.left_stick_y);
            timer5 = timer3.time();
            gravacao();
            telemetry.update();
            sleep(20);
        }
    }

    public void gravacao() {
        if (gamepad1.y) {
            if (gravar) {
                gravar2 = !gravar2;
            }
            gravar = false;
        } else {
            gravar = true;
        }
        if (gravar2) {
            telemetry.addLine("GRAVANDO VALOR");
            telemetry.addLine("To gravando");            // GRAVAR
            valorArmazenado[iGravacao][0] = ((fMD-forcax2) * speed);   //RMF
            valorArmazenado[iGravacao][1] = ((fME+forcax2) * speed);   //LMF
            valorArmazenado[iGravacao][2] = ((fME-forcax2) * speed);  // RMB
            valorArmazenado[iGravacao][3] = ((fMD+forcax2) * speed);  // LMB
            valorArmazenado[iGravacao][4] = timer3.time() - timer5;
            telemetry.addData("TIMER: ", timer3.time() - timer5);
            telemetry.addData("TIMER GRAVADO: ", new Double(valorArmazenado[iGravacao][4]).longValue());

            iGravacao++;
            jGravacao = iGravacao;
        } else {
            iGravacao = 0;

        }
    }
    public void setPowerGravado() {
        timer3.time();
        for (int i = 0; i < jGravacao; i++) {
            long timerLoop = (new Double(valorArmazenado[iGravacao][4]).longValue());
            telemetry.addData("TEMPO CICLO FOR: ", timer3.time() - timer5);
            telemetry.addLine("SET POWER GRAVADOR");
//            telemetry.addData("valor armazenado: ", valorArmazenado[i][0]);
//            telemetry.addData("valor armazenado: ", valorArmazenado[i][1]);
//            telemetry.addData("valor armazenado: ", valorArmazenado[i][2]);
//            telemetry.addData("valor armazenado: ", valorArmazenado[i][3]);
//            RobotLog.d("valor_armazenado_RMF: " + i + " Valor" + timerLoop);
            telemetry.update();
            RMF.setPower(valorArmazenado[i][0]);
            LMF.setPower(valorArmazenado[i][1]);
            RMB.setPower(valorArmazenado[i][2]);
            LMB.setPower(valorArmazenado[i][3]);
//            if( i == jGravacao - 1){
//                sleep();
//            }
            timer5 = timer3.time();
            sleep(20);
        }
        try {
            this.saveGravacao(valorArmazenado);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void saveGravacao(double [][] storedValue) throws IOException {
        System.out.println("SAVE_GRAVACAO_WORKING");
        StringBuffer builder = new StringBuffer();
        for(int i = 0; i < jGravacao; i++) {
            System.out.println("SAVE_GRAVACAO_LOOP_ONE");
            for(int j = 0; j < 4; j++) {
                System.out.println("SAVE_GRAVACAO_LOOP_TWO");
                builder.append(storedValue[i][j] + "");
                if(j < storedValue.length - 1)
                    builder.append(",");
            }
            builder.append("\n");
        }
        Date date = new Date();
        BufferedWriter writer = new BufferedWriter(new FileWriter("/storage/emulated/0/FIRST/RecordedGravacao/AutonomoGeduc.txt"));
        writer.write((builder.toString()));
        writer.close();
    }


    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------



    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getAngle(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }


}



//            if (qqw) {
//                if (gamepad2.a) {
//                    nivel = "Coletar";
//                } else if (gamepad2.x) {
//                    nivel = "3";
//                } else if (gamepad2.y) {
//                    nivel = "2";
//                } else if (gamepad2.b) {
//                    nivel = "1";
//                } else {
//                    nivel = "NA";
//                }

//                if (nivel == "Coletar") {
//                    pointBraco = 20;
//                } else if (nivel == "1") {
//                    pointBraco = -800;
//                } else if (nivel == "2") {
//                    pointBraco = -750;
//                } else if (nivel == "3") {
//                    pointBraco = -700;
//                }
//                else if (nivel == "NA") {
//                    pointBraco = -10;
//                }

//                ang_pointBraco = pointBraco / angMOTOR_TICKS_MB_20;
// Dps irei fazer uma prporcional relacionando a força em função da distancia do setPoint

//                if(MB.getCurrentPosition() <=0 && MB.getCurrentPosition() >= -130){
//                    powerBraco = 0.5;
//                }else {
//                    powerBraco = 0.2;
//                }
//


//                kP = 0.05;
//                kD = 0; // 1

//                positionPaleta = gamepad2.dpad_up ? 0 : gamepad2.dpad_down ? 1 : positionPaleta;

//                    qqw2 = true;
//                    angAtualBraco2 = angAtualBraco;
////                    while (qqw2) {
//
//
////                        PID
//                    telemetry.addLine("PID");
//                    difAngular = ang_pointBraco - angAtualBraco;
//                    proporcional = difAngular * kP;
//
//                    derivada = (difAngular - erro2) * kD;

//                        powerBraco = proporcional + derivada;
//                            ALTERAR POSICAO BRACO

//                    telemetry.addLine("Alterar Posicao Braco");
//                    if(nivel == "Coleta"){
//                        powerBraco = 0.3;
//                    }
//                    erro2 = difAngular;
//                    telemetry.addData("Erro2:", erro2);

//                    angAtualBraco = MB.getCurrentPosition() / angMOTOR_TICKS_MB_20;

//                            ALTERAR POSICAO SERVO

//                    telemetry.addLine("Alterar Posicao Servo");
//
//                    posicionAtualServo = SB.getPosition();
//                    difPosicionServo = (int) (angAtualBraco - angAtualBraco2);
//                    difPosicionServo = Math.toRadians(difPosicionServo);
//                    pointServo = posicionAtualServo + difPosicionServo;
//                    if (posicionAtualServo != difPosicionServo) {
//                        SB.setPosition(pointServo);
//                    }

//                    if(MB.getCurrentPosition() == pointBraco){
//                        qqw2 = false;
//                    }else{
//                        qqw2 = true;
//                    }

//                    }