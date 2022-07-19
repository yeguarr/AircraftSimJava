import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.geom.Point2D;
import java.util.concurrent.atomic.AtomicReference;

public class MainFrame extends JFrame {
    Updater updater;
    Camera camera;
    ControlsGUI controlsGUI;
    Viewer3D viewer3D;
    JTextArea textArea;

    public MainFrame() {
        super("3D Viewer");
        updater = new Updater(80);
        camera = new Camera();
        controlsGUI = new ControlsGUI(camera);
        viewer3D = new Viewer3D(controlsGUI);
        textArea = new JTextArea("Hello World!");
        setup();
    }

    public static void main(String[] args) {
        MainFrame mainFrame = new MainFrame();
        mainFrame.updater.start();
        mainFrame.display();
        /*for(int i = 0; i <10000; i++) {
            mainFrame.updater.doTasks();
        }*/
    }

    public void setup() {
        camera.setPosition(new Point3D(0,0,-30));
        controlsGUI.bindWASD(camera, 0.7f);

        /////// TEST ZONE

        //создаем параметры пространства
        SimulationEnvironment simulationEnvironment = new SimulationEnvironment();

        //создаем пропеллеры
        double lPitch = 2.5;
        double lRoll = 2.5;
        AircraftBase.Propeller m1 = new AircraftBase.Propeller(0.1,new Point3D(lPitch,-0.3,lRoll),new Point3D(0,1,0),false,10,"propellerB.obj", simulationEnvironment);
        AircraftBase.Propeller m2 = new AircraftBase.Propeller(0.1,new Point3D(lPitch,-0.3,-lRoll),new Point3D(0,1,0),true,10,"propellerG.obj",simulationEnvironment);
        AircraftBase.Propeller m3 = new AircraftBase.Propeller(0.1,new Point3D(-lPitch,-0.3,-lRoll),new Point3D(0,1,0),false,10,"propellerY.obj",simulationEnvironment);
        AircraftBase.Propeller m4 = new AircraftBase.Propeller(0.1,new Point3D(-lPitch,-0.3,lRoll),new Point3D(0,1,0),true,10,"propellerM.obj",simulationEnvironment);
        //QuadCopter.Propeller m5 = new QuadCopter.Propeller(1,new Point3D(0,0,2),new Point3D(0,0,-1),true,1000,simulationEnvironment);

        //создаем сам коптер
        Matrix.m4x4 J = new Matrix.m4x4(0.64,0 ,0, 0, 0, 0.64, 0, 0, 0,0, 1.2,0, 0, 0 ,0 ,1 );
        AircraftBase copter = new AircraftBase(3,0.1, J, simulationEnvironment);
        //добавляем пропеллеры на коптер
        copter.addPropeller(m1);
        copter.addPropeller(m2);
        copter.addPropeller(m3);
        copter.addPropeller(m4);
        //copter.addPropeller(m5);
        //m5.setSpeed(1);

        updater.addTask(copter::calculateForces);
        //добавляем коптер и пропеллеры в пространство
        viewer3D.addObject3D(copter.getCopterObject());
        for (int i = 0; i < copter.getPropellers().size(); i++) {
            viewer3D.addObject3D(copter.getPropellers().get(i).getObjectStick());
            viewer3D.addObject3D(copter.getPropellers().get(i).getPropeller3D());
        }

        //создаем и добавляем в пространство точку, в которурую должен прийти коптер
        Object3D OBJRef = new ReaderOBJ("arrow.obj").getObject();
        viewer3D.addObject3D(OBJRef);

        //добовляем следящую за коптером линию
        LineTracer lineTracer = new LineTracer(500,4,copter.getPosition());
        updater.addTask(()->lineTracer.updatePosition(copter.getPosition()));
        viewer3D.addObject3D(lineTracer.getLinesObject());

        ///////////////////////// CONTROLLER
        PID pid1 = new PID(0.8,0.4,2, simulationEnvironment);
        PID pid2 = new PID(0.1,0.3,4, simulationEnvironment);
        PID pid3 = new PID(0.1,0.3,4, simulationEnvironment);
        PID pid4 = new PID(0.1,0.3,8, simulationEnvironment);

        AtomicReference<Double> pitch = new AtomicReference<>(0.);

        // странный баг если Point3D(0,90,0) java.lang.IllegalArgumentException: Comparison method violates its general contract!
        copter.setAngle(Utils.eulerAnglesToQuaternion(new Point3D(0,90,0)));


        //Point3D refPosition = new Point3D(10,5,10);

        Object3D lineArrowObject = new Object3D();
        viewer3D.addObject3D(lineArrowObject);


        updater.addTask( () -> {
            /*if (pitch.get()>1)
                pitch.set(-1.);
            pitch.updateAndGet(v -> (v + 0.0005));*/
            /*Point3D refPosition;
            if (pitch.get()<30)
                refPosition = new Point3D(10*Math.cos(pitch.get()),0,10*Math.sin(pitch.get()));
            else if (pitch.get()<50)
                refPosition = new Point3D(10*Math.cos(pitch.get())+40,0,10*Math.sin(pitch.get()));
            else
                refPosition = new Point3D(10*Math.cos(pitch.get())+20,0,20+20*Math.sin(pitch.get()));*/


            //Point3D refPosition = new Point3D(10*Math.cos(pitch.get()),10,10*Math.sin(pitch.get()));
            //Point3D refPosition = copter.getPosition().scale(1,0,1);
            //refPosition = Utils.rotate(new Point3D(0,copter.getAngle().getY(),0)).multiply(refPosition);

            Point3D refPosition = new Point3D(-20,100,0);
            OBJRef.setPosition(refPosition.scale(-1,0,-1));

            Point3D angle = Utils.quaternionToEulerAngles(copter.getAngle());
            Quaternion quaternionAngleXZ = Utils.eulerAnglesToQuaternion( new Point3D(0,90,0));
            //System.out.println(angle.getY());

            //System.out.println( "1) "+ refPosition.add(Utils.rotY(0).multiply(copter.getPosition())));
            //System.out.println( "2) "+ refPosition.add(copter.getPosition()));

            //System.out.println("1) "+ copter.getAngle());
            //System.out.println("2) "+ Utils.eulerAnglesToQuaternion(angle));
            //System.out.println(Utils.quaternionToEulerAngles(copter.getAngle().multiply(quatY)));

            /*Point3D refAngle = new Point3D(
                    Math.min(5, Math.hypot(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ()))
                            *(Math.cos(Math.atan2(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ()))),
                    pitch.get(),
                    Math.min(5,Math.hypot(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ()))
                            *(Math.sin(Math.atan2(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ())))
            );*/


            lineArrowObject.setFaces(new Line3D[]{new Line3D(copter.getPosition(),Utils.getNormalVectorToPoint(refPosition))});

            //System.out.println((copter.getAngle()));
            //System.out.println(Utils.getNormalVectorToPoint(refPosition));

            //OBJRef.setPosition(Utils.getNormalVectorToPoint(refPosition).multiply(3).add(copter.getPosition()));
            Quaternion angleRotated = Utils.eulerAnglesToQuaternion(new Point3D(0,90,0)).multiply(copter.getAngle()).multiply(Utils.eulerAnglesToQuaternion(new Point3D(0,90,0)).conjugate());
            Quaternion errorRotationOther = Utils.getQuaternionToPoint( refPosition.add(Utils.rotY(90).multiply(copter.getPosition())) ).multiply(angleRotated.conjugate());

            Point3D value1 = Utils.quaternionToEulerAngles(copter.getAngle());
            Point3D value2 = Utils.quaternionToEulerAngles(Utils.eulerAnglesToQuaternion(new Point3D(0,90,0)).multiply(copter.getAngle()).multiply(Utils.eulerAnglesToQuaternion(new Point3D(-180,0,-180))));
            //Point3D value = Utils.quaternionToEulerAngles(copter.getAngle());
            /*System.out.printf("angle: %.5f", value1.getX());
            System.out.printf(" %.5f", value1.getY());
            System.out.printf(" %.5f\t", value1.getZ());
            System.out.printf("%.5f", value2.getX());
            System.out.printf(" %.5f", value2.getY());
            System.out.printf(" %.5f\n", value2.getZ());*/

            System.out.println(Utils.quaternionToEulerAngles(Utils.eulerAnglesToQuaternion(new Point3D(0,-90,0)).multiply(copter.getAngle()).multiply(Utils.eulerAnglesToQuaternion(new Point3D(0,0,0)))));

            //System.out.println(refPosition.add(refPosition.add(Utils.rotY(0).multiply(copter.getPosition()))));
            //System.out.println(refPosition.add(refPosition.add(Utils.rotY(90).multiply(copter.getPosition()))));




            //System.out.println(euler);

            Point3D vector = new Point3D(errorRotationOther.getX(),0,errorRotationOther.getZ());
            //Point3D acceleration = J.inverse().multiply(vector.subtract((copter.getAngleVelocity().cross(J.multiply(copter.getAngleVelocity())))).subtract(copter.getAngleVelocity().multiply(simulationEnvironment.getMomentFriction())));
            //vector = acceleration.multiply(0.1).add(copter.getAngleVelocity().multiply(0.01));

            double err1 = pid1.calculateControl(-copter.getPosition().getY(),100-refPosition.getY());
            double err2 = pid2.calculateControl(-vector.getX(),0);
            double err3 = pid3.calculateControl(-vector.getZ(),0);
            double err4 = 0;//pid4.calculateControl(+vector.getY(),0);

            m1.setSpeed(err1-err4+err2-err3);
            m2.setSpeed(err1+err4-err2-err3);
            m3.setSpeed(err1-err4-err2+err3);
            m4.setSpeed(err1+err4+err2+err3);

            /*m1.setSpeed(err1);
            m2.setSpeed(err1);
            m3.setSpeed(err1);
            m4.setSpeed(err1);*/

        });
        ///////////////////////// CONTROLLER END


        //обновляем управление
        updater.addTask(controlsGUI::updateControls);

        //обнавляем пропеллеры
        updater.addTask(() -> {
            for (AircraftBase.Propeller propeller : copter.getPropellers())
                propeller.updateThrust();
        });

        //обналяем и интегрируем силы коптера

        updater.addTask(copter::updateIntegration);

        //управление с клавиатуры

        controlsGUI.bindAKey(KeyEvent.VK_Y, () -> {
            pitch.updateAndGet(v -> (v + 0.0005));
            m1.setForce(m1.getForce().add(new Point3D(0, 0,pitch.get())));
        });

        controlsGUI.bindAKey(KeyEvent.VK_SHIFT, () -> {
            m1.setSpeed(m1.getThrust()-1.1f);
            m2.setSpeed(m2.getThrust()+1.1f);
            m3.setSpeed(m3.getThrust()-1.1f);
            m4.setSpeed(m4.getThrust()+1.1f);
        });
        controlsGUI.bindAKey(KeyEvent.VK_CONTROL, () -> {
            m1.setSpeed(m1.getThrust()+1.1f);
            m2.setSpeed(m2.getThrust()-1.1f);
            m3.setSpeed(m3.getThrust()+1.1f);
            m4.setSpeed(m4.getThrust()-1.1f);
        });
        controlsGUI.bindAKey(KeyEvent.VK_LEFT, () -> {
            m1.setSpeed(m1.getThrust()+1.1f);
            m2.setSpeed(m2.getThrust()-1.1f);
            m3.setSpeed(m3.getThrust()-1.1f);
            m4.setSpeed(m4.getThrust()+1.1f);
        });
        controlsGUI.bindAKey(KeyEvent.VK_UP, () -> {
            m1.setSpeed(m1.getThrust()-1.1f);
            m2.setSpeed(m2.getThrust()-1.1f);
            m3.setSpeed(m3.getThrust()+1.1f);
            m4.setSpeed(m4.getThrust()+1.1f);
        });
        controlsGUI.bindAKey(KeyEvent.VK_RIGHT, () -> {
            m1.setSpeed(m1.getThrust()-1.1f);
            m2.setSpeed(m2.getThrust()+1.1f);
            m3.setSpeed(m3.getThrust()+1.1f);
            m4.setSpeed(m4.getThrust()-1.1f);
        });
        controlsGUI.bindAKey(KeyEvent.VK_DOWN, () -> {
            m1.setSpeed(m1.getThrust()+1.1f);
            m2.setSpeed(m2.getThrust()+1.1f);
            m3.setSpeed(m3.getThrust()-1.1f);
            m4.setSpeed(m4.getThrust()-1.1f);
        });
        controlsGUI.bindAKey(KeyEvent.VK_N, () -> {
            m1.setSpeed(0);
            m2.setSpeed(0);
            m3.setSpeed(0);
            m4.setSpeed(0);
        });

        // поворот камеры при зажатой кнопке M
        controlsGUI.bindAKey(KeyEvent.VK_M,
                () -> camera.setPosition(copter.getPosition().multiply(-1).add(
                new Point3D((20*Math.sin(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.sin(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.cos(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX())))))));
        // Cam rotation end

        //обнавляем позицию объектов коптера и пропеллеров
        updater.addTask(copter::updateObjectAndPropellers);

        //выводим FPS
        updater.addTask(() -> {
            textArea.setText(String.valueOf(updater.getFrames()));
        });

        ///////// END OF TEST ZONE

        //обнавляем отрисовку объектов
        updater.addTask(viewer3D::updateComponent);
    }

    void display() {
        setSize(500, 500);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
        setBackground(Color.BLACK);

        textArea.setEditable(false);
        textArea.setBackground(Color.BLACK);
        textArea.setFont( new Font(Font.DIALOG, Font.PLAIN, 30 ));
        textArea.setForeground(Color.WHITE);

        //add(textArea, BorderLayout.PAGE_START);
        add(viewer3D, BorderLayout.CENTER);

        viewer3D.setFocusable(true);
        viewer3D.grabFocus();

        setVisible(true);
    }
}
