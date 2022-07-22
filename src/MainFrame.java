import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.geom.Point2D;
import java.util.Arrays;
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
        AircraftBase.Propeller m1 = new AircraftBase.Propeller(0.5,new Point3D(0,-0.5,0),new Point3D(0,1,0),false,10,"propellerM.obj", simulationEnvironment);
        AircraftBase.Propeller m2 = new AircraftBase.Propeller(0.01,new Point3D(-2,0.,0),new Point3D(0,0,-1),true,10,"propellerG.obj",simulationEnvironment);

        //создаем сам коптер
        Matrix.m4x4 J = new Matrix.m4x4(0.64,0 ,0, 0, 0, 0.64, 0, 0, 0,0, 1.2,0, 0, 0 ,0 ,1 );
        AircraftBase helicopter = new AircraftBase(3,0.001, J, simulationEnvironment);

        //добавляем пропеллеры на коптер
        helicopter.addPropeller(m1);
        helicopter.addPropeller(m2);

        //Рассчитываем силы и моменты
        updater.addTask(helicopter::calculateForces);

        //добавляем коптер и пропеллеры в пространство
        viewer3D.addObject3D(helicopter.getCopterObject());
        for (int i = 0; i < helicopter.getPropellers().size(); i++) {
            viewer3D.addObject3D(helicopter.getPropellers().get(i).getObjectStick());
            viewer3D.addObject3D(helicopter.getPropellers().get(i).getPropeller3D());
        }

        //создаем и добавляем в пространство точку, в которурую должен прийти коптер
        Object3D OBJRef = new ReaderOBJ("arrow.obj").getObject();
        viewer3D.addObject3D(OBJRef);

        //добовляем следящую за коптером линию
        LineTracer lineTracer = new LineTracer(500,4,helicopter.getPosition());
        updater.addTask(()->lineTracer.updatePosition(helicopter.getPosition()));
        viewer3D.addObject3D(lineTracer.getLinesObject());

        ///////////////////////// CONTROLLER
        PID pid1 = new PID(0.8,0.4,2, simulationEnvironment);
        PID pid2 = new PID(1,0.01,0, simulationEnvironment);

        helicopter.setAngle(Utils.eulerAnglesToQuaternion(new Point3D(0,0,0)));
        Object3D lineArrowObject = new Object3D();
        viewer3D.addObject3D(lineArrowObject);

        updater.addTask( () -> {
            Point3D position = new Point3D(10,10,10);
            OBJRef.setPosition(position.scale(1,-1,1));
            double neededAngle = 45;

            double errX = position.getX()-helicopter.getPosition().getX();
            double errY = -(position.getY()-helicopter.getPosition().getY());
            double errZ = position.getZ()-helicopter.getPosition().getZ();

            Point3D errVec = Utils.rotY(-(neededAngle)).multiply(new Point3D(errX,errY,errZ));

            double angleX = Math.cos(Math.atan2(errVec.getY(),errVec.getX()));
            double angleZ = Math.cos(Math.atan2(errVec.getY(),errVec.getZ()));

            double errHover = pid1.calculateControl(-helicopter.getPosition().getY(),position.getY());
            double errRot = pid2.calculateControl(Math.acos(helicopter.getAngle().getW())*2, Math.toRadians(neededAngle));
            m1.setSpeed(errHover);
            m2.setSpeed(errRot);

            Quaternion q = new Quaternion(Utils.eulerAnglesToQuaternion(new Point3D(-angleZ,0,angleX))).multiply(helicopter.getAngle().conjugate());
            m1.setForce(Utils.quaternionRotation(q).multiply(new Point3D(0,1,0)));
        });
        ///////////////////////// CONTROLLER END


        //обновляем управление
        updater.addTask(controlsGUI::updateControls);

        //обнавляем пропеллеры
        updater.addTask(() -> {
            for (AircraftBase.Propeller propeller : helicopter.getPropellers())
                propeller.updateThrust();
        });

        //обналяем и интегрируем силы коптера

        updater.addTask(helicopter::updateIntegration);

        // поворот камеры при зажатой кнопке M
        controlsGUI.bindAKey(KeyEvent.VK_M,
                () -> camera.setPosition(helicopter.getPosition().multiply(-1).add(
                new Point3D((20*Math.sin(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.sin(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.cos(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX())))))));
        // Cam rotation end

        //обнавляем позицию объектов коптера и пропеллеров
        updater.addTask(helicopter::updateObjectAndPropellers);

        //выводим FPS
        updater.addTask(() -> textArea.setText(String.valueOf(updater.getFrames())));

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
