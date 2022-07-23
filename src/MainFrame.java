import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

public class MainFrame extends JFrame {
    Updater updater;
    Camera camera;
    ControlsGUI controlsGUI;
    Viewer3D viewer3D;

    public MainFrame() {
        super("3D Viewer");
        updater = new Updater(80);
        camera = new Camera();
        controlsGUI = new ControlsGUI(camera);
        viewer3D = new Viewer3D(controlsGUI);
        setup();
    }

    public static void main(String[] args) {
        MainFrame mainFrame = new MainFrame();
        mainFrame.updater.start();
        mainFrame.display();
    }

    public void setup() {
        camera.setPosition(new Point3D(0,0,-30));
        controlsGUI.bindWASD(camera, 0.7f);

        //создаем параметры пространства
        SimulationEnvironment simulationEnvironment = new SimulationEnvironment();

        //создаем вертолет (хз откуда брать параметры момента инерции)
        Matrix.m4x4 J = new Matrix.m4x4(0.64,0 ,0, 0, 0, 0.64, 0, 0, 0,0, 1.2,0, 0, 0 ,0 ,1 );
        AircraftBase helicopter = new AircraftBase(3,0.001, J,"HelicopterBase.obj", simulationEnvironment);

        //создаем пропеллеры
        AircraftBase.Propeller mainRotor = new AircraftBase.Propeller(0.5,new Point3D(0,-0.8,0),new Point3D(0,1,0),false,10,"MainRotor.obj", simulationEnvironment);
        AircraftBase.Propeller tailRotor = new AircraftBase.Propeller(0.01,new Point3D(-3.54,-0.6,0),new Point3D(0,0,-1),true,10,"TailRotor.obj",simulationEnvironment);

        //добавляем пропеллеры на коптер
        helicopter.addPropeller(mainRotor);
        helicopter.addPropeller(tailRotor);

        //добавляем внртолёт и пропеллеры в пространство
        viewer3D.addObject3D(helicopter.getCopterObject());
        for (int i = 0; i < helicopter.getPropellers().size(); i++) {
            //viewer3D.addObject3D(helicopter.getPropellers().get(i).getObjectStick());
            viewer3D.addObject3D(helicopter.getPropellers().get(i).getPropeller3D());
        }

        //создаем и добавляем в пространство точку, в которурую должен прийти коптер
        Object3D OBJRef = new ReaderOBJ("arrow.obj").getObject();
        viewer3D.addObject3D(OBJRef);

        //добовляем следящую за вертолетом линию
        LineTracer lineTracer = new LineTracer(500,4,helicopter.getPosition());
        updater.addTask(()->lineTracer.updatePosition(helicopter.getPosition()));
        viewer3D.addObject3D(lineTracer.getLinesObject());

        //to do настроить пиды
        PID pid1 = new PID(0.8,0.4,2, simulationEnvironment);
        PID pid2 = new PID(1,0.01,0.1, simulationEnvironment);
        QuaternionPID quaternionPID = new QuaternionPID(1,0,0, simulationEnvironment);

        helicopter.setAngle(Utils.eulerAnglesToQuaternion(new Point3D(0,0,0)));

        Point3D position = new Point3D(10,10,0);
        double neededAngle = 0;

        // Контроллер для вертолета
        HelicopterController helicopterController = new HelicopterController(position,neededAngle,pid1,pid2,quaternionPID);

        updater.addTask( () -> {
            mainRotor.setSpeed(helicopterController.mainRotorSpeed(helicopter.getPosition()));
            tailRotor.setSpeed(helicopterController.tailRotorSpeed(helicopter.getAngle()));
            mainRotor.setForce(helicopterController.getForceAngle(helicopter.getPosition(),helicopter.getAngle()));

            OBJRef.setPosition(position.scale(1,-1,1));
            OBJRef.setRotation(Utils.rotate(new Point3D(0,neededAngle,0)));
        });

        //обновляем управление
        updater.addTask(controlsGUI::updateControls);

        //Рассчитываем силы и моменты
        updater.addTask(helicopter::calculateForces);

        //обналяем и интегрируем силы коптера
        updater.addTask(helicopter::updateIntegration);

        //обнавляем пропеллеры
        updater.addTask(helicopter::updateThrust);

        // поворот камеры при зажатой кнопке M
        controlsGUI.bindAKey(KeyEvent.VK_M,
                () -> camera.setPosition(helicopter.getPosition().multiply(-1).add(
                new Point3D((20*Math.sin(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.sin(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.cos(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX())))))));

        //обнавляем позицию объектов коптера и пропеллеров
        updater.addTask(helicopter::updateObjectAndPropellers);

        //выводим FPS и ошибка.
        updater.addTask(()-> {
            Point3D error = position.subtract(helicopter.getPosition().scale(1,-1,1));
            String x = String.format("%.2f", error.getX());
            String y = String.format("%.2f", error.getY());
            String z = String.format("%.2f", error.getZ());
            viewer3D.setText("FPS: "+(updater.getFrames())+"\nError: "+x+" "+y+" "+z);
        });

        //обнавляем отрисовку объектов
        updater.addTask(viewer3D::updateComponent);
    }

    void display() {
        setSize(500, 500);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
        setBackground(Color.BLACK);
        addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent windowEvent) {
                updater.stop();
            }
        });

        add(viewer3D, BorderLayout.CENTER);

        viewer3D.setFocusable(true);
        viewer3D.grabFocus();

        setVisible(true);
    }
}
