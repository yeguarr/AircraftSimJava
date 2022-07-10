import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

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
        controlsGUI.bindWASD(camera, 0.75f);

        /////// TEST ZONE

        SimulationEnvironment simulationEnvironment = new SimulationEnvironment();

        QuadCopter.Propeller m1 = new QuadCopter.Propeller(simulationEnvironment);
        QuadCopter.Propeller m2 = new QuadCopter.Propeller(simulationEnvironment);
        QuadCopter.Propeller m3 = new QuadCopter.Propeller(simulationEnvironment);
        QuadCopter.Propeller m4 = new QuadCopter.Propeller(simulationEnvironment);

        QuadCopter copter = new QuadCopter(1,2.5f,2.5f,0.8f, Utils.eye4x4(), m1, m2, m3, m4, simulationEnvironment);

        Object3D copterObject = new ReaderOBJ("copter.obj").getObject();
        Object3D propeller1 = new ReaderOBJ("propeller.obj").getObject();
        Object3D OBJRef = new ReaderOBJ("ico.obj").getObject();
        Object3D propeller2 = new Object3D(propeller1);
        Object3D propeller3 = new Object3D(propeller1);
        Object3D propeller4 = new Object3D(propeller1);

        PID pid1 = new PID(0.8,0.4,1.3, simulationEnvironment);
        PID pid2 = new PID(0.8,0.4,1.8, simulationEnvironment);
        PID pid3 = new PID(0.8,0.4,1.8, simulationEnvironment);
        PID pid4 = new PID(0.8,0.4,1.3, simulationEnvironment);

        AtomicReference<Double> pitch = new AtomicReference<>(0.);
        copter.setAngle(Utils.eulerAnglesToQuaternion(new Point3D(0,pitch.get(),0)));
        //Point3D refPosition = new Point3D(10,5,10);

        updater.addTask( () -> {
            if (pitch.get()<30)
                pitch.updateAndGet(v -> (v + 0.005));

            //Point3D refPosition = new Point3D(10*Math.cos(pitch.get()),10,10*Math.sin(pitch.get()));
            Point3D refPosition = new Point3D(0,0,0);

            OBJRef.setPosition(refPosition.scale(1,-1,1));

            Point3D refAngle = new Point3D(
                    Math.min(10,Math.hypot(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ()))
                            *Math.cos(Math.atan2(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ())),
                    pitch.get(),
                    Math.min(10,Math.hypot(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ()))
                            *Math.sin(Math.atan2(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ()))
            );

            Quaternion errorRotationPitch = Utils.eulerAnglesToQuaternion(refAngle).multiply(copter.getAngle().conjugate());
            Quaternion errorRotationOther = Utils.eulerAnglesToQuaternion(refAngle.scale(1,1,1)).multiply((copter.getAngle().conjugate()));

            //System.out.println(errorRotationOther);
            //System.out.println(pitch);
            System.out.println(copter.getPosition().subtract(refPosition.scale(1,-1,1)));
            System.out.println(pitch.get());
            System.out.println(errorRotationPitch.getY());
            double a = refAngle.getY()/90;
            //System.out.println(a);

            double err1 = pid1.calculateControl(-copter.getPosition().getY()-refPosition.getY(),0);
            double err2 = pid2.calculateControl(-errorRotationOther.getX()*(1-a)+errorRotationOther.getZ()*a,0);
            double err3 = pid3.calculateControl(-errorRotationOther.getZ()*(1-a)+errorRotationOther.getX()*a,0);
            double err4 = pid4.calculateControl(-errorRotationOther.getY(),0);

            m1.setSpeed(err1+err2-err3-err4);
            m2.setSpeed(err1-err2-err3+err4);
            m3.setSpeed(err1-err2+err3-err4);
            m4.setSpeed(err1+err2+err3+err4);
        });

        LinkedList<Point3D> linesList = new LinkedList<>();
        for (int i = 0; i < 500; i++) {
            linesList.add(copter.getPosition());
        }
        Object3D linesObject = new Object3D();
        viewer3D.addObject3D(linesObject);
        AtomicInteger count = new AtomicInteger();

        updater.addTask(() -> {
                    count.getAndIncrement();
                    if(count.get() % 4 != 0)
                        return;
                    linesList.pollFirst();
                    linesList.add(copter.getPosition());
                    Line3D[] line3DS = new Line3D[linesList.size()-2];
                    for (int i =0; i <line3DS.length; i++) {
                        line3DS[i] = new Line3D(linesList.get(i),linesList.get(i+1),new Color((float) i/line3DS.length,(float)i/line3DS.length,(float)i/line3DS.length), 5);
                    }
                    linesObject.setFaces(line3DS);
                });

        updater.addTask(controlsGUI::updateControls);

        updater.addTask(m1::updateThrust);
        updater.addTask(m2::updateThrust);
        updater.addTask(m3::updateThrust);
        updater.addTask(m4::updateThrust);

        updater.addTask(copter::calculateForces);
        updater.addTask(copter::updateIntegration);
        updater.addTask(viewer3D::updateComponent);

        controlsGUI.bindAKey(KeyEvent.VK_SHIFT, () -> {
            pitch.updateAndGet(v ->  (v + 0.5));

            /*m1.setSpeed(m1.getThrust()-1.1f);
            m2.setSpeed(m2.getThrust()+1.1f);
            m3.setSpeed(m3.getThrust()-1.1f);
            m4.setSpeed(m4.getThrust()+1.1f);*/
        });
        controlsGUI.bindAKey(KeyEvent.VK_CONTROL, () -> {
            pitch.updateAndGet(v ->  (v - 0.5));

            /*m1.setSpeed(m1.getThrust()+1.1f);
            m2.setSpeed(m2.getThrust()-1.1f);
            m3.setSpeed(m3.getThrust()+1.1f);
            m4.setSpeed(m4.getThrust()-1.1f);*/
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

        controlsGUI.bindAKey(KeyEvent.VK_M, () -> camera.setPosition(copterObject.getPosition().multiply(-1).add(
                new Point3D((20*Math.sin(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.sin(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.cos(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX())))))));

        viewer3D.addObject3D(copterObject);
        viewer3D.addObject3D(propeller1);
        viewer3D.addObject3D(propeller2);
        viewer3D.addObject3D(propeller3);
        viewer3D.addObject3D(propeller4);
        viewer3D.addObject3D(OBJRef);

        updater.addTask(() -> {
            copterObject.setRotation(Utils.quaternionRotation(copter.getAngle()));
            propeller1.setRotation(Utils.quaternionRotation(copter.getAngle()).multiply(Utils.rotate(new Point3D(0,-m1.getAngle(),0))));
            propeller2.setRotation(Utils.quaternionRotation(copter.getAngle()).multiply(Utils.rotate(new Point3D(0,m2.getAngle(),0))));
            propeller3.setRotation(Utils.quaternionRotation(copter.getAngle()).multiply(Utils.rotate(new Point3D(0,-m3.getAngle(),0))));
            propeller4.setRotation(Utils.quaternionRotation(copter.getAngle()).multiply(Utils.rotate(new Point3D(0,m4.getAngle(),0))));

        });
        updater.addTask(() -> {
            copterObject.setPosition(copter.getPosition());
            propeller1.setPosition(copter.getPosition().add(Utils.quaternionRotation(copter.getAngle()).multiply(m1.getPosition())));
            propeller2.setPosition(copter.getPosition().add(Utils.quaternionRotation(copter.getAngle()).multiply(m2.getPosition())));
            propeller3.setPosition(copter.getPosition().add(Utils.quaternionRotation(copter.getAngle()).multiply(m3.getPosition())));
            propeller4.setPosition(copter.getPosition().add(Utils.quaternionRotation(copter.getAngle()).multiply(m4.getPosition())));
        });

        //updater.addTask(() -> System.out.println(updater.getFrames()));

        ///////// END OF TEST ZONE
    }

    void display() {
        setSize(500, 500);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);

        viewer3D.setFocusable(true);
        viewer3D.grabFocus();

        add(viewer3D, BorderLayout.CENTER);
        setVisible(true);
    }
}
