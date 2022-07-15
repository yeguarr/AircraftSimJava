import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.nio.channels.Pipe;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
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
        controlsGUI.bindWASD(camera, 0.7f);

        /////// TEST ZONE

        SimulationEnvironment simulationEnvironment = new SimulationEnvironment();
        double lPitch = 2.5;
        double lRoll = 2.5;

        QuadCopter.Propeller m1 = new QuadCopter.Propeller(new Point3D(lPitch,0,lRoll),new Point3D(0,1,0),false,100, simulationEnvironment);
        QuadCopter.Propeller m2 = new QuadCopter.Propeller(new Point3D(lPitch,0,-lRoll),new Point3D(0,1,0),true,100,simulationEnvironment);
        QuadCopter.Propeller m3 = new QuadCopter.Propeller(new Point3D(-lPitch,0,-lRoll),new Point3D(0,1,0),false,100,simulationEnvironment);
        QuadCopter.Propeller m4 = new QuadCopter.Propeller(new Point3D(-lPitch,0,lRoll),new Point3D(0,1,0),true,100,simulationEnvironment);
        //QuadCopter.Propeller m5 = new QuadCopter.Propeller(new Point3D(0,-5,0),new Point3D(0,1,0),true,1000,simulationEnvironment);

        QuadCopter copter = new QuadCopter(1,0.1, Utils.eye4x4(), simulationEnvironment);
        copter.propellers.add(m1);
        copter.propellers.add(m2);
        copter.propellers.add(m3);
        copter.propellers.add(m4);
        //copter.propellers.add(m5);
        //m5.setSpeed(1);

        Object3D OBJRef = new ReaderOBJ("ico.obj").getObject();

        /*List<Line3D> line3D = new LinkedList<>();
        for (int i = 0; i < copter.propellers.size(); i++) {
            line3D.add(new Line3D(copter.getPosition(),copter.propellers.get(i).getPosition()));

        }*/
        //Object3D sticks = new Object3D(line3D.toArray(new Line3D[0]));
        for (int i = 0; i < copter.propellers.size(); i++) {
            viewer3D.addObject3D(copter.propellers.get(i).getObjectStick());
            viewer3D.addObject3D(copter.propellers.get(i).getPropeller3D());
        }
        viewer3D.addObject3D(copter.copterObject);
        viewer3D.addObject3D(OBJRef);
        updater.addTask(viewer3D::updateComponent);

        PID pid1 = new PID(0.8,0.4,1.8, simulationEnvironment);
        PID pid2 = new PID(0.8,0.4,1.8, simulationEnvironment);
        PID pid3 = new PID(0.8,0.4,1.8, simulationEnvironment);
        PID pid4 = new PID(0.8,0.4,1.3, simulationEnvironment);

        AtomicReference<Double> pitch = new AtomicReference<>(45.);
        copter.setAngle(Utils.eulerAnglesToQuaternion(new Point3D(0,pitch.get(),0)));
        //Point3D refPosition = new Point3D(10,5,10);

        updater.addTask( () -> {

            //pitch.updateAndGet(v -> (v + 0.005ы));
            /*Point3D refPosition;
            if (pitch.get()<30)
                refPosition = new Point3D(10*Math.cos(pitch.get()),0,10*Math.sin(pitch.get()));
            else if (pitch.get()<50)
                refPosition = new Point3D(10*Math.cos(pitch.get())+40,0,10*Math.sin(pitch.get()));
            else
                refPosition = new Point3D(10*Math.cos(pitch.get())+20,0,20+20*Math.sin(pitch.get()));*/


            //Point3D refPosition = new Point3D(10*Math.cos(pitch.get()),10,10*Math.sin(pitch.get()));
            Point3D refPosition = new Point3D(10,10,10);

            OBJRef.setPosition(refPosition.scale(1,-1,1));

            Point3D refAngle = new Point3D(
                    Math.min(20,Math.hypot(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ()))
                            *(Math.cos(Math.atan2(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ())+pitch.get())),
                    pitch.get(),
                    Math.min(20,Math.hypot(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ()))
                            *(Math.sin(Math.atan2(refPosition.getX()-copter.getPosition().getX(),-refPosition.getZ()+copter.getPosition().getZ())+pitch.get()))
            );
            refAngle = Utils.rotate(new Point3D(0, -pitch.get()*1,0)).multiply(refAngle);
            //System.out.println( refAngle);

            Quaternion errorRotationPitch = Utils.eulerAnglesToQuaternion(refAngle).multiply(copter.getAngle().conjugate());
            Quaternion errorRotationOther = Utils.eulerAnglesToQuaternion(refAngle.scale(1,1,1)).multiply((copter.getAngle().conjugate()));

            /*Point3D torque2 = Utils.quaternionRotation(copter.getAngle()).multiply(
                    m1.getPosition().cross(m1.getForce().multiply(m1.getThrust()).add(m1.getMoment())).add(
                    m2.getPosition().cross(m2.getForce().multiply(m2.getThrust()).add(m2.getMoment()))).add(
                    m3.getPosition().cross(m3.getForce().multiply(m3.getThrust()).add(m3.getMoment()))).add(
                    m4.getPosition().cross(m4.getForce().multiply(m4.getThrust()).add(m4.getMoment()))));*/

           // System.out.println(copter.getTorque());
            //System.out.println(torque2);
            //System.out.println(copter.getTorque().add(torque2));

            double err1 = pid1.calculateControl(-copter.getPosition().getY()-refPosition.getY(),0);
            double err2 = pid2.calculateControl(-errorRotationOther.getX(),0);
            double err3 = pid3.calculateControl(-errorRotationOther.getZ(),0);
            double err4 = pid4.calculateControl(-errorRotationOther.getY(),0);

            m1.setSpeed(err1+err2-err3-err4);
            m2.setSpeed(err1-err2-err3+err4);
            m3.setSpeed(err1-err2+err3-err4);
            m4.setSpeed(err1+err2+err3+err4);

        });

        //Line tracer
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
        //Line tracer end

        updater.addTask(controlsGUI::updateControls);

        updater.addTask(() -> {
                for (QuadCopter.Propeller propeller : copter.propellers)
                    propeller.updateThrust();
        });

        updater.addTask(copter::calculateForces);
        updater.addTask(copter::updateIntegration);

        /*controlsGUI.bindAKey(KeyEvent.VK_U, () -> {
            m5.setSpeed(1);
        });
        controlsGUI.bindAKey(KeyEvent.VK_I, () -> {
            m5.setSpeed(0);
        });*/
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

        // Cam rotation
        controlsGUI.bindAKey(KeyEvent.VK_M, () -> camera.setPosition(copter.getPosition().multiply(-1).add(
                new Point3D((20*Math.sin(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.sin(Math.toRadians(camera.getRotation().getX()))),
                         (-20*Math.cos(Math.toRadians(camera.getRotation().getY()))*Math.cos(Math.toRadians(camera.getRotation().getX())))))));
        // Cam rotation end

        updater.addTask(() -> {
            copter.rotationCopterObject(Utils.quaternionRotation(copter.getAngle()));
            copter.positionCopterObject(copter.getPosition());
            for (int i = 0; i < copter.propellers.size(); i++) {
                copter.propellers.get(i).rotationPropeller3D(Utils.quaternionRotation(copter.getAngle()).multiply(Utils.quaternionRotation(Utils.normalToQuaternion(copter.propellers.get(i).getForce()))).multiply(Utils.rotate(new Point3D(0, -copter.propellers.get(i).getAngle(), 0))));
                Point3D temp = copter.getPosition().add(Utils.quaternionRotation(copter.getAngle()).multiply(copter.propellers.get(i).getPosition()));
                copter.propellers.get(i).positionPropeller3D(temp);
                copter.propellers.get(i).updateStick(copter.getPosition(),Utils.quaternionRotation(copter.getAngle()).multiply(copter.propellers.get(i).getPosition()));
            }
            //sticks.setFaces(line3D.toArray(new Line3D[0]));
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
