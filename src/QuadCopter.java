import java.awt.*;
import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.List;

public class QuadCopter {
    private Point3D position;
    private Point3D velocity;
    private Point3D acceleration;
    private Quaternion angle;
    private Point3D angleVelocity;
    private Point3D torque;

    public List<Propeller> propellers = new LinkedList<>();

    private final double mass;
    private final double trustToMoment;

    private final Matrix.m4x4 inertia;
    private final Matrix.m4x4 inertiaInv;

    private final SimulationEnvironment simulationEnvironment;

    Object3D copterObject;

    public QuadCopter(double mass, double trustToMoment, Matrix.m4x4 inertia, SimulationEnvironment simulationEnvironment) {
        this.mass = mass;
        this.trustToMoment = trustToMoment;
        this.inertia = inertia;
        this.inertiaInv = inertia.inverse();
        this.simulationEnvironment = simulationEnvironment;

        position = new Point3D();
        velocity = new Point3D();
        acceleration = new Point3D();
        angle = new Quaternion(1,0,0,0);
        angleVelocity = new Point3D();
        torque = new Point3D();
        copterObject = new ReaderOBJ("copter2.obj").getObject();
    }

    void updateIntegration() {
        position = position.add(velocity.multiply(simulationEnvironment.getDeltaTime()));
        velocity = velocity.add(acceleration.multiply(simulationEnvironment.getDeltaTime()));

        angle = angle.add(((angle.multiply(new Quaternion(0,angleVelocity))).multiply(0.5f)).multiply(simulationEnvironment.getDeltaTime())).normalise();
        angleVelocity = angleVelocity.add(torque.multiply(simulationEnvironment.getDeltaTime()));
    }

    void calculateForces() {
        Point3D propForce = new Point3D();
        for (Propeller propeller : propellers) {
            propForce = propForce.add(propeller.getForce().multiply(propeller.getThrust()));
        }
        Point3D propMoment = new Point3D();
        for (Propeller propeller : propellers) {
            propMoment = propMoment.add(propeller.getForce().multiply(propeller.getThrust()*trustToMoment*propeller.getDirection()));
        }
        acceleration=  simulationEnvironment.getGravityForce().multiply(mass)
                .subtract(velocity.multiply(simulationEnvironment.getForceFriction()))
                .subtract(Utils.quaternionRotation(angle).multiply(propForce)).multiply(1/mass);

        Point3D moment = new Point3D(0,0,0);
        for (Propeller propeller : propellers) {
            Point3D pos = propeller.getPosition().cross(propeller.getForce().multiply(propeller.getThrust()));
            moment = new Point3D(moment.getX()-pos.getX(),moment.getY()-pos.getY(),moment.getZ()-pos.getZ());
        }
        moment = moment.add(propMoment);

        this.torque = inertiaInv.multiply(moment
                .subtract(angleVelocity.multiply(simulationEnvironment.getMomentFriction()))
                .subtract(angleVelocity.cross(inertia.multiply(angleVelocity))));
    }
    public Object3D getCopterObject() {
        return copterObject;
    }

    void rotationCopterObject(Matrix.m4x4 rotation) {
         copterObject.setRotation(rotation);
    }
    void positionCopterObject(Point3D position) {
        copterObject.setPosition(position);
    }
    public Point3D getPosition() {
        return position;
    }

    public Quaternion getAngle() {
        return angle;
    }

    public Point3D getVelocity() {
        return velocity;
    }

    public Point3D getAcceleration() {
        return acceleration;
    }

    public Point3D getGyroscopeData() {
        return angleVelocity; //+n
    }
    public Point3D getCameraPositionData() {
        return position; //+n
    }
    public Point3D getCameraVelocityData() {
        return velocity; //+n
    }
    public Point3D getSensorAccelerationData() {
        return Utils.quaternionRotation(angle).inverse().multiply(acceleration); //+n
    }

    public Point3D getAngleVelocity() {
        return angleVelocity;
    }

    public Point3D getTorque() {
        return torque;
    }

    public void setPosition(Point3D position) {
        this.position = position;
    }

    public void setAngle(Quaternion angle) {
        this.angle = angle;
    }

    public static class Propeller {
        private double thrust = 0;
        private double speed = 0;
        private double angle = 0;
        private double maxSpeed;
        private int direction;
        private SimulationEnvironment se;
        private Point3D position;
        private Point3D force;
        private Point3D moment;
        private Object3D propeller3D;
        private Line3D[] line3D;
        private Object3D stick;

        public void setPosition(Point3D position) {
            this.position = position;
        }
        void rotationPropeller3D(Matrix.m4x4 rotation) {
            propeller3D.setRotation(rotation);
        }
        void positionPropeller3D(Point3D position) {
            propeller3D.setPosition(position);
        }

        Propeller(Point3D position, Point3D force, boolean direction, double maxSpeed, SimulationEnvironment se) {
            this.position = position;
            this.force = force.normalise();
            this.se = se;
            this.direction = (direction? -1 : 1);
            this.maxSpeed = maxSpeed;
            //this.moment = position.cross(force).multiply(this.direction);
            this.propeller3D = new ReaderOBJ("propeller.obj").getObject();
            propeller3D.setRotation(Utils.quaternionRotation(Utils.normalToQuaternion(force)));
            line3D = new Line3D[21];
            for (int i =0; i <= 20; i++) {
                line3D[i] = new Line3D(new Point3D(),new Point3D(),new Color(90,90,90),10);
            }
            stick = new Object3D(line3D);
        }

        public void updateStick(Point3D start, Point3D end) {
            line3D = new Line3D[21];
            for (int i =0; i <= 20; i++) {
                line3D[i] = new Line3D(start.add(end.multiply(i/20.)),start.add(end.multiply((i+1)/20.)),new Color(90,90,90),10);
            }
            stick.setFaces(line3D);
        }
        public Object3D getObjectStick() {
            return stick;
        }

        public double getThrust() {
            return thrust;
        }

        public void setSpeed(double speed) {
            this.speed = speed;
        }

        public void updateThrust() {
            thrust += Math.max(Math.min((speed-thrust)*0.99f*se.getDeltaTime(),maxSpeed),-maxSpeed);
            angle += 200*thrust*se.getDeltaTime()*Math.signum(direction);
        }

        public double getAngle() {
            return angle;
        }

        public Point3D getPosition() {
            return position;
        }

        public Point3D getForce() {
            return force;
        }

        public Point3D getMoment() {
            return moment;
        }

        public Object3D getPropeller3D() {
            return propeller3D;
        }

        public double[][] getRotation() {
            return propeller3D.getRotation().getMatrixArray();
        }

        public int getDirection() {
            return direction;
        }
    }
}
