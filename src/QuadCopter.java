public class QuadCopter {
    private Point3D position;
    private Point3D velocity;
    private Point3D acceleration;
    private Quaternion angle;
    private Point3D angleVelocity;
    private Point3D torque;

    private Propeller firstPropeller;
    private Propeller secondPropeller;
    private Propeller thirdPropeller;
    private Propeller forthPropeller;

    private final double mass;
    private final double lRoll;
    private final double lPitch;
    private final double trustToMoment;

    private final Matrix.m4x4 inertia;
    private final Matrix.m4x4 inertiaInv;

    private final SimulationEnvironment simulationEnvironment;

    public QuadCopter(double mass, double lRoll, double lPitch, double trustToMoment, Matrix.m4x4 inertia, Propeller firstPropeller, Propeller secondPropeller, Propeller thirdPropeller, Propeller forthPropeller, SimulationEnvironment simulationEnvironment) {
        this.mass = mass;
        this.lRoll = lRoll;
        this.lPitch = lPitch;
        this.trustToMoment = trustToMoment;
        this.inertia = inertia;
        this.inertiaInv = inertia.inverse();
        this.firstPropeller = firstPropeller;
        this.secondPropeller = secondPropeller;
        this.thirdPropeller = thirdPropeller;
        this.forthPropeller = forthPropeller;
        this.firstPropeller.setPosition(new Point3D(lPitch,0.0f,lRoll));
        this.secondPropeller.setPosition(new Point3D(lPitch,0.0f,-lRoll));
        this.thirdPropeller.setPosition(new Point3D(-lPitch,0.0f,-lRoll));
        this.forthPropeller.setPosition(new Point3D(-lPitch,0.0f,lRoll));
        this.simulationEnvironment = simulationEnvironment;

        position = new Point3D();
        velocity = new Point3D();
        acceleration = new Point3D();
        angle = new Quaternion(1,0,0,0);
        angleVelocity = new Point3D();
        torque = new Point3D();
    }

    void updateIntegration() {
        position = position.add(velocity.multiply(simulationEnvironment.getDeltaTime()));
        velocity = velocity.add(acceleration.multiply(simulationEnvironment.getDeltaTime()));

        angle = angle.add(((angle.multiply(new Quaternion(0,angleVelocity))).multiply(0.5f)).multiply(simulationEnvironment.getDeltaTime())).normalise();
        angleVelocity = angleVelocity.add(torque.multiply(simulationEnvironment.getDeltaTime()));
    }

    void calculateForces() {

        acceleration = simulationEnvironment.getGravityForce().multiply(mass)
                .subtract(velocity.multiply(simulationEnvironment.getForceFriction()))
                .subtract(Utils.quaternionRotation(angle).multiply(
                        new Point3D(
                                0,
                                firstPropeller.getThrust() + secondPropeller.getThrust() + thirdPropeller.getThrust() + forthPropeller.getThrust(),
                                0
                        )))
                .multiply(1/mass);

        torque = inertiaInv.multiply(new Point3D(
                lRoll*(firstPropeller.getThrust() - secondPropeller.getThrust() - thirdPropeller.getThrust() + forthPropeller.getThrust()),
                trustToMoment*(-firstPropeller.getThrust() + secondPropeller.getThrust() - thirdPropeller.getThrust() + forthPropeller.getThrust()),
                lPitch*(-firstPropeller.getThrust() - secondPropeller.getThrust() + thirdPropeller.getThrust() + forthPropeller.getThrust())
                )
                .subtract(angleVelocity.multiply(simulationEnvironment.getMomentFriction()))
                .subtract(angleVelocity.cross(inertia.multiply(angleVelocity))));
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

    public Propeller getFirstPropeller() {
        return firstPropeller;
    }

    public Propeller getSecondPropeller() {
        return secondPropeller;
    }

    public Propeller getThirdPropeller() {
        return thirdPropeller;
    }

    public Propeller getForthPropeller() {
        return forthPropeller;
    }

    public static class Propeller {
        private double thrust = 0;
        private double speed = 0;
        private double angle = 0;
        private SimulationEnvironment se;
        private Point3D position;

        public void setPosition(Point3D position) {
            this.position = position;
        }

        Propeller(Point3D position, SimulationEnvironment se) {
            this.position = position;
            this.se = se;
        }
        Propeller(SimulationEnvironment se) {
            position = new Point3D();
            this.se = se;
        }

        public double getThrust() {
            return thrust;
        }

        public void setSpeed(double speed) {
            this.speed = speed;
        }

        public void updateThrust() {
            thrust += (speed-thrust)*0.99f*se.getDeltaTime();
            angle += 200*thrust*se.getDeltaTime();
        }

        public double getAngle() {
            return angle;
        }

        public Point3D getPosition() {
            return position;
        }
    }
}
