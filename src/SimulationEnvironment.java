public class SimulationEnvironment {
    private final Point3D gravityForce;
    private final double deltaTime;
    private final double forceFriction;
    private final double momentFriction;

    public SimulationEnvironment() {
        //gravityForce = new Point3D(0,9.81f, 0);
        gravityForce = new Point3D(0,0,0);
        deltaTime = 0.01;
        forceFriction = 0.5;
        momentFriction = 0.5;
    }

    public SimulationEnvironment(Point3D gravityForce, double deltaTime, double forceFriction, double momentFriction) {
        this.gravityForce = gravityForce;
        this.deltaTime = deltaTime;
        this.forceFriction = forceFriction;
        this.momentFriction = momentFriction;
    }

    public Point3D getGravityForce() {
        return gravityForce;
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    public double getForceFriction() {
        return forceFriction;
    }

    public double getMomentFriction() {
        return momentFriction;
    }
}
