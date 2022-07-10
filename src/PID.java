public class PID {
    private final SimulationEnvironment simulationEnvironment;

    private double lastState;
    private double sum;

    private final double p, i, d;

    public PID(double p, double i, double d, SimulationEnvironment simulationEnvironment) {
        this.simulationEnvironment = simulationEnvironment;
        lastState = 0;
        sum = 0;
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double calculateControl(double current, double destination) {
        double error = destination - current;
        sum += error*simulationEnvironment.getDeltaTime();

        double alpha = p*error;
        double betta = i*sum;
        double gamma = d*(error - lastState)/simulationEnvironment.getDeltaTime();

        lastState = error;

        return alpha + betta + gamma;
    }
}
