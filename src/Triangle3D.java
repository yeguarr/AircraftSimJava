import java.awt.*;

public class Triangle3D extends Shape3D {
    private Point3D first;
    private Point3D second;
    private Point3D third;
    private Color color;

    public Triangle3D(Point3D first, Point3D second, Point3D third) {
        this.first = first;
        this.second = second;
        this.third = third;
        this.color = Color.WHITE;
    }

    public Triangle3D(Point3D first, Point3D second, Point3D third, Color color) {
        this.first = first;
        this.second = second;
        this.third = third;
        this.color = color;
    }

    public Shape3D getTransformed(Matrix.m4x4 transformation) {
        return new Triangle3D(transformation.multiply(this.first), transformation.multiply(this.second), transformation.multiply(this.third),color);
    }

    @Override
    public Polygon getPolygon() {
        return new Polygon(new int[]{(int) this.first.getX(), (int) this.second.getX(), (int) this.third.getX()},
                new int[]{(int) this.first.getY(), (int) this.second.getY(), (int) this.third.getY()}, 3);
    }

    public double getZCentre() {
        return (this.first.getZ() + this.second.getZ() + this.third.getZ()) / 3;
    }

    @Override
    public Point3D[] getVertices() {
        return new Point3D[]{first, second, third};
    }

    public Point3D getNormal() {
        Point3D a = second.add(-first.getX(),-first.getY(),-first.getZ());
        Point3D b = third.add(-first.getX(),-first.getY(),-first.getZ());
        return a.cross(b).normalise();
    }

    @Override
    public Color getColor() {
        return color;
    }

    @Override
    public void drawOnComponent(Graphics2D g2d) {
        g2d.setPaint(this.getColor());
        g2d.fillPolygon(this.getPolygon());
    }
}
