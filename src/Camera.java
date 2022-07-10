public class Camera {
    private Point3D position;
    private Point3D rotation;

    public Camera(Point3D position, Point3D rotation) {
        this.position = position;
        this.rotation = rotation;
    }

    public Camera() {
        this.position = new Point3D();
        this.rotation = new Point3D();
    }

    public Matrix.m4x4 getTransformMatrix() {
        return Utils.rotX(rotation.getX()).multiply(Utils.rotY(rotation.getY())).multiply(Utils.rotZ(rotation.getZ())).multiply(Utils.move(position));
    }

    public Point3D getPosition() {
        return position;
    }

    public void setPosition(Point3D position) {
        this.position = position;
    }

    public Point3D getRotation() {
        return rotation;
    }

    public void setRotation(Point3D rotation) {
        this.rotation = rotation;
    }

}
