#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

int main() {
    // Create two transforms
    tf2::Transform transform_msg, transform_init;

    // Set the translation and rotation of the transforms
    transform_msg.setOrigin(tf2::Vector3(1.0, 0.0, 0.0));
    transform_msg.setRotation(tf2::Quaternion(0.707, 0.0, 0.0, 0.707));

    transform_init.setOrigin(tf2::Vector3(0.8, 0.0, 0.5));
    transform_init.setRotation(tf2::Quaternion(0.4963, 0.4963, 0.0, -0.71236));

    // Perform the transformation multiplication
    tf2::Transform t = transform_init * transform_msg;

    // Print the result
    std::cout << "Resulting transform: " << std::endl << t.getOrigin().x() << "," << t.getOrigin().y() << "," << t.getOrigin().z() << std::endl;
    return 0;
}
