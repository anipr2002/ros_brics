// #include <QApplication>
// #include <QWidget>
// #include <QVBoxLayout>
// #include <QFormLayout>
// #include <QLineEdit>
// #include <QPushButton>
// #include <QLabel>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// // Function to execute the motion plan
// void executeMotionPlan(
//     double x, double y, double z,
//     double qx, double qy, double qz, double qw,
//     rclcpp::Node::SharedPtr node, rclcpp::Logger logger)
// {
//     using namespace moveit::planning_interface;

//     // Create the MoveGroupInterface
//     MoveGroupInterface moveGroup(node, "ur_manipulator");

//     // Define the goal pose
//     geometry_msgs::msg::Pose goalPose;
//     goalPose.position.x = x;
//     goalPose.position.y = y;
//     goalPose.position.z = z;
//     goalPose.orientation.x = qx;
//     goalPose.orientation.y = qy;
//     goalPose.orientation.z = qz;
//     goalPose.orientation.w = qw;

//     // Set the pose target
//     moveGroup.setPoseTarget(goalPose);

//     // Plan and execute the motion
//     MoveGroupInterface::Plan plan;
//     if (moveGroup.plan(plan))
//     {
//         moveGroup.execute(plan);
//         RCLCPP_INFO(logger, "Plan executed successfully.");
//     }
//     else
//     {
//         RCLCPP_ERROR(logger, "Failed to plan and execute motion!");
//     }
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>(
//         "move_program_gui",
//         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
//     auto logger = rclcpp::get_logger("move_program_gui");

//     QApplication app(argc, argv);

//     // Create the main window
//     QWidget window;
//     window.setWindowTitle("Robot Motion Planner");

//     QVBoxLayout *layout = new QVBoxLayout(&window);

//     // Input fields for position and orientation
//     QFormLayout *formLayout = new QFormLayout;
//     QLineEdit *xInput = new QLineEdit;
//     QLineEdit *yInput = new QLineEdit;
//     QLineEdit *zInput = new QLineEdit;
//     QLineEdit *qxInput = new QLineEdit;
//     QLineEdit *qyInput = new QLineEdit;
//     QLineEdit *qzInput = new QLineEdit;
//     QLineEdit *qwInput = new QLineEdit;

//     formLayout->addRow("Position X:", xInput);
//     formLayout->addRow("Position Y:", yInput);
//     formLayout->addRow("Position Z:", zInput);
//     formLayout->addRow("Orientation X:", qxInput);
//     formLayout->addRow("Orientation Y:", qyInput);
//     formLayout->addRow("Orientation Z:", qzInput);
//     formLayout->addRow("Orientation W:", qwInput);
//     layout->addLayout(formLayout);

//     // Execute button
//     QPushButton *executeButton = new QPushButton("Execute Motion");
//     QLabel *statusLabel = new QLabel;
//     layout->addWidget(executeButton);
//     layout->addWidget(statusLabel);

//     // Connect the execute button to the motion planning function
//     QObject::connect(executeButton, &QPushButton::clicked, [&]() {
//         double x = xInput->text().toDouble();
//         double y = yInput->text().toDouble();
//         double z = zInput->text().toDouble();
//         double qx = qxInput->text().toDouble();
//         double qy = qyInput->text().toDouble();
//         double qz = qzInput->text().toDouble();
//         double qw = qwInput->text().toDouble();

//         try
//         {
//             executeMotionPlan(x, y, z, qx, qy, qz, qw, node, logger);
//             statusLabel->setText("Motion executed successfully!");
//         }
//         catch (const std::exception &e)
//         {
//             statusLabel->setText(QString("Error: ") + e.what());
//         }
//     });

//     // Show the GUI
//     window.show();
//     return app.exec();
// }

#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

// Function to convert degrees to radians
double degreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

// Function to execute the motion plan
void executeMotionPlan(
    double x, double y, double z,
    double roll_deg, double pitch_deg, double yaw_deg,
    rclcpp::Node::SharedPtr node, rclcpp::Logger logger)
{
    using namespace moveit::planning_interface;

    // Convert Euler angles (in degrees) to quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(
        degreesToRadians(roll_deg),
        degreesToRadians(pitch_deg),
        degreesToRadians(-yaw_deg)
    );

    // Create the MoveGroupInterface
    MoveGroupInterface moveGroup(node, "ur_manipulator");

    // Define the goal pose
    geometry_msgs::msg::Pose goalPose;
    goalPose.position.x = x;
    goalPose.position.y = y;
    goalPose.position.z = z;
    goalPose.orientation.x = quaternion.x();
    goalPose.orientation.y = quaternion.y();
    goalPose.orientation.z = quaternion.z();
    goalPose.orientation.w = quaternion.w();

    // Set the pose target
    moveGroup.setPoseTarget(goalPose);

    // Plan and execute the motion
    MoveGroupInterface::Plan plan;
    if (moveGroup.plan(plan))
    {
        moveGroup.execute(plan);
        RCLCPP_INFO(logger, "Plan executed successfully.");
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to plan and execute motion!");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "move_program_gui",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto logger = rclcpp::get_logger("move_program_gui");

    QApplication app(argc, argv);

    // Create the main window
    QWidget window;
    window.setWindowTitle("Robot Motion Planner");

    QVBoxLayout *layout = new QVBoxLayout(&window);

    // Input fields for position and Euler angles
    QFormLayout *formLayout = new QFormLayout;
    QLineEdit *xInput = new QLineEdit;
    QLineEdit *yInput = new QLineEdit;
    QLineEdit *zInput = new QLineEdit;
    QLineEdit *rollInput = new QLineEdit;
    QLineEdit *pitchInput = new QLineEdit;
    QLineEdit *yawInput = new QLineEdit;

    formLayout->addRow("Position X:", xInput);
    formLayout->addRow("Position Y:", yInput);
    formLayout->addRow("Position Z:", zInput);
    formLayout->addRow("Roll (degrees):", rollInput);
    formLayout->addRow("Pitch (degrees):", pitchInput);
    formLayout->addRow("Yaw (degrees):", yawInput);
    layout->addLayout(formLayout);

    // Execute button
    QPushButton *executeButton = new QPushButton("Execute Motion");
    QLabel *statusLabel = new QLabel;
    layout->addWidget(executeButton);
    layout->addWidget(statusLabel);

    // Connect the execute button to the motion planning function
    QObject::connect(executeButton, &QPushButton::clicked, [&]() {
        double x = xInput->text().toDouble();
        double y = yInput->text().toDouble();
        double z = zInput->text().toDouble();
        double roll = rollInput->text().toDouble();
        double pitch = pitchInput->text().toDouble();
        double yaw = yawInput->text().toDouble();

        try
        {
            executeMotionPlan(x, y, z, roll, pitch, yaw, node, logger);
            statusLabel->setText("Motion executed successfully!");
        }
        catch (const std::exception &e)
        {
            statusLabel->setText(QString("Error: ") + e.what());
        }
    });

    // Show the GUI
    window.show();
    return app.exec();
}
