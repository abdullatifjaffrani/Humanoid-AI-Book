"""
Script to index textbook content into the vector store
"""
import sys
import os
# Add the backend/src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

# Import the text indexer
from src.vector_store.indexer import text_indexer


def index_textbook_content():
    """
    Index textbook content from files into the vector store
    """
    print("Indexing textbook content...")

    # In a real implementation, this would read from actual textbook files
    # For now, we'll use the same sample content as in setup_vector_store.py
    textbook_content = [
        {
            "document_id": "ch1_intro_robotics",
            "content": "Robotics is an interdisciplinary branch of engineering and science that includes mechanical engineering, electrical engineering, computer science, and others. Robotics deals with the design, construction, operation, and use of robots, as well as computer systems for their control, sensory feedback, and information processing. Key components of robotic systems include sensors, actuators, control systems, and power sources.",
            "source_reference": "Chapter 1: Introduction to Robotics",
            "metadata": {
                "chapter": "1",
                "section": "1.1",
                "page_number": 1
            }
        },
        {
            "document_id": "ch2_humanoid_design",
            "content": "Humanoid robots are robots with physical human-like features such as a head, two arms, and two legs. The purpose of humanoid robots is to make human-robot interaction more natural. Humanoid robots can be used for various purposes including research, education, entertainment, and assistance. Design considerations include kinematics, dynamics, balance, and human-robot interaction.",
            "source_reference": "Chapter 2: Humanoid Robot Design",
            "metadata": {
                "chapter": "2",
                "section": "2.1",
                "page_number": 25
            }
        },
        {
            "document_id": "ch3_sensors",
            "content": "Sensors in humanoid robots provide perception capabilities. Common sensor types include cameras for vision, IMUs for orientation, force/torque sensors for contact detection, and encoders for joint position feedback. Sensor fusion combines data from multiple sensors to improve perception accuracy.",
            "source_reference": "Chapter 3: Sensors and Perception",
            "metadata": {
                "chapter": "3",
                "section": "3.2",
                "page_number": 42
            }
        },
        {
            "document_id": "ch5_control_systems",
            "content": "Control systems in humanoid robots typically include sensors for feedback and actuators for movement. The control system processes sensor data to make decisions about motor commands. Common control approaches include PID control, adaptive control, and model predictive control. Stability and balance control are critical for humanoid locomotion.",
            "source_reference": "Chapter 5: Control Systems for Humanoid Robots",
            "metadata": {
                "chapter": "5",
                "section": "5.2",
                "page_number": 87
            }
        },
        {
            "document_id": "ch7_actuators",
            "content": "Actuators in humanoid robots convert energy into mechanical motion. Common types include servo motors, pneumatic actuators, and hydraulic actuators. The choice of actuator affects the robot's strength, speed, and precision of movement. Series elastic actuators provide better force control for safe human-robot interaction.",
            "source_reference": "Chapter 7: Actuators and Movement Systems",
            "metadata": {
                "chapter": "7",
                "section": "7.1",
                "page_number": 134
            }
        },
        {
            "document_id": "ch9_ai_integration",
            "content": "Artificial intelligence integration in humanoid robots enables higher-level capabilities such as learning, planning, and decision making. Machine learning techniques including reinforcement learning and deep learning are used for various robotic tasks. Natural language processing allows for human-robot communication.",
            "source_reference": "Chapter 9: AI Integration in Humanoid Robots",
            "metadata": {
                "chapter": "9",
                "section": "9.3",
                "page_number": 189
            }
        },
        {
            "document_id": "ch12_simulation_environments",
            "content": "Creating realistic and useful simulation environments is crucial for effective robotics development. This includes using tools like Gazebo, which provides physics simulation, 3D rendering, and sensor simulation capabilities. Simulation environments allow for testing robot algorithms, control systems, and behaviors without the risk and cost of physical hardware. Key features of simulation environments include accurate physics modeling, realistic sensor simulation, and support for various robot models. Advanced techniques for designing Gazebo environments include creating detailed world files, configuring physics parameters, and integrating with ROS/ROS2 for robot control. Simulation environments should accurately represent real-world scenarios to ensure effective transfer of learned behaviors from simulation to reality.",
            "source_reference": "Chapter 12: Simulation Environments for Robotics",
            "metadata": {
                "chapter": "12",
                "section": "12.1",
                "page_number": 245
            }
        },
        {
            "document_id": "ch12_gazebo_techniques",
            "content": "Gazebo is a powerful 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. Advanced techniques for designing Gazebo environments involve creating detailed world files with accurate physical properties, configuring lighting and visual effects, and setting up sensor models that match real hardware. To create realistic simulation scenarios, developers should focus on modeling environmental conditions that match real-world deployment scenarios. This includes terrain modeling, lighting conditions, and dynamic elements that robots might encounter. Gazebo worlds can be designed to test specific capabilities such as navigation, manipulation, or human-robot interaction in controlled settings.",
            "source_reference": "Chapter 12: Advanced Gazebo Techniques",
            "metadata": {
                "chapter": "12",
                "section": "12.3",
                "page_number": 267
            }
        }
    ]

    print(f"Indexing {len(textbook_content)} textbook sections...")
    text_indexer.index_textbook_content(textbook_content)
    print("Textbook content indexing complete!")


if __name__ == "__main__":
    index_textbook_content()