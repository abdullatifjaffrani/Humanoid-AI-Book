// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction'
    },
    {
      type: 'doc',
      id: 'quickstart',
      label: 'Quick Start Guide'
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Foundations',
      items: [
        {
          type: 'doc',
          id: 'modules/module-1-ros-foundations/week-1-introduction',
          label: 'Week 1: Introduction to ROS 2'
        },
        {
          type: 'doc',
          id: 'modules/module-1-ros-foundations/week-2-ros-nodes',
          label: 'Week 2: ROS Nodes and Services'
        },
        {
          type: 'doc',
          id: 'modules/module-1-ros-foundations/week-3-ros-topics',
          label: 'Week 3: ROS Topics and Messages'
        },
        {
          type: 'doc',
          id: 'modules/module-1-ros-foundations/lab-1-ros-basics',
          label: 'Lab 1: ROS Basics Exercise'
        }
      ],
      link: {
        type: 'generated-index',
        title: 'Module 1: ROS 2 Foundations',
        description: 'Learn the fundamentals of ROS 2, including nodes, services, topics, and messages.',
        slug: '/modules/module-1-ros-foundations'
      }
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo/Unity Simulation',
      items: [
        {
          type: 'doc',
          id: 'modules/module-2-gazebo-unity/week-4-simulation-basics',
          label: 'Week 4: Simulation Basics'
        },
        {
          type: 'doc',
          id: 'modules/module-2-gazebo-unity/week-5-gazebo-environments',
          label: 'Week 5: Gazebo Environments'
        },
        {
          type: 'doc',
          id: 'modules/module-2-gazebo-unity/lab-2-simulation',
          label: 'Lab 2: Simulation Exercise'
        }
      ],
      link: {
        type: 'generated-index',
        title: 'Module 2: Gazebo/Unity Simulation',
        description: 'Explore robotics simulation with Gazebo and Unity environments.',
        slug: '/modules/module-2-gazebo-unity'
      }
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac Platform',
      items: [
        {
          type: 'doc',
          id: 'modules/module-3-nvidia-isaac/week-6-isaac-platform',
          label: 'Week 6: Isaac Platform Overview'
        },
        {
          type: 'doc',
          id: 'modules/module-3-nvidia-isaac/week-7-navigation-systems',
          label: 'Week 7: Navigation Systems'
        },
        {
          type: 'doc',
          id: 'modules/module-3-nvidia-isaac/lab-3-isaac-navigation',
          label: 'Lab 3: Isaac Navigation Exercise'
        }
      ],
      link: {
        type: 'generated-index',
        title: 'Module 3: NVIDIA Isaac Platform',
        description: 'Learn about NVIDIA Isaac robotics platform and navigation systems.',
        slug: '/modules/module-3-nvidia-isaac'
      }
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Systems',
      items: [
        {
          type: 'doc',
          id: 'modules/module-4-vla-systems/week-8-vision-processing',
          label: 'Week 8: Vision Processing'
        },
        {
          type: 'doc',
          id: 'modules/module-4-vla-systems/week-9-language-integration',
          label: 'Week 9: Language Integration'
        },
        {
          type: 'doc',
          id: 'modules/module-4-vla-systems/week-10-humanoid-control',
          label: 'Week 10: Humanoid Robot Control'
        },
        {
          type: 'doc',
          id: 'modules/module-4-vla-systems/week-11-multimodal-integration',
          label: 'Week 11: Multimodal Integration'
        },
        {
          type: 'doc',
          id: 'modules/module-4-vla-systems/week-12-humanoid-manipulation',
          label: 'Week 12: Humanoid Manipulation'
        },
        {
          type: 'doc',
          id: 'modules/module-4-vla-systems/week-13-humanoid-locomotion',
          label: 'Week 13: Humanoid Locomotion'
        },
        {
          type: 'doc',
          id: 'modules/module-4-vla-systems/week-14-system-integration',
          label: 'Week 14: System Integration'
        },
        {
          type: 'doc',
          id: 'modules/module-4-vla-systems/lab-4-vla-integration',
          label: 'Lab 4: VLA Integration Exercise'
        }
      ],
      link: {
        type: 'generated-index',
        title: 'Module 4: Vision-Language-Action Systems',
        description: 'Explore modern Vision-Language-Action systems in robotics.',
        slug: '/modules/module-4-vla-systems'
      }
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        {
          type: 'doc',
          id: 'modules/capstone/capstone-project-overview',
          label: 'Capstone Project Overview'
        },
        {
          type: 'doc',
          id: 'modules/capstone/capstone-implementation',
          label: 'Capstone Implementation Guide'
        },
        {
          type: 'doc',
          id: 'modules/capstone/capstone-evaluation',
          label: 'Capstone Evaluation Criteria'
        }
      ],
      link: {
        type: 'generated-index',
        title: 'Capstone Project: Autonomous Humanoid Robot',
        description: 'Complete project integrating all concepts learned in the textbook.',
        slug: '/modules/capstone'
      }
    },
    {
      type: 'category',
      label: 'References & Resources',
      items: [
        {
          type: 'doc',
          id: 'references/ros-bibliography',
          label: 'ROS Bibliography'
        },
        {
          type: 'doc',
          id: 'references/simulation-bibliography',
          label: 'Simulation Bibliography'
        },
        {
          type: 'doc',
          id: 'references/isaac-bibliography',
          label: 'Isaac Bibliography'
        },
        {
          type: 'doc',
          id: 'references/vla-bibliography',
          label: 'VLA Bibliography'
        },
        {
          type: 'doc',
          id: 'references/humanoid-bibliography',
          label: 'Humanoid Robotics Bibliography'
        },
        {
          type: 'doc',
          id: 'references/manipulation-bibliography',
          label: 'Manipulation Systems Bibliography'
        },
        {
          type: 'doc',
          id: 'references/humanoid-locomotion-bibliography',
          label: 'Humanoid Locomotion Bibliography'
        },
        {
          type: 'doc',
          id: 'references/sensor-fusion-bibliography',
          label: 'Sensor Fusion Bibliography'
        },
        {
          type: 'doc',
          id: 'references/system-integration-bibliography',
          label: 'System Integration Bibliography'
        },
        {
          type: 'doc',
          id: 'glossary',
          label: 'Glossary of Terms'
        },
        {
          type: 'doc',
          id: 'index-of-concepts',
          label: 'Index of Concepts and Terms'
        }
      ]
    }
  ],
};

module.exports = sidebars;