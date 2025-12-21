// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: [
        'introduction',
        'why-physical-ai-matters',
        'learning-outcomes'
      ],
    },
    {
      type: 'category',
      label: 'Weekly Breakdown',
      items: [
        'weekly-breakdown/week-1-2',
        'weekly-breakdown/week-3-4',
        'weekly-breakdown/week-5-6',
        'weekly-breakdown/week-7-8',
        'weekly-breakdown/week-9-10',
        'weekly-breakdown/week-11-12'
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        'module1/introduction',
        'module1/ros2-concepts',
        'module1/ros2-architecture',
        'module1/ros2-packages',
        'module1/ros2-exercises'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      items: [
        'module2/introduction',
        'module2/gazebo-simulation',
        'module2/unity-integration',
        'module2/digital-twin-exercises'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsed: false,
      items: [
        'module3/introduction',
        'module3/isaac-architecture',
        'module3/ai-models',
        'module3/robot-control'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: false,
      items: [
        'module4/introduction',
        'module4/vla-models',
        'module4/conversational-robotics',
        'module4/vla-exercises'
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      collapsed: false,
      items: [
        'capstone/introduction',
        'capstone/project-requirements',
        'capstone/implementation',
        'capstone/results'
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      items: [
        'assessments/introduction',
        'assessments/midterm',
        'assessments/rubrics'
      ],
    },
    {
      type: 'category',
      label: 'Hardware Requirements',
      items: [
        'hardware/introduction',
        'hardware/compute-requirements',
        'hardware/sensor-specifications',
        'hardware/actuator-specifications'
      ],
    },
    {
      type: 'category',
      label: 'Lab Architecture Options',
      items: [
        'lab-architecture/local-setup',
        'lab-architecture/cloud-options',
        'lab-architecture/containerization'
      ],
    },
  ],
};

export default sidebars;