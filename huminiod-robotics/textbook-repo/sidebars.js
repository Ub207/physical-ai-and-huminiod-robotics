// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: Foundations',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 1: Physical AI Overview',
          collapsible: true,
          collapsed: false,
          items: [
            'module-1/week-1/physical-ai-overview',
            'module-1/week-1/vision-language-action-framework',
            'module-1/week-1/embodied-intelligence-concepts',
          ],
        },
        {
          type: 'category',
          label: 'Week 2: Humanoid Robotics Fundamentals',
          collapsible: true,
          collapsed: false,
          items: [
            'module-1/week-2/humanoid-robot-design',
            'module-1/week-2/kinematics-dynamics',
            'module-1/week-2/locomotion-control',
          ],
        },
        {
          type: 'category',
          label: 'Week 3: ROS 2 Ecosystem',
          collapsible: true,
          collapsed: false,
          items: [
            'module-1/week-3/ros2-jazzy-kilted-kaiju-setup',
            'module-1/week-3/rclpy-basics',
            'module-1/week-3/ros2-nodes-services-actions',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 4: Gazebo Simulation',
          collapsible: true,
          collapsed: false,
          items: [
            'module-2/week-4/gazebo-garden-harmonic-setup',
            'module-2/week-4/creating-humanoid-models',
            'module-2/week-4/simulation-scenarios',
          ],
        },
        {
          type: 'category',
          label: 'Week 5: Isaac Sim',
          collapsible: true,
          collapsed: false,
          items: [
            'module-2/week-5/isaac-sim-5-0-setup',
            'module-2/week-5/advanced-humanoid-simulation',
            'module-2/week-5/perception-integration',
          ],
        },
        {
          type: 'category',
          label: 'Week 6: Simulation Comparison',
          collapsible: true,
          collapsed: false,
          items: [
            'module-2/week-6/gazebo-vs-isaac-sim',
            'module-2/week-6/choosing-right-platform',
            'module-2/week-6/transfer-learning-sim2real',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Perception and Control',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 7: Sensor Integration',
          collapsible: true,
          collapsed: false,
          items: [
            'module-3/week-7/camera-lidar-fusion',
            'module-3/week-7/imu-force-torque-sensors',
            'module-3/week-7/sensor-calibration',
          ],
        },
        {
          type: 'category',
          label: 'Week 8: Computer Vision',
          collapsible: true,
          collapsed: false,
          items: [
            'module-3/week-8/object-detection-tracking',
            'module-3/week-8/pose-estimation',
            'module-3/week-8/vision-processing-pipelines',
          ],
        },
        {
          type: 'category',
          label: 'Week 9: Control Systems',
          collapsible: true,
          collapsed: false,
          items: [
            'module-3/week-9/feedback-control',
            'module-3/week-9/model-predictive-control',
            'module-3/week-9/adaptive-control',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Integration',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Week 10: VLA Models',
          collapsible: true,
          collapsed: false,
          items: [
            'module-4/week-10/vla-architecture-overview',
            'module-4/week-10/pretrained-vla-models',
            'module-4/week-10/fine-tuning-strategies',
          ],
        },
        {
          type: 'category',
          label: 'Week 11: Human-Robot Interaction',
          collapsible: true,
          collapsed: false,
          items: [
            'module-4/week-11/voice-command-processing',
            'module-4/week-11/natural-language-understanding',
            'module-4/week-11/multimodal-interaction',
          ],
        },
        {
          type: 'category',
          label: 'Week 12: Advanced Applications',
          collapsible: true,
          collapsed: false,
          items: [
            'module-4/week-12/manipulation-tasks',
            'module-4/week-12/navigation-planning',
            'module-4/week-12/autonomous-behaviors',
          ],
        },
        {
          type: 'category',
          label: 'Week 13: Project Integration',
          collapsible: true,
          collapsed: false,
          items: [
            'module-4/week-13/integrated-project-setup',
            'module-4/week-13/troubleshooting-guide',
            'module-4/week-13/future-directions',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      collapsible: true,
      collapsed: true,
      items: [
        'hardware-appendix/hardware-setup-guide',
        'hardware-appendix/troubleshooting-common-issues',
        'hardware-appendix/supplier-recommendations',
        'hardware-appendix/calibration-procedures',
      ],
    },
    {
      type: 'link',
      label: 'GitHub Repository',
      href: 'https://github.com/your-organization/physical-ai-humanoid-robotics',
    },
  ],
};

module.exports = sidebars;