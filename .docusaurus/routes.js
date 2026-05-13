import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug', 'a7d'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/config', '542'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/content', 'e1e'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/globalData', 'ade'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/metadata', '3e3'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/registry', 'a96'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-humanoid-robotics/__docusaurus/debug/routes', 'b0c'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/',
    component: ComponentCreator('/physical-ai-humanoid-robotics/', '96a'),
    exact: true
  },
  {
    path: '/physical-ai-humanoid-robotics/',
    component: ComponentCreator('/physical-ai-humanoid-robotics/', 'a8e'),
    routes: [
      {
        path: '/physical-ai-humanoid-robotics/',
        component: ComponentCreator('/physical-ai-humanoid-robotics/', '8a9'),
        routes: [
          {
            path: '/physical-ai-humanoid-robotics/',
            component: ComponentCreator('/physical-ai-humanoid-robotics/', '98f'),
            routes: [
              {
                path: '/physical-ai-humanoid-robotics/assessments/introduction',
                component: ComponentCreator('/physical-ai-humanoid-robotics/assessments/introduction', '198'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/assessments/midterm',
                component: ComponentCreator('/physical-ai-humanoid-robotics/assessments/midterm', '5b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/assessments/rubrics',
                component: ComponentCreator('/physical-ai-humanoid-robotics/assessments/rubrics', '660'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/capstone/implementation',
                component: ComponentCreator('/physical-ai-humanoid-robotics/capstone/implementation', 'a59'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/capstone/introduction',
                component: ComponentCreator('/physical-ai-humanoid-robotics/capstone/introduction', 'f4e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/capstone/project-requirements',
                component: ComponentCreator('/physical-ai-humanoid-robotics/capstone/project-requirements', '32f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/capstone/results',
                component: ComponentCreator('/physical-ai-humanoid-robotics/capstone/results', '196'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/hardware/actuator-specifications',
                component: ComponentCreator('/physical-ai-humanoid-robotics/hardware/actuator-specifications', '64f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/hardware/compute-requirements',
                component: ComponentCreator('/physical-ai-humanoid-robotics/hardware/compute-requirements', 'a2e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/hardware/introduction',
                component: ComponentCreator('/physical-ai-humanoid-robotics/hardware/introduction', '1cf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/hardware/sensor-specifications',
                component: ComponentCreator('/physical-ai-humanoid-robotics/hardware/sensor-specifications', '980'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/introduction',
                component: ComponentCreator('/physical-ai-humanoid-robotics/introduction', '840'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/lab-architecture/cloud-options',
                component: ComponentCreator('/physical-ai-humanoid-robotics/lab-architecture/cloud-options', 'c53'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/lab-architecture/containerization',
                component: ComponentCreator('/physical-ai-humanoid-robotics/lab-architecture/containerization', '954'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/lab-architecture/local-setup',
                component: ComponentCreator('/physical-ai-humanoid-robotics/lab-architecture/local-setup', '273'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/learning-outcomes',
                component: ComponentCreator('/physical-ai-humanoid-robotics/learning-outcomes', 'd14'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module1/introduction',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module1/introduction', 'ca7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module1/ros2-architecture',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module1/ros2-architecture', '733'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module1/ros2-concepts',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module1/ros2-concepts', 'd0a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module1/ros2-exercises',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module1/ros2-exercises', 'd18'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module1/ros2-packages',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module1/ros2-packages', 'c3d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module2/digital-twin-exercises',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module2/digital-twin-exercises', 'b6c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module2/gazebo-simulation',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module2/gazebo-simulation', '19d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module2/introduction',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module2/introduction', 'f3e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module2/unity-integration',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module2/unity-integration', 'ea7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module3/ai-models',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module3/ai-models', '84e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module3/introduction',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module3/introduction', 'e75'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module3/isaac-architecture',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module3/isaac-architecture', 'f60'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module3/robot-control',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module3/robot-control', '917'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module4/conversational-robotics',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module4/conversational-robotics', 'd5e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module4/introduction',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module4/introduction', '7d8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module4/vla-exercises',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module4/vla-exercises', '015'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/module4/vla-models',
                component: ComponentCreator('/physical-ai-humanoid-robotics/module4/vla-models', '1a2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/weekly-breakdown/week-1-2',
                component: ComponentCreator('/physical-ai-humanoid-robotics/weekly-breakdown/week-1-2', '3b9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/weekly-breakdown/week-11-12',
                component: ComponentCreator('/physical-ai-humanoid-robotics/weekly-breakdown/week-11-12', '4b8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/weekly-breakdown/week-3-4',
                component: ComponentCreator('/physical-ai-humanoid-robotics/weekly-breakdown/week-3-4', '897'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/weekly-breakdown/week-5-6',
                component: ComponentCreator('/physical-ai-humanoid-robotics/weekly-breakdown/week-5-6', '8d5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/weekly-breakdown/week-7-8',
                component: ComponentCreator('/physical-ai-humanoid-robotics/weekly-breakdown/week-7-8', 'd47'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/weekly-breakdown/week-9-10',
                component: ComponentCreator('/physical-ai-humanoid-robotics/weekly-breakdown/week-9-10', '208'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-humanoid-robotics/why-physical-ai-matters',
                component: ComponentCreator('/physical-ai-humanoid-robotics/why-physical-ai-matters', '317'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
