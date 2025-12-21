import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '61c'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'a29'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '005'),
            routes: [
              {
                path: '/docs/assessments/introduction',
                component: ComponentCreator('/docs/assessments/introduction', '67d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/assessments/midterm',
                component: ComponentCreator('/docs/assessments/midterm', '126'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/assessments/rubrics',
                component: ComponentCreator('/docs/assessments/rubrics', 'd52'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/implementation',
                component: ComponentCreator('/docs/capstone/implementation', '8be'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/introduction',
                component: ComponentCreator('/docs/capstone/introduction', '2f3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/project-requirements',
                component: ComponentCreator('/docs/capstone/project-requirements', 'e2f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/capstone/results',
                component: ComponentCreator('/docs/capstone/results', 'ee4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/hardware/actuator-specifications',
                component: ComponentCreator('/docs/hardware/actuator-specifications', '3e8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/hardware/compute-requirements',
                component: ComponentCreator('/docs/hardware/compute-requirements', '2ea'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/hardware/introduction',
                component: ComponentCreator('/docs/hardware/introduction', 'a22'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/hardware/sensor-specifications',
                component: ComponentCreator('/docs/hardware/sensor-specifications', '24b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/introduction',
                component: ComponentCreator('/docs/introduction', 'f7d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/lab-architecture/cloud-options',
                component: ComponentCreator('/docs/lab-architecture/cloud-options', 'bc7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/lab-architecture/containerization',
                component: ComponentCreator('/docs/lab-architecture/containerization', 'b9a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/lab-architecture/local-setup',
                component: ComponentCreator('/docs/lab-architecture/local-setup', '1c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/learning-outcomes',
                component: ComponentCreator('/docs/learning-outcomes', '769'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/introduction',
                component: ComponentCreator('/docs/module1/introduction', '8b3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-architecture',
                component: ComponentCreator('/docs/module1/ros2-architecture', '3f7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-concepts',
                component: ComponentCreator('/docs/module1/ros2-concepts', '526'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-exercises',
                component: ComponentCreator('/docs/module1/ros2-exercises', '3dc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/ros2-packages',
                component: ComponentCreator('/docs/module1/ros2-packages', '24a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/digital-twin-exercises',
                component: ComponentCreator('/docs/module2/digital-twin-exercises', 'e95'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/gazebo-simulation',
                component: ComponentCreator('/docs/module2/gazebo-simulation', '954'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/introduction',
                component: ComponentCreator('/docs/module2/introduction', 'cee'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/unity-integration',
                component: ComponentCreator('/docs/module2/unity-integration', 'c67'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/ai-models',
                component: ComponentCreator('/docs/module3/ai-models', '6de'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/introduction',
                component: ComponentCreator('/docs/module3/introduction', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/isaac-architecture',
                component: ComponentCreator('/docs/module3/isaac-architecture', '7e5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/robot-control',
                component: ComponentCreator('/docs/module3/robot-control', '734'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/conversational-robotics',
                component: ComponentCreator('/docs/module4/conversational-robotics', '01e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/introduction',
                component: ComponentCreator('/docs/module4/introduction', 'b3d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/vla-exercises',
                component: ComponentCreator('/docs/module4/vla-exercises', '8cc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/vla-models',
                component: ComponentCreator('/docs/module4/vla-models', '809'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/weekly-breakdown/week-1-2',
                component: ComponentCreator('/docs/weekly-breakdown/week-1-2', '517'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/weekly-breakdown/week-11-12',
                component: ComponentCreator('/docs/weekly-breakdown/week-11-12', '501'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/weekly-breakdown/week-3-4',
                component: ComponentCreator('/docs/weekly-breakdown/week-3-4', 'b7a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/weekly-breakdown/week-5-6',
                component: ComponentCreator('/docs/weekly-breakdown/week-5-6', '16c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/weekly-breakdown/week-7-8',
                component: ComponentCreator('/docs/weekly-breakdown/week-7-8', 'd72'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/weekly-breakdown/week-9-10',
                component: ComponentCreator('/docs/weekly-breakdown/week-9-10', '9cf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/why-physical-ai-matters',
                component: ComponentCreator('/docs/why-physical-ai-matters', '2f4'),
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
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
