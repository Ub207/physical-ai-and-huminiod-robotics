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
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', '23a'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', '663'),
        routes: [
          {
            path: '/',
            component: ComponentCreator('/', '286'),
            routes: [
              {
                path: '/assessments/introduction',
                component: ComponentCreator('/assessments/introduction', '858'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/assessments/midterm',
                component: ComponentCreator('/assessments/midterm', '2d4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/assessments/rubrics',
                component: ComponentCreator('/assessments/rubrics', 'e89'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/capstone/implementation',
                component: ComponentCreator('/capstone/implementation', '5ac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/capstone/introduction',
                component: ComponentCreator('/capstone/introduction', '199'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/capstone/project-requirements',
                component: ComponentCreator('/capstone/project-requirements', '58c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/capstone/results',
                component: ComponentCreator('/capstone/results', '259'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hardware/actuator-specifications',
                component: ComponentCreator('/hardware/actuator-specifications', '2e9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hardware/compute-requirements',
                component: ComponentCreator('/hardware/compute-requirements', '260'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hardware/introduction',
                component: ComponentCreator('/hardware/introduction', 'c48'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hardware/sensor-specifications',
                component: ComponentCreator('/hardware/sensor-specifications', '7aa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/introduction',
                component: ComponentCreator('/introduction', '363'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/lab-architecture/cloud-options',
                component: ComponentCreator('/lab-architecture/cloud-options', '5b8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/lab-architecture/containerization',
                component: ComponentCreator('/lab-architecture/containerization', '7d8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/lab-architecture/local-setup',
                component: ComponentCreator('/lab-architecture/local-setup', 'da4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/learning-outcomes',
                component: ComponentCreator('/learning-outcomes', '76d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module1/introduction',
                component: ComponentCreator('/module1/introduction', '1df'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module1/ros2-architecture',
                component: ComponentCreator('/module1/ros2-architecture', 'beb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module1/ros2-concepts',
                component: ComponentCreator('/module1/ros2-concepts', '8ac'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module1/ros2-exercises',
                component: ComponentCreator('/module1/ros2-exercises', 'ccf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module1/ros2-packages',
                component: ComponentCreator('/module1/ros2-packages', 'f8d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module2/digital-twin-exercises',
                component: ComponentCreator('/module2/digital-twin-exercises', '98c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module2/gazebo-simulation',
                component: ComponentCreator('/module2/gazebo-simulation', '149'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module2/introduction',
                component: ComponentCreator('/module2/introduction', 'd96'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module2/unity-integration',
                component: ComponentCreator('/module2/unity-integration', 'b8c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module3/ai-models',
                component: ComponentCreator('/module3/ai-models', '192'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module3/introduction',
                component: ComponentCreator('/module3/introduction', '639'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module3/isaac-architecture',
                component: ComponentCreator('/module3/isaac-architecture', 'd20'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module3/robot-control',
                component: ComponentCreator('/module3/robot-control', 'ea2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module4/conversational-robotics',
                component: ComponentCreator('/module4/conversational-robotics', '113'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module4/introduction',
                component: ComponentCreator('/module4/introduction', 'bd5'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module4/vla-exercises',
                component: ComponentCreator('/module4/vla-exercises', '434'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module4/vla-models',
                component: ComponentCreator('/module4/vla-models', '938'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/weekly-breakdown/week-1-2',
                component: ComponentCreator('/weekly-breakdown/week-1-2', '9a6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/weekly-breakdown/week-11-12',
                component: ComponentCreator('/weekly-breakdown/week-11-12', 'a06'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/weekly-breakdown/week-3-4',
                component: ComponentCreator('/weekly-breakdown/week-3-4', 'd51'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/weekly-breakdown/week-5-6',
                component: ComponentCreator('/weekly-breakdown/week-5-6', '531'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/weekly-breakdown/week-7-8',
                component: ComponentCreator('/weekly-breakdown/week-7-8', '8c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/weekly-breakdown/week-9-10',
                component: ComponentCreator('/weekly-breakdown/week-9-10', '161'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/why-physical-ai-matters',
                component: ComponentCreator('/why-physical-ai-matters', '8fa'),
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
