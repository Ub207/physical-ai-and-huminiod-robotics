// @ts-check
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'Embodied Intelligence in the Physical World',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-site.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-org', // Usually your GitHub org/user name.
  projectName: 'physical-ai', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Remove this to remove the "edit this page" links.
          editUrl: 'https://github.com/your-org/physical-ai/edit/main/',
          // Only enable docs, no blog or pages in preset
        },
        blog: false, // Blog is disabled
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI and Humanoid Robotics',
        logo: {
          alt: 'Physical AI and Humanoid Robotics Logo',
          src: 'img/logo.webp',
          srcDark: 'img/logo.webp', // Use same image for dark mode
          href: '/docs/introduction', // Link to the new introduction page
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/your-org/physical-ai',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },

      footer: {
        style: 'dark',
        links: [
          {
            title: 'Textbook',
            items: [
              {
                label: 'Introduction',
                to: '/docs/introduction',
              },
              {
                label: 'Module 1: ROS 2',
                to: '/docs/module1/introduction',
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/docs/module2/introduction',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/physical-ai',
              },
              {
                label: 'Robotics Stack Exchange',
                href: 'https://robotics.stackexchange.com/',
              },
              {
                label: 'ROS Documentation',
                href: 'https://docs.ros.org/',
              },
            ],
          },
          {
            title: 'Related',
            items: [
              {
                label: 'NVIDIA Isaac',
                href: 'https://developer.nvidia.com/isaac-ros',
              },
              {
                label: 'Gazebo Sim',
                href: 'https://gazebosim.org/',
              },
              {
                label: 'Unity Robotics',
                href: 'https://unity.com/solutions/industrial-automation',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI and Humanoid Robotics Project. Built with Docusaurus.`,
      },

      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;