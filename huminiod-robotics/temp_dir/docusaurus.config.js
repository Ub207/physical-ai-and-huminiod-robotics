/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'Advanced Textbook on Physical Intelligence and Humanoid Systems',
  favicon: 'img/favicon.ico',


  baseUrl: '/',

  organizationName: 'Ub207',
  projectName: 'physical-ai-and-huminiod-robotics',
  deploymentBranch: 'gh-pages',

  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Modules',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook`,
    },
    prism: {
      theme: 'github',
      darkTheme: 'dracula',
    },
  },

  themes: ['@docusaurus/theme-mermaid'],

  markdown: {
    mermaid: true,
  },
};

module.exports = config;