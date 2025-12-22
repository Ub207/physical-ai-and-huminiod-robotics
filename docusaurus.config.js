/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'Advanced Textbook on Physical Intelligence and Humanoid Systems',
  favicon: 'img/favicon.ico',

  // Set the base URL to match Vercel subpath deployment
  url: 'https://physical-ai-and-huminiod-robotics-utke-ne3dzmm3y.vercel.app',
  baseUrl: '/docs/',

  // GitHub pages deployment config
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
      /** @type {import('@docusaurus/preset-classic').Options} */
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

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
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
        theme: require('prism-react-renderer/themes/github'),
        darkTheme: require('prism-react-renderer/themes/dracula'),
      },
    }),

  themes: ['@docusaurus/theme-mermaid'],

  markdown: {
    mermaid: true,
  },
};

module.exports = config;