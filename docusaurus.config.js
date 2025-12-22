// docusaurus.config.js
// FINAL corrected version for /docs/ baseUrl on Vercel

const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'Your comprehensive AI & robotics textbook',
  url: 'https://physical-ai-and-huminiod-robotics-gqmo-cg2vqq8hz.vercel.app',
  baseUrl: '/docs/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  trailingSlash: false,

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/', // serve docs at baseUrl /docs/
          editUrl: 'https://github.com/Ub207/physical-ai-and-huminiod-robotics/edit/main/',
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig: ({
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        { type: 'doc', docId: 'intro', position: 'left', label: 'Docs' },
        { href: 'https://github.com/Ub207/physical-ai-and-huminiod-robotics', label: 'GitHub', position: 'right' },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [{ label: 'Module 4 Intro', to: '/docs/module-4/intro' }],
        },
        {
          title: 'Community',
          items: [
            { label: 'GitHub', href: 'https://github.com/Ub207/physical-ai-and-huminiod-robotics' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Ubaid ur Rahman.`,
    },
  }),
};

module.exports = config;