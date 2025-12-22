// docusaurus.config.js
// FINAL corrected version for /docs/ baseUrl

const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'Your comprehensive AI & robotics textbook',
  url: 'https://physical-ai-and-huminiod-robotics-utke-ne3dzmm3y.vercel.app',
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
          routeBasePath: '/', // serve docs at baseUrl
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
    }),
  },
};

module.exports = config;