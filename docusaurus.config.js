// docusaurus.config.js
// FINAL corrected version for /docs/ baseUrl on Vercel

const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'Your comprehensive AI & robotics textbook',
  url: 'https://vercel.com/new/import?framework=docusaurus-2&hasTrialAvailable=1&id=1120776753&name=physical-ai-and-huminiod-robotics&owner=Ub207&project-name=physical-ai-and-huminiod-robotics&provider=github&remainingProjects=1&s=https%3A%2F%2Fgithub.com%2FUb207%2Fphysical-ai-and-huminiod-robotics&teamSlug=ubaid-ur-rahmans-projects-6b672f56&totalProjects=1&deploymentIds=dpl_7Qu7DpxxTPbKnBh4M4KBpivHBgVB',
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
          items: [{ label: 'Module 4 Intro', to: '/module-4/intro' }],
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