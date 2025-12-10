// @ts-check
// Docusaurus config for Physical AI & Humanoid Robotics Textbook

const { themes } = require("prism-react-renderer");
const lightCodeTheme = themes.github;
const darkCodeTheme = themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Physical AI & Humanoid Robotics Textbook",
  tagline:
    "A comprehensive guide to Physical AI, ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action Systems",
  favicon: "img/favicon.ico",

  // Deployment settings
  url: "https://humanide-ai.github.io",
  baseUrl: "/Humanide-AI-Book/",

  organizationName: "humanide-ai",
  projectName: "Humanide-AI-Book",

  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: "./sidebars.js",
          editUrl:
            "https://github.com/humanide-ai/Humanide-AI-Book/tree/main/",
        },
        blog: false, // No blog
        theme: {
          customCss: "./src/css/custom.css",
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: "img/docusaurus-social-card.jpg",

      navbar: {
        title: "Physical AI & Humanoid Robotics",
        logo: {
          alt: "Physical AI & Humanoid Robotics Logo",
          src: "img/logo.svg",
        },
        items: [
          {
            type: "doc",
            docId: "intro",
            position: "left",
            label: "Textbook",
          },
          {
            href: "https://github.com/humanide-ai/Humanide-AI-Book",
            label: "GitHub",
            position: "right",
          },
        ],
      },

      footer: {
        style: "dark",
        links: [
          {
            title: "Textbook",
            items: [
              {
                label: "Introduction",
                to: "/docs/intro",
              },
            ],
          },
          {
            title: "Community",
            items: [
              {
                label: "Stack Overflow",
                href: "https://stackoverflow.com/questions/tagged/docusaurus",
              },
              {
                label: "Discord",
                href: "https://discordapp.com/invite/docusaurus",
              },
            ],
          },
          {
            title: "More",
            items: [
              {
                label: "GitHub",
                href: "https://github.com/humanide-ai/Humanide-AI-Book",
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },

      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
      },
    }),
};

module.exports = config;
