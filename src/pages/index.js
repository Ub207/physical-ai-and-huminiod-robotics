import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';
import RagChatWidget from '@site/src/components/chat/RagChatWidget';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/introduction">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Comprehensive textbook on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>Physical Intelligence</h3>
                  <p>Learn how AI algorithms can be embodied in physical systems and humanoid robots.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>Modern Robotics Stack</h3>
                  <p>Explore ROS 2, NVIDIA Isaac, Gazebo, Unity, and Vision-Language-Action models.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>Hands-On Learning</h3>
                  <p>Complete with practical exercises, capstone projects, and hardware specifications.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
      {/* RAG Chatbot Widget - This will use the default from Root.js in production */}
      <RagChatWidget bookId="physical-ai-textbook" />
    </Layout>
  );
}