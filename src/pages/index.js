import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started with Textbook 📚
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Comprehensive textbook on Physical AI, ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action Systems">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h2>ROS 2 Foundations</h2>
                <p>Master the Robot Operating System 2 with comprehensive modules covering nodes, services, topics, and messages.</p>
              </div>
              <div className="col col--4">
                <h2>Simulation & Gazebo</h2>
                <p>Learn robotics simulation with Gazebo and Unity environments for safe and efficient development.</p>
              </div>
              <div className="col col--4">
                <h2>NVIDIA Isaac Platform</h2>
                <p>Accelerate your robotics development with GPU-accelerated perception and navigation systems.</p>
              </div>
            </div>
            <div className="row" style={{ marginTop: '2rem' }}>
              <div className="col col--4">
                <h2>Vision-Language-Action Systems</h2>
                <p>Integrate vision, language, and action for advanced embodied AI applications.</p>
              </div>
              <div className="col col--4">
                <h2>Humanoid Robotics</h2>
                <p>Explore humanoid robot control, locomotion, and manipulation systems.</p>
              </div>
              <div className="col col--4">
                <h2>Capstone Project</h2>
                <p>Apply all concepts in a comprehensive Autonomous Humanoid Robot project.</p>
              </div>
            </div>
          </div>
        </section>

        <section style={{ padding: '4rem 0', backgroundColor: '#f6f6f6' }}>
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2 style={{ textAlign: 'center', marginBottom: '2rem' }}>About This Textbook</h2>
                <p style={{ textAlign: 'center' }}>
                  This comprehensive textbook guides students, instructors, and lab engineers through a 14-week curriculum
                  covering ROS 2, Gazebo/Unity simulation, NVIDIA Isaac Platform, Vision-Language-Action systems,
                  and culminating in a capstone Autonomous Humanoid Robot project.
                </p>
                <div style={{ textAlign: 'center', marginTop: '2rem' }}>
                  <Link
                    className="button button--primary button--lg"
                    to="/docs/intro">
                    Start Learning
                  </Link>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}