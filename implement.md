# Implementation Guide: Physical AI & Humanoid Robotics Textbook

## Docusaurus Configuration

### docusaurus.config.js
```javascript
// Already created in previous step
```

### Custom Components
Create reusable React components for robotics-specific elements:

#### src/components/RobotSimulation.js
```jsx
import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const RobotSimulation = ({ simulationType, config }) => {
  return (
    <BrowserOnly>
      {() => {
        // Dynamic simulation loading based on type
        // ROS 2, Gazebo, Unity integration
        return (
          <div className="simulation-container">
            <h3>{simulationType} Simulation</h3>
            <div className="simulation-embed">
              {/* Simulation iframe or WebGL component */}
            </div>
          </div>
        );
      }}
    </BrowserOnly>
  );
};

export default RobotSimulation;
```

#### src/components/AIModelDemo.js
```jsx
import React, { useState } from 'react';

const AIModelDemo = ({ modelType }) => {
  const [input, setInput] = useState('');
  const [output, setOutput] = useState('');

  const runModel = async () => {
    // API call to run AI model
    // Handle different model types (VLA, Isaac, etc.)
  };

  return (
    <div className="ai-model-demo">
      <h3>{modelType} Model Demo</h3>
      <textarea
        value={input}
        onChange={(e) => setInput(e.target.value)}
        placeholder="Enter input for the model..."
      />
      <button onClick={runModel}>Run Model</button>
      <div className="model-output">
        {output}
      </div>
    </div>
  );
};

export default AIModelDemo;
```

## Content Templates

### Chapter Template: docs/chapter-template.md
```markdown
---
sidebar_position: [NUMBER]
title: [TITLE]
description: [SHORT DESCRIPTION]
keywords: [comma, separated, keywords]
---

# [TITLE]

## Introduction

[Chapter introduction content]

## [Section 1]

[Content for first section]

### Code Example

```python
# Example code for this section
def example_function():
    pass
```

## [Section 2]

[Content for second section]

## Exercises

1. [Exercise 1 description]
2. [Exercise 2 description]

## Summary

[Chapter summary]
```

### ROS 2 Integration Template
```python
# ros2_example.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist

class PhysicalAINode(Node):
    def __init__(self):
        super().__init__('physical_ai_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        # Process image for AI model
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## GitHub Actions Workflow

### .github/workflows/deploy.yml
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: npm

      - name: Install dependencies
        run: npm ci
      - name: Build website
        run: npm run build

      # Popular action to deploy to GitHub Pages:
      # Docs: https://github.com/peaceiris/actions-gh-pages#%EF%B8%8F-docusaurus
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          # Build output to publish to the `gh-pages` branch:
          publish_dir: ./build
          # The following lines assign commit authorship to the official
          # GH-Actions bot for deploys to `gh-pages` branch:
          # https://github.com/actions/checkout/issues/13#issuecomment-724415212
          # The GH actions bot is used by default if you didn't specify the two fields.
          # You can swap them with your own user credentials.
          user_name: github-actions[bot]
          user_email: 41898282+github-actions[bot]@users.noreply.github.com
```

## Docker Configuration

### Dockerfile
```dockerfile
FROM node:18

WORKDIR /app

COPY package*.json ./
RUN npm ci

COPY . .

EXPOSE 3000

CMD ["npm", "start"]
```

### docker-compose.yml
```yaml
version: '3.8'

services:
  docusaurus:
    build: .
    ports:
      - 3000:3000
    volumes:
      - .:/app
    stdin_open: true
    tty: true
```

## Spec-Kit Plus Integration

### .specify/config.json
```json
{
  "project": "Physical AI & Humanoid Robotics",
  "version": "1.0.0",
  "specification": {
    "content": {
      "chapters": 6,
      "sections_per_chapter": 5,
      "exercises_per_chapter": 10
    },
    "technical": {
      "frameworks": ["docusaurus", "ros2", "gazebo", "unity", "nvidia-isaac"],
      "deployment": "github-pages"
    },
    "quality": {
      "completeness": "high",
      "accuracy": "verified",
      "accessibility": "wcag-aa"
    }
  }
}
```

## Testing Framework

### tests/content-validation.js
```javascript
// Content validation tests
const fs = require('fs');
const path = require('path');

function validateChapter(chapterPath) {
  const content = fs.readFileSync(chapterPath, 'utf8');
  const requiredSections = ['Introduction', 'Summary'];

  // Check for required sections
  for (const section of requiredSections) {
    if (!content.includes(section)) {
      console.warn(`Missing ${section} in ${chapterPath}`);
    }
  }

  return true;
}

module.exports = { validateChapter };
```

## Build Scripts

### package.json additional scripts
```json
{
  "scripts": {
    "start": "docusaurus start",
    "build": "docusaurus build",
    "serve": "docusaurus serve",
    "swizzle": "docusaurus swizzle",
    "deploy": "docusaurus deploy",
    "clear": "docusaurus clear",
    "test": "npm run test:content && npm run test:links",
    "test:content": "node tests/content-validation.js",
    "test:links": "find . -name '*.md' -exec grep -l 'http' {} \\; | xargs curl -I"
  }
}
```