# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Overview
This guide provides a quick setup process to start working with the Physical AI & Humanoid Robotics Textbook project. Follow these steps to get your development environment ready and begin contributing to the textbook.

## Prerequisites

### System Requirements
- Node.js (version 18.x or higher)
- npm or yarn package manager
- Git version control system
- Modern web browser for preview

### Optional (for simulation content)
- ROS 2 (Humble Hawksbill or later)
- Gazebo Garden for robotics simulation
- Python 3.8+ for ROS 2 packages

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/[organization]/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Install Dependencies
```bash
npm install
```

### 3. Verify Installation
```bash
npm run build
```
This should build the textbook without errors.

## Local Development

### 1. Start Development Server
```bash
npm start
```
This will start a local server at `http://localhost:3000` with live reloading.

### 2. Create a New Chapter
1. Navigate to the appropriate module directory in `docs/modules/`
2. Create a new markdown file following the naming convention: `week-{number}-{topic}.md`
3. Include the proper frontmatter:

```markdown
---
title: Chapter Title
week: 1
module: module-1-ros-foundations
learningObjectives:
  - Objective 1
  - Objective 2
  - Objective 3
prerequisites:
  - Prerequisite knowledge
description: Brief description of the chapter
---

# Chapter Title

## Learning Objectives
- Objective 1
- Objective 2
- Objective 3

## Content
[Your chapter content here]

## Key Takeaways
- Key point 1
- Key point 2
- Key point 3

## References
[APA-formatted references]
```

### 3. Add a Lab Exercise
Create a lab file in the same module directory with the naming convention: `lab-{number}-{topic}.md`

## Content Guidelines

### Chapter Structure
Each chapter should follow this structure:
1. **Concept Overview** - Introduction to the topic
2. **Technical Breakdown** - Detailed technical explanation
3. **Hands-On Lab** - Practical exercise
4. **Diagrams/Code** - Visual aids and examples
5. **Key Takeaways** - Summary of main points
6. **References** - APA-formatted citations

### Writing Standards
- Use clear, accessible language
- Define technical terms when first used
- Include at least 12 authoritative sources for major sections
- Follow APA 7 citation format
- Provide practical examples and applications

### Technical Requirements
- All content must be in Markdown format
- Images should include alt text for accessibility
- Code examples should specify the language
- Links should be verified and functional

## Building and Deployment

### 1. Local Build
```bash
npm run build
```

### 2. Serve Built Version Locally
```bash
npm run serve
```

### 3. Deploy to GitHub Pages
The project is configured for GitHub Pages deployment. Push changes to the main branch to trigger deployment.

## Contributing

### 1. Branch Naming Convention
Use the format: `[issue-number]-[brief-description]`
Example: `42-add-ros-navigation-content`

### 2. Commit Messages
Follow conventional commits format:
```
feat: Add new chapter on ROS navigation
fix: Correct citation format in week 3
docs: Update installation instructions
```

### 3. Pull Request Process
1. Ensure all content follows the textbook standards
2. Verify the build passes without errors
3. Include appropriate tests/validations
4. Link to any related issues

## Testing Your Changes

### 1. Validate Content Structure
```bash
npm run validate
```

### 2. Check Links
```bash
npm run link-check
```

### 3. Verify Build
```bash
npm run build
```

## Common Tasks

### Adding References
When adding references, ensure they follow APA 7 format:
```
Author, A. A. (Year). Title of work. Publisher. URL (if applicable)
```

### Including Code Examples
Use fenced code blocks with language specification:
```python
# Example Python code
def example_function():
    return "Hello, ROS 2!"
```

### Adding Diagrams
Place diagrams in the `static/img/diagrams/` directory and reference them:
```markdown
![Diagram Description](/img/diagrams/example.svg)
```

## Troubleshooting

### Build Errors
If the build fails:
1. Check for syntax errors in your Markdown files
2. Verify all image paths are correct
3. Ensure all required frontmatter fields are present

### Local Server Issues
If the local server doesn't start:
1. Clear npm cache: `npm cache clean --force`
2. Delete node_modules: `rm -rf node_modules`
3. Reinstall dependencies: `npm install`

## Getting Help

- Check the [Documentation](../README.md) for detailed information
- Review existing chapters for examples of proper formatting
- Contact the maintainers via GitHub Issues for technical questions