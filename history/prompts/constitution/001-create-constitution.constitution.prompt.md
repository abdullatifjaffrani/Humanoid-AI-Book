---
id: 1
title: Create Constitution
stage: constitution
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, textbook, ai, robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 - null
---

## Prompt

— Physical AI & Humanoid Robotics Textbook
Purpose

Create a high-quality, academically rigorous textbook and Docusaurus-powered site for teaching Physical AI & Humanoid Robotics, aligned with your 14-week course and SpecKitPlus workflow.

Core Principles

Accuracy — All technical claims must be verified with credible sources.

Clarity — Explanations must be accessible to instructors, students, and practitioners.

Modularity — Content should be structured so chapters, labs, and assets can stand alone.

Practicality — Emphasize hands-on labs, clear examples, and real humanoid robot systems.

Spec-Driven — All work must follow Constitution → Specify → Plan → Tasks → Published Output.

Standards

Citation Style: APA 7

Evidence: Every chapter requires peer-reviewed or authoritative technical sources.

Diagrams: Must be accurate, labelled, and reproducible in Docusaurus.

Writing Style: Concise, structured, and accessible; avoid unnecessary jargon.

Quality Checks: Each chapter must pass clarity, accuracy, and structural validation.

Constraints

Build using Docusaurus + GitHub Pages.

Organize content according to course structure (14 weeks).

Ensure compatibility for both local robot labs and cloud-native labs.

Only include hardware/software workflows that are reproducible.

Exclusions

Vendor comparisons

Low-level implementation code

Ethical deep-dives (can be referenced, not detailed)

Full robotics literature surveys

Success Criteria

Textbook is complete, accurate, visually clear, and fully navigable.

All chapters meet APA + validation requirements.

Book mapped 1:1 to course learning outcomes.

Deployable with one command on GitHub Pages.

Usable by any university or robotics lab with required hardware.

## Response snapshot

Created a comprehensive constitution for the Physical AI & Humanoid Robotics Textbook project based on user requirements. The constitution includes:

- Core principles: Accuracy (NON-NEGOTIABLE), Clarity, Modularity, Practicality, Spec-Driven Development, and Academic Rigor
- Standards section with APA 7 citation requirements, evidence standards, diagram standards, writing style guidelines, and quality checks
- Constraints section outlining technology stack, course structure, lab compatibility, and reproducibility requirements
- Exclusions section defining what's out of scope
- Success criteria section with measurable outcomes
- Governance section with amendment procedures

Updated dependent templates to align with the new constitution:
- plan-template.md: Added constitution check with academic compliance requirements
- spec-template.md: Added academic alignment section
- tasks-template.md: Added academic compliance requirements

## Outcome

- ✅ Impact: Complete constitution created with 6 core principles and aligned templates
- 🧪 Tests: No tests required for constitution creation
- 📁 Files: 4 files updated (.specify/memory/constitution.md, .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md)
- 🔁 Next prompts: Can proceed with feature specification using /sp.specify
- 🧠 Reflection: Constitution provides clear academic and technical guidelines for the textbook project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification for first textbook chapter