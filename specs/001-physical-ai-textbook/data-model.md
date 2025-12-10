# Data Model: Physical AI & Humanoid Robotics Textbook

## Overview
This document defines the data structures and relationships for the Physical AI & Humanoid Robotics Textbook project. Since this is a documentation project, the "data model" represents the content structure and metadata schemas.

## Core Entities

### 1. Chapter
**Description**: A modular unit of content covering specific topics within the 14-week curriculum

**Fields**:
- `id`: Unique identifier (string, required)
- `title`: Chapter title (string, required)
- `module`: Parent module identifier (string, required)
- `week`: Week number in curriculum (integer, required)
- `learningObjectives`: List of learning objectives (array of strings, required)
- `prerequisites`: Prerequisites for this chapter (array of strings, optional)
- `content`: Main content in Markdown format (string, required)
- `examples`: Code examples and diagrams (array of objects, optional)
- `labs`: Associated lab exercises (array of lab objects, required)
- `keyTakeaways`: Key concepts summary (array of strings, required)
- `references`: APA-formatted citations (array of reference objects, required)
- `createdAt`: Creation timestamp (ISO 8601, required)
- `updatedAt`: Last modification timestamp (ISO 8601, required)
- `version`: Content version (string, required)

**Validation Rules**:
- Title must be 5-100 characters
- Week must be 1-14 for standard curriculum
- Must have at least 1 learning objective
- Must have at least 1 reference
- Content must be valid Markdown

### 2. Lab Exercise
**Description**: A practical activity that allows students to apply theoretical concepts

**Fields**:
- `id`: Unique identifier (string, required)
- `title`: Lab title (string, required)
- `chapterId`: Parent chapter identifier (string, required)
- `description`: Lab overview (string, required)
- `objectives`: Lab learning objectives (array of strings, required)
- `prerequisites`: Required setup or knowledge (array of strings, required)
- `steps`: Sequential steps for completion (array of step objects, required)
- `expectedOutcome`: What students should achieve (string, required)
- `troubleshooting`: Common issues and solutions (array of troubleshooting objects, optional)
- `duration`: Estimated completion time in minutes (integer, required)
- `difficulty`: Difficulty level (enum: beginner, intermediate, advanced, required)

**Validation Rules**:
- Title must be 5-100 characters
- Must have 3-15 steps
- Duration must be 30-240 minutes
- Difficulty must be one of the allowed values

### 3. Reference
**Description**: Academic or technical source cited in APA 7 format

**Fields**:
- `id`: Unique identifier (string, required)
- `apaCitation`: Full APA 7 citation (string, required)
- `url`: Source URL if available (string, optional)
- `accessDate`: Date source was accessed (ISO 8601, required for online sources)
- `type`: Source type (enum: academicPaper, technicalDoc, book, website, required)
- `primarySource`: Whether this is a primary/authoritative source (boolean, required)
- `relevance`: Brief explanation of relevance to content (string, required)

**Validation Rules**:
- APA citation must follow APA 7 guidelines
- URL must be valid if provided
- Type must be one of the allowed values

### 4. Module
**Description**: A collection of chapters focused on a specific topic area

**Fields**:
- `id`: Unique identifier (string, required)
- `title`: Module title (string, required)
- `description`: Module overview (string, required)
- `chapters`: Array of chapter IDs in this module (array of strings, required)
- `learningOutcomes`: Module-level learning outcomes (array of strings, required)
- `duration`: Estimated completion time in weeks (integer, required)
- `prerequisites`: Prerequisites for the module (array of strings, optional)

**Validation Rules**:
- Title must be 5-100 characters
- Must have 1-5 chapters
- Duration must be 1-4 weeks for standard modules

### 5. Curriculum
**Description**: The complete 14-week course structure that organizes chapters in a logical learning progression

**Fields**:
- `id`: Unique identifier (string, required)
- `title`: Curriculum title (string, required)
- `modules`: Array of module IDs in sequence (array of strings, required)
- `totalWeeks`: Total duration in weeks (integer, required)
- `learningObjectives`: Overall curriculum objectives (array of strings, required)
- `assessmentMethods`: How learning is evaluated (array of strings, required)
- `prerequisites`: Overall curriculum prerequisites (array of strings, optional)

**Validation Rules**:
- Total weeks must be 14 for standard curriculum
- Must have 4-6 modules
- Must have 3-10 learning objectives

### 6. Project Template
**Description**: Pre-configured code or simulation environments that students can use as starting points

**Fields**:
- `id`: Unique identifier (string, required)
- `name`: Template name (string, required)
- `description`: Template overview (string, required)
- `type`: Template category (enum: code, simulation, configuration, required)
- `technologies`: Technologies used (array of strings, required)
- `files`: List of files included in template (array of file objects, required)
- `instructions`: Setup and usage instructions (string, required)
- `compatibility`: Compatible environments (array of strings, required)
- `license`: Usage license information (string, required)

**Validation Rules**:
- Name must be 3-50 characters
- Type must be one of the allowed values
- Must have at least 1 file

## Relationships

### Chapter Relationships
- Chapter belongs to one Module (many-to-one)
- Chapter has many Lab Exercises (one-to-many)
- Chapter has many References (one-to-many)

### Module Relationships
- Module contains many Chapters (one-to-many)
- Module belongs to one Curriculum (many-to-one)

### Lab Exercise Relationships
- Lab Exercise belongs to one Chapter (many-to-one)

### Reference Relationships
- Reference may be associated with many Chapters (many-to-many)

### Curriculum Relationships
- Curriculum contains many Modules (one-to-many)

## State Transitions

### Chapter States
- `draft`: Initial state, content being created
- `review`: Content under review
- `approved`: Content approved for publication
- `published`: Content published and available
- `deprecated`: Content no longer current

### Lab Exercise States
- `design`: Lab being designed
- `tested`: Lab tested and validated
- `published`: Lab ready for use
- `archived`: Lab no longer used

## Content Validation Rules

### General Content Rules
- All content must be in valid Markdown format
- Images must have appropriate alt text
- All technical claims must be supported by references
- Content must be accessible to the target audience

### Academic Standards
- All references must follow APA 7 format
- Minimum of 12 authoritative sources per major section
- All diagrams must be clear and properly labeled
- Content must be factually accurate and verifiable