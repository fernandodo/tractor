# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This directory contains Vim learning materials and documentation, specifically focused on Vim editor commands and usage patterns. It's part of an Obsidian knowledge vault documenting various programming tools and techniques.

## File Structure

- **Vim 从入门到精通.md** - Comprehensive Vim tutorial in Chinese (104KB) covering everything from basics to advanced usage, sourced from vim-galore
- **vim_motion_commands.md** - Systematic categorization of all Vim motion commands (Basic, Word, f/t, Jump, Scrolling, Search, Paragraph/Sentence/Structure, Marks, Screen), structured as a quick reference guide with Chinese explanations
- **vscode_vim_stage12_cheatsheet_with_copy_paste.md** - Practical cheat sheet with checkbox-based learning progression and real-world code examples, includes sections on mode switching, movement, basic editing, copy/paste (yank/put), operators+motions, search, and text objects
- **Vim 修改命令 c 系列完整对照表.md** - Complete reference table for Vim's `c` (change) command series organized into: basic modifications (cc, cw, c$), text objects (ciw, ci", ci(, ci{), special character find (ct_, cf), paragraph/range (cip, cG), and numeric combinations
- **c-command.ods** - LibreOffice spreadsheet containing structured data for change commands

## Content Architecture

### Learning Progression Structure
The materials are organized in a pedagogical hierarchy:

1. **Comprehensive Reference** (Vim 从入门到精通.md) - Full tutorial covering all concepts
2. **Motion Commands** (vim_motion_commands.md) - Categorized by motion type (10 categories: Basic, Word, f/t, Jump, Line number, Scrolling, Search, Large-range, Marks, Screen)
3. **Change Commands** (Vim 修改命令 c 系列完整对照表.md) - Structured tables with 5 categories and real code examples
4. **Progressive Cheatsheet** (vscode_vim_stage12_cheatsheet_with_copy_paste.md) - Checkbox-based learning tracker with practice plan

### Key Command Categories
- **Motions**: `hjkl`, `w/e/b`, `0/^/$`, `gg/G`, `H/M/L`, `f/t/F/T`, `{/}`, `(/)`
- **Operators**: `d` (delete/cut), `y` (yank/copy), `c` (change), `p/P` (put/paste)
- **Text Objects**: `iw/aw` (word), `i"/a"` (quotes), `i(/a(`, `i{/a{`, `i[/a[` (brackets), `it/at` (tags), `ip/ap` (paragraph)
- **Jump/Navigation**: `%` (matching bracket), `gd/gD` (definition), `Ctrl-o/Ctrl-i` (jump history), `` ` ` `` (previous position)
- **Search**: `/` (forward), `?` (backward), `n/N` (next/prev), `*/#` (word under cursor)
- **VSCode Integration**: `gd` maps to LSP, `Ctrl-Shift-O` for symbol navigation

## Working with These Files

When editing or enhancing these materials:
- Maintain the pedagogical structure with examples
- Keep command explanations concise but practical
- Include real code examples for context (preferably C/C++ as seen in existing examples)
- Preserve the bilingual nature (Chinese main guide, English cheat sheets)
- Focus on practical, commonly-used commands over obscure features
- Maintain checkbox format `- [ ]` and `- [x]` for tracking learning progress in cheat sheets
- Use consistent table formatting with markdown tables (command, range, explanation, example)
- Include both command syntax and practical usage examples with before/after states
- When adding new commands, categorize them appropriately (motions vs operators vs text objects)
- Preserve the learning progression from basic to advanced
- Keep VSCode Vim integration notes separate from pure Vim commands

## Special Formatting Conventions

- **Command notation**: Use backticks for commands: `` `ciw` ``, `` `gd` ``
- **Examples**: Use format `original code` → `result after command` with arrow (→)
- **Checkbox status**: `- [x]` for mastered, `- [ ]` for learning
- **Table structure**: | Command | Range | Explanation | Example | with Chinese headers
- **Code blocks**: Use language-specific code blocks for examples (```c, ```python)
- **Emphasis**: Use bold (**) for key terms like **复制 = yank**, not for decoration

## Git Context

This is part of a larger Obsidian vault with automated backups. The directory structure suggests this is educational content rather than executable code. When making changes, respect the learning state tracked by checkboxes in the cheatsheet file.