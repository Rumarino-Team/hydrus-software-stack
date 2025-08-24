#!/usr/bin/env python3
"""
TODO and FIXME tracker for the Hydrus Software Stack.

This module provides functionality to scan the repository for TODO, FIXME,
HACK, NOTE, and XXX comments, generate reports, and optionally link them
to GitHub issues.
"""

import json
import os
import re
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import typer
from rich.console import Console
from rich.table import Table

# Create the todo subcommand app
todo_app = typer.Typer()

console = Console()

# Default file extensions to search
DEFAULT_EXTENSIONS = [
    '.py', '.cpp', '.c', '.h', '.hpp', '.js', '.ts', '.md', '.yaml', '.yml',
    '.launch', '.xml', '.sh', '.bash', '.json', '.txt', '.rst'
]

# Default comment patterns to search for
DEFAULT_PATTERNS = ['TODO', 'FIXME', 'HACK', 'NOTE', 'XXX', 'BUG']


class TodoItem:
    """Represents a TODO/FIXME item found in the codebase."""
    
    def __init__(self, file_path: str, line_number: int, content: str, 
                 pattern: str, context: str = "", issue_number: Optional[int] = None):
        self.file_path = file_path
        self.line_number = line_number
        self.content = content.strip()
        self.pattern = pattern
        self.context = context.strip()
        self.issue_number = issue_number
        
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            'file_path': self.file_path,
            'line_number': self.line_number,
            'content': self.content,
            'pattern': self.pattern,
            'context': self.context,
            'issue_number': self.issue_number
        }


class TodoTracker:
    """Main class for scanning and tracking TODO items."""
    
    def __init__(self, root_path: str = ".", 
                 extensions: List[str] = None,
                 patterns: List[str] = None):
        self.root_path = Path(root_path).resolve()
        self.extensions = extensions or DEFAULT_EXTENSIONS
        self.patterns = patterns or DEFAULT_PATTERNS
        self.exclude_dirs = {'.git', '__pycache__', '.venv', 'venv', 'node_modules', 
                           'build', 'dist', '.pytest_cache', 'hydrus_software_stack.egg-info'}
        
    def _should_exclude_path(self, path: Path) -> bool:
        """Check if a path should be excluded from scanning."""
        for part in path.parts:
            if part in self.exclude_dirs or part.startswith('.'):
                return True
        
        # Exclude the todo.py file itself to avoid self-references
        if path.name == 'todo.py':
            return True
            
        return False
    
    def _extract_issue_number(self, content: str) -> Optional[int]:
        """Extract GitHub issue number from comment if present."""
        # Look for patterns like #123, issue #123, fixes #123, etc.
        issue_patterns = [
            r'#(\d+)',
            r'issue\s*#?(\d+)',
            r'fixes?\s*#?(\d+)', 
            r'closes?\s*#?(\d+)',
            r'resolves?\s*#?(\d+)'
        ]
        
        for pattern in issue_patterns:
            match = re.search(pattern, content, re.IGNORECASE)
            if match:
                return int(match.group(1))
        return None
    
    def _scan_file(self, file_path: Path) -> List[TodoItem]:
        """Scan a single file for TODO patterns."""
        todos = []
        
        try:
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
                
            for line_num, line in enumerate(lines, 1):
                line_clean = line.strip()
                if not line_clean:
                    continue
                    
                # Check for any of our patterns
                for pattern in self.patterns:
                    # Create case-insensitive pattern matching for comment-style TODO/FIXME
                    # Look for the pattern preceded by comment markers and optionally followed by a colon
                    regex_pattern = rf'(?i)(?://|#|\*|<!--)\s*{pattern}(?:\s*:?\s*(.*))?'
                    match = re.search(regex_pattern, line_clean)
                    
                    if match:
                        content = match.group(1) if match.group(1) else line_clean
                        issue_number = self._extract_issue_number(content)
                        
                        # Get context (previous and next line if available)
                        context_lines = []
                        if line_num > 1:
                            context_lines.append(f"{line_num-1}: {lines[line_num-2].strip()}")
                        context_lines.append(f"{line_num}: {line.strip()}")
                        if line_num < len(lines):
                            context_lines.append(f"{line_num+1}: {lines[line_num].strip()}")
                        
                        context = "\n".join(context_lines)
                        
                        todo_item = TodoItem(
                            file_path=str(file_path.relative_to(self.root_path)),
                            line_number=line_num,
                            content=content,
                            pattern=pattern.upper(),
                            context=context,
                            issue_number=issue_number
                        )
                        todos.append(todo_item)
                        break  # Only match one pattern per line
                        
        except Exception as e:
            console.print(f"[yellow]Warning: Could not read {file_path}: {e}[/yellow]")
            
        return todos
    
    def scan_repository(self) -> List[TodoItem]:
        """Scan the entire repository for TODO items."""
        todos = []
        
        for root, dirs, files in os.walk(self.root_path):
            root_path = Path(root)
            
            # Filter out excluded directories
            dirs[:] = [d for d in dirs if not self._should_exclude_path(root_path / d)]
            
            for file in files:
                file_path = root_path / file
                
                # Check if file has a supported extension
                if file_path.suffix.lower() in self.extensions:
                    if not self._should_exclude_path(file_path):
                        todos.extend(self._scan_file(file_path))
                        
        return todos
    
    def generate_report(self, todos: List[TodoItem]) -> Dict:
        """Generate a comprehensive report from TODO items."""
        report = {
            'summary': {
                'total_todos': len(todos),
                'by_pattern': defaultdict(int),
                'by_directory': defaultdict(int),
                'by_file_type': defaultdict(int),
                'with_issues': 0
            },
            'todos': [todo.to_dict() for todo in todos]
        }
        
        for todo in todos:
            # Count by pattern
            report['summary']['by_pattern'][todo.pattern] += 1
            
            # Count by directory
            dir_path = os.path.dirname(todo.file_path) or '.'
            report['summary']['by_directory'][dir_path] += 1
            
            # Count by file type
            ext = Path(todo.file_path).suffix or 'no_extension'
            report['summary']['by_file_type'][ext] += 1
            
            # Count items with issue references
            if todo.issue_number:
                report['summary']['with_issues'] += 1
                
        # Convert defaultdicts to regular dicts for JSON serialization
        report['summary']['by_pattern'] = dict(report['summary']['by_pattern'])
        report['summary']['by_directory'] = dict(report['summary']['by_directory'])
        report['summary']['by_file_type'] = dict(report['summary']['by_file_type'])
        
        return report


def _print_summary_table(report: Dict):
    """Print a summary table to the console."""
    summary = report['summary']
    
    # Overall summary
    console.print(f"\n[bold cyan]📋 TODO Tracker Summary[/bold cyan]")
    console.print(f"Total items found: [bold]{summary['total_todos']}[/bold]")
    console.print(f"Items with GitHub issues: [bold]{summary['with_issues']}[/bold]")
    
    # Pattern breakdown
    if summary['by_pattern']:
        console.print(f"\n[bold yellow]🏷️  By Pattern:[/bold yellow]")
        pattern_table = Table()
        pattern_table.add_column("Pattern", style="cyan")
        pattern_table.add_column("Count", style="magenta")
        
        for pattern, count in sorted(summary['by_pattern'].items(), 
                                   key=lambda x: x[1], reverse=True):
            pattern_table.add_row(pattern, str(count))
        console.print(pattern_table)
    
    # Directory breakdown  
    if summary['by_directory']:
        console.print(f"\n[bold yellow]📁 By Directory:[/bold yellow]")
        dir_table = Table()
        dir_table.add_column("Directory", style="cyan")
        dir_table.add_column("Count", style="magenta")
        
        for directory, count in sorted(summary['by_directory'].items(), 
                                     key=lambda x: x[1], reverse=True):
            dir_table.add_row(directory, str(count))
        console.print(dir_table)


def _print_detailed_todos(todos: List[TodoItem], limit: Optional[int] = None):
    """Print detailed TODO items."""
    if not todos:
        console.print("[green]🎉 No TODO items found![/green]")
        return
        
    console.print(f"\n[bold cyan]📝 TODO Items Detail:[/bold cyan]")
    
    displayed_todos = todos[:limit] if limit else todos
    
    for i, todo in enumerate(displayed_todos, 1):
        console.print(f"\n[bold]{i}. {todo.pattern}[/bold] in [cyan]{todo.file_path}[/cyan]:[magenta]{todo.line_number}[/magenta]")
        console.print(f"   💬 {todo.content}")
        
        if todo.issue_number:
            console.print(f"   🔗 Related to issue #{todo.issue_number}")
            
    if limit and len(todos) > limit:
        console.print(f"\n[dim]... and {len(todos) - limit} more items. Use --limit to see more.[/dim]")


@todo_app.command("scan")
def scan_todos(
    path: str = typer.Option(".", "--path", "-p", help="Path to scan (default: current directory)"),
    patterns: str = typer.Option(
        ",".join(DEFAULT_PATTERNS), 
        "--patterns", 
        help="Comma-separated list of patterns to search for"
    ),
    extensions: str = typer.Option(
        ",".join(DEFAULT_EXTENSIONS),
        "--extensions", "-e",
        help="Comma-separated list of file extensions to scan"
    ),
    output_format: str = typer.Option(
        "console", 
        "--format", "-f",
        help="Output format: console, json, markdown"
    ),
    output_file: Optional[str] = typer.Option(
        None,
        "--output", "-o", 
        help="Output file path (optional)"
    ),
    limit: Optional[int] = typer.Option(
        None,
        "--limit", "-l",
        help="Limit number of detailed items to show"
    ),
    no_summary: bool = typer.Option(
        False,
        "--no-summary",
        help="Skip summary output"
    )
):
    """Scan repository for TODO, FIXME, and other patterns."""
    
    pattern_list = [p.strip().upper() for p in patterns.split(",") if p.strip()]
    extension_list = [e.strip() if e.startswith('.') else f'.{e.strip()}' 
                     for e in extensions.split(",") if e.strip()]
    
    console.print(f"[bold green]🔍 Scanning {path} for patterns: {', '.join(pattern_list)}[/bold green]")
    
    tracker = TodoTracker(path, extension_list, pattern_list)
    todos = tracker.scan_repository()
    report = tracker.generate_report(todos)
    
    # Output based on format
    if output_format.lower() == "json":
        output_content = json.dumps(report, indent=2)
        if output_file:
            with open(output_file, 'w') as f:
                f.write(output_content)
            console.print(f"[green]✅ JSON report saved to {output_file}[/green]")
        else:
            console.print(output_content)
            
    elif output_format.lower() == "markdown":
        # Generate markdown output
        md_lines = [
            "# TODO Tracker Report",
            f"\nGenerated on: {__import__('datetime').datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"\n## Summary",
            f"- **Total items:** {report['summary']['total_todos']}",
            f"- **Items with GitHub issues:** {report['summary']['with_issues']}"
        ]
        
        if report['summary']['by_pattern']:
            md_lines.append("\n### By Pattern")
            for pattern, count in sorted(report['summary']['by_pattern'].items(), 
                                       key=lambda x: x[1], reverse=True):
                md_lines.append(f"- **{pattern}:** {count}")
                
        if report['summary']['by_directory']:
            md_lines.append("\n### By Directory")
            for directory, count in sorted(report['summary']['by_directory'].items(), 
                                         key=lambda x: x[1], reverse=True):
                md_lines.append(f"- **{directory}:** {count}")
        
        if todos:
            md_lines.append("\n## Detailed Items")
            for todo in todos:
                md_lines.append(f"\n### {todo.pattern} in `{todo.file_path}:{todo.line_number}`")
                md_lines.append(f"{todo.content}")
                if todo.issue_number:
                    md_lines.append(f"*Related to issue #{todo.issue_number}*")
        
        output_content = "\n".join(md_lines)
        if output_file:
            with open(output_file, 'w') as f:
                f.write(output_content)
            console.print(f"[green]✅ Markdown report saved to {output_file}[/green]")
        else:
            console.print(output_content)
            
    else:  # console format
        if not no_summary:
            _print_summary_table(report)
        _print_detailed_todos(todos, limit)
        
        if output_file:
            # Save console output as text
            with open(output_file, 'w') as f:
                f.write(f"TODO Tracker Report - {__import__('datetime').datetime.now()}\n")
                f.write("=" * 50 + "\n\n")
                f.write(f"Total items: {report['summary']['total_todos']}\n")
                f.write(f"Items with issues: {report['summary']['with_issues']}\n\n")
                
                for todo in todos:
                    f.write(f"{todo.pattern} in {todo.file_path}:{todo.line_number}\n")
                    f.write(f"  {todo.content}\n")
                    if todo.issue_number:
                        f.write(f"  Related to issue #{todo.issue_number}\n")
                    f.write("\n")
            console.print(f"[green]✅ Text report saved to {output_file}[/green]")


@todo_app.command("stats")
def show_stats(
    path: str = typer.Option(".", "--path", "-p", help="Path to analyze"),
    patterns: str = typer.Option(
        ",".join(DEFAULT_PATTERNS), 
        "--patterns", 
        help="Comma-separated list of patterns to search for"
    )
):
    """Show statistics about TODO items in the repository."""
    
    pattern_list = [p.strip().upper() for p in patterns.split(",") if p.strip()]
    
    tracker = TodoTracker(path, patterns=pattern_list)
    todos = tracker.scan_repository()
    report = tracker.generate_report(todos)
    
    _print_summary_table(report)


@todo_app.command("issues")
def show_linked_issues(
    path: str = typer.Option(".", "--path", "-p", help="Path to scan")
):
    """Show TODO items that reference GitHub issues."""
    
    tracker = TodoTracker(path)
    todos = tracker.scan_repository()
    
    linked_todos = [todo for todo in todos if todo.issue_number]
    
    if not linked_todos:
        console.print("[yellow]No TODO items with GitHub issue references found.[/yellow]")
        return
        
    console.print(f"[bold cyan]🔗 TODO Items linked to GitHub Issues ({len(linked_todos)} found):[/bold cyan]")
    
    # Group by issue number
    by_issue = defaultdict(list)
    for todo in linked_todos:
        by_issue[todo.issue_number].append(todo)
    
    for issue_num in sorted(by_issue.keys()):
        console.print(f"\n[bold magenta]Issue #{issue_num}:[/bold magenta]")
        for todo in by_issue[issue_num]:
            console.print(f"  📁 {todo.file_path}:{todo.line_number} ({todo.pattern})")
            console.print(f"     💬 {todo.content}")


if __name__ == "__main__":
    todo_app()