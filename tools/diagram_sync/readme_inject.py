#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - README Diagram Injection Tool

Automatically injects and updates diagram strips in README files based on
.readme.diagrams.yaml manifest files.

Usage:
    python tools/diagram_sync/readme_inject.py [--check] [--folder PATH]
    
Options:
    --check     Validate that all READMEs have correct diagram strips (CI mode)
    --folder    Process specific folder only (default: all folders)
    --dry-run   Show what would be changed without making changes
    --verbose   Enable verbose logging
    
Examples:
    # Update all READMEs
    python tools/diagram_sync/readme_inject.py
    
    # Check specific service
    python tools/diagram_sync/readme_inject.py --check --folder services/fleet-manager
    
    # Dry run for testing
    python tools/diagram_sync/readme_inject.py --dry-run --verbose
"""

import os
import sys
import re
import yaml
import argparse
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from datetime import datetime
import subprocess
import json

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class DiagramInjector:
    """Main class for README diagram injection and validation."""
    
    def __init__(self, repo_root: Path, dry_run: bool = False, verbose: bool = False):
        self.repo_root = repo_root
        self.dry_run = dry_run
        self.verbose = verbose
        self.templates_dir = repo_root / "_templates"
        self.diagrams_dir = repo_root / "docs" / "diagrams"
        
        if verbose:
            logger.setLevel(logging.DEBUG)
            
        # Statistics
        self.stats = {
            'processed': 0,
            'updated': 0,
            'errors': 0,
            'missing_diagrams': 0,
            'missing_manifests': 0
        }
    
    def find_manifest_files(self, folder_path: Optional[Path] = None) -> List[Path]:
        """Find all .readme.diagrams.yaml manifest files."""
        search_path = folder_path or self.repo_root
        manifest_files = []
        
        # Search patterns for different folder types
        search_patterns = [
            "services/*/",
            "edge/*/", 
            "ui/*/",
            "data/*/",
            "infrastructure/*/",
            "deployment/*/"
        ]
        
        if folder_path:
            # Search specific folder
            manifest_file = folder_path / ".readme.diagrams.yaml"
            if manifest_file.exists():
                manifest_files.append(manifest_file)
        else:
            # Search all patterns
            for pattern in search_patterns:
                for path in self.repo_root.glob(pattern):
                    if path.is_dir():
                        manifest_file = path / ".readme.diagrams.yaml"
                        if manifest_file.exists():
                            manifest_files.append(manifest_file)
        
        logger.debug(f"Found {len(manifest_files)} manifest files")
        return manifest_files
    
    def load_manifest(self, manifest_path: Path) -> Optional[Dict]:
        """Load and validate a manifest file."""
        try:
            with open(manifest_path, 'r', encoding='utf-8') as f:
                manifest = yaml.safe_load(f)
            
            # Basic validation
            required_fields = ['type', 'strip', 'metadata']
            for field in required_fields:
                if field not in manifest:
                    logger.error(f"Missing required field '{field}' in {manifest_path}")
                    return None
            
            # Validate strip has 3 diagrams
            strip = manifest['strip']
            required_diagrams = ['diagram1', 'diagram2', 'diagram3']
            for diagram in required_diagrams:
                if diagram not in strip:
                    logger.error(f"Missing diagram '{diagram}' in strip for {manifest_path}")
                    return None
            
            logger.debug(f"Loaded manifest: {manifest_path}")
            return manifest
            
        except yaml.YAMLError as e:
            logger.error(f"YAML error in {manifest_path}: {e}")
            return None
        except Exception as e:
            logger.error(f"Error loading {manifest_path}: {e}")
            return None
    
    def validate_diagram_paths(self, manifest: Dict, manifest_path: Path) -> List[str]:
        """Validate that all diagram paths exist."""
        errors = []
        strip = manifest['strip']
        
        for diagram_key in ['diagram1', 'diagram2', 'diagram3']:
            diagram = strip[diagram_key]
            diagram_path = diagram['path']
            
            # Resolve relative path from manifest location
            full_path = (manifest_path.parent / diagram_path).resolve()
            
            if not full_path.exists():
                errors.append(f"Diagram not found: {diagram_path} (resolved to {full_path})")
                self.stats['missing_diagrams'] += 1
        
        return errors
    
    def load_template(self, template_type: str) -> Optional[str]:
        """Load the appropriate README template."""
        template_map = {
            'service': 'README.tpl.md',
            'edge': 'README.edge.tpl.md', 
            'ui': 'README.ui.tpl.md',
            'data': 'README.tpl.md',  # Use base template for now
            'infrastructure': 'README.tpl.md'  # Use base template for now
        }
        
        template_file = template_map.get(template_type, 'README.tpl.md')
        template_path = self.templates_dir / template_file
        
        try:
            with open(template_path, 'r', encoding='utf-8') as f:
                return f.read()
        except Exception as e:
            logger.error(f"Error loading template {template_path}: {e}")
            return None
    
    def generate_diagram_strip(self, manifest: Dict, manifest_path: Path) -> str:
        """Generate the HTML diagram strip."""
        strip = manifest['strip']
        metadata = manifest['metadata']
        
        # Generate the diagram strip HTML
        diagram_strip = '<p align="center">\n'
        
        for i, diagram_key in enumerate(['diagram1', 'diagram2', 'diagram3'], 1):
            diagram = strip[diagram_key]
            path = diagram['path']
            alt_text = diagram['alt_text']
            title = diagram.get('title', alt_text)
            
            diagram_strip += f'  <a href="{path}">\n'
            diagram_strip += f'    <img src="{path}" alt="{alt_text}" title="{title}" width="32%" />\n'
            diagram_strip += f'  </a>\n'
            
            # Add spacing between diagrams (except last one)
            if i < 3:
                diagram_strip += '  '
        
        diagram_strip += '</p>\n\n'
        
        # Add description based on type
        type_descriptions = {
            'service': '**📊 Service Overview:** *Component architecture* • *Primary flows* • *Data model*',
            'edge': '**🤖 Edge Overview:** *ROS2 nodes* • *Lifecycle flow* • *Edge↔Cloud data*',
            'ui': '**🎨 UI Overview:** *User flows* • *API calls* • *Component tree*',
            'data': '**📈 Data Overview:** *Pipeline architecture* • *Data flows* • *Schema model*',
            'infrastructure': '**🏗️ Infrastructure Overview:** *System topology* • *Deployment flow* • *Resource model*'
        }
        
        description = type_descriptions.get(manifest['type'], '**📊 Overview:** *Architecture* • *Flows* • *Data model*')
        diagram_strip += f'{description}\n'
        
        return diagram_strip
    
    def substitute_template_variables(self, template: str, manifest: Dict, manifest_path: Path) -> str:
        """Substitute template variables with manifest data."""
        metadata = manifest['metadata']
        
        # Basic substitutions
        substitutions = {
            'SERVICE_NAME': metadata['name'],
            'SERVICE_DESCRIPTION': metadata['description'],
            'SERVICE_OWNER': metadata['owner'],
            'CONTACT_EMAIL': metadata.get('contact', 'team@atlasmesh.com'),
            'LAST_UPDATED': datetime.now().strftime('%Y-%m-%d %H:%M:%S UTC'),
            'MANIFEST_PATH': str(manifest_path.relative_to(self.repo_root))
        }
        
        # Service-specific substitutions
        if 'service' in metadata:
            service = metadata['service']
            substitutions.update({
                'SERVICE_SLUG': service.get('slug', metadata['name'].lower().replace(' ', '-')),
                'OPENAPI_PATH': service.get('api_path', ''),
                'EVENTS_PATH': service.get('events_path', ''),
                'DATABASE_TYPE': service.get('database_type', 'PostgreSQL'),
                'PRIMARY_ENDPOINT': service.get('primary_endpoint', ''),
                'HEALTH_ENDPOINT': service.get('health_endpoint', '/health'),
                'METRICS_ENDPOINT': service.get('metrics_endpoint', '/metrics')
            })
        
        # Edge-specific substitutions
        if 'edge' in metadata:
            edge = metadata['edge']
            substitutions.update({
                'PACKAGE_NAME': edge.get('package_name', metadata['name']),
                'ROS2_VERSION': edge.get('ros2_version', 'humble'),
                'LAUNCH_FILE': edge.get('launch_file', ''),
                'SAFETY_CLASSIFICATION': edge.get('safety_classification', 'non_safety')
            })
        
        # UI-specific substitutions
        if 'ui' in metadata:
            ui = metadata['ui']
            substitutions.update({
                'UI_COMPONENT_NAME': metadata['name'],
                'UI_FRAMEWORK': ui.get('framework', 'React'),
                'FRAMEWORK_VERSION': ui.get('framework_version', '18.2.0'),
                'DEV_PORT': str(ui.get('dev_port', 3000))
            })
        
        # SLO substitutions
        if 'slos' in metadata:
            slos = metadata['slos']
            substitutions.update({
                'LATENCY_TARGET': slos.get('latency_p95', 'TBD'),
                'AVAILABILITY_TARGET': slos.get('availability', 'TBD'),
                'ERROR_RATE_TARGET': slos.get('error_rate', 'TBD'),
                'THROUGHPUT_TARGET': slos.get('throughput', 'TBD')
            })
        
        # Apply substitutions
        result = template
        for key, value in substitutions.items():
            result = result.replace(f'{{{{{key}}}}}', str(value))
        
        # Remove any remaining template variables (set to TBD)
        result = re.sub(r'\{\{[^}]+\}\}', 'TBD', result)
        
        return result
    
    def inject_diagram_strip(self, readme_content: str, diagram_strip: str) -> str:
        """Inject or update the diagram strip in README content."""
        begin_marker = '<!-- BEGIN:DIAGRAM-STRIP -->'
        end_marker = '<!-- END:DIAGRAM-STRIP -->'
        
        # Check if markers exist
        if begin_marker in readme_content and end_marker in readme_content:
            # Replace existing strip
            pattern = f'{re.escape(begin_marker)}.*?{re.escape(end_marker)}'
            replacement = f'{begin_marker}\n{diagram_strip}{end_marker}'
            return re.sub(pattern, replacement, readme_content, flags=re.DOTALL)
        else:
            # Insert after title (first # line)
            lines = readme_content.split('\n')
            insert_index = 1  # After title
            
            # Find the title line
            for i, line in enumerate(lines):
                if line.startswith('# '):
                    insert_index = i + 1
                    break
            
            # Insert diagram strip
            strip_lines = [
                '',
                begin_marker,
                diagram_strip.rstrip(),
                end_marker,
                ''
            ]
            
            for i, strip_line in enumerate(strip_lines):
                lines.insert(insert_index + i, strip_line)
            
            return '\n'.join(lines)
    
    def process_folder(self, manifest_path: Path) -> bool:
        """Process a single folder with a manifest file."""
        folder_path = manifest_path.parent
        readme_path = folder_path / "README.md"
        
        logger.info(f"Processing {folder_path.relative_to(self.repo_root)}")
        self.stats['processed'] += 1
        
        # Load manifest
        manifest = self.load_manifest(manifest_path)
        if not manifest:
            self.stats['errors'] += 1
            return False
        
        # Validate diagram paths
        diagram_errors = self.validate_diagram_paths(manifest, manifest_path)
        if diagram_errors:
            for error in diagram_errors:
                logger.error(f"  {error}")
            self.stats['errors'] += 1
            return False
        
        # Load template
        template = self.load_template(manifest['type'])
        if not template:
            self.stats['errors'] += 1
            return False
        
        # Generate diagram strip
        diagram_strip = self.generate_diagram_strip(manifest, manifest_path)
        
        # Check if README exists
        if readme_path.exists():
            # Update existing README
            with open(readme_path, 'r', encoding='utf-8') as f:
                readme_content = f.read()
            
            # Inject diagram strip
            updated_content = self.inject_diagram_strip(readme_content, diagram_strip)
            
            # Check if content changed
            if updated_content != readme_content:
                if not self.dry_run:
                    with open(readme_path, 'w', encoding='utf-8') as f:
                        f.write(updated_content)
                logger.info(f"  ✅ Updated diagram strip in {readme_path.name}")
                self.stats['updated'] += 1
            else:
                logger.debug(f"  ⏭️  No changes needed for {readme_path.name}")
        else:
            # Create new README from template
            template_content = self.substitute_template_variables(template, manifest, manifest_path)
            final_content = self.inject_diagram_strip(template_content, diagram_strip)
            
            if not self.dry_run:
                with open(readme_path, 'w', encoding='utf-8') as f:
                    f.write(final_content)
            logger.info(f"  ✨ Created new README: {readme_path.name}")
            self.stats['updated'] += 1
        
        return True
    
    def check_mode(self, folder_path: Optional[Path] = None) -> bool:
        """Check that all READMEs have correct diagram strips (CI mode)."""
        logger.info("🔍 Running in check mode - validating README diagram strips")
        
        manifest_files = self.find_manifest_files(folder_path)
        if not manifest_files:
            logger.warning("No manifest files found")
            return True
        
        all_valid = True
        
        for manifest_path in manifest_files:
            folder_path = manifest_path.parent
            readme_path = folder_path / "README.md"
            
            # Check if README exists
            if not readme_path.exists():
                logger.error(f"❌ Missing README.md in {folder_path.relative_to(self.repo_root)}")
                all_valid = False
                continue
            
            # Check if README has diagram strip markers
            with open(readme_path, 'r', encoding='utf-8') as f:
                readme_content = f.read()
            
            if '<!-- BEGIN:DIAGRAM-STRIP -->' not in readme_content:
                logger.error(f"❌ Missing diagram strip in {readme_path.relative_to(self.repo_root)}")
                all_valid = False
                continue
            
            # Validate manifest and diagrams
            manifest = self.load_manifest(manifest_path)
            if not manifest:
                all_valid = False
                continue
            
            diagram_errors = self.validate_diagram_paths(manifest, manifest_path)
            if diagram_errors:
                for error in diagram_errors:
                    logger.error(f"❌ {error}")
                all_valid = False
        
        if all_valid:
            logger.info("✅ All README diagram strips are valid")
        else:
            logger.error("❌ Some README diagram strips are invalid")
        
        return all_valid
    
    def run(self, folder_path: Optional[Path] = None, check_only: bool = False) -> bool:
        """Main execution method."""
        if check_only:
            return self.check_mode(folder_path)
        
        logger.info("🚀 Starting README diagram injection")
        
        # Generate diagrams first
        if not self.dry_run:
            self.generate_diagrams()
        
        # Find and process manifest files
        manifest_files = self.find_manifest_files(folder_path)
        
        if not manifest_files:
            logger.warning("No manifest files found")
            if folder_path:
                logger.info(f"Create a .readme.diagrams.yaml file in {folder_path}")
            return True
        
        logger.info(f"Found {len(manifest_files)} folders to process")
        
        success = True
        for manifest_path in manifest_files:
            if not self.process_folder(manifest_path):
                success = False
        
        # Print statistics
        self.print_statistics()
        
        return success and self.stats['errors'] == 0
    
    def generate_diagrams(self):
        """Generate diagrams using the Makefile."""
        logger.info("📊 Generating diagrams...")
        try:
            result = subprocess.run(
                ['make', '-f', 'Makefile.diagrams', 'all'],
                cwd=self.repo_root,
                capture_output=True,
                text=True,
                timeout=300  # 5 minute timeout
            )
            
            if result.returncode == 0:
                logger.info("✅ Diagrams generated successfully")
            else:
                logger.warning(f"⚠️  Diagram generation had issues: {result.stderr}")
        except subprocess.TimeoutExpired:
            logger.error("❌ Diagram generation timed out")
        except Exception as e:
            logger.error(f"❌ Error generating diagrams: {e}")
    
    def print_statistics(self):
        """Print processing statistics."""
        logger.info("\n📈 Processing Statistics:")
        logger.info(f"  📁 Folders processed: {self.stats['processed']}")
        logger.info(f"  ✅ READMEs updated: {self.stats['updated']}")
        logger.info(f"  ❌ Errors: {self.stats['errors']}")
        logger.info(f"  📊 Missing diagrams: {self.stats['missing_diagrams']}")
        logger.info(f"  📋 Missing manifests: {self.stats['missing_manifests']}")

def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="AtlasMesh Fleet OS - README Diagram Injection Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument(
        '--check',
        action='store_true',
        help='Validate that all READMEs have correct diagram strips (CI mode)'
    )
    
    parser.add_argument(
        '--folder',
        type=Path,
        help='Process specific folder only (default: all folders)'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what would be changed without making changes'
    )
    
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose logging'
    )
    
    args = parser.parse_args()
    
    # Find repository root
    repo_root = Path.cwd()
    while not (repo_root / ".git").exists() and repo_root != repo_root.parent:
        repo_root = repo_root.parent
    
    if not (repo_root / ".git").exists():
        logger.error("❌ Not in a git repository")
        sys.exit(1)
    
    # Initialize injector
    injector = DiagramInjector(
        repo_root=repo_root,
        dry_run=args.dry_run,
        verbose=args.verbose
    )
    
    # Run processing
    success = injector.run(
        folder_path=args.folder,
        check_only=args.check
    )
    
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
