#!/usr/bin/env python3
"""
AtlasMesh Fleet OS - Map Validator

This script validates map files in Lanelet2 and OpenDRIVE formats,
checking for common errors and compliance with AtlasMesh requirements.

Usage:
  python validate_map.py --format <lanelet2|opendrive> --file <path_to_map_file>
  python validate_map.py --format lanelet2 --file configs/map/examples/lanelet2/intersection_example.osm
  python validate_map.py --format opendrive --file configs/map/examples/opendrive/straight_road_example.xodr

Requirements:
  - lanelet2 (for Lanelet2 validation)
  - pyodrx (for OpenDRIVE validation)
  - lxml
"""

import argparse
import os
import sys
import json
import logging
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any
import xml.etree.ElementTree as ET

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("map_validator")

# Map format validators
class MapValidator:
    """Base class for map validators"""
    
    def __init__(self, map_file: str):
        self.map_file = map_file
        self.errors = []
        self.warnings = []
    
    def validate(self) -> bool:
        """Validate the map file"""
        raise NotImplementedError("Subclasses must implement validate()")
    
    def report(self) -> None:
        """Print validation report"""
        if not self.errors and not self.warnings:
            logger.info(f"✅ Map file {self.map_file} is valid")
            return True
        
        if self.errors:
            logger.error(f"❌ Found {len(self.errors)} errors in {self.map_file}:")
            for i, error in enumerate(self.errors, 1):
                logger.error(f"  {i}. {error}")
        
        if self.warnings:
            logger.warning(f"⚠️ Found {len(self.warnings)} warnings in {self.map_file}:")
            for i, warning in enumerate(self.warnings, 1):
                logger.warning(f"  {i}. {warning}")
        
        return len(self.errors) == 0


class Lanelet2Validator(MapValidator):
    """Validator for Lanelet2 maps (.osm format)"""
    
    def validate(self) -> bool:
        """Validate a Lanelet2 map file"""
        try:
            # Check if file exists
            if not os.path.exists(self.map_file):
                self.errors.append(f"File not found: {self.map_file}")
                return False
            
            # Parse XML
            try:
                tree = ET.parse(self.map_file)
                root = tree.getroot()
            except ET.ParseError as e:
                self.errors.append(f"Invalid XML: {e}")
                return False
            
            # Check for OSM format
            if root.tag != "osm":
                self.errors.append(f"Not a valid Lanelet2 map: Root element should be 'osm', found '{root.tag}'")
                return False
            
            # Collect all elements
            nodes = {}
            ways = {}
            relations = {}
            
            for elem in root:
                if elem.tag == "node":
                    node_id = elem.get("id")
                    if node_id in nodes:
                        self.errors.append(f"Duplicate node ID: {node_id}")
                    else:
                        nodes[node_id] = elem
                        
                    # Check for required attributes
                    if not elem.get("lat") or not elem.get("lon"):
                        self.errors.append(f"Node {node_id} missing lat/lon attributes")
                
                elif elem.tag == "way":
                    way_id = elem.get("id")
                    if way_id in ways:
                        self.errors.append(f"Duplicate way ID: {way_id}")
                    else:
                        ways[way_id] = elem
                    
                    # Check for nd references
                    nd_refs = [nd.get("ref") for nd in elem.findall("nd")]
                    if len(nd_refs) < 2:
                        self.errors.append(f"Way {way_id} has fewer than 2 nodes")
                    
                    # Check if all referenced nodes exist
                    for ref in nd_refs:
                        if ref not in nodes:
                            self.errors.append(f"Way {way_id} references non-existent node {ref}")
                
                elif elem.tag == "relation":
                    relation_id = elem.get("id")
                    if relation_id in relations:
                        self.errors.append(f"Duplicate relation ID: {relation_id}")
                    else:
                        relations[relation_id] = elem
                    
                    # Check relation type
                    relation_type = None
                    for tag in elem.findall("tag"):
                        if tag.get("k") == "type":
                            relation_type = tag.get("v")
                            break
                    
                    if not relation_type:
                        self.errors.append(f"Relation {relation_id} missing type tag")
                    
                    # Check lanelet structure
                    if relation_type == "lanelet":
                        left_found = False
                        right_found = False
                        for member in elem.findall("member"):
                            if member.get("role") == "left":
                                left_found = True
                                way_ref = member.get("ref")
                                if way_ref not in ways:
                                    self.errors.append(f"Lanelet {relation_id} references non-existent left way {way_ref}")
                            elif member.get("role") == "right":
                                right_found = True
                                way_ref = member.get("ref")
                                if way_ref not in ways:
                                    self.errors.append(f"Lanelet {relation_id} references non-existent right way {way_ref}")
                        
                        if not left_found or not right_found:
                            self.errors.append(f"Lanelet {relation_id} missing left or right boundary")
            
            # Check for AtlasMesh-specific requirements
            self._check_atlasmesh_requirements(root, nodes, ways, relations)
            
            return self.report()
        
        except Exception as e:
            self.errors.append(f"Validation failed with exception: {str(e)}")
            return False
    
    def _check_atlasmesh_requirements(self, root, nodes, ways, relations):
        """Check AtlasMesh-specific requirements for Lanelet2 maps"""
        
        # Check for map metadata
        metadata_found = False
        for relation in relations.values():
            for tag in relation.findall("tag"):
                if tag.get("k") == "type" and tag.get("v") == "multipolygon":
                    metadata_found = True
                    
                    # Check for required metadata fields
                    required_fields = ["name", "version", "date", "source"]
                    found_fields = set()
                    
                    for meta_tag in relation.findall("tag"):
                        if meta_tag.get("k") in required_fields:
                            found_fields.add(meta_tag.get("k"))
                    
                    missing_fields = set(required_fields) - found_fields
                    if missing_fields:
                        self.warnings.append(f"Map metadata missing required fields: {', '.join(missing_fields)}")
        
        if not metadata_found:
            self.warnings.append("Map metadata relation not found")
        
        # Check for regulatory elements
        regulatory_elements = []
        for relation in relations.values():
            for tag in relation.findall("tag"):
                if tag.get("k") == "type" and tag.get("v") == "regulatory_element":
                    regulatory_elements.append(relation)
        
        if not regulatory_elements:
            self.warnings.append("No regulatory elements found in the map")
        
        # Check for speed limits
        speed_limits_found = False
        for relation in relations.values():
            for tag in relation.findall("tag"):
                if tag.get("k") == "speed_limit":
                    speed_limits_found = True
                    break
        
        if not speed_limits_found:
            self.warnings.append("No speed limits defined in the map")


class OpenDriveValidator(MapValidator):
    """Validator for OpenDRIVE maps (.xodr format)"""
    
    def validate(self) -> bool:
        """Validate an OpenDRIVE map file"""
        try:
            # Check if file exists
            if not os.path.exists(self.map_file):
                self.errors.append(f"File not found: {self.map_file}")
                return False
            
            # Parse XML
            try:
                tree = ET.parse(self.map_file)
                root = tree.getroot()
            except ET.ParseError as e:
                self.errors.append(f"Invalid XML: {e}")
                return False
            
            # Check for OpenDRIVE format
            if root.tag != "OpenDRIVE":
                self.errors.append(f"Not a valid OpenDRIVE map: Root element should be 'OpenDRIVE', found '{root.tag}'")
                return False
            
            # Check header
            header = root.find("header")
            if header is None:
                self.errors.append("Missing header element")
            else:
                required_attrs = ["revMajor", "revMinor", "name", "version", "date"]
                for attr in required_attrs:
                    if attr not in header.attrib:
                        self.errors.append(f"Header missing required attribute: {attr}")
            
            # Check roads
            roads = root.findall("road")
            if not roads:
                self.errors.append("No roads defined in the map")
            
            road_ids = set()
            for road in roads:
                road_id = road.get("id")
                if road_id is None:
                    self.errors.append("Road missing ID attribute")
                elif road_id in road_ids:
                    self.errors.append(f"Duplicate road ID: {road_id}")
                else:
                    road_ids.add(road_id)
                
                # Check required road attributes
                if "length" not in road.attrib:
                    self.errors.append(f"Road {road_id} missing length attribute")
                
                # Check planView
                plan_view = road.find("planView")
                if plan_view is None:
                    self.errors.append(f"Road {road_id} missing planView element")
                else:
                    geometries = plan_view.findall("geometry")
                    if not geometries:
                        self.errors.append(f"Road {road_id} planView has no geometry elements")
                
                # Check lanes
                lanes = road.find("lanes")
                if lanes is None:
                    self.errors.append(f"Road {road_id} missing lanes element")
                else:
                    lane_sections = lanes.findall("laneSection")
                    if not lane_sections:
                        self.errors.append(f"Road {road_id} has no lane sections")
            
            # Check for AtlasMesh-specific requirements
            self._check_atlasmesh_requirements(root)
            
            return self.report()
        
        except Exception as e:
            self.errors.append(f"Validation failed with exception: {str(e)}")
            return False
    
    def _check_atlasmesh_requirements(self, root):
        """Check AtlasMesh-specific requirements for OpenDRIVE maps"""
        
        # Check for AtlasMesh metadata in header userData
        header = root.find("header")
        if header is not None:
            user_data = header.find("userData")
            if user_data is None:
                self.warnings.append("Header missing userData element with AtlasMesh metadata")
            else:
                vector_scene = user_data.find("vectorScene")
                if vector_scene is None or vector_scene.get("program") is None:
                    self.warnings.append("Header userData missing vectorScene element with program attribute")
        
        # Check for lane markings
        roads = root.findall("road")
        for road in roads:
            road_id = road.get("id", "unknown")
            lanes = road.find("lanes")
            if lanes is not None:
                lane_sections = lanes.findall("laneSection")
                for lane_section in lane_sections:
                    center = lane_section.find("center")
                    if center is not None:
                        center_lane = center.find("lane")
                        if center_lane is not None:
                            road_mark = center_lane.find("roadMark")
                            if road_mark is None:
                                self.warnings.append(f"Road {road_id} center lane missing roadMark element")
        
        # Check for signals
        signal_count = 0
        for road in roads:
            signals = road.find("signals")
            if signals is not None:
                signal_count += len(signals.findall("signal"))
        
        if signal_count == 0:
            self.warnings.append("No signals defined in the map")
        
        # Check for controllers
        controllers = root.findall("controller")
        if not controllers:
            self.warnings.append("No controllers defined in the map")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Validate map files for AtlasMesh Fleet OS")
    parser.add_argument("--format", choices=["lanelet2", "opendrive"], required=True,
                        help="Map format to validate")
    parser.add_argument("--file", required=True, help="Path to map file")
    
    args = parser.parse_args()
    
    # Select validator based on format
    if args.format == "lanelet2":
        validator = Lanelet2Validator(args.file)
    elif args.format == "opendrive":
        validator = OpenDriveValidator(args.file)
    else:
        logger.error(f"Unsupported map format: {args.format}")
        return 1
    
    # Validate map
    if validator.validate():
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())
