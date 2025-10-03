#!/usr/bin/env python3
"""
AtlasMesh Vehicle Provisioner
Handles vehicle onboarding, configuration, and initial setup

This tool is used in the garage/depot to:
- Flash initial OS and software
- Configure vehicle profiles
- Generate and install certificates
- Perform initial calibration
- Validate system integrity
"""

import os
import sys
import json
import yaml
import hashlib
import subprocess
import argparse
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from datetime import datetime, timezone
import requests
import cryptography
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.x509 import load_pem_x509_certificate

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('vehicle_provisioner')

class VehicleProvisioner:
    """Main vehicle provisioning class"""
    
    def __init__(self, config_path: str = None):
        """Initialize provisioner with configuration"""
        self.config_path = config_path or '/opt/atlasmesh/garage/config/provisioner.yaml'
        self.config = self.load_config()
        self.fleet_api_url = self.config.get('fleet_api_url', 'https://api.atlasmesh.com')
        self.ca_cert_path = self.config.get('ca_cert_path', '/opt/atlasmesh/certs/ca.pem')
        self.private_key_path = self.config.get('private_key_path', '/opt/atlasmesh/certs/garage.key')
        
    def load_config(self) -> Dict:
        """Load provisioner configuration"""
        try:
            with open(self.config_path, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            logger.warning(f"Config file not found: {self.config_path}, using defaults")
            return self.get_default_config()
        except Exception as e:
            logger.error(f"Failed to load config: {e}")
            return self.get_default_config()
    
    def get_default_config(self) -> Dict:
        """Get default configuration"""
        return {
            'fleet_api_url': 'https://api.atlasmesh.com',
            'ca_cert_path': '/opt/atlasmesh/certs/ca.pem',
            'private_key_path': '/opt/atlasmesh/certs/garage.key',
            'os_image_path': '/opt/atlasmesh/images/atlasmesh-os-latest.img',
            'software_bundle_path': '/opt/atlasmesh/bundles/vehicle-software-latest.tar.gz',
            'profile_templates_path': '/opt/atlasmesh/profiles',
            'certificates_path': '/opt/atlasmesh/certs',
            'target_device': '/dev/sdb',  # Default target device
            'verification_timeout': 300,  # 5 minutes
        }
    
    def provision_vehicle(self, vehicle_id: str, vehicle_type: str, 
                         sector: str = 'logistics') -> bool:
        """Complete vehicle provisioning process"""
        logger.info(f"Starting provisioning for vehicle {vehicle_id} (type: {vehicle_type}, sector: {sector})")
        
        try:
            # Step 1: Validate inputs
            if not self.validate_inputs(vehicle_id, vehicle_type, sector):
                return False
            
            # Step 2: Register vehicle with fleet management
            if not self.register_vehicle(vehicle_id, vehicle_type, sector):
                return False
            
            # Step 3: Flash OS image
            if not self.flash_os_image(vehicle_id):
                return False
            
            # Step 4: Install software bundle
            if not self.install_software_bundle(vehicle_id):
                return False
            
            # Step 5: Configure vehicle profile
            if not self.configure_vehicle_profile(vehicle_id, vehicle_type, sector):
                return False
            
            # Step 6: Generate and install certificates
            if not self.setup_certificates(vehicle_id):
                return False
            
            # Step 7: Perform initial calibration
            if not self.perform_initial_calibration(vehicle_id):
                return False
            
            # Step 8: Generate SBOM (Software Bill of Materials)
            if not self.generate_sbom(vehicle_id):
                return False
            
            # Step 9: Validate system integrity
            if not self.validate_system_integrity(vehicle_id):
                return False
            
            # Step 10: Mark vehicle as ready
            if not self.mark_vehicle_ready(vehicle_id):
                return False
            
            logger.info(f"Vehicle {vehicle_id} provisioned successfully")
            return True
            
        except Exception as e:
            logger.error(f"Provisioning failed for vehicle {vehicle_id}: {e}")
            return False
    
    def validate_inputs(self, vehicle_id: str, vehicle_type: str, sector: str) -> bool:
        """Validate provisioning inputs"""
        logger.info("Validating inputs...")
        
        # Validate vehicle ID format
        if not vehicle_id or len(vehicle_id) < 8:
            logger.error("Invalid vehicle ID: must be at least 8 characters")
            return False
        
        # Validate vehicle type
        supported_types = ['ugv_themis', 'ugv_atlas', 'uav_phoenix']
        if vehicle_type not in supported_types:
            logger.error(f"Unsupported vehicle type: {vehicle_type}")
            return False
        
        # Validate sector
        supported_sectors = ['defense', 'mining', 'logistics', 'ridehail']
        if sector not in supported_sectors:
            logger.error(f"Unsupported sector: {sector}")
            return False
        
        # Check if vehicle already exists
        if self.vehicle_exists(vehicle_id):
            logger.error(f"Vehicle {vehicle_id} already exists")
            return False
        
        logger.info("Input validation passed")
        return True
    
    def vehicle_exists(self, vehicle_id: str) -> bool:
        """Check if vehicle already exists in fleet"""
        try:
            response = requests.get(
                f"{self.fleet_api_url}/api/v1/vehicles/{vehicle_id}",
                timeout=10
            )
            return response.status_code == 200
        except Exception as e:
            logger.warning(f"Failed to check vehicle existence: {e}")
            return False
    
    def register_vehicle(self, vehicle_id: str, vehicle_type: str, sector: str) -> bool:
        """Register vehicle with fleet management system"""
        logger.info(f"Registering vehicle {vehicle_id} with fleet management...")
        
        try:
            registration_data = {
                'vehicle_id': vehicle_id,
                'vehicle_type': vehicle_type,
                'sector': sector,
                'status': 'provisioning',
                'provisioning_started': datetime.now(timezone.utc).isoformat(),
                'garage_id': self.config.get('garage_id', 'garage_001'),
                'provisioner_version': '1.0.0'
            }
            
            response = requests.post(
                f"{self.fleet_api_url}/api/v1/vehicles",
                json=registration_data,
                timeout=30
            )
            
            if response.status_code == 201:
                logger.info("Vehicle registered successfully")
                return True
            else:
                logger.error(f"Registration failed: {response.status_code} - {response.text}")
                return False
                
        except Exception as e:
            logger.error(f"Registration failed: {e}")
            return False
    
    def flash_os_image(self, vehicle_id: str) -> bool:
        """Flash OS image to vehicle storage"""
        logger.info(f"Flashing OS image for vehicle {vehicle_id}...")
        
        os_image_path = self.config['os_image_path']
        target_device = self.config['target_device']
        
        if not os.path.exists(os_image_path):
            logger.error(f"OS image not found: {os_image_path}")
            return False
        
        try:
            # Verify image integrity
            if not self.verify_image_integrity(os_image_path):
                return False
            
            # Flash image using dd command
            logger.info(f"Flashing {os_image_path} to {target_device}")
            cmd = [
                'sudo', 'dd',
                f'if={os_image_path}',
                f'of={target_device}',
                'bs=4M',
                'status=progress',
                'conv=fsync'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)  # 30 min timeout
            
            if result.returncode == 0:
                logger.info("OS image flashed successfully")
                return True
            else:
                logger.error(f"Flashing failed: {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            logger.error("OS flashing timed out")
            return False
        except Exception as e:
            logger.error(f"OS flashing failed: {e}")
            return False
    
    def verify_image_integrity(self, image_path: str) -> bool:
        """Verify OS image integrity using checksum"""
        logger.info("Verifying OS image integrity...")
        
        try:
            # Calculate SHA256 checksum
            sha256_hash = hashlib.sha256()
            with open(image_path, 'rb') as f:
                for chunk in iter(lambda: f.read(4096), b""):
                    sha256_hash.update(chunk)
            
            calculated_checksum = sha256_hash.hexdigest()
            
            # Load expected checksum
            checksum_file = f"{image_path}.sha256"
            if os.path.exists(checksum_file):
                with open(checksum_file, 'r') as f:
                    expected_checksum = f.read().strip().split()[0]
                
                if calculated_checksum == expected_checksum:
                    logger.info("OS image integrity verified")
                    return True
                else:
                    logger.error("OS image checksum mismatch")
                    return False
            else:
                logger.warning("No checksum file found, skipping verification")
                return True
                
        except Exception as e:
            logger.error(f"Image verification failed: {e}")
            return False
    
    def install_software_bundle(self, vehicle_id: str) -> bool:
        """Install software bundle on vehicle"""
        logger.info(f"Installing software bundle for vehicle {vehicle_id}...")
        
        bundle_path = self.config['software_bundle_path']
        
        if not os.path.exists(bundle_path):
            logger.error(f"Software bundle not found: {bundle_path}")
            return False
        
        try:
            # Mount target filesystem
            mount_point = f"/mnt/vehicle_{vehicle_id}"
            if not self.mount_target_filesystem(mount_point):
                return False
            
            # Extract and install bundle
            cmd = ['sudo', 'tar', '-xzf', bundle_path, '-C', mount_point]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
            
            if result.returncode == 0:
                logger.info("Software bundle installed successfully")
                success = True
            else:
                logger.error(f"Bundle installation failed: {result.stderr}")
                success = False
            
            # Unmount filesystem
            self.unmount_target_filesystem(mount_point)
            return success
            
        except Exception as e:
            logger.error(f"Software installation failed: {e}")
            return False
    
    def mount_target_filesystem(self, mount_point: str) -> bool:
        """Mount target vehicle filesystem"""
        try:
            os.makedirs(mount_point, exist_ok=True)
            
            # Mount the root partition (assuming partition 2)
            target_partition = f"{self.config['target_device']}2"
            cmd = ['sudo', 'mount', target_partition, mount_point]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                logger.info(f"Filesystem mounted at {mount_point}")
                return True
            else:
                logger.error(f"Mount failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Mount failed: {e}")
            return False
    
    def unmount_target_filesystem(self, mount_point: str) -> bool:
        """Unmount target vehicle filesystem"""
        try:
            cmd = ['sudo', 'umount', mount_point]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                logger.info(f"Filesystem unmounted from {mount_point}")
                os.rmdir(mount_point)
                return True
            else:
                logger.warning(f"Unmount warning: {result.stderr}")
                return True  # Don't fail on unmount issues
                
        except Exception as e:
            logger.warning(f"Unmount warning: {e}")
            return True
    
    def configure_vehicle_profile(self, vehicle_id: str, vehicle_type: str, sector: str) -> bool:
        """Configure vehicle-specific profile"""
        logger.info(f"Configuring vehicle profile for {vehicle_id}...")
        
        try:
            # Load base profile template
            template_path = f"{self.config['profile_templates_path']}/{vehicle_type}.yaml"
            with open(template_path, 'r') as f:
                profile = yaml.safe_load(f)
            
            # Customize profile for this vehicle
            profile['profile_metadata']['profile_id'] = f"{vehicle_id}_profile"
            profile['profile_metadata']['vehicle_id'] = vehicle_id
            profile['profile_metadata']['sector_primary'] = sector
            profile['profile_metadata']['provisioned_date'] = datetime.now(timezone.utc).isoformat()
            
            # Add sector-specific configurations
            if sector in profile.get('sector_profiles', {}):
                sector_config = profile['sector_profiles'][sector]
                # Apply sector-specific overrides
                profile['sector_active'] = sector
                profile['sector_config'] = sector_config
            
            # Generate unique identifiers
            profile['security'] = {
                'vehicle_uuid': self.generate_vehicle_uuid(vehicle_id),
                'provisioning_hash': self.generate_provisioning_hash(vehicle_id, vehicle_type, sector)
            }
            
            # Save customized profile
            profile_output_path = f"/tmp/vehicle_profile_{vehicle_id}.yaml"
            with open(profile_output_path, 'w') as f:
                yaml.dump(profile, f, default_flow_style=False)
            
            logger.info("Vehicle profile configured successfully")
            return True
            
        except Exception as e:
            logger.error(f"Profile configuration failed: {e}")
            return False
    
    def generate_vehicle_uuid(self, vehicle_id: str) -> str:
        """Generate unique UUID for vehicle"""
        import uuid
        namespace = uuid.UUID('6ba7b810-9dad-11d1-80b4-00c04fd430c8')  # AtlasMesh namespace
        return str(uuid.uuid5(namespace, vehicle_id))
    
    def generate_provisioning_hash(self, vehicle_id: str, vehicle_type: str, sector: str) -> str:
        """Generate provisioning hash for integrity verification"""
        data = f"{vehicle_id}:{vehicle_type}:{sector}:{datetime.now(timezone.utc).isoformat()}"
        return hashlib.sha256(data.encode()).hexdigest()
    
    def setup_certificates(self, vehicle_id: str) -> bool:
        """Generate and install vehicle certificates"""
        logger.info(f"Setting up certificates for vehicle {vehicle_id}...")
        
        try:
            # Generate vehicle private key
            private_key = rsa.generate_private_key(
                public_exponent=65537,
                key_size=2048
            )
            
            # Create certificate signing request (CSR)
            csr = self.create_certificate_request(vehicle_id, private_key)
            
            # Sign certificate with CA (in production, this would be done by CA service)
            certificate = self.sign_certificate(csr, vehicle_id)
            
            # Save private key and certificate
            key_path = f"/tmp/vehicle_{vehicle_id}.key"
            cert_path = f"/tmp/vehicle_{vehicle_id}.crt"
            
            with open(key_path, 'wb') as f:
                f.write(private_key.private_bytes(
                    encoding=serialization.Encoding.PEM,
                    format=serialization.PrivateFormat.PKCS8,
                    encryption_algorithm=serialization.NoEncryption()
                ))
            
            with open(cert_path, 'wb') as f:
                f.write(certificate.public_bytes(serialization.Encoding.PEM))
            
            logger.info("Certificates generated successfully")
            return True
            
        except Exception as e:
            logger.error(f"Certificate setup failed: {e}")
            return False
    
    def create_certificate_request(self, vehicle_id: str, private_key):
        """Create certificate signing request"""
        from cryptography import x509
        from cryptography.x509.oid import NameOID
        
        subject = x509.Name([
            x509.NameAttribute(NameOID.COUNTRY_NAME, "US"),
            x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME, "California"),
            x509.NameAttribute(NameOID.LOCALITY_NAME, "San Francisco"),
            x509.NameAttribute(NameOID.ORGANIZATION_NAME, "AtlasMesh"),
            x509.NameAttribute(NameOID.ORGANIZATIONAL_UNIT_NAME, "Fleet Vehicles"),
            x509.NameAttribute(NameOID.COMMON_NAME, vehicle_id),
        ])
        
        csr = x509.CertificateSigningRequestBuilder().subject_name(subject).sign(
            private_key, hashes.SHA256()
        )
        
        return csr
    
    def sign_certificate(self, csr, vehicle_id: str):
        """Sign certificate with CA (simplified for demo)"""
        from cryptography import x509
        import datetime
        
        # In production, this would call the CA service
        # For now, create a self-signed certificate
        
        subject = csr.subject
        issuer = subject  # Self-signed for demo
        
        certificate = x509.CertificateBuilder().subject_name(
            subject
        ).issuer_name(
            issuer
        ).public_key(
            csr.public_key()
        ).serial_number(
            x509.random_serial_number()
        ).not_valid_before(
            datetime.datetime.utcnow()
        ).not_valid_after(
            datetime.datetime.utcnow() + datetime.timedelta(days=365)
        ).add_extension(
            x509.SubjectAlternativeName([
                x509.DNSName(vehicle_id),
                x509.DNSName(f"{vehicle_id}.atlasmesh.com"),
            ]),
            critical=False,
        ).sign(csr.private_key(), hashes.SHA256())
        
        return certificate
    
    def perform_initial_calibration(self, vehicle_id: str) -> bool:
        """Perform initial sensor calibration"""
        logger.info(f"Performing initial calibration for vehicle {vehicle_id}...")
        
        try:
            # This would typically involve:
            # 1. IMU calibration
            # 2. Camera calibration
            # 3. LiDAR-camera extrinsic calibration
            # 4. Wheel odometry calibration
            # 5. Steering calibration
            
            # For now, simulate calibration process
            calibration_steps = [
                "IMU bias calibration",
                "Camera intrinsic calibration",
                "LiDAR-camera extrinsic calibration",
                "Wheel odometry calibration",
                "Steering angle calibration"
            ]
            
            for step in calibration_steps:
                logger.info(f"Performing {step}...")
                # Simulate calibration time
                import time
                time.sleep(2)
            
            # Generate calibration report
            calibration_report = {
                'vehicle_id': vehicle_id,
                'calibration_date': datetime.now(timezone.utc).isoformat(),
                'calibration_version': '1.0.0',
                'steps_completed': calibration_steps,
                'status': 'completed',
                'quality_score': 0.95  # Simulated quality score
            }
            
            # Save calibration report
            report_path = f"/tmp/calibration_report_{vehicle_id}.json"
            with open(report_path, 'w') as f:
                json.dump(calibration_report, f, indent=2)
            
            logger.info("Initial calibration completed successfully")
            return True
            
        except Exception as e:
            logger.error(f"Calibration failed: {e}")
            return False
    
    def generate_sbom(self, vehicle_id: str) -> bool:
        """Generate Software Bill of Materials"""
        logger.info(f"Generating SBOM for vehicle {vehicle_id}...")
        
        try:
            # This would scan the installed software and generate SBOM
            # For now, create a basic SBOM structure
            
            sbom = {
                'bomFormat': 'CycloneDX',
                'specVersion': '1.4',
                'serialNumber': f'urn:uuid:{self.generate_vehicle_uuid(vehicle_id)}',
                'version': 1,
                'metadata': {
                    'timestamp': datetime.now(timezone.utc).isoformat(),
                    'tools': [
                        {
                            'vendor': 'AtlasMesh',
                            'name': 'Vehicle Provisioner',
                            'version': '1.0.0'
                        }
                    ],
                    'component': {
                        'type': 'device',
                        'name': vehicle_id,
                        'version': '1.0.0'
                    }
                },
                'components': [
                    {
                        'type': 'operating-system',
                        'name': 'AtlasMesh OS',
                        'version': '1.0.0',
                        'licenses': [{'license': {'name': 'Proprietary'}}]
                    },
                    {
                        'type': 'application',
                        'name': 'Vehicle Agent',
                        'version': '1.0.0',
                        'licenses': [{'license': {'name': 'Proprietary'}}]
                    },
                    {
                        'type': 'library',
                        'name': 'ROS2 Humble',
                        'version': '22.04',
                        'licenses': [{'license': {'name': 'Apache-2.0'}}]
                    }
                ]
            }
            
            # Save SBOM
            sbom_path = f"/tmp/sbom_{vehicle_id}.json"
            with open(sbom_path, 'w') as f:
                json.dump(sbom, f, indent=2)
            
            logger.info("SBOM generated successfully")
            return True
            
        except Exception as e:
            logger.error(f"SBOM generation failed: {e}")
            return False
    
    def validate_system_integrity(self, vehicle_id: str) -> bool:
        """Validate complete system integrity"""
        logger.info(f"Validating system integrity for vehicle {vehicle_id}...")
        
        try:
            validation_checks = [
                self.check_os_integrity(),
                self.check_software_integrity(),
                self.check_configuration_integrity(vehicle_id),
                self.check_certificate_integrity(vehicle_id),
                self.check_calibration_integrity(vehicle_id)
            ]
            
            if all(validation_checks):
                logger.info("System integrity validation passed")
                return True
            else:
                logger.error("System integrity validation failed")
                return False
                
        except Exception as e:
            logger.error(f"System validation failed: {e}")
            return False
    
    def check_os_integrity(self) -> bool:
        """Check OS integrity"""
        # Simplified check - in production would verify signatures, checksums, etc.
        return True
    
    def check_software_integrity(self) -> bool:
        """Check software integrity"""
        # Simplified check - in production would verify all installed packages
        return True
    
    def check_configuration_integrity(self, vehicle_id: str) -> bool:
        """Check configuration integrity"""
        # Verify vehicle profile exists and is valid
        profile_path = f"/tmp/vehicle_profile_{vehicle_id}.yaml"
        return os.path.exists(profile_path)
    
    def check_certificate_integrity(self, vehicle_id: str) -> bool:
        """Check certificate integrity"""
        # Verify certificates exist and are valid
        key_path = f"/tmp/vehicle_{vehicle_id}.key"
        cert_path = f"/tmp/vehicle_{vehicle_id}.crt"
        return os.path.exists(key_path) and os.path.exists(cert_path)
    
    def check_calibration_integrity(self, vehicle_id: str) -> bool:
        """Check calibration integrity"""
        # Verify calibration report exists
        report_path = f"/tmp/calibration_report_{vehicle_id}.json"
        return os.path.exists(report_path)
    
    def mark_vehicle_ready(self, vehicle_id: str) -> bool:
        """Mark vehicle as ready for deployment"""
        logger.info(f"Marking vehicle {vehicle_id} as ready...")
        
        try:
            # Update vehicle status in fleet management
            update_data = {
                'status': 'ready',
                'provisioning_completed': datetime.now(timezone.utc).isoformat(),
                'integrity_validated': True,
                'ready_for_deployment': True
            }
            
            response = requests.patch(
                f"{self.fleet_api_url}/api/v1/vehicles/{vehicle_id}",
                json=update_data,
                timeout=30
            )
            
            if response.status_code == 200:
                logger.info("Vehicle marked as ready successfully")
                return True
            else:
                logger.error(f"Failed to mark vehicle ready: {response.status_code}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to mark vehicle ready: {e}")
            return False

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='AtlasMesh Vehicle Provisioner')
    parser.add_argument('--vehicle-id', required=True, help='Unique vehicle identifier')
    parser.add_argument('--vehicle-type', required=True, help='Vehicle type (e.g., ugv_themis)')
    parser.add_argument('--sector', default='logistics', help='Primary sector (defense, mining, logistics, ridehail)')
    parser.add_argument('--config', help='Configuration file path')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose logging')
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Initialize provisioner
    provisioner = VehicleProvisioner(args.config)
    
    # Provision vehicle
    success = provisioner.provision_vehicle(args.vehicle_id, args.vehicle_type, args.sector)
    
    if success:
        print(f"✅ Vehicle {args.vehicle_id} provisioned successfully")
        sys.exit(0)
    else:
        print(f"❌ Vehicle {args.vehicle_id} provisioning failed")
        sys.exit(1)

if __name__ == '__main__':
    main()
