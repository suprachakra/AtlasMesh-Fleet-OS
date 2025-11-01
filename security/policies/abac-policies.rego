# AtlasMesh Fleet OS - Attribute-Based Access Control (ABAC) Policies
# Advanced authorization policies based on user, resource, environment, and action attributes

package atlasmesh.abac

import rego.v1

# Default deny - explicit allow required
default allow := false

# Main authorization decision point
allow if {
    # Check all applicable policies
    policy_allows_access
    
    # Ensure no explicit deny
    not policy_denies_access
    
    # Check environmental constraints
    environment_constraints_satisfied
    
    # Check data classification constraints
    data_classification_allows_access
}

# Policy allows access based on attributes
policy_allows_access if {
    some policy in applicable_policies
    evaluate_policy(policy)
}

# Policy explicitly denies access
policy_denies_access if {
    some policy in applicable_policies
    policy.effect == "deny"
    evaluate_policy_conditions(policy.conditions)
}

# Environment constraints are satisfied
environment_constraints_satisfied if {
    # Time-based constraints
    time_constraints_satisfied
    
    # Location-based constraints  
    location_constraints_satisfied
    
    # Network-based constraints
    network_constraints_satisfied
    
    # Device-based constraints
    device_constraints_satisfied
}

# Data classification allows access
data_classification_allows_access if {
    # Check if user clearance level is sufficient
    user_clearance_sufficient
    
    # Check data handling requirements
    data_handling_compliant
    
    # Check data residency requirements
    data_residency_compliant
}

# Get applicable policies based on context
applicable_policies := policies if {
    policies := [policy |
        some policy in all_policies
        policy_applies_to_context(policy)
    ]
}

# Check if policy applies to current context
policy_applies_to_context(policy) if {
    # Check subject (user) attributes
    subject_matches(policy.subject, input.user)
    
    # Check resource attributes
    resource_matches(policy.resource, input.resource)
    
    # Check action attributes
    action_matches(policy.action, input.action)
    
    # Check environment attributes
    environment_matches(policy.environment, input.environment)
}

# Evaluate policy conditions
evaluate_policy(policy) if {
    policy.effect == "allow"
    evaluate_policy_conditions(policy.conditions)
}

evaluate_policy_conditions(conditions) if {
    every condition in conditions {
        evaluate_condition(condition)
    }
}

# Evaluate individual condition
evaluate_condition(condition) if {
    condition.operator == "equals"
    get_attribute_value(condition.attribute) == condition.value
}

evaluate_condition(condition) if {
    condition.operator == "not_equals"
    get_attribute_value(condition.attribute) != condition.value
}

evaluate_condition(condition) if {
    condition.operator == "in"
    get_attribute_value(condition.attribute) in condition.values
}

evaluate_condition(condition) if {
    condition.operator == "not_in"
    not get_attribute_value(condition.attribute) in condition.values
}

evaluate_condition(condition) if {
    condition.operator == "greater_than"
    get_attribute_value(condition.attribute) > condition.value
}

evaluate_condition(condition) if {
    condition.operator == "less_than"
    get_attribute_value(condition.attribute) < condition.value
}

evaluate_condition(condition) if {
    condition.operator == "contains"
    contains(get_attribute_value(condition.attribute), condition.value)
}

evaluate_condition(condition) if {
    condition.operator == "regex_match"
    regex.match(condition.pattern, get_attribute_value(condition.attribute))
}

# Get attribute value from input context
get_attribute_value(attribute) := value if {
    attribute_path := split(attribute, ".")
    value := get_nested_value(input, attribute_path)
}

# Helper to get nested value from object
get_nested_value(obj, [key]) := obj[key]

get_nested_value(obj, [key | rest]) := get_nested_value(obj[key], rest)

# Subject matching
subject_matches(policy_subject, user) if {
    # Match by user ID
    policy_subject.id == user.id
}

subject_matches(policy_subject, user) if {
    # Match by role
    some role in policy_subject.roles
    role in user.roles
}

subject_matches(policy_subject, user) if {
    # Match by department/sector
    policy_subject.sector == user.sector
}

subject_matches(policy_subject, user) if {
    # Match by clearance level
    policy_subject.clearance_level <= user.clearance_level
}

subject_matches(policy_subject, user) if {
    # Match by group membership
    some group in policy_subject.groups
    group in user.groups
}

# Resource matching
resource_matches(policy_resource, resource) if {
    # Match by resource type
    policy_resource.type == resource.type
}

resource_matches(policy_resource, resource) if {
    # Match by resource owner
    policy_resource.owner == resource.owner_id
}

resource_matches(policy_resource, resource) if {
    # Match by resource classification
    policy_resource.classification == resource.classification
}

resource_matches(policy_resource, resource) if {
    # Match by resource sector
    policy_resource.sector == resource.sector
}

resource_matches(policy_resource, resource) if {
    # Match by resource tags
    every tag in policy_resource.tags {
        tag in resource.tags
    }
}

# Action matching
action_matches(policy_action, action) if {
    policy_action == action
}

action_matches(policy_actions, action) if {
    is_array(policy_actions)
    action in policy_actions
}

# Environment matching
environment_matches(policy_env, env) if {
    # Match by network location
    policy_env.network == env.network
}

environment_matches(policy_env, env) if {
    # Match by time range
    time_in_range(policy_env.time_range, env.current_time)
}

environment_matches(policy_env, env) if {
    # Match by geographic location
    location_in_bounds(policy_env.location, env.location)
}

# Time constraints
time_constraints_satisfied if {
    not input.environment.time_restrictions
}

time_constraints_satisfied if {
    input.environment.time_restrictions
    current_time := time.now_ns()
    some restriction in input.environment.time_restrictions
    time_in_range(restriction, current_time)
}

time_in_range(time_range, current_time) if {
    current_time >= time_range.start
    current_time <= time_range.end
}

# Location constraints
location_constraints_satisfied if {
    not input.environment.location_restrictions
}

location_constraints_satisfied if {
    input.environment.location_restrictions
    some restriction in input.environment.location_restrictions
    location_in_bounds(restriction, input.environment.location)
}

location_in_bounds(bounds, location) if {
    location.latitude >= bounds.min_latitude
    location.latitude <= bounds.max_latitude
    location.longitude >= bounds.min_longitude
    location.longitude <= bounds.max_longitude
}

# Network constraints
network_constraints_satisfied if {
    not input.environment.network_restrictions
}

network_constraints_satisfied if {
    input.environment.network_restrictions
    input.environment.client_ip
    some allowed_network in input.environment.network_restrictions.allowed_networks
    net.cidr_contains(allowed_network, input.environment.client_ip)
}

# Device constraints
device_constraints_satisfied if {
    not input.environment.device_restrictions
}

device_constraints_satisfied if {
    input.environment.device_restrictions
    device_is_trusted(input.environment.device)
}

device_is_trusted(device) if {
    device.is_managed == true
    device.compliance_status == "compliant"
    device.last_security_scan
    time.now_ns() - device.last_security_scan < 86400000000000 # 24 hours in nanoseconds
}

# User clearance level sufficient
user_clearance_sufficient if {
    not input.resource.required_clearance
}

user_clearance_sufficient if {
    input.resource.required_clearance
    input.user.clearance_level >= input.resource.required_clearance
}

# Data handling compliance
data_handling_compliant if {
    not input.resource.data_handling_requirements
}

data_handling_compliant if {
    input.resource.data_handling_requirements
    every requirement in input.resource.data_handling_requirements {
        user_meets_data_handling_requirement(requirement)
    }
}

user_meets_data_handling_requirement(requirement) if {
    requirement.type == "training"
    requirement.name in input.user.completed_training
}

user_meets_data_handling_requirement(requirement) if {
    requirement.type == "certification"
    some cert in input.user.certifications
    cert.name == requirement.name
    cert.expires_at > time.now_ns()
}

# Data residency compliance
data_residency_compliant if {
    not input.resource.data_residency_requirements
}

data_residency_compliant if {
    input.resource.data_residency_requirements
    input.environment.location
    input.environment.location.country in input.resource.data_residency_requirements.allowed_countries
}

# Policy definitions
all_policies := [
    # High-security defense sector policy
    {
        "id": "defense_classified_access",
        "effect": "allow",
        "subject": {
            "sector": "defense",
            "clearance_level": 3,
            "roles": ["operator", "supervisor", "admin"]
        },
        "resource": {
            "type": "vehicle_telemetry",
            "classification": "classified",
            "sector": "defense"
        },
        "action": ["read", "update"],
        "environment": {
            "network": "secure_network",
            "location": {
                "min_latitude": 24.0,
                "max_latitude": 49.0,
                "min_longitude": -125.0,
                "max_longitude": -66.0
            }
        },
        "conditions": [
            {
                "attribute": "user.mfa_verified",
                "operator": "equals",
                "value": true
            },
            {
                "attribute": "environment.device.is_managed",
                "operator": "equals", 
                "value": true
            }
        ]
    },
    
    # Mining sector operational data access
    {
        "id": "mining_operational_access",
        "effect": "allow",
        "subject": {
            "sector": "mining",
            "roles": ["operator", "supervisor", "admin"]
        },
        "resource": {
            "type": "vehicle_telemetry",
            "classification": "internal",
            "sector": "mining"
        },
        "action": ["read", "create", "update"],
        "environment": {
            "time_range": {
                "start": 21600000000000,  # 6 AM in nanoseconds from midnight
                "end": 79200000000000    # 10 PM in nanoseconds from midnight
            }
        },
        "conditions": [
            {
                "attribute": "user.employment_status",
                "operator": "equals",
                "value": "active"
            }
        ]
    },
    
    # Logistics cross-sector data sharing
    {
        "id": "logistics_cross_sector_sharing",
        "effect": "allow",
        "subject": {
            "sector": "logistics",
            "roles": ["supervisor", "admin"]
        },
        "resource": {
            "type": "route_data",
            "classification": "public"
        },
        "action": ["read"],
        "conditions": [
            {
                "attribute": "resource.sharing_agreement",
                "operator": "equals",
                "value": true
            },
            {
                "attribute": "user.data_sharing_training",
                "operator": "in",
                "values": ["completed", "current"]
            }
        ]
    },
    
    # Ride-hail passenger data protection
    {
        "id": "ridehail_passenger_data_protection",
        "effect": "allow",
        "subject": {
            "sector": "ridehail",
            "roles": ["operator", "supervisor", "admin"]
        },
        "resource": {
            "type": "passenger_data",
            "classification": "pii"
        },
        "action": ["read"],
        "conditions": [
            {
                "attribute": "user.privacy_training_completed",
                "operator": "equals",
                "value": true
            },
            {
                "attribute": "resource.consent_given",
                "operator": "equals",
                "value": true
            },
            {
                "attribute": "user.background_check_status",
                "operator": "equals",
                "value": "cleared"
            }
        ]
    },
    
    # Emergency override policy
    {
        "id": "emergency_override",
        "effect": "allow",
        "subject": {
            "roles": ["supervisor", "admin"]
        },
        "resource": {
            "type": "*"
        },
        "action": ["read", "update", "emergency_stop"],
        "conditions": [
            {
                "attribute": "environment.emergency_declared",
                "operator": "equals",
                "value": true
            },
            {
                "attribute": "user.emergency_response_certified",
                "operator": "equals",
                "value": true
            }
        ]
    },
    
    # Deny policy for suspended users
    {
        "id": "suspended_user_deny",
        "effect": "deny",
        "subject": {
            "employment_status": "suspended"
        },
        "resource": {
            "type": "*"
        },
        "action": ["*"],
        "conditions": []
    },
    
    # Deny policy for expired certifications
    {
        "id": "expired_certification_deny",
        "effect": "deny",
        "subject": {},
        "resource": {
            "classification": ["classified", "secret"]
        },
        "action": ["*"],
        "conditions": [
            {
                "attribute": "user.security_clearance_expired",
                "operator": "equals",
                "value": true
            }
        ]
    }
]

# Audit information for policy decisions
audit_info := {
    "timestamp": time.now_ns(),
    "decision": allow,
    "applicable_policies": [policy.id | some policy in applicable_policies],
    "user_attributes": {
        "id": input.user.id,
        "sector": input.user.sector,
        "roles": input.user.roles,
        "clearance_level": input.user.clearance_level
    },
    "resource_attributes": {
        "type": input.resource.type,
        "classification": input.resource.classification,
        "sector": input.resource.sector
    },
    "environment_attributes": {
        "client_ip": input.environment.client_ip,
        "location": input.environment.location,
        "device_trusted": device_is_trusted(input.environment.device)
    },
    "constraints_satisfied": {
        "time": time_constraints_satisfied,
        "location": location_constraints_satisfied,
        "network": network_constraints_satisfied,
        "device": device_constraints_satisfied,
        "clearance": user_clearance_sufficient,
        "data_handling": data_handling_compliant,
        "data_residency": data_residency_compliant
    }
}
