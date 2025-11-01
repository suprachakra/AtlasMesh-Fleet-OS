# AtlasMesh Fleet OS - Role-Based Access Control (RBAC) Policies
# Open Policy Agent (OPA) Rego policies for authorization decisions

package atlasmesh.rbac

import rego.v1

# Default deny - explicit allow required
default allow := false

# Allow if user has required role
allow if {
    user_has_role(input.user.roles, input.required_role)
}

# Allow if user has required permission
allow if {
    user_has_permission(input.user.permissions, input.required_permission)
}

# Allow if user has admin role (admin can do anything)
allow if {
    "admin" in input.user.roles
}

# Allow if user is accessing their own resources
allow if {
    input.resource.owner_id == input.user.id
    input.action in ["read", "update"]
}

# Sector-specific access control
allow if {
    input.user.sector == input.resource.sector
    user_has_permission(input.user.permissions, input.required_permission)
}

# Service-to-service authentication
allow if {
    input.user.type == "service"
    input.user.service_name in valid_services
    service_has_permission(input.user.service_name, input.required_permission)
}

# Helper functions

# Check if user has specific role
user_has_role(user_roles, required_role) if {
    required_role in user_roles
}

# Check if user has specific permission
user_has_permission(user_permissions, required_permission) if {
    required_permission in user_permissions
}

# Check if user has any of the required roles
user_has_any_role(user_roles, required_roles) if {
    some role in required_roles
    role in user_roles
}

# Check if user has all required permissions
user_has_all_permissions(user_permissions, required_permissions) if {
    every permission in required_permissions {
        permission in user_permissions
    }
}

# Service permissions mapping
service_permissions := {
    "api-gateway": [
        "auth:validate",
        "user:read",
        "service:proxy"
    ],
    "policy-engine": [
        "policy:evaluate",
        "policy:read",
        "vehicle:read"
    ],
    "trip-service": [
        "trip:create",
        "trip:read",
        "trip:update",
        "vehicle:read",
        "route:read"
    ],
    "dispatch-service": [
        "dispatch:create",
        "dispatch:read",
        "dispatch:update",
        "vehicle:read",
        "trip:read"
    ],
    "fleet-manager": [
        "vehicle:create",
        "vehicle:read",
        "vehicle:update",
        "vehicle:delete",
        "fleet:manage"
    ],
    "routing-service": [
        "route:create",
        "route:read",
        "route:update",
        "map:read",
        "traffic:read"
    ],
    "telemetry-ingestion": [
        "telemetry:write",
        "telemetry:read",
        "schema:validate"
    ],
    "data-lineage": [
        "lineage:track",
        "lineage:read",
        "data:read"
    ]
}

# Valid services that can authenticate
valid_services := {
    "api-gateway",
    "policy-engine", 
    "trip-service",
    "dispatch-service",
    "fleet-manager",
    "routing-service",
    "telemetry-ingestion",
    "data-lineage",
    "auth-service"
}

# Check if service has specific permission
service_has_permission(service_name, required_permission) if {
    service_name in service_permissions
    required_permission in service_permissions[service_name]
}

# Role hierarchy - higher roles inherit permissions from lower roles
role_hierarchy := {
    "admin": ["supervisor", "operator", "viewer"],
    "supervisor": ["operator", "viewer"],
    "operator": ["viewer"],
    "viewer": []
}

# Get all roles including inherited ones
effective_roles(user_roles) := roles if {
    roles := {role |
        some user_role in user_roles
        role := user_role
    } | {inherited_role |
        some user_role in user_roles
        some inherited_role in role_hierarchy[user_role]
    }
}

# Sector-specific role permissions
sector_role_permissions := {
    "defense": {
        "admin": [
            "vehicle:*",
            "trip:*", 
            "dispatch:*",
            "security:*",
            "user:*",
            "system:*"
        ],
        "supervisor": [
            "vehicle:read",
            "vehicle:update",
            "trip:*",
            "dispatch:*",
            "security:read",
            "user:read"
        ],
        "operator": [
            "vehicle:read",
            "trip:read",
            "trip:create",
            "dispatch:read",
            "dispatch:create"
        ],
        "viewer": [
            "vehicle:read",
            "trip:read",
            "dispatch:read"
        ]
    },
    "mining": {
        "admin": [
            "vehicle:*",
            "trip:*",
            "dispatch:*",
            "maintenance:*",
            "user:*",
            "system:*"
        ],
        "supervisor": [
            "vehicle:read",
            "vehicle:update",
            "trip:*",
            "dispatch:*",
            "maintenance:read",
            "maintenance:schedule",
            "user:read"
        ],
        "operator": [
            "vehicle:read",
            "trip:read",
            "trip:create",
            "dispatch:read",
            "dispatch:create",
            "maintenance:read"
        ],
        "viewer": [
            "vehicle:read",
            "trip:read",
            "dispatch:read",
            "maintenance:read"
        ]
    },
    "logistics": {
        "admin": [
            "vehicle:*",
            "trip:*",
            "dispatch:*",
            "route:*",
            "warehouse:*",
            "user:*",
            "system:*"
        ],
        "supervisor": [
            "vehicle:read",
            "vehicle:update",
            "trip:*",
            "dispatch:*",
            "route:read",
            "route:create",
            "warehouse:read",
            "user:read"
        ],
        "operator": [
            "vehicle:read",
            "trip:read",
            "trip:create",
            "dispatch:read",
            "dispatch:create",
            "route:read",
            "warehouse:read"
        ],
        "viewer": [
            "vehicle:read",
            "trip:read",
            "dispatch:read",
            "route:read",
            "warehouse:read"
        ]
    },
    "ridehail": {
        "admin": [
            "vehicle:*",
            "trip:*",
            "dispatch:*",
            "passenger:*",
            "payment:*",
            "user:*",
            "system:*"
        ],
        "supervisor": [
            "vehicle:read",
            "vehicle:update",
            "trip:*",
            "dispatch:*",
            "passenger:read",
            "passenger:update",
            "payment:read",
            "user:read"
        ],
        "operator": [
            "vehicle:read",
            "trip:read",
            "trip:create",
            "dispatch:read",
            "dispatch:create",
            "passenger:read"
        ],
        "viewer": [
            "vehicle:read",
            "trip:read",
            "dispatch:read",
            "passenger:read"
        ]
    }
}

# Get effective permissions for user based on roles and sector
effective_permissions(user_roles, user_sector) := permissions if {
    user_sector in sector_role_permissions
    permissions := {permission |
        some role in user_roles
        role in sector_role_permissions[user_sector]
        some permission in sector_role_permissions[user_sector][role]
    }
}

# Time-based access control
allow if {
    input.user.roles
    time_based_access_allowed(input.user, input.resource, input.action)
}

# Check if access is allowed based on time constraints
time_based_access_allowed(user, resource, action) if {
    # Allow 24/7 access for critical roles
    "admin" in user.roles
}

time_based_access_allowed(user, resource, action) if {
    # Allow supervisor access during extended hours (6 AM - 10 PM)
    "supervisor" in user.roles
    current_hour := time.now_ns() / 1000000000 / 3600 % 24
    current_hour >= 6
    current_hour <= 22
}

time_based_access_allowed(user, resource, action) if {
    # Allow operator access during business hours (8 AM - 6 PM)
    "operator" in user.roles
    current_hour := time.now_ns() / 1000000000 / 3600 % 24
    current_hour >= 8
    current_hour <= 18
}

# Emergency access override
allow if {
    input.emergency_override == true
    input.user.roles
    "supervisor" in input.user.roles
    input.justification
}

# Audit logging for all authorization decisions
audit_log := {
    "timestamp": time.now_ns(),
    "user_id": input.user.id,
    "user_email": input.user.email,
    "user_roles": input.user.roles,
    "user_sector": input.user.sector,
    "resource": input.resource,
    "action": input.action,
    "decision": allow,
    "reason": decision_reason
}

# Provide reason for authorization decision
decision_reason := "admin_access" if {
    "admin" in input.user.roles
}

decision_reason := "role_based_access" if {
    not "admin" in input.user.roles
    user_has_role(input.user.roles, input.required_role)
}

decision_reason := "permission_based_access" if {
    not "admin" in input.user.roles
    not user_has_role(input.user.roles, input.required_role)
    user_has_permission(input.user.permissions, input.required_permission)
}

decision_reason := "owner_access" if {
    input.resource.owner_id == input.user.id
    input.action in ["read", "update"]
}

decision_reason := "sector_access" if {
    input.user.sector == input.resource.sector
    user_has_permission(input.user.permissions, input.required_permission)
}

decision_reason := "service_access" if {
    input.user.type == "service"
    service_has_permission(input.user.service_name, input.required_permission)
}

decision_reason := "emergency_override" if {
    input.emergency_override == true
    "supervisor" in input.user.roles
}

decision_reason := "access_denied" if {
    not allow
}
