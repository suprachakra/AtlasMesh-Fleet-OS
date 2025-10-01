package models

import (
	"time"
)

// Policy represents a policy definition
type Policy struct {
	ID          string                 `json:"id" db:"id"`
	Name        string                 `json:"name" db:"name"`
	Description string                 `json:"description" db:"description"`
	Type        PolicyType             `json:"type" db:"type"`
	Content     string                 `json:"content" db:"content"`
	Status      PolicyStatus           `json:"status" db:"status"`
	Tags        []string               `json:"tags" db:"tags"`
	Scope       PolicyScope            `json:"scope" db:"scope"`
	Priority    PolicyPriority         `json:"priority" db:"priority"`
	Metadata    map[string]interface{} `json:"metadata" db:"metadata"`
	Version     int                    `json:"version" db:"version"`
	CreatedAt   time.Time              `json:"created_at" db:"created_at"`
	UpdatedAt   time.Time              `json:"updated_at" db:"updated_at"`
	CreatedBy   string                 `json:"created_by" db:"created_by"`
	UpdatedBy   string                 `json:"updated_by" db:"updated_by"`
}

// PolicyDecision represents the result of a policy evaluation
type PolicyDecision struct {
	Result          DecisionResult         `json:"result"`
	Reason          string                 `json:"reason"`
	Evidence        map[string]interface{} `json:"evidence"`
	ApplicableRules []string               `json:"applicable_rules"`
	Confidence      DecisionConfidence     `json:"confidence"`
}

// PolicyViolation represents a policy violation
type PolicyViolation struct {
	RuleID          string            `json:"rule_id"`
	Message         string            `json:"message"`
	Severity        ViolationSeverity `json:"severity"`
	Context         map[string]interface{} `json:"context"`
	SuggestedAction string            `json:"suggested_action"`
}

// EvaluationContext provides context for policy evaluation
type EvaluationContext struct {
	RequestID   string                 `json:"request_id"`
	UserID      string                 `json:"user_id"`
	TenantID    string                 `json:"tenant_id"`
	Sector      string                 `json:"sector"`
	Environment string                 `json:"environment"`
	Timestamp   time.Time              `json:"timestamp"`
	Additional  map[string]interface{} `json:"additional"`
}

// AuditEntry represents an audit log entry
type AuditEntry struct {
	ID        string                 `json:"id" db:"id"`
	Type      AuditType              `json:"type" db:"type"`
	Timestamp time.Time              `json:"timestamp" db:"timestamp"`
	UserID    string                 `json:"user_id" db:"user_id"`
	TenantID  string                 `json:"tenant_id" db:"tenant_id"`
	Action    string                 `json:"action" db:"action"`
	Resource  string                 `json:"resource" db:"resource"`
	Details   map[string]interface{} `json:"details" db:"details"`
	Signature string                 `json:"signature" db:"signature"`
}

// Enums

type PolicyType int

const (
	PolicyTypeUnspecified PolicyType = iota
	PolicyTypeODD
	PolicyTypeSafety
	PolicyTypeRouting
	PolicyTypeDispatch
	PolicyTypeRegulatory
	PolicyTypeSector
	PolicyTypeVehicle
	PolicyTypeEnvironmental
	PolicyTypeSecurity
	PolicyTypePrivacy
)

func (pt PolicyType) String() string {
	switch pt {
	case PolicyTypeODD:
		return "ODD"
	case PolicyTypeSafety:
		return "SAFETY"
	case PolicyTypeRouting:
		return "ROUTING"
	case PolicyTypeDispatch:
		return "DISPATCH"
	case PolicyTypeRegulatory:
		return "REGULATORY"
	case PolicyTypeSector:
		return "SECTOR"
	case PolicyTypeVehicle:
		return "VEHICLE"
	case PolicyTypeEnvironmental:
		return "ENVIRONMENTAL"
	case PolicyTypeSecurity:
		return "SECURITY"
	case PolicyTypePrivacy:
		return "PRIVACY"
	default:
		return "UNSPECIFIED"
	}
}

type PolicyStatus int

const (
	PolicyStatusUnspecified PolicyStatus = iota
	PolicyStatusDraft
	PolicyStatusActive
	PolicyStatusInactive
	PolicyStatusDeprecated
	PolicyStatusArchived
)

func (ps PolicyStatus) String() string {
	switch ps {
	case PolicyStatusDraft:
		return "DRAFT"
	case PolicyStatusActive:
		return "ACTIVE"
	case PolicyStatusInactive:
		return "INACTIVE"
	case PolicyStatusDeprecated:
		return "DEPRECATED"
	case PolicyStatusArchived:
		return "ARCHIVED"
	default:
		return "UNSPECIFIED"
	}
}

type PolicyScope int

const (
	PolicyScopeUnspecified PolicyScope = iota
	PolicyScopeGlobal
	PolicyScopeTenant
	PolicyScopeSector
	PolicyScopeSite
	PolicyScopeVehicle
)

func (ps PolicyScope) String() string {
	switch ps {
	case PolicyScopeGlobal:
		return "GLOBAL"
	case PolicyScopeTenant:
		return "TENANT"
	case PolicyScopeSector:
		return "SECTOR"
	case PolicyScopeSite:
		return "SITE"
	case PolicyScopeVehicle:
		return "VEHICLE"
	default:
		return "UNSPECIFIED"
	}
}

type PolicyPriority int

const (
	PolicyPriorityUnspecified PolicyPriority = iota
	PolicyPriorityLow
	PolicyPriorityMedium
	PolicyPriorityHigh
	PolicyPriorityCritical
)

func (pp PolicyPriority) String() string {
	switch pp {
	case PolicyPriorityLow:
		return "LOW"
	case PolicyPriorityMedium:
		return "MEDIUM"
	case PolicyPriorityHigh:
		return "HIGH"
	case PolicyPriorityCritical:
		return "CRITICAL"
	default:
		return "UNSPECIFIED"
	}
}

type DecisionResult int

const (
	DecisionResultUnspecified DecisionResult = iota
	DecisionResultAllow
	DecisionResultDeny
	DecisionResultAbstain
	DecisionResultError
)

func (dr DecisionResult) String() string {
	switch dr {
	case DecisionResultAllow:
		return "ALLOW"
	case DecisionResultDeny:
		return "DENY"
	case DecisionResultAbstain:
		return "ABSTAIN"
	case DecisionResultError:
		return "ERROR"
	default:
		return "UNSPECIFIED"
	}
}

type DecisionConfidence int

const (
	DecisionConfidenceUnspecified DecisionConfidence = iota
	DecisionConfidenceLow
	DecisionConfidenceMedium
	DecisionConfidenceHigh
	DecisionConfidenceAbsolute
)

func (dc DecisionConfidence) String() string {
	switch dc {
	case DecisionConfidenceLow:
		return "LOW"
	case DecisionConfidenceMedium:
		return "MEDIUM"
	case DecisionConfidenceHigh:
		return "HIGH"
	case DecisionConfidenceAbsolute:
		return "ABSOLUTE"
	default:
		return "UNSPECIFIED"
	}
}

type ViolationSeverity int

const (
	ViolationSeverityUnspecified ViolationSeverity = iota
	ViolationSeverityInfo
	ViolationSeverityWarning
	ViolationSeverityError
	ViolationSeverityCritical
)

func (vs ViolationSeverity) String() string {
	switch vs {
	case ViolationSeverityInfo:
		return "INFO"
	case ViolationSeverityWarning:
		return "WARNING"
	case ViolationSeverityError:
		return "ERROR"
	case ViolationSeverityCritical:
		return "CRITICAL"
	default:
		return "UNSPECIFIED"
	}
}

type AuditType int

const (
	AuditTypeUnspecified AuditType = iota
	AuditTypeEvaluation
	AuditTypePolicyChange
	AuditTypeSystemEvent
	AuditTypeSecurityEvent
)

func (at AuditType) String() string {
	switch at {
	case AuditTypeEvaluation:
		return "EVALUATION"
	case AuditTypePolicyChange:
		return "POLICY_CHANGE"
	case AuditTypeSystemEvent:
		return "SYSTEM_EVENT"
	case AuditTypeSecurityEvent:
		return "SECURITY_EVENT"
	default:
		return "UNSPECIFIED"
	}
}
