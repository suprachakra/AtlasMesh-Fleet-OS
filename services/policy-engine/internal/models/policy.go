package models

import (
	"encoding/json"
	"time"
)

// Policy represents a policy definition
// Content stores the Rego module as a string (JSON persisted as TEXT/JSONB)
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

// Clone returns a deep copy of the policy (useful when mutating metadata)
func (p *Policy) Clone() *Policy {
	if p == nil {
		return nil
	}
	clone := *p
	if p.Metadata != nil {
		clone.Metadata = make(map[string]interface{}, len(p.Metadata))
		for k, v := range p.Metadata {
			clone.Metadata[k] = v
		}
	}
	if p.Tags != nil {
		clone.Tags = append([]string(nil), p.Tags...)
	}
	return &clone
}

func (p *Policy) ContentBytes() []byte {
	return []byte(p.Content)
}

func (p *Policy) SetContentBytes(b []byte) {
	p.Content = string(b)
}

// Utility helpers to marshal/unmarshal arbitrary metadata slices
func ToJSON(v interface{}) []byte {
	if v == nil {
		return []byte("{}")
	}
	b, err := json.Marshal(v)
	if err != nil {
		return []byte("{}")
	}
	return b
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

func PolicyTypeFromString(value string) PolicyType {
	switch value {
	case "ODD":
		return PolicyTypeODD
	case "SAFETY":
		return PolicyTypeSafety
	case "ROUTING":
		return PolicyTypeRouting
	case "DISPATCH":
		return PolicyTypeDispatch
	case "REGULATORY":
		return PolicyTypeRegulatory
	case "SECTOR":
		return PolicyTypeSector
	case "VEHICLE":
		return PolicyTypeVehicle
	case "ENVIRONMENTAL":
		return PolicyTypeEnvironmental
	case "SECURITY":
		return PolicyTypeSecurity
	case "PRIVACY":
		return PolicyTypePrivacy
	default:
		return PolicyTypeUnspecified
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

func PolicyStatusFromString(value string) PolicyStatus {
	switch value {
	case "DRAFT":
		return PolicyStatusDraft
	case "ACTIVE":
		return PolicyStatusActive
	case "INACTIVE":
		return PolicyStatusInactive
	case "DEPRECATED":
		return PolicyStatusDeprecated
	case "ARCHIVED":
		return PolicyStatusArchived
	default:
		return PolicyStatusUnspecified
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

func PolicyScopeFromString(value string) PolicyScope {
	switch value {
	case "GLOBAL":
		return PolicyScopeGlobal
	case "TENANT":
		return PolicyScopeTenant
	case "SECTOR":
		return PolicyScopeSector
	case "SITE":
		return PolicyScopeSite
	case "VEHICLE":
		return PolicyScopeVehicle
	default:
		return PolicyScopeUnspecified
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

func PolicyPriorityFromString(value string) PolicyPriority {
	switch value {
	case "LOW":
		return PolicyPriorityLow
	case "MEDIUM":
		return PolicyPriorityMedium
	case "HIGH":
		return PolicyPriorityHigh
	case "CRITICAL":
		return PolicyPriorityCritical
	default:
		return PolicyPriorityUnspecified
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

func DecisionResultFromString(value string) DecisionResult {
	switch value {
	case "ALLOW":
		return DecisionResultAllow
	case "DENY":
		return DecisionResultDeny
	case "ABSTAIN":
		return DecisionResultAbstain
	case "ERROR":
		return DecisionResultError
	default:
		return DecisionResultUnspecified
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

func DecisionConfidenceFromString(value string) DecisionConfidence {
	switch value {
	case "LOW":
		return DecisionConfidenceLow
	case "MEDIUM":
		return DecisionConfidenceMedium
	case "HIGH":
		return DecisionConfidenceHigh
	case "ABSOLUTE":
		return DecisionConfidenceAbsolute
	default:
		return DecisionConfidenceUnspecified
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

func ViolationSeverityFromString(value string) ViolationSeverity {
	switch value {
	case "INFO":
		return ViolationSeverityInfo
	case "WARNING":
		return ViolationSeverityWarning
	case "ERROR":
		return ViolationSeverityError
	case "CRITICAL":
		return ViolationSeverityCritical
	default:
		return ViolationSeverityUnspecified
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

func AuditTypeFromString(value string) AuditType {
	switch value {
	case "EVALUATION":
		return AuditTypeEvaluation
	case "POLICY_CHANGE":
		return AuditTypePolicyChange
	case "SYSTEM_EVENT":
		return AuditTypeSystemEvent
	case "SECURITY_EVENT":
		return AuditTypeSecurityEvent
	default:
		return AuditTypeUnspecified
	}
}
