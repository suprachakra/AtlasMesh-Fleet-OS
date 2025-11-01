package enforcer

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"time"

	"github.com/atlasmesh/fleet-os/services/variant-budget/internal/budget"
)

// Enforcer enforces variant budget policies and triggers CCB workflow.
type Enforcer struct {
	config  Config
	tracker *budget.Tracker
}

// Config holds enforcer configuration.
type Config struct {
	Enabled        bool
	CCBWebhookURL  string
	MetricsEnabled bool
	AlertThresholds map[string]float64
}

// EnforcementResult represents the result of budget enforcement.
type EnforcementResult struct {
	AllowMerge      bool                   `json:"allow_merge"`
	BlockReason     string                 `json:"block_reason,omitempty"`
	Violations      []BudgetViolation      `json:"violations"`
	Warnings        []BudgetWarning        `json:"warnings"`
	CCBTriggered    bool                   `json:"ccb_triggered"`
	EnforcedAt      time.Time              `json:"enforced_at"`
	Metadata        map[string]interface{} `json:"metadata"`
}

// BudgetViolation represents a hard budget limit violation.
type BudgetViolation struct {
	Dimension       string  `json:"dimension"`
	CurrentDelta    float64 `json:"current_delta_pct"`
	Limit           float64 `json:"limit_pct"`
	Excess          float64 `json:"excess_pct"`
	RequiresAction  string  `json:"requires_action"`
}

// BudgetWarning represents a soft budget limit breach.
type BudgetWarning struct {
	Dimension      string  `json:"dimension"`
	CurrentDelta   float64 `json:"current_delta_pct"`
	SoftLimit      float64 `json:"soft_limit_pct"`
	Recommendation string  `json:"recommendation"`
}

// CCBRequest represents a Change Control Board review request.
type CCBRequest struct {
	RequestID       string              `json:"request_id"`
	CommitSHA       string              `json:"commit_sha"`
	Violations      []BudgetViolation   `json:"violations"`
	Requester       string              `json:"requester"`
	Justification   string              `json:"justification"`
	ImpactAnalysis  string              `json:"impact_analysis"`
	RemediationPlan string              `json:"remediation_plan"`
	CreatedAt       time.Time           `json:"created_at"`
}

// New creates a new policy enforcer.
func New(cfg Config, tracker *budget.Tracker) *Enforcer {
	return &Enforcer{
		config:  cfg,
		tracker: tracker,
	}
}

// Enforce evaluates budget compliance and determines if merge/deployment is allowed.
func (e *Enforcer) Enforce(ctx context.Context, commitSHA string) (*EnforcementResult, error) {
	if !e.config.Enabled {
		return &EnforcementResult{
			AllowMerge:   true,
			EnforcedAt:   time.Now(),
			CCBTriggered: false,
			Metadata: map[string]interface{}{
				"enforcement_disabled": true,
			},
		}, nil
	}

	// Get overall budget status
	status := e.tracker.GetOverallBudgetStatus()

	result := &EnforcementResult{
		AllowMerge:   true,
		Violations:   []BudgetViolation{},
		Warnings:     []BudgetWarning{},
		CCBTriggered: false,
		EnforcedAt:   time.Now(),
		Metadata: map[string]interface{}{
			"commit_sha":       commitSHA,
			"overall_status":   status.OverallStatus,
			"violation_count":  status.ViolationCount,
			"warning_count":    status.WarningCount,
		},
	}

	// Check each dimension for violations
	for dimension, budgetStatus := range status.AllDimensions {
		// Hard limit violation
		if budgetStatus.CurrentDelta >= budgetStatus.Limit {
			result.AllowMerge = false
			result.Violations = append(result.Violations, BudgetViolation{
				Dimension:      dimension,
				CurrentDelta:   budgetStatus.CurrentDelta,
				Limit:          budgetStatus.Limit,
				Excess:         budgetStatus.CurrentDelta - budgetStatus.Limit,
				RequiresAction: "CCB approval required",
			})
		} else if budgetStatus.CurrentDelta >= budgetStatus.SoftLimit {
			// Soft limit warning
			result.Warnings = append(result.Warnings, BudgetWarning{
				Dimension:      dimension,
				CurrentDelta:   budgetStatus.CurrentDelta,
				SoftLimit:      budgetStatus.SoftLimit,
				Recommendation: fmt.Sprintf("Consider refactoring to reduce %s delta", dimension),
			})
		}
	}

	// If violations exist, trigger CCB workflow
	if len(result.Violations) > 0 {
		result.CCBTriggered = true
		result.BlockReason = fmt.Sprintf("Budget violations detected in %d dimension(s). CCB approval required.", len(result.Violations))

		// Send CCB notification
		if err := e.triggerCCB(ctx, commitSHA, result.Violations); err != nil {
			// Log error but don't fail enforcement
			result.Metadata["ccb_trigger_error"] = err.Error()
		}
	}

	return result, nil
}

// triggerCCB sends a notification to the Change Control Board.
func (e *Enforcer) triggerCCB(ctx context.Context, commitSHA string, violations []BudgetViolation) error {
	if e.config.CCBWebhookURL == "" {
		return nil // CCB webhook not configured
	}

	ccbRequest := CCBRequest{
		RequestID:       fmt.Sprintf("ccb-%s-%d", commitSHA[:8], time.Now().Unix()),
		CommitSHA:       commitSHA,
		Violations:      violations,
		Requester:       "variant-budget-service",
		Justification:   "Automated variant budget violation detection",
		ImpactAnalysis:  fmt.Sprintf("%d dimension(s) exceeded budget limits", len(violations)),
		RemediationPlan: "Manual review required by Change Control Board",
		CreatedAt:       time.Now(),
	}

	// Send webhook notification
	payload, err := json.Marshal(ccbRequest)
	if err != nil {
		return fmt.Errorf("failed to marshal CCB request: %w", err)
	}

	req, err := http.NewRequestWithContext(ctx, "POST", e.config.CCBWebhookURL, bytes.NewBuffer(payload))
	if err != nil {
		return fmt.Errorf("failed to create CCB request: %w", err)
	}

	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("X-Event-Type", "variant-budget-violation")

	client := &http.Client{Timeout: 10 * time.Second}
	resp, err := client.Do(req)
	if err != nil {
		return fmt.Errorf("failed to send CCB notification: %w", err)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK && resp.StatusCode != http.StatusAccepted {
		return fmt.Errorf("CCB webhook returned non-success status: %d", resp.StatusCode)
	}

	return nil
}

// ApproveException records a CCB-approved budget exception.
func (e *Enforcer) ApproveException(ctx context.Context, commitSHA, approver, justification string) error {
	// TODO: Store exception in database/audit log
	return nil
}

// RejectException records a CCB-rejected budget exception.
func (e *Enforcer) RejectException(ctx context.Context, commitSHA, rejector, reason string) error {
	// TODO: Store rejection in database/audit log
	return nil
}

